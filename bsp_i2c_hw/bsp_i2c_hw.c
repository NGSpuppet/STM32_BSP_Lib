/*
*程序说明：	该程序为硬件I2C的支持程序
*			阻塞传输和DMA传输已经可以使用
*			中断传输存在问题无法传输
*建议：I2C对传感器进行小数据传输优先使用阻塞传输
*	   中断已经可以使用，推荐用在突发的传感器数据读取
*	   对EEPROM存储器大数据传输可以用DMA传输（要注意不要让局部变量过早释放）	
*/
#include "bsp.h"
#include "bsp_i2c_hw.h"

/************************** 变量宏定义 ****************************/
I2C_HandleTypeDef hi2c1;				/* 硬件I2C结构体 */


#ifndef I2C_BLOCKING_MODE
/*
*因为非阻塞传输过快会导致局部变量释放
*所有在这里开辟缓存区来存放需要传输的变量
*/
static __IO uint8_t I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_IDLE;	/* 硬件I2C状态 */
uint8_t I2C_RX_DATA[I2C_DATA_BUFFER];	/* 接收缓存 */
uint8_t I2C_TX_DATA[I2C_DATA_BUFFER];	/* 发送缓存 */

#ifdef I2C_DMA_MODE
DMA_HandleTypeDef hdma_i2c1_rx;			/* I2C接收DMA结构体 */
DMA_HandleTypeDef hdma_i2c1_tx;			/* I2C发送DMA结构体 */
#endif
#endif

/************************* I2C 初始化配置 **************************/
/*
*函数说明：	硬件I2C配置初始化
*输入形参：	无
*输出数据：	无
*/
void bsp_I2C_HW_Init(void)		
{
	/* 使能I2C1时钟 */
	__HAL_RCC_I2C1_CLK_ENABLE();

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00E00D29;				/* cubemx生成(1000KHz) */
	hi2c1.Init.OwnAddress1 = 0;					/* 第一地址 */
	hi2c1.Init.OwnAddress2 = 0;					/* 第二地址（单地址不用配置） */
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;		/* 7位本地地址 */
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;		/* 失能双地址 */
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;				/* 单地址不用配置 */
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;		/* 失能广播模式 */
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;			/* 失能时钟不拉伸 */
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
	  Error_Handler(__FILE__, __LINE__);
	}

	/* 配置模拟滤波 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
	  Error_Handler(__FILE__, __LINE__);
	}

	/* 配置数字滤波 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 5) != HAL_OK)
	{
	  Error_Handler(__FILE__, __LINE__);
	}
}

/*
*函数说明：	I2C初始化调用
*调用条件：	“HAL_I2C_Init()”会自动调用本函数
*输入形参：	i2cHandle：	I2C hal库结构体
*输出数据：	无
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();		/* 使能PB端口时钟 */
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#ifdef I2C_IT_MODE
    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 13, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 13, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
#endif

#ifdef I2C_DMA_MODE

	__HAL_RCC_DMA1_CLK_ENABLE();		/* 使能DMA1时钟 */
    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Stream0;							/* DMA1数据流0 */
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_I2C1_RX;				/* I2C1接收触发 */
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;				/* 外设到寄存器 */
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;					/* 外设地址不增 */
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;						/* 寄存器地址增 */
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;	/* 单字节传输 */
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;		/* 单字节传输 */
    hdma_i2c1_rx.Init.Mode = DMA_CIRCULAR;							
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;					/* 优先级低 */
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;				/* 不适用FIFO */
    
	/* 配置DMA */
	if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler(__FILE__, __LINE__);
    }

	/* 将DMA1数据流0映射到I2C1结构体 */
    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Stream1;							/* DMA1数据流0 */
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_I2C1_TX;				/* I2C1接收触发 */
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;				/* 外设到寄存器 */
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;					/* 外设地址不增 */
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;						/* 寄存器地址增 */
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;	/* 单字节传输 */
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;		/* 单字节传输 */
    hdma_i2c1_tx.Init.Mode = DMA_CIRCULAR;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;					/* 优先级低 */
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;				/* 不适用FIFO */
    
	/* 配置DMA */
	if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      Error_Handler(__FILE__, __LINE__);
    }

	/* 将DMA1数据流1映射到I2C1结构体 */
    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

	/* DMA一定要开启的中断 */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 13, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

	/* 配置DMA中断优先级 */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 13, 0);
	/* 使能DMA */
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

	/* 配置DMA中断优先级 */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 13, 0);
	/* 使能DMA */
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
#endif
  }
}

/*
*函数说明：	I2C复位初始化调用
*调用条件：	“HAL_I2C_DeInit()”会自动调用本函数
*输入形参：	i2cHandle：	I2C hal库结构体
*输出数据：	无	
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {

    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

#ifdef I2C_IT_MODE
	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
#endif

#ifdef I2C_DMA_MODE
	/* DMA一定要开启的中断 */
	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);
#endif
  }
}

/**************************** I2C HAL库重封装 *****************************/
/*
*函数说明：	检测I2C总线设备
*补充说明：	函数设定的是10次多次检测，0xFF的超时值
*输入形参：	i2c_addr：设备的I2C总线地址(经过左移)
*输出数据：	I2C_HW_OK：检测到设备	I2C_HW_ERROR：未检测到设备
*/
I2C_HW_STATE bsp_I2C_HW_CheckDevice(uint8_t i2c_addr)
{
	if(HAL_I2C_IsDeviceReady(&hi2c1, i2c_addr, 5, 0XFFFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
}

/*
*函数说明：	使用硬件I2C向从地址寄存器读1个字节数据
*输入形参：	i2c_addr：设备的I2C总线地址(经过左移)
*			REG_addr：寄存器地址
*			*read_data：读取数据的存放指针
*输出数据：	I2C_HW_OK：成功	I2C_HW_ERROR：失败
*/
I2C_HW_STATE bsp_I2C_HW_Read_Byte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *read_data)
{
#ifdef I2C_BLOCKING_MODE		/* 阻塞传输模式 */
	if(HAL_I2C_Mem_Read(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, 1, 0xFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_IT_MODE				/* 中断传输模式 */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* 判断现在的状态 */
		return I2C_HW_ERROR;	/* 表示上一次未发送完 */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* 将状态改为忙 */

	if(HAL_I2C_Mem_Read_IT(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, 1) == HAL_OK)
	{
		// while(I2C_NO_BLOCKING_STATE != I2C_NO_BLOCKING_IDLE);
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_DMA_MODE				/* DMA传输莫 */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* 判断现在的状态 */
		return I2C_HW_ERROR;	/* 表示上一次未发送完 */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* 将状态改为忙 */

	if(HAL_I2C_Mem_Read_DMA(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, 1) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif
}

/*
*函数说明：	使用硬件I2C向从地址寄存器写1个字节数据
*输入形参：	i2c_addr：设备的I2C总线地址(经过左移)
*			REG_addr：寄存器地址
*			write_data：要写入的数据
*输出数据：	I2C_HW_OK：成功	I2C_HW_ERROR：失败
*/
I2C_HW_STATE bsp_I2C_HW_Write_Byte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t write_data)
{
#ifdef I2C_BLOCKING_MODE		/* 阻塞传输模式 */
	if(HAL_I2C_Mem_Write(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, &write_data, 1, 0xFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_IT_MODE				/* 中断传输模式 */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* 判断现在的状态 */
		return I2C_HW_ERROR;	/* 表示上一次未发送完 */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* 将状态改为忙 */

	if(HAL_I2C_Mem_Write_IT(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, &write_data, 1) == HAL_OK)
	{
		// while(I2C_NO_BLOCKING_STATE != I2C_NO_BLOCKING_IDLE);
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;

#endif

#ifdef I2C_DMA_MODE				/* MDA传输模式 */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* 判断现在的状态 */
		return I2C_HW_ERROR;	/* 表示上一次未发送完 */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* 将状态改为忙 */

	if(HAL_I2C_Mem_Write_DMA(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, &write_data, 1) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;

#endif
}

/*
*函数说明：	使用硬件I2C向从地址寄存器读n个字节数据
*输入形参：	i2c_addr：设备的I2C总线地址(经过左移)
*			REG_addr：寄存器地址
*			*read_data：读取数据的存放指针
*			Length：写入数量（1~65535）
*输出数据：	I2C_HW_OK：成功	I2C_HW_ERROR：失败
*/
I2C_HW_STATE bsp_I2C_HW_Read_nByte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *read_data, uint16_t Length)
{
#ifdef I2C_BLOCKING_MODE		/* 阻塞传输模式 */
	if(HAL_I2C_Mem_Read(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, Length, 0xFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_IT_MODE				/* 中断传输模式 */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* 判断现在的状态 */
		return I2C_HW_ERROR;	/* 表示上一次未发送完 */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* 将状态改为忙 */

	if(HAL_I2C_Mem_Read_IT(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, Length) == HAL_OK)
	{
		// while(I2C_NO_BLOCKING_STATE != I2C_NO_BLOCKING_IDLE);
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_DMA_MODE				/* DMA传输莫 */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* 判断现在的状态 */
		return I2C_HW_ERROR;	/* 表示上一次未发送完 */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* 将状态改为忙 */

	if(HAL_I2C_Mem_Read_DMA(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, Length) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif
}


/*
*函数说明：	使用硬件I2C向从地址寄存器写n个字节数据
*输入形参：	i2c_addr：设备的I2C总线地址(经过左移)
*			REG_addr：寄存器地址
*			write_data：要写入的数据
*			Length：写入数量（1~65535）
*输出数据：	I2C_HW_OK：成功	I2C_HW_ERROR：失败
*/
I2C_HW_STATE bsp_I2C_HW_Write_nByte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *write_data, uint16_t Length)
{
#ifdef I2C_BLOCKING_MODE		/* 阻塞传输模式 */
	if(HAL_I2C_Mem_Write(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, write_data, Length, 0xFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_IT_MODE				/* 中断传输模式 */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* 判断现在的状态 */
		return I2C_HW_ERROR;	/* 表示上一次未发送完 */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* 将状态改为忙 */

	if(HAL_I2C_Mem_Write_IT(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, write_data, Length) == HAL_OK)
	{
		while(I2C_NO_BLOCKING_STATE != I2C_NO_BLOCKING_IDLE);
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_DMA_MODE				/* MDA传输模式 */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* 判断现在的状态 */
		return I2C_HW_ERROR;	/* 表示上一次未发送完 */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* 将状态改为忙 */

	if(HAL_I2C_Mem_Write_DMA(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, write_data, Length) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif
}


/**************************** I2C 中断服务函数 *****************************/

#ifdef I2C_IT_MODE		/* 中断传输用到的中断 */
/* 事件中断 */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}
/* 错误中断 */
void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c1);
}
#endif


#ifdef I2C_DMA_MODE		/* DMA传输用到的中断 */

/* DMA一定要开启的中断 */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}
/* DMA1数据流0的中断服务函数 */
void DMA1_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(hi2c1.hdmarx);
}

/* DMA1数据流1的中断服务函数 */
void DMA1_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(hi2c1.hdmatx);
}
#endif

/**************************** I2C 回调函数 *****************************/
#ifndef I2C_BLOCKING_MODE
/* 发送完成回调函数 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_IDLE;	/* 将状态改为空闲 */
}
/* 接收完成回调函数 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_IDLE;	/* 将状态改为空闲 */
}
#endif

/* I2C错误回调函数 */
void HAL_I2C_HW_ERRORCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1)
		Error_Handler(__FILE__, __LINE__);	/* 发生错误 */
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
