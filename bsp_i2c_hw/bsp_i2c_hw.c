/*
*����˵����	�ó���ΪӲ��I2C��֧�ֳ���
*			���������DMA�����Ѿ�����ʹ��
*			�жϴ�����������޷�����
*���飺I2C�Դ���������С���ݴ�������ʹ����������
*	   �ж��Ѿ�����ʹ�ã��Ƽ�����ͻ���Ĵ��������ݶ�ȡ
*	   ��EEPROM�洢�������ݴ��������DMA���䣨Ҫע�ⲻҪ�þֲ����������ͷţ�	
*/
#include "bsp.h"
#include "bsp_i2c_hw.h"

/************************** �����궨�� ****************************/
I2C_HandleTypeDef hi2c1;				/* Ӳ��I2C�ṹ�� */


#ifndef I2C_BLOCKING_MODE
/*
*��Ϊ�������������ᵼ�¾ֲ������ͷ�
*���������￪�ٻ������������Ҫ����ı���
*/
static __IO uint8_t I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_IDLE;	/* Ӳ��I2C״̬ */
uint8_t I2C_RX_DATA[I2C_DATA_BUFFER];	/* ���ջ��� */
uint8_t I2C_TX_DATA[I2C_DATA_BUFFER];	/* ���ͻ��� */

#ifdef I2C_DMA_MODE
DMA_HandleTypeDef hdma_i2c1_rx;			/* I2C����DMA�ṹ�� */
DMA_HandleTypeDef hdma_i2c1_tx;			/* I2C����DMA�ṹ�� */
#endif
#endif

/************************* I2C ��ʼ������ **************************/
/*
*����˵����	Ӳ��I2C���ó�ʼ��
*�����βΣ�	��
*������ݣ�	��
*/
void bsp_I2C_HW_Init(void)		
{
	/* ʹ��I2C1ʱ�� */
	__HAL_RCC_I2C1_CLK_ENABLE();

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00E00D29;				/* cubemx����(1000KHz) */
	hi2c1.Init.OwnAddress1 = 0;					/* ��һ��ַ */
	hi2c1.Init.OwnAddress2 = 0;					/* �ڶ���ַ������ַ�������ã� */
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;		/* 7λ���ص�ַ */
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;		/* ʧ��˫��ַ */
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;				/* ����ַ�������� */
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;		/* ʧ�ܹ㲥ģʽ */
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;			/* ʧ��ʱ�Ӳ����� */
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
	  Error_Handler(__FILE__, __LINE__);
	}

	/* ����ģ���˲� */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
	  Error_Handler(__FILE__, __LINE__);
	}

	/* ���������˲� */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 5) != HAL_OK)
	{
	  Error_Handler(__FILE__, __LINE__);
	}
}

/*
*����˵����	I2C��ʼ������
*����������	��HAL_I2C_Init()�����Զ����ñ�����
*�����βΣ�	i2cHandle��	I2C hal��ṹ��
*������ݣ�	��
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();		/* ʹ��PB�˿�ʱ�� */
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

	__HAL_RCC_DMA1_CLK_ENABLE();		/* ʹ��DMA1ʱ�� */
    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Stream0;							/* DMA1������0 */
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_I2C1_RX;				/* I2C1���մ��� */
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;				/* ���赽�Ĵ��� */
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;					/* �����ַ���� */
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;						/* �Ĵ�����ַ�� */
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;	/* ���ֽڴ��� */
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;		/* ���ֽڴ��� */
    hdma_i2c1_rx.Init.Mode = DMA_CIRCULAR;							
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;					/* ���ȼ��� */
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;				/* ������FIFO */
    
	/* ����DMA */
	if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler(__FILE__, __LINE__);
    }

	/* ��DMA1������0ӳ�䵽I2C1�ṹ�� */
    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Stream1;							/* DMA1������0 */
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_I2C1_TX;				/* I2C1���մ��� */
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;				/* ���赽�Ĵ��� */
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;					/* �����ַ���� */
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;						/* �Ĵ�����ַ�� */
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;	/* ���ֽڴ��� */
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;		/* ���ֽڴ��� */
    hdma_i2c1_tx.Init.Mode = DMA_CIRCULAR;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;					/* ���ȼ��� */
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;				/* ������FIFO */
    
	/* ����DMA */
	if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      Error_Handler(__FILE__, __LINE__);
    }

	/* ��DMA1������1ӳ�䵽I2C1�ṹ�� */
    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

	/* DMAһ��Ҫ�������ж� */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 13, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

	/* ����DMA�ж����ȼ� */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 13, 0);
	/* ʹ��DMA */
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

	/* ����DMA�ж����ȼ� */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 13, 0);
	/* ʹ��DMA */
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
#endif
  }
}

/*
*����˵����	I2C��λ��ʼ������
*����������	��HAL_I2C_DeInit()�����Զ����ñ�����
*�����βΣ�	i2cHandle��	I2C hal��ṹ��
*������ݣ�	��	
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
	/* DMAһ��Ҫ�������ж� */
	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);
#endif
  }
}

/**************************** I2C HAL���ط�װ *****************************/
/*
*����˵����	���I2C�����豸
*����˵����	�����趨����10�ζ�μ�⣬0xFF�ĳ�ʱֵ
*�����βΣ�	i2c_addr���豸��I2C���ߵ�ַ(��������)
*������ݣ�	I2C_HW_OK����⵽�豸	I2C_HW_ERROR��δ��⵽�豸
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
*����˵����	ʹ��Ӳ��I2C��ӵ�ַ�Ĵ�����1���ֽ�����
*�����βΣ�	i2c_addr���豸��I2C���ߵ�ַ(��������)
*			REG_addr���Ĵ�����ַ
*			*read_data����ȡ���ݵĴ��ָ��
*������ݣ�	I2C_HW_OK���ɹ�	I2C_HW_ERROR��ʧ��
*/
I2C_HW_STATE bsp_I2C_HW_Read_Byte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *read_data)
{
#ifdef I2C_BLOCKING_MODE		/* ��������ģʽ */
	if(HAL_I2C_Mem_Read(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, 1, 0xFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_IT_MODE				/* �жϴ���ģʽ */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* �ж����ڵ�״̬ */
		return I2C_HW_ERROR;	/* ��ʾ��һ��δ������ */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* ��״̬��Ϊæ */

	if(HAL_I2C_Mem_Read_IT(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, 1) == HAL_OK)
	{
		// while(I2C_NO_BLOCKING_STATE != I2C_NO_BLOCKING_IDLE);
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_DMA_MODE				/* DMA����Ī */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* �ж����ڵ�״̬ */
		return I2C_HW_ERROR;	/* ��ʾ��һ��δ������ */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* ��״̬��Ϊæ */

	if(HAL_I2C_Mem_Read_DMA(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, 1) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif
}

/*
*����˵����	ʹ��Ӳ��I2C��ӵ�ַ�Ĵ���д1���ֽ�����
*�����βΣ�	i2c_addr���豸��I2C���ߵ�ַ(��������)
*			REG_addr���Ĵ�����ַ
*			write_data��Ҫд�������
*������ݣ�	I2C_HW_OK���ɹ�	I2C_HW_ERROR��ʧ��
*/
I2C_HW_STATE bsp_I2C_HW_Write_Byte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t write_data)
{
#ifdef I2C_BLOCKING_MODE		/* ��������ģʽ */
	if(HAL_I2C_Mem_Write(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, &write_data, 1, 0xFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_IT_MODE				/* �жϴ���ģʽ */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* �ж����ڵ�״̬ */
		return I2C_HW_ERROR;	/* ��ʾ��һ��δ������ */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* ��״̬��Ϊæ */

	if(HAL_I2C_Mem_Write_IT(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, &write_data, 1) == HAL_OK)
	{
		// while(I2C_NO_BLOCKING_STATE != I2C_NO_BLOCKING_IDLE);
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;

#endif

#ifdef I2C_DMA_MODE				/* MDA����ģʽ */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* �ж����ڵ�״̬ */
		return I2C_HW_ERROR;	/* ��ʾ��һ��δ������ */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* ��״̬��Ϊæ */

	if(HAL_I2C_Mem_Write_DMA(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, &write_data, 1) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;

#endif
}

/*
*����˵����	ʹ��Ӳ��I2C��ӵ�ַ�Ĵ�����n���ֽ�����
*�����βΣ�	i2c_addr���豸��I2C���ߵ�ַ(��������)
*			REG_addr���Ĵ�����ַ
*			*read_data����ȡ���ݵĴ��ָ��
*			Length��д��������1~65535��
*������ݣ�	I2C_HW_OK���ɹ�	I2C_HW_ERROR��ʧ��
*/
I2C_HW_STATE bsp_I2C_HW_Read_nByte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *read_data, uint16_t Length)
{
#ifdef I2C_BLOCKING_MODE		/* ��������ģʽ */
	if(HAL_I2C_Mem_Read(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, Length, 0xFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_IT_MODE				/* �жϴ���ģʽ */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* �ж����ڵ�״̬ */
		return I2C_HW_ERROR;	/* ��ʾ��һ��δ������ */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* ��״̬��Ϊæ */

	if(HAL_I2C_Mem_Read_IT(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, Length) == HAL_OK)
	{
		// while(I2C_NO_BLOCKING_STATE != I2C_NO_BLOCKING_IDLE);
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_DMA_MODE				/* DMA����Ī */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* �ж����ڵ�״̬ */
		return I2C_HW_ERROR;	/* ��ʾ��һ��δ������ */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* ��״̬��Ϊæ */

	if(HAL_I2C_Mem_Read_DMA(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, read_data, Length) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif
}


/*
*����˵����	ʹ��Ӳ��I2C��ӵ�ַ�Ĵ���дn���ֽ�����
*�����βΣ�	i2c_addr���豸��I2C���ߵ�ַ(��������)
*			REG_addr���Ĵ�����ַ
*			write_data��Ҫд�������
*			Length��д��������1~65535��
*������ݣ�	I2C_HW_OK���ɹ�	I2C_HW_ERROR��ʧ��
*/
I2C_HW_STATE bsp_I2C_HW_Write_nByte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *write_data, uint16_t Length)
{
#ifdef I2C_BLOCKING_MODE		/* ��������ģʽ */
	if(HAL_I2C_Mem_Write(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, write_data, Length, 0xFF) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_IT_MODE				/* �жϴ���ģʽ */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* �ж����ڵ�״̬ */
		return I2C_HW_ERROR;	/* ��ʾ��һ��δ������ */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* ��״̬��Ϊæ */

	if(HAL_I2C_Mem_Write_IT(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, write_data, Length) == HAL_OK)
	{
		while(I2C_NO_BLOCKING_STATE != I2C_NO_BLOCKING_IDLE);
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif

#ifdef I2C_DMA_MODE				/* MDA����ģʽ */

	if(I2C_NO_BLOCKING_STATE == I2C_NO_BLOCKING_BUSY)	/* �ж����ڵ�״̬ */
		return I2C_HW_ERROR;	/* ��ʾ��һ��δ������ */

	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_BUSY;	/* ��״̬��Ϊæ */

	if(HAL_I2C_Mem_Write_DMA(&hi2c1, i2c_addr, REG_addr, I2C_MEMADD_SIZE_8BIT, write_data, Length) == HAL_OK)
	{
		return I2C_HW_OK;
	}
	else
		return I2C_HW_ERROR;
#endif
}


/**************************** I2C �жϷ����� *****************************/

#ifdef I2C_IT_MODE		/* �жϴ����õ����ж� */
/* �¼��ж� */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}
/* �����ж� */
void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c1);
}
#endif


#ifdef I2C_DMA_MODE		/* DMA�����õ����ж� */

/* DMAһ��Ҫ�������ж� */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}
/* DMA1������0���жϷ����� */
void DMA1_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(hi2c1.hdmarx);
}

/* DMA1������1���жϷ����� */
void DMA1_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(hi2c1.hdmatx);
}
#endif

/**************************** I2C �ص����� *****************************/
#ifndef I2C_BLOCKING_MODE
/* ������ɻص����� */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_IDLE;	/* ��״̬��Ϊ���� */
}
/* ������ɻص����� */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	I2C_NO_BLOCKING_STATE = I2C_NO_BLOCKING_IDLE;	/* ��״̬��Ϊ���� */
}
#endif

/* I2C����ص����� */
void HAL_I2C_HW_ERRORCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1)
		Error_Handler(__FILE__, __LINE__);	/* �������� */
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
