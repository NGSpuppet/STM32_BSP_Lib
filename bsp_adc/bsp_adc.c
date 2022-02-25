/*
*********************************************************************************************************
*
*	模块名称 : ADC驱动
*	文件名称 : bsp_adc.c
*	版    本 : V1.0
*	说    明 : ADC多通道采样
*              1. 例子默认用的PLL时钟供ADC使用，大家可以通过bsp_adc.c文件开头宏定义切换到AHB时钟。
*              2、采用DMA方式进行多通道采样，采集了PC0, Vbat/4, VrefInt和温度。
*              3、ADC配置做了一个16倍过采样，使得采样数据比较稳定，最高支持1024倍过采样。
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2018-12-12 armfly  正式发布
*
*	Copyright (C), 2018-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "bsp.h"

/* 选择ADC的时钟源 */
//#define ADC_CLOCK_SOURCE_AHB     /* 选择AHB时钟源 */
#define ADC_CLOCK_SOURCE_PLL     /* 选择PLL时钟源 */

#define adc_channel_num		6	//ADC通道数量

__IO uint16_t ADCxValues[adc_channel_num];

ADC_HandleTypeDef   AdcHandle;
DMA_HandleTypeDef   ADC1_DMA_Handle;


/*
*********************************************************************************************************
*	函 数 名: bsp_InitADC
*	功能说明: 初始化ADC，采用DMA方式进行多通道采样，采集了PC0, Vbat/4, VrefInt和温度
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitADC(void)
{
	ADC_MultiModeTypeDef multimode = {0};
//	ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

  /* ## - 1 - 配置ADC采样的时钟 ####################################### */
#if defined (ADC_CLOCK_SOURCE_PLL)
/*
	ADC时钟在“bsp.c”文件中的“SystemClock_Config”函数进行配置
	ADC时钟配置为PLL2P源 时钟 = 72MHz
*/
#elif defined (ADC_CLOCK_SOURCE_AHB)
  
  /* 使用AHB时钟的话，无需配置，默认选择*/
  
#endif

	/* ## - 2 - 配置ADC采样使用的GPIO ####################################### */
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_4);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_7);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_8);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_11);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_14);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_17);
	// bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_18);
	// bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_19);
  
	/* ## - 3 - 配置ADC采样使用的时钟 ####################################### */
	__HAL_RCC_DMA1_CLK_ENABLE();	/* 开启DMA时钟 */

	ADC1_DMA_Handle.Instance                 = DMA1_Stream7;            /* 使用的DMA2 Stream7 */
	ADC1_DMA_Handle.Init.Request             = DMA_REQUEST_ADC1;  	   	/* 请求类型采用DMA_REQUEST_ADC1 */  
	ADC1_DMA_Handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* 传输方向是从存储器到外设 */  
	ADC1_DMA_Handle.Init.PeriphInc           = DMA_PINC_DISABLE;        /* 外设地址自增禁止 */ 
	ADC1_DMA_Handle.Init.MemInc              = DMA_MINC_ENABLE;         /* 存储器地址自增使能 */  
	ADC1_DMA_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* 外设数据传输位宽选择半字，即16bit */     
	ADC1_DMA_Handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;	/* 存储器数据传输位宽选择半字，即16bit */    
	ADC1_DMA_Handle.Init.Mode                = DMA_CIRCULAR;            /* 循环模式 */   
	ADC1_DMA_Handle.Init.Priority            = DMA_PRIORITY_MEDIUM;     /* 优先级低 */  
	ADC1_DMA_Handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;    /* 禁止FIFO*/
	ADC1_DMA_Handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; /* 禁止FIFO此位不起作用，用于设置阀值 */
	ADC1_DMA_Handle.Init.MemBurst            = DMA_MBURST_SINGLE;       /* 禁止FIFO此位不起作用，用于存储器突发 */
	ADC1_DMA_Handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;       /* 禁止FIFO此位不起作用，用于外设突发 */

	/* 初始化DMA */
	if(HAL_DMA_Init(&ADC1_DMA_Handle) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);     
	}
	/* 关联ADC句柄和DMA句柄 */
	__HAL_LINKDMA(&AdcHandle, DMA_Handle, ADC1_DMA_Handle);

	/* ## - 4 - 配置ADC ########################################################### */
	__HAL_RCC_ADC12_CLK_ENABLE();	/* 开启ADC1、2的时钟 */

	AdcHandle.Instance = ADC1;		/* 选择为ADC1 */

#if defined (ADC_CLOCK_SOURCE_PLL)
	AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV8;          /* 采用PLL异步时钟，8分频，即72MHz/8 = 36MHz */
#elif defined (ADC_CLOCK_SOURCE_AHB)
	AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;      /* 采用AHB同步时钟，4分频，即200MHz/4 = 50MHz */
#endif
	
	AdcHandle.Init.Resolution            = ADC_RESOLUTION_16B;        /* 16位分辨率 */
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;           /* 启用扫描 */
	AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;       /* EOC转换结束标志 */
	AdcHandle.Init.LowPowerAutoWait      = DISABLE;                   /* 禁止低功耗自动延迟特性 */
	AdcHandle.Init.ContinuousConvMode    = ENABLE;                    /* 禁止自动转换，采用的软件触发 */
	AdcHandle.Init.NbrOfConversion       = adc_channel_num;           /* 使用了4个转换通道 */
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;                   /* 禁止不连续模式 */
	AdcHandle.Init.NbrOfDiscConversion   = 1;                         /* 禁止不连续模式后，此参数忽略，此位是用来配置不连续子组中通道数 */
	AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;        /* 采用软件触发 */
	AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;    /* 采用软件触发的话，此位忽略 */
	AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; /* DMA循环模式接收ADC转换的数据 */
	AdcHandle.Init.BoostMode             = DISABLE;                            /* ADC时钟低于20MHz的话，可以禁止boost */
	AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;     	   /* ADC转换溢出的话，覆盖ADC的数据寄存器 */
	AdcHandle.Init.OversamplingMode         		  = ENABLE;                        /* 使能过采样 */
	AdcHandle.Init.Oversampling.Ratio                 = 15;                            /* 15+1倍过采样 */
	AdcHandle.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_4;           /* 数据右移4bit，即除以16 */
	AdcHandle.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;   /* 触发后连续完成每个通道的所有过采样转换 */
	AdcHandle.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE; /* 如果触发注入转换，过采样会暂时停止，并会在注
                                                                                            //    入序列完成后继续，注入序列过程中会保留过采样缓冲区*/    
  /* 初始化ADC */
	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}

 	multimode.Mode = ADC_MODE_INDEPENDENT;	/* 选择为独立模式 */
	if (HAL_ADCEx_MultiModeConfigChannel(&AdcHandle, &multimode) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	/* ## - 5 - 配置ADC规则通道 ###################################################### */
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_4, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_7, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_8, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_11, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_14, ADC_REGULAR_RANK_5, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_17, ADC_REGULAR_RANK_6, ADC_SAMPLETIME_810CYCLES_5);
	// bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_18, ADC_REGULAR_RANK_7, ADC_SAMPLETIME_810CYCLES_5);
	// bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_19, ADC_REGULAR_RANK_8, ADC_SAMPLETIME_810CYCLES_5);

	/* 模拟看门狗配置 */
	// AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
	// AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
	// AnalogWDGConfig.Channel = ADC_CHANNEL_4;
	// AnalogWDGConfig.ITMode = ENABLE;
	// AnalogWDGConfig.HighThreshold = 30000;
	// AnalogWDGConfig.LowThreshold = 0;
	// if (HAL_ADC_AnalogWDGConfig(&AdcHandle, &AnalogWDGConfig) != HAL_OK)
	// {
	//   Error_Handler(__FILE__, __LINE__);
	// }

	/* ## - 6 - 配置ADC和DMA中断优先级 ##################################### */

	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	// HAL_NVIC_SetPriority(ADC_IRQn,0,0);
	// HAL_NVIC_EnableIRQ(ADC_IRQn);

	/* ## - 7 - 启动ADC的DMA方式传输 ####################################### */

	/* 校准ADC，采用偏移校准 */
	if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	__HAL_ADC_ENABLE_IT(&AdcHandle,ADC_IT_AWD1);

	/* 开启ADC的DMA传输 */
	if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)ADCxValues, adc_channel_num) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}
}

/*
*函数说明：	开启ADC通道对应的GPIO配置
*输入形参：	ADCx：ADC外设（x可以是1、2、3）
*			ADC_CHANNEL_x：通道（x可以是0~19）
*输出数据：	无
*/
void bsp_ADCx_CH_GPIO_Config(ADC_TypeDef* ADCx, uint8_t ADC_CHANNEL_x)
{
	GPIO_TypeDef* GPIOx;
	uint32_t GPIO_PIN_x;
	GPIO_InitTypeDef ADC_GPIO = {0};

	if(ADCx == ADC1)
	{
		switch(ADC_CHANNEL_x)
		{
			case ADC_CHANNEL_4:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_4;
				break;

			case ADC_CHANNEL_7:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_7;
				break;

			case ADC_CHANNEL_8:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_5;
				break;

			case ADC_CHANNEL_11:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_1;
				break;

			case ADC_CHANNEL_14:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_2;
				break;

			case ADC_CHANNEL_17:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_1;
				break;

			case ADC_CHANNEL_18:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_4;
				break;

			case ADC_CHANNEL_19:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_5;
				break;

			default:
				Error_Handler(__FILE__, __LINE__);		/* ADC1其他IO已经被占用 */
		}

	}
	else if(ADCx == ADC2)
	{
		switch(ADC_CHANNEL_x)
		{
			case ADC_CHANNEL_3:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_6;
				break;

			case ADC_CHANNEL_4:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_4;
				break;

			case ADC_CHANNEL_7:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_7;
				break;

			case ADC_CHANNEL_8:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_5;
				break;

			case ADC_CHANNEL_11:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_1;
				break;

			case ADC_CHANNEL_14:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_2;
				break;

			case ADC_CHANNEL_18:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_4;
				break;

			case ADC_CHANNEL_19:
				GPIOx = GPIOA;
				GPIO_PIN_x = GPIO_PIN_5;
				break;

			default :
				Error_Handler(__FILE__, __LINE__);		/* ADC2其他IO已经被占用 */
				break;
		}
	}
	else if (ADCx == ADC3)
	{
		switch(ADC_CHANNEL_x)
		{
			case ADC_CHANNEL_0:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_2;
				break;

			case ADC_CHANNEL_1:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_3;
				break;

			case ADC_CHANNEL_11:
				GPIOx = GPIOC;
				GPIO_PIN_x = GPIO_PIN_1;
				break;

			default :
				Error_Handler(__FILE__, __LINE__);		/* ADC3其他IO已经被占用 */
				break;
		}
	}
	
	if (GPIOx == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
	else if (GPIOx == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
	else if (GPIOx == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
	else if (GPIOx == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
	else if (GPIOx == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
	else if (GPIOx == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
	else if (GPIOx == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
	else if (GPIOx == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
	else if (GPIOx == GPIOI) __HAL_RCC_GPIOI_CLK_ENABLE();
	else if (GPIOx == GPIOJ) __HAL_RCC_GPIOJ_CLK_ENABLE();		/* H743II用不到 */
	else if (GPIOx == GPIOK) __HAL_RCC_GPIOK_CLK_ENABLE();		/* H743II用不到 */

	ADC_GPIO.Pull = GPIO_NOPULL;
	ADC_GPIO.Mode = GPIO_MODE_ANALOG;
	ADC_GPIO.Pin  = GPIO_PIN_x;

	HAL_GPIO_Init(GPIOx,&ADC_GPIO);
}

/*
*函数说明：	对ADCx通道配置规则转换组
*输入形参：	HADCx：ADC_HandleTypeDef结构体
*			ADC_CHANNEL_x：通道（x可以是0~19，@ref ADC_channels）
*			ADC_REGULAR_RANK_x：规则转换组（x可以是1~16，@ref ADC_regular_rank）
*			ADC_SAMPLETIME：采样时间配置（可以是 @ref ADC_sampling_times）
*输出数据：	无
*/
void bsp_ADCx_Set_Rule_CH(ADC_HandleTypeDef* HADCx, uint32_t ADC_CHANNEL_x, uint32_t ADC_REGULAR_RANK_x, uint32_t ADC_SAMPLETIME)
{
	ADC_ChannelConfTypeDef   ADCx_Channel = {0};

	ADCx_Channel.Channel      = ADC_CHANNEL_x;         		/* 配置使用的ADC通道 */
	ADCx_Channel.Rank         = ADC_REGULAR_RANK_x;       	/* 采样序列 */
	ADCx_Channel.SamplingTime = ADC_SAMPLETIME;  			/* 采样周期 */
	ADCx_Channel.SingleDiff   = ADC_SINGLE_ENDED;          	/* 单端输入 */
	ADCx_Channel.OffsetNumber = ADC_OFFSET_NONE;           	/* 无偏移 */ 
	ADCx_Channel.Offset = 0;                               	/* 无偏移的情况下，此参数忽略 */
	ADCx_Channel.OffsetRightShift       = DISABLE;         	/* 禁止右移 */
	ADCx_Channel.OffsetSignedSaturation = DISABLE;         	/* 禁止有符号饱和 */

	if (HAL_ADC_ConfigChannel(HADCx, &ADCx_Channel) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}
}



/*
*函数说明：	ADC转换数据测试函数
*输入形参：	无
*输出数据：	无
*/
void bsp_ADC_Test(void)
{
  float AdcValues[adc_channel_num];

	AdcValues[0] = ADCxValues[0] * 3.3 / 65536;
	AdcValues[1] = ADCxValues[1] * 3.3 / 65536;
	AdcValues[2] = ADCxValues[2] * 3.3 / 65536;
	AdcValues[3] = ADCxValues[3] * 3.3 / 65536;
	AdcValues[4] = ADCxValues[4] * 3.3 / 65536;
	AdcValues[5] = ADCxValues[5] * 3.3 / 65536;
	// AdcValues[6] = ADCxValues[6] * 3.3 / 65536;
	// AdcValues[7] = ADCxValues[7] * 3.3 / 65536;

	//芯片温度获取
  // TS_CAL1 = *(__IO uint16_t *)(0x1FF1E820);
  // TS_CAL2 = *(__IO uint16_t *)(0x1FF1E840);
  
  // AdcValues[5] = (110.0 - 30.0) * (ADCxValues[5] - TS_CAL1)/ (TS_CAL2 - TS_CAL1) + 30; 
  
	printf("ADC1 CH4： %5.3fV\r\n",AdcValues[0]);
	printf("ADC1 CH7： %5.3fV\r\n",AdcValues[1]);
	printf("ADC1 CH8： %5.3fV\r\n",AdcValues[2]);
	printf("ADC1 CH11：%5.3fV\r\n",AdcValues[3]);
	printf("ADC1 CH14：%5.3fV\r\n",AdcValues[4]);
	printf("ADC1 CH17：%5.3fV\r\n",AdcValues[5]);
	// printf("ADC1 CH18：%5.3fV\r\n",AdcValues[6]);
	// printf("ADC1 CH19：%5.3fV\r\n",AdcValues[7]);
	printf("\r\n");


}

/*
*函数说明：	DMA1数据流通道1的中断服务函数（这里用于ADC1的数据传输）
*输入形参：	无
*输出数据：	无
*/
void DMA1_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}

/*
*函数说明：	ADC1、2的中断服务函数
*输入形参：	无
*输出数据：	无
*/
void ADC_IRQHandler(void)
{
	// if(__HAL_ADC_GET_FLAG(&AdcHandle,ADC_IT_AWD1) == TRUE)
	// {
	// 	HAL_Delay(100);
	// 	printf("电压超出\r\n");
	// 	__HAL_ADC_CLEAR_FLAG(&AdcHandle,ADC_IT_AWD1);
	// }
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
