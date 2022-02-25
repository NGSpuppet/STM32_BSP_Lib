/*
*********************************************************************************************************
*
*	ģ������ : ADC����
*	�ļ����� : bsp_adc.c
*	��    �� : V1.0
*	˵    �� : ADC��ͨ������
*              1. ����Ĭ���õ�PLLʱ�ӹ�ADCʹ�ã���ҿ���ͨ��bsp_adc.c�ļ���ͷ�궨���л���AHBʱ�ӡ�
*              2������DMA��ʽ���ж�ͨ���������ɼ���PC0, Vbat/4, VrefInt���¶ȡ�
*              3��ADC��������һ��16����������ʹ�ò������ݱȽ��ȶ������֧��1024����������
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2018-12-12 armfly  ��ʽ����
*
*	Copyright (C), 2018-2030, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "bsp.h"

/* ѡ��ADC��ʱ��Դ */
//#define ADC_CLOCK_SOURCE_AHB     /* ѡ��AHBʱ��Դ */
#define ADC_CLOCK_SOURCE_PLL     /* ѡ��PLLʱ��Դ */

#define adc_channel_num		6	//ADCͨ������

__IO uint16_t ADCxValues[adc_channel_num];

ADC_HandleTypeDef   AdcHandle;
DMA_HandleTypeDef   ADC1_DMA_Handle;


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitADC
*	����˵��: ��ʼ��ADC������DMA��ʽ���ж�ͨ���������ɼ���PC0, Vbat/4, VrefInt���¶�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitADC(void)
{
	ADC_MultiModeTypeDef multimode = {0};
//	ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

  /* ## - 1 - ����ADC������ʱ�� ####################################### */
#if defined (ADC_CLOCK_SOURCE_PLL)
/*
	ADCʱ���ڡ�bsp.c���ļ��еġ�SystemClock_Config��������������
	ADCʱ������ΪPLL2PԴ ʱ�� = 72MHz
*/
#elif defined (ADC_CLOCK_SOURCE_AHB)
  
  /* ʹ��AHBʱ�ӵĻ����������ã�Ĭ��ѡ��*/
  
#endif

	/* ## - 2 - ����ADC����ʹ�õ�GPIO ####################################### */
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_4);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_7);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_8);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_11);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_14);
	bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_17);
	// bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_18);
	// bsp_ADCx_CH_GPIO_Config(ADC1, ADC_CHANNEL_19);
  
	/* ## - 3 - ����ADC����ʹ�õ�ʱ�� ####################################### */
	__HAL_RCC_DMA1_CLK_ENABLE();	/* ����DMAʱ�� */

	ADC1_DMA_Handle.Instance                 = DMA1_Stream7;            /* ʹ�õ�DMA2 Stream7 */
	ADC1_DMA_Handle.Init.Request             = DMA_REQUEST_ADC1;  	   	/* �������Ͳ���DMA_REQUEST_ADC1 */  
	ADC1_DMA_Handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* ���䷽���ǴӴ洢�������� */  
	ADC1_DMA_Handle.Init.PeriphInc           = DMA_PINC_DISABLE;        /* �����ַ������ֹ */ 
	ADC1_DMA_Handle.Init.MemInc              = DMA_MINC_ENABLE;         /* �洢����ַ����ʹ�� */  
	ADC1_DMA_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* �������ݴ���λ��ѡ����֣���16bit */     
	ADC1_DMA_Handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;	/* �洢�����ݴ���λ��ѡ����֣���16bit */    
	ADC1_DMA_Handle.Init.Mode                = DMA_CIRCULAR;            /* ѭ��ģʽ */   
	ADC1_DMA_Handle.Init.Priority            = DMA_PRIORITY_MEDIUM;     /* ���ȼ��� */  
	ADC1_DMA_Handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;    /* ��ֹFIFO*/
	ADC1_DMA_Handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; /* ��ֹFIFO��λ�������ã��������÷�ֵ */
	ADC1_DMA_Handle.Init.MemBurst            = DMA_MBURST_SINGLE;       /* ��ֹFIFO��λ�������ã����ڴ洢��ͻ�� */
	ADC1_DMA_Handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;       /* ��ֹFIFO��λ�������ã���������ͻ�� */

	/* ��ʼ��DMA */
	if(HAL_DMA_Init(&ADC1_DMA_Handle) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);     
	}
	/* ����ADC�����DMA��� */
	__HAL_LINKDMA(&AdcHandle, DMA_Handle, ADC1_DMA_Handle);

	/* ## - 4 - ����ADC ########################################################### */
	__HAL_RCC_ADC12_CLK_ENABLE();	/* ����ADC1��2��ʱ�� */

	AdcHandle.Instance = ADC1;		/* ѡ��ΪADC1 */

#if defined (ADC_CLOCK_SOURCE_PLL)
	AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV8;          /* ����PLL�첽ʱ�ӣ�8��Ƶ����72MHz/8 = 36MHz */
#elif defined (ADC_CLOCK_SOURCE_AHB)
	AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;      /* ����AHBͬ��ʱ�ӣ�4��Ƶ����200MHz/4 = 50MHz */
#endif
	
	AdcHandle.Init.Resolution            = ADC_RESOLUTION_16B;        /* 16λ�ֱ��� */
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;           /* ����ɨ�� */
	AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;       /* EOCת��������־ */
	AdcHandle.Init.LowPowerAutoWait      = DISABLE;                   /* ��ֹ�͹����Զ��ӳ����� */
	AdcHandle.Init.ContinuousConvMode    = ENABLE;                    /* ��ֹ�Զ�ת�������õ�������� */
	AdcHandle.Init.NbrOfConversion       = adc_channel_num;           /* ʹ����4��ת��ͨ�� */
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;                   /* ��ֹ������ģʽ */
	AdcHandle.Init.NbrOfDiscConversion   = 1;                         /* ��ֹ������ģʽ�󣬴˲������ԣ���λ���������ò�����������ͨ���� */
	AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;        /* ����������� */
	AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;    /* ������������Ļ�����λ���� */
	AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; /* DMAѭ��ģʽ����ADCת�������� */
	AdcHandle.Init.BoostMode             = DISABLE;                            /* ADCʱ�ӵ���20MHz�Ļ������Խ�ֹboost */
	AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;     	   /* ADCת������Ļ�������ADC�����ݼĴ��� */
	AdcHandle.Init.OversamplingMode         		  = ENABLE;                        /* ʹ�ܹ����� */
	AdcHandle.Init.Oversampling.Ratio                 = 15;                            /* 15+1�������� */
	AdcHandle.Init.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_4;           /* ��������4bit��������16 */
	AdcHandle.Init.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;   /* �������������ÿ��ͨ�������й�����ת�� */
	AdcHandle.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE; /* �������ע��ת��������������ʱֹͣ��������ע
                                                                                            //    ��������ɺ������ע�����й����лᱣ��������������*/    
  /* ��ʼ��ADC */
	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}

 	multimode.Mode = ADC_MODE_INDEPENDENT;	/* ѡ��Ϊ����ģʽ */
	if (HAL_ADCEx_MultiModeConfigChannel(&AdcHandle, &multimode) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	/* ## - 5 - ����ADC����ͨ�� ###################################################### */
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_4, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_7, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_8, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_11, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_14, ADC_REGULAR_RANK_5, ADC_SAMPLETIME_810CYCLES_5);
	bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_17, ADC_REGULAR_RANK_6, ADC_SAMPLETIME_810CYCLES_5);
	// bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_18, ADC_REGULAR_RANK_7, ADC_SAMPLETIME_810CYCLES_5);
	// bsp_ADCx_Set_Rule_CH(&AdcHandle, ADC_CHANNEL_19, ADC_REGULAR_RANK_8, ADC_SAMPLETIME_810CYCLES_5);

	/* ģ�⿴�Ź����� */
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

	/* ## - 6 - ����ADC��DMA�ж����ȼ� ##################################### */

	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	// HAL_NVIC_SetPriority(ADC_IRQn,0,0);
	// HAL_NVIC_EnableIRQ(ADC_IRQn);

	/* ## - 7 - ����ADC��DMA��ʽ���� ####################################### */

	/* У׼ADC������ƫ��У׼ */
	if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}

	__HAL_ADC_ENABLE_IT(&AdcHandle,ADC_IT_AWD1);

	/* ����ADC��DMA���� */
	if (HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)ADCxValues, adc_channel_num) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}
}

/*
*����˵����	����ADCͨ����Ӧ��GPIO����
*�����βΣ�	ADCx��ADC���裨x������1��2��3��
*			ADC_CHANNEL_x��ͨ����x������0~19��
*������ݣ�	��
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
				Error_Handler(__FILE__, __LINE__);		/* ADC1����IO�Ѿ���ռ�� */
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
				Error_Handler(__FILE__, __LINE__);		/* ADC2����IO�Ѿ���ռ�� */
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
				Error_Handler(__FILE__, __LINE__);		/* ADC3����IO�Ѿ���ռ�� */
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
	else if (GPIOx == GPIOJ) __HAL_RCC_GPIOJ_CLK_ENABLE();		/* H743II�ò��� */
	else if (GPIOx == GPIOK) __HAL_RCC_GPIOK_CLK_ENABLE();		/* H743II�ò��� */

	ADC_GPIO.Pull = GPIO_NOPULL;
	ADC_GPIO.Mode = GPIO_MODE_ANALOG;
	ADC_GPIO.Pin  = GPIO_PIN_x;

	HAL_GPIO_Init(GPIOx,&ADC_GPIO);
}

/*
*����˵����	��ADCxͨ�����ù���ת����
*�����βΣ�	HADCx��ADC_HandleTypeDef�ṹ��
*			ADC_CHANNEL_x��ͨ����x������0~19��@ref ADC_channels��
*			ADC_REGULAR_RANK_x������ת���飨x������1~16��@ref ADC_regular_rank��
*			ADC_SAMPLETIME������ʱ�����ã������� @ref ADC_sampling_times��
*������ݣ�	��
*/
void bsp_ADCx_Set_Rule_CH(ADC_HandleTypeDef* HADCx, uint32_t ADC_CHANNEL_x, uint32_t ADC_REGULAR_RANK_x, uint32_t ADC_SAMPLETIME)
{
	ADC_ChannelConfTypeDef   ADCx_Channel = {0};

	ADCx_Channel.Channel      = ADC_CHANNEL_x;         		/* ����ʹ�õ�ADCͨ�� */
	ADCx_Channel.Rank         = ADC_REGULAR_RANK_x;       	/* �������� */
	ADCx_Channel.SamplingTime = ADC_SAMPLETIME;  			/* �������� */
	ADCx_Channel.SingleDiff   = ADC_SINGLE_ENDED;          	/* �������� */
	ADCx_Channel.OffsetNumber = ADC_OFFSET_NONE;           	/* ��ƫ�� */ 
	ADCx_Channel.Offset = 0;                               	/* ��ƫ�Ƶ�����£��˲������� */
	ADCx_Channel.OffsetRightShift       = DISABLE;         	/* ��ֹ���� */
	ADCx_Channel.OffsetSignedSaturation = DISABLE;         	/* ��ֹ�з��ű��� */

	if (HAL_ADC_ConfigChannel(HADCx, &ADCx_Channel) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}
}



/*
*����˵����	ADCת�����ݲ��Ժ���
*�����βΣ�	��
*������ݣ�	��
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

	//оƬ�¶Ȼ�ȡ
  // TS_CAL1 = *(__IO uint16_t *)(0x1FF1E820);
  // TS_CAL2 = *(__IO uint16_t *)(0x1FF1E840);
  
  // AdcValues[5] = (110.0 - 30.0) * (ADCxValues[5] - TS_CAL1)/ (TS_CAL2 - TS_CAL1) + 30; 
  
	printf("ADC1 CH4�� %5.3fV\r\n",AdcValues[0]);
	printf("ADC1 CH7�� %5.3fV\r\n",AdcValues[1]);
	printf("ADC1 CH8�� %5.3fV\r\n",AdcValues[2]);
	printf("ADC1 CH11��%5.3fV\r\n",AdcValues[3]);
	printf("ADC1 CH14��%5.3fV\r\n",AdcValues[4]);
	printf("ADC1 CH17��%5.3fV\r\n",AdcValues[5]);
	// printf("ADC1 CH18��%5.3fV\r\n",AdcValues[6]);
	// printf("ADC1 CH19��%5.3fV\r\n",AdcValues[7]);
	printf("\r\n");


}

/*
*����˵����	DMA1������ͨ��1���жϷ���������������ADC1�����ݴ��䣩
*�����βΣ�	��
*������ݣ�	��
*/
void DMA1_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}

/*
*����˵����	ADC1��2���жϷ�����
*�����βΣ�	��
*������ݣ�	��
*/
void ADC_IRQHandler(void)
{
	// if(__HAL_ADC_GET_FLAG(&AdcHandle,ADC_IT_AWD1) == TRUE)
	// {
	// 	HAL_Delay(100);
	// 	printf("��ѹ����\r\n");
	// 	__HAL_ADC_CLEAR_FLAG(&AdcHandle,ADC_IT_AWD1);
	// }
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
