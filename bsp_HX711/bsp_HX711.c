
#include "bsp.h"

//调用外部延时函数
extern void delay_ms(uint32_t i);
extern void delay_us(uint32_t i);

ELE_SCALE_Type ele_scale ={0};//结构体初始化


/*
*函数说明：	初始化HX711
*输入形参：	无
*输出数据：	无
*/
void HX711_Init(void)
{
	/* GPIO初始化 */
	HX711_GPIO_Config();
	/* 空秤数据获取 */
  	ele_scale.ADC_initial_value = HX711_ADC_Repeatedly_Average(5);

  	/* 对线性K、B赋值 */
  	ele_scale.B = -0.125282;
  	ele_scale.K = 0.002035;
}

//函数说明：初始化HX711管脚定义
//输入形参：无
//输出数据：无
void HX711_GPIO_Config(void)
{
	//用户自定义HX711端口初始化管脚
	GPIO_InitTypeDef	HX711_GPIO = {0};

	HX711_ADD0_CLK;
	HX711_ADSK_CLK;
	
	HX711_GPIO.Pull = GPIO_NOPULL;
	HX711_GPIO.Mode = GPIO_MODE_OUTPUT_PP;
	HX711_GPIO.Speed = GPIO_SPEED_FREQ_LOW;

	HX711_GPIO.Pin = HX711_ADSK_PIN;
	HAL_GPIO_Init(HX711_ADSK_PORT,&HX711_GPIO);

	HX711_GPIO.Mode = GPIO_MODE_INPUT;
	HX711_GPIO.Pin = HX711_ADD0_PIN;
	HAL_GPIO_Init(HX711_ADD0_PORT,&HX711_GPIO);
}
//函数说明：读取HX711采用值
//输入形参：无
//输出数据：采用值
uint32_t HX711_ReadCount(void)
{
   uint32_t Count;

   ADSK_LOW;
   Count=0;
   while(READ_ADD0);	//当DOUT为高电平时表明A/D转换器还没有准备好！
   for (uint8_t i=0;i<24;i++)
   {
      ADSK_HIGH;
      Count=Count<<1;
      ADSK_LOW;
      if(READ_ADD0) 
	  Count++;
   }
   ADSK_HIGH;	//发送第25个脉冲，表示下次转换使用A通道128db
   Count=Count^0x800000;
   ADSK_LOW;
   return(Count);
}

/*
*函数说明：	HX711的ADC多次采样平均数
*输入形参：	count采样次数（1~65565）
*输出数据：	ADC值
*/
uint32_t HX711_ADC_Repeatedly_Average(uint16_t count)
{
	uint32_t c = 0;
	uint32_t max = 0;
	uint32_t min = 0;
	uint64_t overall = 0;
	
	if(count>0 && count<3)//如果采样次数在1~2，都作为2次采样
	{
		overall += HX711_ReadCount();
		overall += HX711_ReadCount();
		
		return (overall/2);
	}
	else if(count >= 3)
	{
		for(uint32_t i = count; i != 0; i--)
		{
			c = HX711_ReadCount();
			max = (c>max) ? c : max;	//记录最大值
			min = (c<min) ? c : min;	//记录最小值
			
			overall += c;
		}
		return (overall - max - min)/(count - 1);
	}
	return 0;
}

/*
*函数说明： 计算并更新线性的K和B值
*输入形参： X1：样本1的ADC值
*           Y1：样本1的质量（单位g）
*           X2：样本2的ADC值
*           Y2：样本1的质量（单位g）
*输出数据： 无
*/
void HX711_Linear_Calculate(uint32_t X1, float Y1, uint32_t X2, float Y2)
{
	float A = (float)(X2 - ele_scale.ADC_initial_value)/(X1 - ele_scale.ADC_initial_value);
	
	ele_scale.B = (float)((Y2 - Y1 *A)/(1-A));
	ele_scale.K = (float)((Y2 - ele_scale.B)/(X2 - ele_scale.ADC_initial_value));
}

/*
*函数说明： 将ADC值转成质量
*输入形参： adc_data：adc值
*输出数据： 质量（单位g）
*/
float HX711_ADC_to_Weight(uint32_t adc_data)
{
	float data_buf;

   	if(adc_data <= ele_scale.ADC_initial_value)
      	return 0;

	data_buf = ((adc_data - ele_scale.ADC_initial_value)*ele_scale.K + ele_scale.B);
	if(data_buf <= 0)
		return 0;

	return data_buf;
}

/*
*函数说明： 计算物品价格
*输入形参： adc_data：adc值
*输出数据： 单价（元）
*/
float HX711_Price_Total_Prices(uint32_t adc_data)
{
	return (float)(((adc_data - ele_scale.ADC_initial_value)*ele_scale.K + ele_scale.B - ele_scale.pack)*ele_scale.selling_price);
}

/*
*函数说明：	HX711测试程序
*输入形参：	无
*输出数据：	无
*/
void HX711_Test(void)
{
	uint32_t hx711_data;

	hx711_data = HX711_ADC_Repeatedly_Average(2);

	printf("电子秤测量：%f g",HX711_ADC_to_Weight(hx711_data));
}


