
#include "bsp.h"

//�����ⲿ��ʱ����
extern void delay_ms(uint32_t i);
extern void delay_us(uint32_t i);

ELE_SCALE_Type ele_scale ={0};//�ṹ���ʼ��


/*
*����˵����	��ʼ��HX711
*�����βΣ�	��
*������ݣ�	��
*/
void HX711_Init(void)
{
	/* GPIO��ʼ�� */
	HX711_GPIO_Config();
	/* �ճ����ݻ�ȡ */
  	ele_scale.ADC_initial_value = HX711_ADC_Repeatedly_Average(5);

  	/* ������K��B��ֵ */
  	ele_scale.B = -0.125282;
  	ele_scale.K = 0.002035;
}

//����˵������ʼ��HX711�ܽŶ���
//�����βΣ���
//������ݣ���
void HX711_GPIO_Config(void)
{
	//�û��Զ���HX711�˿ڳ�ʼ���ܽ�
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
//����˵������ȡHX711����ֵ
//�����βΣ���
//������ݣ�����ֵ
uint32_t HX711_ReadCount(void)
{
   uint32_t Count;

   ADSK_LOW;
   Count=0;
   while(READ_ADD0);	//��DOUTΪ�ߵ�ƽʱ����A/Dת������û��׼���ã�
   for (uint8_t i=0;i<24;i++)
   {
      ADSK_HIGH;
      Count=Count<<1;
      ADSK_LOW;
      if(READ_ADD0) 
	  Count++;
   }
   ADSK_HIGH;	//���͵�25�����壬��ʾ�´�ת��ʹ��Aͨ��128db
   Count=Count^0x800000;
   ADSK_LOW;
   return(Count);
}

/*
*����˵����	HX711��ADC��β���ƽ����
*�����βΣ�	count����������1~65565��
*������ݣ�	ADCֵ
*/
uint32_t HX711_ADC_Repeatedly_Average(uint16_t count)
{
	uint32_t c = 0;
	uint32_t max = 0;
	uint32_t min = 0;
	uint64_t overall = 0;
	
	if(count>0 && count<3)//�������������1~2������Ϊ2�β���
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
			max = (c>max) ? c : max;	//��¼���ֵ
			min = (c<min) ? c : min;	//��¼��Сֵ
			
			overall += c;
		}
		return (overall - max - min)/(count - 1);
	}
	return 0;
}

/*
*����˵���� ���㲢�������Ե�K��Bֵ
*�����βΣ� X1������1��ADCֵ
*           Y1������1����������λg��
*           X2������2��ADCֵ
*           Y2������1����������λg��
*������ݣ� ��
*/
void HX711_Linear_Calculate(uint32_t X1, float Y1, uint32_t X2, float Y2)
{
	float A = (float)(X2 - ele_scale.ADC_initial_value)/(X1 - ele_scale.ADC_initial_value);
	
	ele_scale.B = (float)((Y2 - Y1 *A)/(1-A));
	ele_scale.K = (float)((Y2 - ele_scale.B)/(X2 - ele_scale.ADC_initial_value));
}

/*
*����˵���� ��ADCֵת������
*�����βΣ� adc_data��adcֵ
*������ݣ� ��������λg��
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
*����˵���� ������Ʒ�۸�
*�����βΣ� adc_data��adcֵ
*������ݣ� ���ۣ�Ԫ��
*/
float HX711_Price_Total_Prices(uint32_t adc_data)
{
	return (float)(((adc_data - ele_scale.ADC_initial_value)*ele_scale.K + ele_scale.B - ele_scale.pack)*ele_scale.selling_price);
}

/*
*����˵����	HX711���Գ���
*�����βΣ�	��
*������ݣ�	��
*/
void HX711_Test(void)
{
	uint32_t hx711_data;

	hx711_data = HX711_ADC_Repeatedly_Average(2);

	printf("���ӳӲ�����%f g",HX711_ADC_to_Weight(hx711_data));
}


