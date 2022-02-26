
#ifndef __BSP_HX711_H__
#define __BSP_HX711_H__

#include "stm32h7xx_hal.h"

#define HX711_ADD0_CLK          __HAL_RCC_GPIOE_CLK_ENABLE()
#define HX711_ADD0_PIN		    GPIO_PIN_2
#define HX711_ADD0_PORT         GPIOE

#define HX711_ADSK_CLK          __HAL_RCC_GPIOE_CLK_ENABLE()
#define HX711_ADSK_PIN			GPIO_PIN_3
#define HX711_ADSK_PORT         GPIOE

#define ADSK_HIGH				(HX711_ADSK_PORT->ODR |= GPIO_PIN_3)
#define ADSK_LOW				(HX711_ADSK_PORT->ODR &= ~GPIO_PIN_3)

#define READ_ADD0				(HX711_ADD0_PORT->IDR & GPIO_PIN_2)


typedef struct
{
	uint32_t ADC_initial_value; /* ADC空秤值(adc值) */
	float pack;     /* 包装值(克) */
	float K;        /* 线性K值 */
	float B;        /* 线性B值 */
	float selling_price;        /* 计价（元/克） */
}
ELE_SCALE_Type;

extern ELE_SCALE_Type ele_scale;

//函数或者变量声明
void HX711_GPIO_Config(void);
uint32_t HX711_ReadCount(void);

uint32_t HX711_ADC_Repeatedly_Average(uint16_t count);
void HX711_Linear_Calculate(uint32_t X1, float Y1, uint32_t X2, float Y2);
float HX711_ADC_to_Weight(uint32_t adc_data);
float HX711_Price_Total_Prices(uint32_t adc_data);

void HX711_Init(void);
void HX711_Test(void);
#endif
