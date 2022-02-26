#ifndef __BSP_HC_RS04_H
#define __BSP_HC_RS04_H

#include "stm32h7xx_hal.h"

/*
    VCC   ->  5V
    Trig  ->  PI1
    Echo  ->  PI3
    GND   ->  0V
*/

/* 版本宏 */
#define bsp_HC_RS04_V   "版本：21.09.07"

/* HC_RS40 GPIO宏定义 */
#define HC_RS04_Trig_CLK                __HAL_RCC_GPIOI_CLK_ENABLE()
#define HC_RS04_Trig_PORT               GPIOI
#define HC_RS04_Trig_PIN                GPIO_PIN_1

#define HC_RS04_Echo_CLK			    __HAL_RCC_GPIOI_CLK_ENABLE()
#define HC_RS04_Echo_PORT               GPIOI
#define HC_RS04_Echo_PIN                GPIO_PIN_3

/* HC_RS40 GPIO电平宏定义 */
#define HC_RS04_Trig_HIGH               (HC_RS04_Trig_PORT->ODR |= HC_RS04_Trig_PIN)
#define HC_RS04_Trig_LOW                (HC_RS04_Trig_PORT->ODR &= ~(HC_RS04_Trig_PIN))

#define HC_RS04_Echo_LEVEL				((HC_RS04_Echo_PORT->IDR & HC_RS04_Echo_PIN) == HC_RS04_Echo_PIN)

/* HC_RS40 定时器宏定义 */
#define HC_RS04_TIMx                    TIM7
#define HC_RS04_TIMx_CLK                __HAL_RCC_TIM7_CLK_ENABLE()

#define HC_RS04_TIMx_IRQ                TIM7_IRQn
#define HC_RS04_TIMx_IRQHandler         TIM7_IRQHandler


typedef struct{
uint8_t achieve_flag;           /* 完成标志 */
uint32_t distance_count;       /* us时间计数 */

}HC_RS04_TypeDef;

void HC_RS04_Start(void);
float HC_RS04_Distance_Calculate_cm(void);
float HC_RS04_Distance_Calculate_in(void);
void HC_RS04_Init(void);

void HC_RS04_Test(void);

#endif
