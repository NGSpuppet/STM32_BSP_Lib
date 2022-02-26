/**
  ******************************************************************************
  * @author  泽耀科技 ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   SPI配置H文件
  ******************************************************************************
  * @attention
  *
  * 官网	:	http://www.ashining.com
  * 淘宝	:	https://shop105912646.taobao.com
  * 阿里巴巴:	https://cdzeyao.1688.com
  ******************************************************************************
  */


#ifndef __BSP_SPI_SW_H
#define __BSP_SPI_SW_H

#include "stm32h7xx_hal.h"


//SPI引脚定义
#define SPI_CLK_GPIO_CLK			__HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI_CLK_GPIO_PORT			GPIOB
#define SPI_CLK_GPIO_PIN			GPIO_PIN_3

#define SPI_MISO_GPIO_CLK			__HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI_MISO_GPIO_PORT		GPIOB
#define SPI_MISO_GPIO_PIN			GPIO_PIN_4

#define SPI_MOSI_GPIO_CLK			__HAL_RCC_GPIOB_CLK_ENABLE()
#define SPI_MOSI_GPIO_PORT		GPIOB
#define SPI_MOSI_GPIO_PIN			GPIO_PIN_5


#define SPI_SET_CLK_H		      SPI_CLK_GPIO_PORT->ODR |= SPI_CLK_GPIO_PIN								//时钟置高
#define SPI_SET_CLK_L		      SPI_CLK_GPIO_PORT->ODR &= ~SPI_CLK_GPIO_PIN	              //时钟置低

#define SPI_SET_MOSI_H			  SPI_MOSI_GPIO_PORT->ODR |= SPI_MOSI_GPIO_PIN							//发送脚置高
#define SPI_SET_MOSI_L			  SPI_MOSI_GPIO_PORT->ODR &= ~SPI_MOSI_GPIO_PIN	            //发送脚置低

#define SPI_GET_MISO			((SPI_MISO_GPIO_PORT->IDR & SPI_MISO_GPIO_PIN) != SPI_MISO_GPIO_PIN) ? 0 : 1 // 若相应输入位为低则得到0，相应输入位为高则得到1


void bsp_SPI_SW_Init(void);
uint8_t bsp_SPI_SW_Read_Byte(void);
void bsp_SPI_SW_Write_Byte(uint8_t data);
uint8_t bsp_SPI_SW_Read_Write_Byte(uint8_t TxByte);
void bsp_SPI_SW_Read_Block(uint8_t* ReadBuffer, uint16_t Length);
void bsp_SPI_SW_Write_Block(uint8_t* WriteBuffer, uint16_t Length);
void bsp_SPI_SW_Read_Write_Block(uint8_t* ReadBuffer, uint8_t* WriteBuffer, uint16_t Length);

#endif


