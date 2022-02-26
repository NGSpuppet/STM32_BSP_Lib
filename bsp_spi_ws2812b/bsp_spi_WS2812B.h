
#ifndef __BSP_WS2812B_H
#define __BSP_WS2812B_H

#include "stdint.h"
#include "stm32h7xx_hal.h"

#define RGB_LED_quantity        4   /* LED灯的数量 */

/* 与硬件SPI的MOSI口相同 */
#define WS2812B_PROT                    GPIOA
#define WS2812B_PIN                     GPIO_PIN_7

#define WS2812B_PIN_SET                 WS2812B_PROT->ODR |= WS2812B_PIN
#define WS2812B_PIN_RESET               WS2812B_PROT->ODR &= ~WS2812B_PIN

void bsp_WS2812B_WriteRGB(uint8_t red, uint8_t green, uint8_t blue);
void bsp_WS2812B_nWriteRGB(uint8_t *RGB_addr);
void bsp_WS2812B_Reset(void);

#endif
