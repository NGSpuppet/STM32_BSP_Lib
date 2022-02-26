#ifndef __BSP_I2C_HW_H
#define __BSP_I2C_HW_H


#include "stdint.h"
#include "stm32h7xx_hal_i2c.h"

#define BSP_I2C_HW_V  "BSP_I2C_HW版本：21.09.12"

/*
	i2c总线GPIO:
 		PB6/I2C1_SCL
 		PB7/I2C1_SDA
*/

/********************* 硬件I2C传输模式宏定义 ***********************/

#define I2C_BLOCKING_MODE		/* 阻塞模式 */
// #define I2C_IT_MODE			/* 中断模式 */
// #define I2C_DMA_MODE			/* DMA模式 */


#define I2C_DATA_BUFFER		1024      	/* 数据缓存空间（单位B） */
#define I2C_NO_BLOCKING_IDLE	0U		/* 闲状态 */
#define I2C_NO_BLOCKING_BUSY	1U		/* 忙状态 */


typedef enum{
  I2C_HW_OK = 0U,
  I2C_HW_ERROR = 1U,
}I2C_HW_STATE;


extern I2C_HandleTypeDef hi2c1;   /* 声明成全局变量 */

/* 函数声明 */
void bsp_I2C_HW_Init(void);
I2C_HW_STATE bsp_I2C_HW_CheckDevice(uint8_t i2c_addr);
I2C_HW_STATE bsp_I2C_HW_Write_Byte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t write_data);
I2C_HW_STATE bsp_I2C_HW_Read_Byte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *read_data);
I2C_HW_STATE bsp_I2C_HW_Write_nByte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *write_data, uint16_t Length);
I2C_HW_STATE bsp_I2C_HW_Read_nByte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *read_data, uint16_t Length);

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
