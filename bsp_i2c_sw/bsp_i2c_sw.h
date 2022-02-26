/*
*********************************************************************************************************
*
*	模块名称 : I2C总线驱动模块
*	文件名称 : bsp_i2c_gpio.h
*	版    本 : V1.0
*	说    明 : 头文件。
*
*	Copyright (C), 2012-2013, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_I2C_SW_H
#define _BSP_I2C_SW_H

#include "stdint.h"

#define BSP_I2C_SW_V	"bsp_i2c_SW版本：21.09.12"

/*
	安富莱STM32-V7开发板 i2c总线GPIO:
 		PB6/I2C1_SCL
 		PB7/I2C1_SDA
*/

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

/* 定义I2C总线连接的GPIO端口, 用户只需要修改下面5行代码即可任意改变SCL和SDA的引脚 */
#define I2C_SCL_GPIO	GPIOB			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_GPIO	GPIOB			/* 连接到SDA数据线的GPIO */

#define I2C_SCL_PIN		GPIO_PIN_6		/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_PIN_7		/* 连接到SDA数据线的GPIO */

#define ALL_I2C_GPIO_CLK_ENABLE		__HAL_RCC_GPIOB_CLK_ENABLE()	/* 对应GPIO端口 */

/* 定义读写SCL和SDA的宏 */
#define I2C_SCL_1()  I2C_SCL_GPIO->ODR |= I2C_SCL_PIN;			/* SCL = 1 */
#define I2C_SCL_0()  I2C_SCL_GPIO->ODR &= ~I2C_SCL_PIN;			/* SCL = 0 */
	
#define I2C_SDA_1()  I2C_SDA_GPIO->ODR |= I2C_SDA_PIN;			/* SDA = 1 */
#define I2C_SDA_0()  I2C_SDA_GPIO->ODR &= ~I2C_SDA_PIN;			/* SDA = 0 */

#define I2C_SCL_READ()  ((I2C_SCL_GPIO->IDR & I2C_SCL_PIN) == I2C_SCL_PIN)		/* 读SCL口线状态 */
#define I2C_SDA_READ()  ((I2C_SDA_GPIO->IDR & I2C_SDA_PIN) == I2C_SDA_PIN)		/* 读SDA口线状态 */

/* 函数声明 */
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(void);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);

/************************* 模拟标准I2C传输 *************************/
typedef enum{
	I2C_SW_OK = 0U,
	I2C_SW_ERROR = 1U
}I2C_SW_STATE;

/* 函数声明 */
void bsp_I2C_SW_Init(void);
I2C_SW_STATE bsp_I2C_SW_CheckDevice(uint8_t _Address);
I2C_SW_STATE bsp_I2C_SW_Write_Byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t write_data);
I2C_SW_STATE bsp_I2C_SW_Read_Byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *read_data);
I2C_SW_STATE bsp_I2C_SW_Write_nByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t Length);
I2C_SW_STATE bsp_I2C_SW_Read_nByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t Length);
/************************ 模拟标准I2C传输end ************************/


#endif
