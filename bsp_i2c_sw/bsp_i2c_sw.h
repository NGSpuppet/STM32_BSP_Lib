/*
*********************************************************************************************************
*
*	ģ������ : I2C��������ģ��
*	�ļ����� : bsp_i2c_gpio.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ���
*
*	Copyright (C), 2012-2013, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_I2C_SW_H
#define _BSP_I2C_SW_H

#include "stdint.h"

#define BSP_I2C_SW_V	"bsp_i2c_SW�汾��21.09.12"

/*
	������STM32-V7������ i2c����GPIO:
 		PB6/I2C1_SCL
 		PB7/I2C1_SDA
*/

#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */

/* ����I2C�������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����5�д��뼴������ı�SCL��SDA������ */
#define I2C_SCL_GPIO	GPIOB			/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_GPIO	GPIOB			/* ���ӵ�SDA�����ߵ�GPIO */

#define I2C_SCL_PIN		GPIO_PIN_6		/* ���ӵ�SCLʱ���ߵ�GPIO */
#define I2C_SDA_PIN		GPIO_PIN_7		/* ���ӵ�SDA�����ߵ�GPIO */

#define ALL_I2C_GPIO_CLK_ENABLE		__HAL_RCC_GPIOB_CLK_ENABLE()	/* ��ӦGPIO�˿� */

/* �����дSCL��SDA�ĺ� */
#define I2C_SCL_1()  I2C_SCL_GPIO->ODR |= I2C_SCL_PIN;			/* SCL = 1 */
#define I2C_SCL_0()  I2C_SCL_GPIO->ODR &= ~I2C_SCL_PIN;			/* SCL = 0 */
	
#define I2C_SDA_1()  I2C_SDA_GPIO->ODR |= I2C_SDA_PIN;			/* SDA = 1 */
#define I2C_SDA_0()  I2C_SDA_GPIO->ODR &= ~I2C_SDA_PIN;			/* SDA = 0 */

#define I2C_SCL_READ()  ((I2C_SCL_GPIO->IDR & I2C_SCL_PIN) == I2C_SCL_PIN)		/* ��SCL����״̬ */
#define I2C_SDA_READ()  ((I2C_SDA_GPIO->IDR & I2C_SDA_PIN) == I2C_SDA_PIN)		/* ��SDA����״̬ */

/* �������� */
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(uint8_t _ucByte);
uint8_t i2c_ReadByte(void);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);

/************************* ģ���׼I2C���� *************************/
typedef enum{
	I2C_SW_OK = 0U,
	I2C_SW_ERROR = 1U
}I2C_SW_STATE;

/* �������� */
void bsp_I2C_SW_Init(void);
I2C_SW_STATE bsp_I2C_SW_CheckDevice(uint8_t _Address);
I2C_SW_STATE bsp_I2C_SW_Write_Byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t write_data);
I2C_SW_STATE bsp_I2C_SW_Read_Byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *read_data);
I2C_SW_STATE bsp_I2C_SW_Write_nByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t Length);
I2C_SW_STATE bsp_I2C_SW_Read_nByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t Length);
/************************ ģ���׼I2C����end ************************/


#endif
