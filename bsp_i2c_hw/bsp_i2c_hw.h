#ifndef __BSP_I2C_HW_H
#define __BSP_I2C_HW_H


#include "stdint.h"
#include "stm32h7xx_hal_i2c.h"

#define BSP_I2C_HW_V  "BSP_I2C_HW�汾��21.09.12"

/*
	i2c����GPIO:
 		PB6/I2C1_SCL
 		PB7/I2C1_SDA
*/

/********************* Ӳ��I2C����ģʽ�궨�� ***********************/

#define I2C_BLOCKING_MODE		/* ����ģʽ */
// #define I2C_IT_MODE			/* �ж�ģʽ */
// #define I2C_DMA_MODE			/* DMAģʽ */


#define I2C_DATA_BUFFER		1024      	/* ���ݻ���ռ䣨��λB�� */
#define I2C_NO_BLOCKING_IDLE	0U		/* ��״̬ */
#define I2C_NO_BLOCKING_BUSY	1U		/* æ״̬ */


typedef enum{
  I2C_HW_OK = 0U,
  I2C_HW_ERROR = 1U,
}I2C_HW_STATE;


extern I2C_HandleTypeDef hi2c1;   /* ������ȫ�ֱ��� */

/* �������� */
void bsp_I2C_HW_Init(void);
I2C_HW_STATE bsp_I2C_HW_CheckDevice(uint8_t i2c_addr);
I2C_HW_STATE bsp_I2C_HW_Write_Byte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t write_data);
I2C_HW_STATE bsp_I2C_HW_Read_Byte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *read_data);
I2C_HW_STATE bsp_I2C_HW_Write_nByte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *write_data, uint16_t Length);
I2C_HW_STATE bsp_I2C_HW_Read_nByte(uint8_t i2c_addr, uint8_t REG_addr, uint8_t *read_data, uint16_t Length);

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
