#ifndef	__BSP_I2C_FDC2214_H
#define __BSP_I2C_FDC2214_H

#include "stdint.h"

/* I2C��ַ */
#define FDC2214_Addr    0x2A						
#define FDC2214_Write   (FDC2214_Addr<<1)
#define FDC2214_Read    (FDC2214_Addr<<1)+1

/* �Ĵ�����ַ�궨�� */
#define FDC2214_DATA_CH0 0x00               //ͨ��0���ݵĸ�12λ
#define FDC2214_DATA_LSB_CH0 0x01			//ͨ��1���ݵĵ�12λ��λ�������ֲ�15ҳ����4��
#define FDC2214_DATA_CH1 0x02
#define FDC2214_DATA_LSB_CH1 0x03
#define FDC2214_DATA_CH2 0x04
#define FDC2214_DATA_LSB_CH2 0x05
#define FDC2214_DATA_CH3 0x06
#define FDC2214_DATA_LSB_CH3 0x07
#define FDC2214_RCOUNT_CH0 0x08 			//ת��ʱ�����üĴ�����
#define FDC2214_RCOUNT_CH1 0x09
#define FDC2214_RCOUNT_CH2 0x0A
#define FDC2214_RCOUNT_CH3 0x0B
#define FDC2214_OFFSET_CH0 0x0C
#define FDC2214_OFFSET_CH1 0x0D
#define FDC2214_OFFSET_CH2 0x0E
#define FDC2214_OFFSET_CH3 0x0F
#define FDC2214_SETTLECOUNT_CH0 0x10		//ͨ������ʱ�����üĴ�����λ�������ֲ�16ҳ����5
#define FDC2214_SETTLECOUNT_CH1 0x11
#define FDC2214_SETTLECOUNT_CH2 0x12
#define FDC2214_SETTLECOUNT_CH3 0x13
#define FDC2214_CLOCK_DIVIDERS_C_CH0 0x14
#define FDC2214_CLOCK_DIVIDERS_C_CH1 0x15
#define FDC2214_CLOCK_DIVIDERS_C_CH2 0x16
#define FDC2214_CLOCK_DIVIDERS_C_CH3 0x17
#define FDC2214_STATUS 0x18
#define FDC2214_ERROR_CONFIG 0x19
#define FDC2214_CONFIG 0x1A
#define FDC2214_MUX_CONFIG 0x1B
#define FDC2214_RESET_DEV 0x1C
#define FDC2214_DRIVE_CURRENT_CH0 0x1E
#define FDC2214_DRIVE_CURRENT_CH1 0x1F
#define FDC2214_DRIVE_CURRENT_CH2 0x20
#define FDC2214_DRIVE_CURRENT_CH3 0x21
#define FDC2214_MANUFACTURER_ID 0x7E
#define FDC2214_DEVICE_ID 0x7F

/* �������� */
void FDC2214_Write_2Byte(uint8_t REG_add,uint16_t value);
uint16_t FDC2214_Read_2Byte(uint8_t REG_add);
int FDC2214_Read_Channl_Data(uint8_t index);
void FDC2214_SingleChannl_Init(void);
void FDC2214_MultiChannl_Init(void);

void FDC2214_Test(void);

#endif
