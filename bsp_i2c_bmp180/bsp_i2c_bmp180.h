
#ifndef __BSP_I2C_BMP180_H
#define __BSP_I2C_BMP180_H

#include "stdint.h"

#define BMP180_ADDRESS              (0x77 << 1)
#define BMP180_Write_ADDRESS        (BMP180_ADDRESS & 0xFE)
#define BMP180_Read_ADDRESS         (BMP180_ADDRESS | 0x01)

#define BMP180_ID_REG               0xD0

typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
    int32_t UT;
    int32_t UP;
    int32_t X1;
    int32_t X2;
    int32_t X3;
    int32_t B3;
    uint32_t B4;
    int32_t B5;
    int32_t B6;
    int32_t B7;
    int32_t p;
    int32_t Temp;
    float altitude;
}_bmp180;

extern _bmp180 bmp180;

uint8_t BMP180_Read_Byte(uint8_t ReadAddr);
void BMP180_Write_Byte(uint8_t WriteAddr,uint8_t DataToWrite);
int16_t BMP180_Read_2Byte(uint8_t ReadAddr);
uint8_t BMP180_Read_ID(void);
int32_t BMP180_Read_UT(void);
int32_t BMP180_Read_UP(void);
void BMP180_UncompemstatedToTrue(void);
void BMP180_Init(void);

void BMP180_Test(void);
#endif 
