#ifndef __BSP_I2C_BMP280_H
#define __BSP_I2C_BMP280_H

#include "stdint.h"

/****************************** 从设备地址 *******************************/
/* SDO电平(选择iic地址) */
#define BMP_SDO_Level   0

//从设备地址(随SDO电平改变)
#if(BMP_SDO_Level)
#define BMP280_ADDRESS                          (0x77 << 1)		/* SDO高电平I2C地址 */
#else
#define BMP280_ADDRESS						    (0x76 << 1)		/* SDO低电平I2C地址 */
#endif
//从设备读/写地址
#define BMP280_Write_ADDRESS                    (BMP280_ADDRESS & 0xFE)
#define BMP280_Read_ADDRESS                     (BMP280_ADDRESS | 0x01)

#define BSP_I2C_BMP280_V	"BSP_I2C_BMP280版本：21.09.13"
/****************************** 从设备地址end ******************************/


/****************************** 变量宏定义 ******************************/
#define BMP280_RESET_VALUE					    0xB6  /* 复位寄存器写入值 */
/****************************** 变量宏定义end ******************************/


/****************************** 寄存器标志宏定义 ******************************/
#define	BMP280_MEASURING					    0x01 /* 转换标志 */
#define	BMP280_IM_UPDATE					    0x08 /* 更新标志 */
/****************************** 寄存器标志宏定义end ******************************/


/****************************** 寄存器地址宏定义 ******************************/
//传感器寄存器地址宏
#define BMP280_CHIPID_REG                       0xD0  /* 芯片 ID 寄存器 */
#define BMP280_RESET_REG                        0xE0  /* 软复位寄存器 */
#define BMP280_STATUS_REG                       0xF3  /* 状态寄存器 */
#define BMP280_CTRLMEAS_REG                     0xF4  /* Ctrl 测量寄存器 */
#define BMP280_CONFIG_REG                       0xF5  /* 配置寄存器 */
#define BMP280_PRESSURE_MSB_REG                 0xF7  /* 压力 MSB 寄存器 */
#define BMP280_PRESSURE_LSB_REG                 0xF8  /* 压力 LSB 寄存器 */
#define BMP280_PRESSURE_XLSB_REG                0xF9  /* 压力 XLSB 寄存器 */
#define BMP280_TEMPERATURE_MSB_REG              0xFA  /* 温度 MSB 寄存器 */
#define BMP280_TEMPERATURE_LSB_REG              0xFB  /* 温度 LSB 寄存器 */
#define BMP280_TEMPERATURE_XLSB_REG             0xFC  /* 温度 XLSB 寄存器 */
//矫正数据寄存器地址宏
#define BMP280_DIG_T1_LSB_REG                   0x88
#define BMP280_DIG_T1_MSB_REG                   0x89
#define BMP280_DIG_T2_LSB_REG                   0x8A
#define BMP280_DIG_T2_MSB_REG                   0x8B
#define BMP280_DIG_T3_LSB_REG                   0x8C
#define BMP280_DIG_T3_MSB_REG                   0x8D
#define BMP280_DIG_P1_LSB_REG                   0x8E
#define BMP280_DIG_P1_MSB_REG                   0x8F
#define BMP280_DIG_P2_LSB_REG                   0x90
#define BMP280_DIG_P2_MSB_REG                   0x91
#define BMP280_DIG_P3_LSB_REG                   0x92
#define BMP280_DIG_P3_MSB_REG                   0x93
#define BMP280_DIG_P4_LSB_REG                   0x94
#define BMP280_DIG_P4_MSB_REG                   0x95
#define BMP280_DIG_P5_LSB_REG                   0x96
#define BMP280_DIG_P5_MSB_REG                   0x97
#define BMP280_DIG_P6_LSB_REG                   0x98
#define BMP280_DIG_P6_MSB_REG                   0x99
#define BMP280_DIG_P7_LSB_REG                   0x9A
#define BMP280_DIG_P7_MSB_REG                   0x9B
#define BMP280_DIG_P8_LSB_REG                   0x9C
#define BMP280_DIG_P8_MSB_REG                   0x9D
#define BMP280_DIG_P9_LSB_REG                   0x9E
#define BMP280_DIG_P9_MSB_REG                   0x9F
/****************************** 寄存器地址宏定义end ******************************/


/****************************** 枚举变量声明 ******************************/
/* BMP工作模式 */
typedef enum {
	BMP280_SLEEP_MODE = 0x0,
	BMP280_FORCED_MODE = 0x1,	//可以说0x2
	BMP280_NORMAL_MODE = 0x3
} BMP280_WORK_MODE;

/* BMP压力过采样因子 */
typedef enum 
{
	BMP280_P_MODE_SKIP = 0x0,	/*skipped*/
	BMP280_P_MODE_1,			/*x1*/
	BMP280_P_MODE_2,			/*x2*/
	BMP280_P_MODE_3,			/*x4*/
	BMP280_P_MODE_4,			/*x8*/
	BMP280_P_MODE_5			    /*x16*/
} BMP280_P_OVERSAMPLING;	

/* BMP温度过采样因子 */
typedef enum {
	BMP280_T_MODE_SKIP = 0x0,	/*skipped*/
	BMP280_T_MODE_1,			/*x1*/
	BMP280_T_MODE_2,			/*x2*/
	BMP280_T_MODE_3,			/*x4*/
	BMP280_T_MODE_4,			/*x8*/
	BMP280_T_MODE_5			    /*x16*/
} BMP280_T_OVERSAMPLING;
									
/* IIR滤波器时间常数 */
typedef enum {
	BMP280_FILTER_OFF = 0x0,	/*filter off*/
	BMP280_FILTER_MODE_1,		/*0.223*ODR*/	/*x2*/
	BMP280_FILTER_MODE_2,		/*0.092*ODR*/	/*x4*/
	BMP280_FILTER_MODE_3,		/*0.042*ODR*/	/*x8*/
	BMP280_FILTER_MODE_4		/*0.021*ODR*/	/*x16*/
} BMP280_FILTER_COEFFICIENT;

/* 保持时间 */
typedef enum {
	BMP280_T_SB1 = 0x0,	    /*0.5ms*/
	BMP280_T_SB2,			/*62.5ms*/
	BMP280_T_SB3,			/*125ms*/
	BMP280_T_SB4,			/*250ms*/
	BMP280_T_SB5,			/*500ms*/
	BMP280_T_SB6,			/*1000ms*/
	BMP280_T_SB7,			/*2000ms*/
	BMP280_T_SB8,			/*4000ms*/
} BMP280_T_SB;
/****************************** 枚举变量声明end ******************************/


/****************************** 结构体变量声明 ******************************/
/* 补偿系数 */
typedef struct  {
	uint16_t T1;
	int16_t	T2;
	int16_t	T3;
	uint16_t P1;
	int16_t	P2;
	int16_t	P3;
	int16_t	P4;
	int16_t	P5;
	int16_t	P6;
	int16_t	P7;
	int16_t	P8;
	int16_t	P9;
}BMP280_Typedef;

/* 过采样因子结构体 */
typedef struct{
	BMP280_P_OVERSAMPLING P_Osample;
	BMP280_T_OVERSAMPLING T_Osample;
	BMP280_WORK_MODE		WORKMODE;
} BMP_OVERSAMPLE_MODE;

/* 保持时间和滤波器分频因子结构体 */
typedef struct{
	BMP280_T_SB 				T_SB;
	BMP280_FILTER_COEFFICIENT 	FILTER_COEFFICIENT;
	FunctionalState				SPI_EN;
} BMP_CONFIG;

/****************************** 结构体变量声明end ******************************/


/****************************** 下面是用来计算补偿值相关 ******************************/
typedef			long signed int				BMP280_S32_t;	//有符号 64位！
typedef			long unsigned int			BMP280_U32_t;	//无符号 32位！
typedef			long long signed int		BMP280_S64_t;

#define	dig_T1			bmp280.T1	
#define	dig_T2			bmp280.T2	
#define	dig_T3			bmp280.T3	

#define	dig_P1			bmp280.P1
#define	dig_P2			bmp280.P2
#define	dig_P3			bmp280.P3
#define	dig_P4			bmp280.P4
#define	dig_P5			bmp280.P5
#define	dig_P6			bmp280.P6
#define	dig_P7			bmp280.P7
#define	dig_P8			bmp280.P8
#define	dig_P9			bmp280.P9
/****************************** 计算补偿值end ******************************/

//全局变量声明
extern BMP280_Typedef bmp280;

//函数声明
uint8_t BMP280_Read_Byte(uint8_t REG_addr);
void BMP280_Write_Byte(uint8_t REG_addr,uint8_t data);
uint8_t BMP280_Read_ID(void);
uint8_t BMP280_GetStatus(uint8_t status_flag);
void BMP280_Set_TemOversamp(BMP_OVERSAMPLE_MODE * Oversample_Mode);
void BMP280_Set_Standby_FILTER(BMP_CONFIG * BMP_Config);
double BMP280_Get_Pressure(void);
double BMP280_Get_StandardPressure(void);
double BMP280_Get_Temperature(void);
double BMP280_Get_Temperature(void);
void BMP280_Init(void);

//测试程序说明
void BMP280_Test(void);
#endif
