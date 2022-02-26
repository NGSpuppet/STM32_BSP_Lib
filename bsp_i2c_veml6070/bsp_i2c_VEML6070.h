#ifndef __I2C_VEML6070
#define __I2C_VEML6070

#include "stdint.h"


//使能中断引脚
#define VEML6070_INT_ENABLE				0
//中断引脚选择
#define VEML6070_INT_CLK				__HAL_RCC_GPIOB_CLK_ENABLE()
#define VEML6070_PORT					GPIOB
#define VEML6070_PIN					GPIO_PIN_8


//发送地址=总线地址+读写位
//实际发送地址数据：
#define VEML6070_Hlight_ADDR    		(0x73) /* 光强度高8位数据地址 */
#define VEML6070_Llight_ADDR    		(0x71) /* 光强度低8位数据地址 */
#define VEML6070_Set_ADDR   			(0x70) /* 配置寄存器地址 */
#define VEML6070_ARA_ADDR				(0x19) /* 中断状态寄存器地址 */

//分辨率枚举
typedef enum 
{
	ACK_THD_108_Steps = 0x00,
	ACK_THD_145_Steps
	
}ACK_THD_Steps;

//采样时间枚举
typedef enum
{
	Time_1_2_T = 0x00,
	Time_1_T,
	Time_2_T,
	Time_4_T
	
}IT_Sampling_Period;

uint16_t VEML6070_Read_Light(void);
uint8_t VEML6070_Read_INT_State(void);
void VEML6070_Set_ACK_Bit(FunctionalState State);
void Set_VEML6070_ACK_THD_Bit(ACK_THD_Steps Steps);
void VEML6070_Set_IT_Bit(IT_Sampling_Period Time);
void Set_VEML6070_SD_Bit(FunctionalState State);
void VEML6070_Config(void);

#endif
