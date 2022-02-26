	
#ifndef __BSP_KEYBOARD4X4_H
#define __BSP_KEYBOARD4X4_H

#include "stm32h7xx_hal.h"

#define uint8_t			unsigned char
#define uint16_t		unsigned short
#define uint32_t		unsigned int

/* 矩阵按键IO口分配 */
/* 	高位	低位 	*/
/*	PB15	PH12	*/
/*	PB14	PH11	*/
/*	PB13	PH10	*/
/*	PB12	PH9		*/

//矩阵按键结构体
typedef struct
{
	uint8_t keyboard_data;//矩阵按键数据
	uint8_t keyboard_updata_flag;//矩阵按键数据更新标志
	uint8_t keyboard_mode_falg;
}
KeyBoard_Typedef;

//矩阵按键数据结构体(全局变量)
extern KeyBoard_Typedef 	KeyBoard_Struct;

/************************** 用户定义端口宏 ***************************/
//选择PB高位端口连接
#define KEYBOARD4X4_GPIO_HPORT		GPIOB
#define KEYBOARD4X4_GPIO_HCLK		__HAL_RCC_GPIOB_CLK_ENABLE()
//选择PH低位端口连接
#define KEYBOARD4X4_GPIO_LPORT		GPIOH
#define KEYBOARD4X4_GPIO_LCLK		__HAL_RCC_GPIOH_CLK_ENABLE()

//矩阵按键的高4位端口
#define KEYBOARD4X4_HIGH_4PIN		0xF000
//矩阵按键的低4位端口
#define KEYBOARD4X4_LOW_4PIN		0x1E00
//矩阵按键的占用端口
#define KEYBOARD4X4_ALL_PIN			(KEYBOARD4X4_HIGH_4PIN | KEYBOARD4X4_LOW_4PIN)
/**************************** 用户端口结束 ***************************/

void KeyBoard_Scan_10ms(void);
char KeyBoard_Read_data(void);
void bsp_KeyBoard_Init(void);


#endif

