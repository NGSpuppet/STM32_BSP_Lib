	
#ifndef __BSP_KEYBOARD4X4_H
#define __BSP_KEYBOARD4X4_H

#include "stm32h7xx_hal.h"

#define uint8_t			unsigned char
#define uint16_t		unsigned short
#define uint32_t		unsigned int

/* ���󰴼�IO�ڷ��� */
/* 	��λ	��λ 	*/
/*	PB15	PH12	*/
/*	PB14	PH11	*/
/*	PB13	PH10	*/
/*	PB12	PH9		*/

//���󰴼��ṹ��
typedef struct
{
	uint8_t keyboard_data;//���󰴼�����
	uint8_t keyboard_updata_flag;//���󰴼����ݸ��±�־
	uint8_t keyboard_mode_falg;
}
KeyBoard_Typedef;

//���󰴼����ݽṹ��(ȫ�ֱ���)
extern KeyBoard_Typedef 	KeyBoard_Struct;

/************************** �û�����˿ں� ***************************/
//ѡ��PB��λ�˿�����
#define KEYBOARD4X4_GPIO_HPORT		GPIOB
#define KEYBOARD4X4_GPIO_HCLK		__HAL_RCC_GPIOB_CLK_ENABLE()
//ѡ��PH��λ�˿�����
#define KEYBOARD4X4_GPIO_LPORT		GPIOH
#define KEYBOARD4X4_GPIO_LCLK		__HAL_RCC_GPIOH_CLK_ENABLE()

//���󰴼��ĸ�4λ�˿�
#define KEYBOARD4X4_HIGH_4PIN		0xF000
//���󰴼��ĵ�4λ�˿�
#define KEYBOARD4X4_LOW_4PIN		0x1E00
//���󰴼���ռ�ö˿�
#define KEYBOARD4X4_ALL_PIN			(KEYBOARD4X4_HIGH_4PIN | KEYBOARD4X4_LOW_4PIN)
/**************************** �û��˿ڽ��� ***************************/

void KeyBoard_Scan_10ms(void);
char KeyBoard_Read_data(void);
void bsp_KeyBoard_Init(void);


#endif

