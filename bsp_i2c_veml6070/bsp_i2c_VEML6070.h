#ifndef __I2C_VEML6070
#define __I2C_VEML6070

#include "stdint.h"


//ʹ���ж�����
#define VEML6070_INT_ENABLE				0
//�ж�����ѡ��
#define VEML6070_INT_CLK				__HAL_RCC_GPIOB_CLK_ENABLE()
#define VEML6070_PORT					GPIOB
#define VEML6070_PIN					GPIO_PIN_8


//���͵�ַ=���ߵ�ַ+��дλ
//ʵ�ʷ��͵�ַ���ݣ�
#define VEML6070_Hlight_ADDR    		(0x73) /* ��ǿ�ȸ�8λ���ݵ�ַ */
#define VEML6070_Llight_ADDR    		(0x71) /* ��ǿ�ȵ�8λ���ݵ�ַ */
#define VEML6070_Set_ADDR   			(0x70) /* ���üĴ�����ַ */
#define VEML6070_ARA_ADDR				(0x19) /* �ж�״̬�Ĵ�����ַ */

//�ֱ���ö��
typedef enum 
{
	ACK_THD_108_Steps = 0x00,
	ACK_THD_145_Steps
	
}ACK_THD_Steps;

//����ʱ��ö��
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
