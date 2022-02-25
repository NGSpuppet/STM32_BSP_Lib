/*
*********************************************************************************************************
*
*	ģ������ : ADC����
*	�ļ����� : bsp_adc.c
*	��    �� : V1.0
*	˵    �� : ADC��ͨ������
*              1. ����Ĭ���õ�PLLʱ�ӹ�ADCʹ�ã���ҿ���ͨ��bsp_adc.c�ļ���ͷ�궨���л���AHBʱ�ӡ�
*              2������DMA��ʽ���ж�ͨ���������ɼ���PC0, Vbat/4, VrefInt���¶ȡ�
*              3��ADC��������һ��16����������ʹ�ò������ݱȽ��ȶ������֧��1024����������
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2018-12-12 armfly  ��ʽ����
*
*	Copyright (C), 2018-2030, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32h7xx_hal.h"

/*****************************ADCͨ����*****************************/
/*
        ADC1               ADC2                 ADC3    
    ͨ��    IO          ͨ��    IO          ͨ��    IO
    CH0     *PA0_C      CH0     *PA0_C      CH0     PC2_C
    CH1     *PA1_C      CH1     *PA1_C      CH1     PC3_C
    CH2     *PF11       CH2     *PF13       CH2     *PF9
    CH3     *PA6        CH3     PA6         CH3     *PF7
    CH4     PC4         CH4     PC4         CH4     *PF5
    CH5     *PB1        CH5     *PB1        CH5     *PF3
    CH6     *PF12       CH6     *PF14       CH6     *PF10
    CH7     PA7         CH7     PA7         CH7     *PF8
    CH8     PC5         CH8     PC5         CH8     *PF6
    CH9     *PB0        CH9     *PB0        CH9     *PF4
    CH10    *PC0        CH10    *PC0        CH10    *PC0
    CH11    PC1         CH11    PC1         CH11    PC1
    CH12    *PC2        CH12    *PC2        CH12    *PC2
    CH13    *PC3        CH13    *PC3        CH13    *PH2
    CH14    PA2         CH14    PA2         CH14    *PH3
    CH15    *PA3        CH15    *PA3        CH15    *PH4
    CH16    *PA0        CH16    DAC1        CH16    *PH5
    CH17    PA1         CH17    DAC2        CH17    VBAT
    CH18    PA4         CH18    PA4         CH18    �ڲ��¶�
    CH19    PA5         CH19    PA5         CH19    �ڲ��ο���ѹ

    "*"��ʾIO��ռ��
*/
/****************************ADCͨ����end****************************/

// /* ADC1ͨ���궨�� */
// #define ADC1_CH4    {ADC1, GPIOC, GPIO_PIN_4}
// #define ADC1_CH7    {ADC1, GPIOA, GPIO_PIN_7}
// #define ADC1_CH8    {ADC1, GPIOC, GPIO_PIN_5}
// #define ADC1_CH11   {ADC1, GPIOC, GPIO_PIN_1}
// #define ADC1_CH14   {ADC1, GPIOA, GPIO_PIN_2}
// #define ADC1_CH17   {ADC1, GPIOA, GPIO_PIN_1}
// #define ADC1_CH18   {ADC1, GPIOA, GPIO_PIN_4}
// #define ADC1_CH19   {ADC1, GPIOA, GPIO_PIN_5}

// /* ADC2ͨ���궨�� */
// #define ADC2_CH3    {ADC2, GPIOA, GPIO_PIN_6}
// #define ADC2_CH4    {ADC2, GPIOC, GPIO_PIN_4}
// #define ADC2_CH7    {ADC2, GPIOA, GPIO_PIN_7}
// #define ADC2_CH8    {ADC2, GPIOC, GPIO_PIN_5}
// #define ADC2_CH11   {ADC2, GPIOC, GPIO_PIN_1}
// #define ADC2_CH14   {ADC2, GPIOA, GPIO_PIN_2}
// #define ADC2_CH18   {ADC2, GPIOA, GPIO_PIN_4}
// #define ADC2_CH19   {ADC2, GPIOA, GPIO_PIN_5}

// /* ADC3ͨ���궨�� */
// #define ADC3_CH0    {GPIOC, GPIO_PIN_2}
// #define ADC3_CH1    {GPIOC, GPIO_PIN_3}
// #define ADC3_CH11   {GPIOC, GPIO_PIN_1}
// #define ADC3_CH13   {GPIOH, GPIO_PIN_2}
// #define ADC3_CH14   {GPIOH, GPIO_PIN_3}
// #define ADC3_CH15   {GPIOH, GPIO_PIN_4}
// #define ADC3_CH16   {GPIOH, GPIO_PIN_5}

typedef struct{
ADC_TypeDef* ADCx;
GPIO_TypeDef* GPIOx;
uint32_t GPIO_PIN_x;

}ADCx_GPIO_Typedef;


void bsp_ADCx_CH_GPIO_Config(ADC_TypeDef* ADCx, uint8_t ADC_CHANNEL_x);
void bsp_ADCx_Set_Rule_CH(ADC_HandleTypeDef* HADCx, uint32_t ADC_CHANNEL_x, uint32_t ADC_REGULAR_RANK_x, uint32_t ADC_SAMPLETIME);
void ADC_NVIC_Set_Config(void);
/* ���ⲿ���õĺ������� */
void bsp_InitADC(void);
void bsp_ADC_Test(void);
void MX_ADC1_Init(void);
#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
