#ifndef	__BSP_UART_DEBUG_VOFA_H
#define __BSP_UART_DEBUG_VOFA_H

#include "stm32h7xx_hal.h"



typedef struct{
    uint32_t IMG_ID;
    uint32_t IMG_SIZE;
    uint32_t IMG_WIDTH;
    uint32_t IMG_HEIGHT;
    uint32_t IMG_FORMAT;
}Image_Start_Frame_Typedef;


void VOFA_JustFloat_Channel(float* float_data,uint8_t channel_count);
void VOFA_JustFloat_Image(Image_Start_Frame_Typedef* Image_start, uint8_t* image_data, uint32_t count);

#endif
