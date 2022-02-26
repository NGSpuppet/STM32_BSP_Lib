#include "bsp.h"

extern 	UART_HandleTypeDef UartHandle;


/*
*函数说明： 向VOFA上位机发送浮点型通道数据
*输入形参： float_data：浮点型通道数据
            channel_count：通道数量
*输出数据： 无
*/
void VOFA_JustFloat_Channel(float* float_data,uint8_t channel_count)
{
    /* JustFloat结束帧 */
    uint8_t JustFloat_finish[] =  {0x00, 0x00, 0x80, 0x7f};

    HAL_UART_Transmit(&UartHandle,(uint8_t*)float_data,(channel_count*4),0xffffff);
    HAL_UART_Transmit(&UartHandle,JustFloat_finish,4,0xffffff);
}

/*
*函数说明： 向VOFA上位机发送图片数据包
*输入形参： Image_Start_Frame_Typedef：图片头帧结构体
*           image_data：图片数据地址
*           count：图片数据量
*输出数据： 无
*/
void VOFA_JustFloat_Image(Image_Start_Frame_Typedef* Image_start, uint8_t* image_data, uint32_t count)
{
    uint32_t Frame[2] = {0x7F800000,0x7F800000}; 

    HAL_UART_Transmit(&UartHandle,(uint8_t*)Image_start, 20, 0xffffff);
    HAL_UART_Transmit(&UartHandle,(uint8_t*)Frame, 8, 0xffffff);

    HAL_UART_Transmit(&UartHandle,image_data, count, 0xffffff);
}
