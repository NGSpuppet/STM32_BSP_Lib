#include "bsp.h"

extern 	UART_HandleTypeDef UartHandle;


/*
*����˵���� ��VOFA��λ�����͸�����ͨ������
*�����βΣ� float_data��������ͨ������
            channel_count��ͨ������
*������ݣ� ��
*/
void VOFA_JustFloat_Channel(float* float_data,uint8_t channel_count)
{
    /* JustFloat����֡ */
    uint8_t JustFloat_finish[] =  {0x00, 0x00, 0x80, 0x7f};

    HAL_UART_Transmit(&UartHandle,(uint8_t*)float_data,(channel_count*4),0xffffff);
    HAL_UART_Transmit(&UartHandle,JustFloat_finish,4,0xffffff);
}

/*
*����˵���� ��VOFA��λ������ͼƬ���ݰ�
*�����βΣ� Image_Start_Frame_Typedef��ͼƬͷ֡�ṹ��
*           image_data��ͼƬ���ݵ�ַ
*           count��ͼƬ������
*������ݣ� ��
*/
void VOFA_JustFloat_Image(Image_Start_Frame_Typedef* Image_start, uint8_t* image_data, uint32_t count)
{
    uint32_t Frame[2] = {0x7F800000,0x7F800000}; 

    HAL_UART_Transmit(&UartHandle,(uint8_t*)Image_start, 20, 0xffffff);
    HAL_UART_Transmit(&UartHandle,(uint8_t*)Frame, 8, 0xffffff);

    HAL_UART_Transmit(&UartHandle,image_data, count, 0xffffff);
}
