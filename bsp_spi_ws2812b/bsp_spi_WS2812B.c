
#include "bsp_WS2812B.h"

/*
����˵����  ������ʹ�õ���Ӳ��SPI��MOSI�ܽ���ΪRGB�ź������
        ��ȷ��SPI�Ĺ���Ƶ����8MHz(���)��ʹ������Ƶ��ʱ����
        ���������ֲ��Ҫ���޸�1��0��ķ���ֵ��

ʹ��ע�⣺  ����Ĭ��LED��������4���������޸������ļ���.h�ļ�
        ���޸�RGB_LED_quantity�ĺ궨��ֵ��
            ��Ϊ����ɫ������ǰ��Ҫ���͸�λ���ݣ�����Ҫ��.h��
        ���н�WS2812B_PROT��WS2812B_PIN�궨���޸ĳ���SPI��
        MOSI�ܽ�һ���ĺꡣ
*/

extern void delay_us(uint32_t i);
extern SPI_HandleTypeDef hspi;

/* RGBɫ�����ݻ��� */
uint8_t n_RGBcode[24 * RGB_LED_quantity] = {0};


/*
*����˵���� WS2812B�ĸ�λ�źź���
*�����βΣ� ��
*������ݣ� ��
*/
void bsp_WS2812B_Reset(void)
{
    WS2812B_PIN_RESET;
    delay_us(500);
}

/*
*����˵���� ����WS2812B������0�źź���
*�����βΣ� low_addr�����ڱ���0��ĵ�ַ
*������ݣ� ��
*/
static void bsp_WS2812B_LowtCode(uint8_t *low_addr)
{
    *low_addr = 0xC0;
}

/*
*����˵���� ����WS2812B������1�źź���
*�����βΣ� high_addr�����ڱ���1��ĵ�ַ
*������ݣ� ��
*/
static void bsp_WS2812B_HighCode(uint8_t *high_addr)
{
    *high_addr = 0xFC;
}

/*
*����˵���� ��WS2812B����8λ����
*�����βΣ� Byte��8λ��ɫ����
*           code_addr��8λɫ����ı����ַ
*������ݣ� ��
*/
static void bsp_WS2812B_WriteByte(uint8_t Byte, uint8_t *code_addr)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        if(Byte & 0x80)
            bsp_WS2812B_HighCode(code_addr);
        else
            bsp_WS2812B_LowtCode(code_addr);
				
		Byte <<= 1;
        code_addr++;

    }
}

/*
*����˵���� ��WS2812B����24λ��ɫ����
*�����βΣ� red��8λ��ɫֵ��
*           green��8λ��ɫֵ��
*           blue��8λ��ɫֵ��
*������ݣ� ��
*/
void bsp_WS2812B_WriteRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t one_RGBcode[24] = {0};
    bsp_WS2812B_Reset();

    bsp_WS2812B_WriteByte(green, (one_RGBcode + 8));
    bsp_WS2812B_WriteByte(red, one_RGBcode);
    bsp_WS2812B_WriteByte(blue, (one_RGBcode + 16));

    HAL_SPI_Transmit(&hspi, one_RGBcode, 24, 0xff);
}

/*
*����˵���� ��WS2812B����n��24λ��ɫ����
*�����βΣ� RGB_addr��RGBɫ���βε�ַ
*������ݣ� ��
*/
void bsp_WS2812B_nWriteRGB(uint8_t *RGB_addr)
{
    bsp_WS2812B_Reset();

    for(uint8_t i = 0; i < RGB_LED_quantity; i++)
    {
        bsp_WS2812B_WriteByte(*(RGB_addr + 1), (n_RGBcode + (i*24)));
        bsp_WS2812B_WriteByte(*RGB_addr, n_RGBcode + 8 + (i*24));
        bsp_WS2812B_WriteByte(*(RGB_addr + 2), (n_RGBcode + 16 + (i*24)));
        RGB_addr += 3;
    }

    HAL_SPI_Transmit(&hspi, n_RGBcode, 24 * RGB_LED_quantity, 0xff);
}


