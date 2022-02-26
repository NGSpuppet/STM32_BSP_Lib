
#include "bsp_WS2812B.h"

/*
程序说明：  本程序使用的是硬件SPI的MOSI管脚作为RGB信号输出端
        请确保SPI的工作频率在8MHz(最好)，使用其他频率时，请
        根据数据手册的要求，修改1、0码的发送值。

使用注意：  程序默认LED的数量是4个，如需修改请在文件的.h文件
        中修改RGB_LED_quantity的宏定义值。
            因为发送色彩数据前需要发送复位数据，所以要在.h文
        件中将WS2812B_PROT和WS2812B_PIN宏定义修改成与SPI的
        MOSI管脚一样的宏。
*/

extern void delay_us(uint32_t i);
extern SPI_HandleTypeDef hspi;

/* RGB色彩数据缓存 */
uint8_t n_RGBcode[24 * RGB_LED_quantity] = {0};


/*
*函数说明： WS2812B的复位信号函数
*输入形参： 无
*输出数据： 无
*/
void bsp_WS2812B_Reset(void)
{
    WS2812B_PIN_RESET;
    delay_us(500);
}

/*
*函数说明： 产生WS2812B的数字0信号函数
*输入形参： low_addr：用于保存0码的地址
*输出数据： 无
*/
static void bsp_WS2812B_LowtCode(uint8_t *low_addr)
{
    *low_addr = 0xC0;
}

/*
*函数说明： 产生WS2812B的数字1信号函数
*输入形参： high_addr：用于保存1码的地址
*输出数据： 无
*/
static void bsp_WS2812B_HighCode(uint8_t *high_addr)
{
    *high_addr = 0xFC;
}

/*
*函数说明： 向WS2812B发送8位数据
*输入形参： Byte：8位颜色数据
*           code_addr：8位色彩码的保存地址
*输出数据： 无
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
*函数说明： 向WS2812B发送24位颜色数据
*输入形参： red（8位红色值）
*           green（8位绿色值）
*           blue（8位蓝色值）
*输出数据： 无
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
*函数说明： 向WS2812B发送n个24位颜色数据
*输入形参： RGB_addr：RGB色彩形参地址
*输出数据： 无
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


