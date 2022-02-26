#ifndef __SPI_OLED_
#define __SPI_OLED_

#include "stm32h7xx_hal.h"

/*OLED端口定义*******************************/
//RES引脚
#define     SPI_OLED_RES_CLK            	__HAL_RCC_GPIOB_CLK_ENABLE()    
#define     SPI_OLED_RES_PORT           	GPIOB
#define     SPI_OLED_RES_PIN            	GPIO_PIN_15
//DC引脚
#define     SPI_OLED_DC_CLK            		__HAL_RCC_GPIOB_CLK_ENABLE()     
#define     SPI_OLED_DC_PORT           		GPIOB 
#define     SPI_OLED_DC_PIN            		GPIO_PIN_14
//CS(NSS)引脚 片选选普通GPIO即可
#define     SPI_OLED_CS_CLK              	__HAL_RCC_GPIOB_CLK_ENABLE()  
#define     SPI_OLED_CS_PORT             	GPIOB
#define     SPI_OLED_CS_PIN              	GPIO_PIN_13

/*OLED端口高低电平定义*************************/
#define     SPI_OLED_RES_LOW()              SPI_OLED_RES_PORT->ODR &= ~SPI_OLED_RES_PIN
#define     SPI_OLED_RES_HIGH()             SPI_OLED_RES_PORT->ODR |= SPI_OLED_RES_PIN

#define     SPI_OLED_DC_LOW()               SPI_OLED_DC_PORT->ODR &= ~SPI_OLED_DC_PIN
#define     SPI_OLED_DC_HIGH()              SPI_OLED_DC_PORT->ODR |= SPI_OLED_DC_PIN

#define     SPI_OLED_CS_LOW()               SPI_OLED_CS_PORT->ODR &= ~SPI_OLED_CS_PIN
#define     SPI_OLED_CS_HIGH()              SPI_OLED_CS_PORT->ODR |= SPI_OLED_CS_PIN

/*OLED命令定义********************************/
#define     SPI_OLDE_CONTRAST       0x81    /* 对比度 */
#define     SPI_OLED_OFF            0xAE    /* 关闭显示 */
#define     SPI_OLED_ON             0xAF    /* 开启显示 */
#define     SPI_OLED_CHARGE_PUMP    0x8D    /* 电荷泵 */
#define     SPI_OLED_PUMP_ON        0x14    /* 开启电荷泵 */
#define     SPI_OLED_PUMP_OFF       0x10    /* 关闭电荷泵 */

/*OLED参数定义********************************/
#define     PAGE_SIZE           8
#define     XLevelL             0x02
#define     XLevelH             0x10
#define     YLevel              0xB0
#define	    Brightness	        0xFF 
#define     WIDTH 	            128
#define     HEIGHT 	            64	
 						  
/*OLED显示参数********************************/
/*中文字库存储在FLASH的起始地址*/
// #define     GBKCODE_START_ADDRESS   387*4096

/*显示中英文字符参数 ***************************/
// #define      WIDTH_CH_CHAR	    16	        //中文字符宽度 
// #define      HEIGHT_CH_CHAR	    16		    //中文字符高度 
    
#define      WIDTH_EN_CHAR	    8	        //中文字符宽度 
#define      HEIGHT_EN_CHAR	    16		    //中文字符高度 


/*函数名重定义 *********************************/
#define     OLED_printf	            OLED_DispString_16X16_EN_CH


/*OLED设置函数 *********************************/
void bsp_OLED_Send_DATA(uint8_t data);
void bsp_OLED_Send_Cmd(uint8_t data);
void bsp_OLED_RESET(void);
void bsp_OLED_Display_On(void);
void bsp_OLED_Display_Off(void);
void bsp_OLED_SetContrast(uint8_t ContrastData);
void bsp_OLED_Set_Pos(uint8_t x, uint8_t y);
void bsp_OLED_Set_Pixel(uint8_t x, uint8_t y,uint8_t color);
void bsp_OLED_UP_Display(void);
void bsp_OLED_Clear(uint8_t dat);

/*OLED字模调用函数 *****************************/
// void GetGBKCode_from_EXFlash( uint8_t * pBuffer, uint16_t c);
// void ILI9341_DispChar_CH(uint8_t X,uint8_t Y,uint16_t CH_ch);
void ILI9341_DispChar_EN(uint8_t x,uint8_t y,char chr);
void OLED_DispString_16X16_EN_CH(uint8_t X, uint8_t Y, char *ch);

void bsp_OLED_Init(void);


#endif
