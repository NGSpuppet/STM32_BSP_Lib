
#include "bsp_spi_sw.h"

extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

/* CP0L=1，CPHA=0，idle时为高电平，数据采样在第一个边沿，此时电平由高到低，下降沿；*/

/**
  * @brief :SPI初始化(软件)
  * @param :无
  * @note  :无
  * @retval:无
  */
void bsp_SPI_SW_Init( void )
{
	/* 开启时钟 */
	SPI_CLK_GPIO_CLK;
	SPI_MISO_GPIO_CLK;
	SPI_MOSI_GPIO_CLK;

	GPIO_InitTypeDef	SPI_GPIO_Struct = {0};
		
	//SCK MOSI配置为推挽输出
	SPI_GPIO_Struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	SPI_GPIO_Struct.Pull = GPIO_NOPULL;
	SPI_GPIO_Struct.Mode = GPIO_MODE_OUTPUT_PP;

	SPI_GPIO_Struct.Pin = SPI_CLK_GPIO_PIN;
	HAL_GPIO_Init(SPI_CLK_GPIO_PORT, &SPI_GPIO_Struct);	
	SPI_SET_CLK_L;

	SPI_GPIO_Struct.Pin = SPI_MOSI_GPIO_PIN;
	HAL_GPIO_Init( SPI_MOSI_GPIO_PORT, &SPI_GPIO_Struct );		//初始化MOSI
	SPI_SET_MOSI_L;
		
	//初始化MISO 上拉输入
	SPI_GPIO_Struct.Mode = GPIO_MODE_INPUT;
	SPI_GPIO_Struct.Pull = GPIO_PULLUP;
	SPI_GPIO_Struct.Pin = SPI_MISO_GPIO_PIN;
	HAL_GPIO_Init(SPI_MISO_GPIO_PORT, &SPI_GPIO_Struct);		
}

/*
*函数说明：	使用SPI读取一个字节数据
*输入形参：	无
*输出数据：	读取的字节
*/
uint8_t bsp_SPI_SW_Read_Byte(void)
{
	uint8_t Data = 0;

	SPI_SET_CLK_L;

	for(uint8_t i = 0; i < 8; i++)
	{
		SPI_SET_CLK_H;				//时钟线置高

		/** 接收 */
		Data <<= 1;					//接收数据左移一位，先接收到的是最高位
		if( 1 == SPI_GET_MISO)
		{
			Data |= 0x01;			//如果接收时IO引脚为高则认为接收到 1
		}
		SPI_SET_CLK_L;
	}
	return Data;
}

/*
*函数说明：	使用SPI发送一个字节的数据
*输入形参：	data：要发送的数据
*输出数据：	无
*/
void bsp_SPI_SW_Write_Byte(uint8_t data)
{	
    for(uint8_t i = 0; i < 8; i++)
    {
        SPI_SET_CLK_L;
        if(data & 0x80)
		{
			SPI_SET_MOSI_H; //从高位到低位
		}
		else 
			SPI_SET_MOSI_L;

        SPI_SET_CLK_H;
        data<<=1;
    }
    SPI_SET_CLK_L;

}

/**
  * @brief :SPI收发一个字节
  * @param :
  *			@TxByte: 发送的数据字节
  * @note  :非堵塞式，一旦等待超时，函数会自动退出
  * @retval:接收到的字节
  */
uint8_t bsp_SPI_SW_Read_Write_Byte(uint8_t TxByte)
{
	uint8_t Data = 0;
		
	for(uint8_t i = 0; i < 8; i++)			//一个字节8byte需要循环8次
	{
		SPI_SET_CLK_L;				//时钟线置低
		/** 发送 */
		if(TxByte & 0x80)
		{
			SPI_SET_MOSI_H;			//如果即将要发送的位为 1 则置高IO引脚
		}
		else
			SPI_SET_MOSI_L;			//如果即将要发送的位为 0 则置低IO引脚

		TxByte <<= 1;				//数据左移一位，先发送的是最高位
		SPI_SET_CLK_H;				//时钟线置高
		/** 接收 */
		Data <<= 1;					//接收数据左移一位，先接收到的是最高位
		if( 1 == SPI_GET_MISO)
		{
			Data |= 0x01;			//如果接收时IO引脚为高则认为接收到 1
		}
	}
	SPI_SET_CLK_L;				//时钟线置低
	return Data;					//返回接收到的字节
}

/*
*函数说明：	使用SPI批量读取读取数据
*输入形参：	ReadBuffer：读取数据缓冲地址
*			Length:数量（1~65535）
*输出数据：	无
*/
void bsp_SPI_SW_Read_Block(uint8_t* ReadBuffer, uint16_t Length)
{
	while(Length--)
	{
		*ReadBuffer = bsp_SPI_SW_Read_Byte();
		ReadBuffer++;
	}
}

/*
*函数说明：	使用SPI批量发送读取数据
*输入形参：	ReadBuffer：发送数据缓冲地址
*			Length:数量（1~65535）
*输出数据：	无
*/
void bsp_SPI_SW_Write_Block(uint8_t* WriteBuffer, uint16_t Length)
{
	while(Length--)
	{
		bsp_SPI_SW_Read_Write_Byte(*WriteBuffer);
		WriteBuffer++;
	}
}


 /*
 *函数说明： 使用SPI批量读写数据
 *输入形参： ReadBuffer：读取数据缓冲区地址
 *			WriteBuffer:发送字节缓冲区地址
 *			Length:数量（1~65535）
 *输出数据： 无
 */
void bsp_SPI_SW_Read_Write_Block(uint8_t* ReadBuffer, uint8_t* WriteBuffer, uint16_t Length)
{
	while(Length--)
	{
		*ReadBuffer = bsp_SPI_SW_Read_Write_Byte(*WriteBuffer);		//收发数据
		ReadBuffer++;
		WriteBuffer++;
	}
}


