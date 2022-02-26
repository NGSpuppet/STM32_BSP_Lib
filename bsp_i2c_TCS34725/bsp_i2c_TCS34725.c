
#include "bsp.h"

/* 调用外部延时函数 */
extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

TCS34725_RGBC TCS34725_rgbc = {0};
TCS34725_HSL  TCS34725_hsl = {0};

/*
*********************************************************************************************************
*	函 数 名: 	TCS34725_Write_Byte
*	功能说明: 	向TCS34725的寄存器写入1byte数据
*	形    参: 	REG_Address：寄存器地址
*				write_byte：准备写入的数据
*	返 回 值: 	无
*********************************************************************************************************
*/
void TCS34725_Write_Byte(uint8_t REG_Address, uint8_t write_byte)
{
	REG_Address |= TCS34725_COMMAND_BIT;
	
	i2c_Start();
	i2c_SendByte(TCS34725_WRITE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(REG_Address);
	i2c_WaitAck();
	i2c_SendByte(write_byte);
	i2c_WaitAck();
	i2c_Stop();
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_Read_Byte
*	功能说明: 	向TCS34725的寄存器读取数据
*	形    参: 	REG_Address：寄存器地址
*				read_data：读取数据的保存指针
*	返 回 值: 无
*********************************************************************************************************
*/
void TCS34725_Read_Byte(uint8_t REG_Address, uint8_t *read_data)
{
	REG_Address |= TCS34725_COMMAND_BIT;
	
	i2c_Start();
	i2c_SendByte(TCS34725_WRITE_ADDRESS);
	i2c_WaitAck();
	i2c_SendByte(REG_Address);
	i2c_WaitAck();

	i2c_Start();
	i2c_SendByte(TCS34725_READ_ADDRESS);
	i2c_WaitAck();
	*read_data = i2c_ReadByte();
	i2c_NAck();
	i2c_Stop();
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_SetIntegrationTime
*	功能说明:	对TCS34725设置积分时间
*	形    参:	time：	积分时间枚举
*	返 回 值: 	无
*********************************************************************************************************
*/
void TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME time)
{
	TCS34725_Write_Byte(TCS34725_ATIME,time);
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_SetGain
*	功能说明:	对TCS34725设置增益
*	形    参:	gain：增益枚举
*	返 回 值: 	无
*********************************************************************************************************
*/
void TCS34725_SetGain(TCS34725_GAIN gain)
{
	TCS34725_Write_Byte(TCS34725_CONTROL, gain);
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_Enable
*	功能说明:	对TCS34725使能
*	形    参:	无
*	返 回 值: 	无
*********************************************************************************************************
*/
void TCS34725_Enable(void)
{
	TCS34725_Write_Byte(TCS34725_ENABLE,TCS34725_ENABLE_PON);
	delay_ms(1);
	TCS34725_Write_Byte(TCS34725_ENABLE,(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_Disable
*	功能说明:	对TCS34725失能
*	形    参:	无
*	返 回 值: 	无
*********************************************************************************************************
*/
void TCS34725_Disable(void)
{
	uint8_t disable_data;
	
	TCS34725_Read_Byte(TCS34725_ENABLE,&disable_data);
	disable_data &= ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
	
	TCS34725_Write_Byte(TCS34725_ENABLE,disable_data);
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_Read_ID
*	功能说明:	读取TCS34725的ID
*	形    参:	无
*	返 回 值: 	ID数据
*********************************************************************************************************
*/
uint8_t TCS34725_Read_ID(void)
{
	uint8_t TCS34725_ID_data;
	
	TCS34725_Read_Byte(TCS34725_ID,&TCS34725_ID_data);
	return TCS34725_ID_data;
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_GetChannelData
*	功能说明:	读取TCS34725的色彩通道数据
*	形    参:	RGB_Channel_addr：色彩通道寄存器地址
*	返 回 值: 	色彩数据
*********************************************************************************************************
*/
uint8_t TCS34725_GetChannelData(uint8_t RGB_Channel_addr)
{
	uint8_t Channel_data;
	
	TCS34725_Read_Byte(RGB_Channel_addr,&Channel_data);
	return Channel_data;
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_GetRawData
*	功能说明:	读取TCS34725的所有色彩通道数据
*	形    参:	无
*	返 回 值: 	1：读取成功，0：读取失败
*********************************************************************************************************
*/
uint8_t TCS34725_GetRawData(void)
{
	uint8_t status;
	
	TCS34725_Read_Byte(TCS34725_STATUS, &status);
	
	if(status & TCS34725_STATUS_AVALID)
	{
		TCS34725_rgbc.c = (uint16_t)(TCS34725_GetChannelData(TCS34725_CDATAH)<<8)+TCS34725_GetChannelData(TCS34725_CDATAL);
		TCS34725_rgbc.r = (uint16_t)(TCS34725_GetChannelData(TCS34725_RDATAH)<<8)+TCS34725_GetChannelData(TCS34725_RDATAL);
		TCS34725_rgbc.g = (uint16_t)(TCS34725_GetChannelData(TCS34725_GDATAH)<<8)+TCS34725_GetChannelData(TCS34725_GDATAL);
		TCS34725_rgbc.b = (uint16_t)(TCS34725_GetChannelData(TCS34725_BDATAH)<<8)+TCS34725_GetChannelData(TCS34725_BDATAL);
		

		return 1;
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_RGBtoHSL
*	功能说明:	将RGB值转换位HSL值
*	形    参:	无
*	返 回 值: 	无
*********************************************************************************************************
*/
void TCS34725_RGBtoHSL(void)
{
	uint8_t maxVal,minVal,difVal;
	uint8_t r = TCS34725_rgbc.r*100/TCS34725_rgbc.c;   //[0-100]
	uint8_t g = TCS34725_rgbc.g*100/TCS34725_rgbc.c;
	uint8_t b = TCS34725_rgbc.b*100/TCS34725_rgbc.c;

	maxVal = max3v(r,g,b);
	minVal = min3v(r,g,b);
	difVal = maxVal-minVal;
	
	TCS34725_hsl.l = (maxVal+minVal)/2;   //[0-100]
	
	if(maxVal == minVal)
	{
		TCS34725_hsl.h = 0; 
		TCS34725_hsl.s = 0;
	}
	else
	{
		if(maxVal==r)
		{
			if(g>=b)
				TCS34725_hsl.h = 60*(g-b)/difVal;
			else
				TCS34725_hsl.h = 60*(g-b)/difVal+360;
		}
		else
			{
				if(maxVal==g)
					TCS34725_hsl.h = 60*(b-r)/difVal+120;
				else if(maxVal==b)
					TCS34725_hsl.h = 60*(r-g)/difVal+240;
			}	
		if(TCS34725_hsl.l<=50)
			TCS34725_hsl.s=difVal*100/(maxVal+minVal);  //[0-100]
		else
			TCS34725_hsl.s=difVal*100/(200-(maxVal+minVal));
	}
}

/*
*********************************************************************************************************
*	函 数 名:	TCS34725_Init
*	功能说明:	初始化TCS34725
*	形    参:	无
*	返 回 值: 	无
*********************************************************************************************************
*/
void TCS34725_Init(void)
{
	if(i2c_CheckDevice(TCS34725_ADDRESS) == 0)	//
	{
		TCS34725_SetIntegrationTime(INTEGRATION_50MS);	//50ms
		TCS34725_SetGain(GAIN_1X);	
		TCS34725_Enable();	
	}
}

/*
*函数说明：	TCS34725测试程序
*输入形参：	无
*输出数据：	无
*/
void TCS34725_Test(void)
{
	TCS34725_GetRawData();
	TCS34725_RGBtoHSL();

	printf("读取ID：%X	\r\n",TCS34725_Read_ID());

	printf("读取RGB数据：R=%d G=%d B=%d C=%d\r\n",TCS34725_rgbc.r,TCS34725_rgbc.g,TCS34725_rgbc.b,TCS34725_rgbc.c);
	printf("读取HSL数据：H=%d S=%d L=%d\r\n",TCS34725_hsl.h,TCS34725_hsl.s,TCS34725_hsl.l);
}


