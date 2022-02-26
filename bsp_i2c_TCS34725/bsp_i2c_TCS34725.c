
#include "bsp.h"

/* �����ⲿ��ʱ���� */
extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

TCS34725_RGBC TCS34725_rgbc = {0};
TCS34725_HSL  TCS34725_hsl = {0};

/*
*********************************************************************************************************
*	�� �� ��: 	TCS34725_Write_Byte
*	����˵��: 	��TCS34725�ļĴ���д��1byte����
*	��    ��: 	REG_Address���Ĵ�����ַ
*				write_byte��׼��д�������
*	�� �� ֵ: 	��
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
*	�� �� ��:	TCS34725_Read_Byte
*	����˵��: 	��TCS34725�ļĴ�����ȡ����
*	��    ��: 	REG_Address���Ĵ�����ַ
*				read_data����ȡ���ݵı���ָ��
*	�� �� ֵ: ��
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
*	�� �� ��:	TCS34725_SetIntegrationTime
*	����˵��:	��TCS34725���û���ʱ��
*	��    ��:	time��	����ʱ��ö��
*	�� �� ֵ: 	��
*********************************************************************************************************
*/
void TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME time)
{
	TCS34725_Write_Byte(TCS34725_ATIME,time);
}

/*
*********************************************************************************************************
*	�� �� ��:	TCS34725_SetGain
*	����˵��:	��TCS34725��������
*	��    ��:	gain������ö��
*	�� �� ֵ: 	��
*********************************************************************************************************
*/
void TCS34725_SetGain(TCS34725_GAIN gain)
{
	TCS34725_Write_Byte(TCS34725_CONTROL, gain);
}

/*
*********************************************************************************************************
*	�� �� ��:	TCS34725_Enable
*	����˵��:	��TCS34725ʹ��
*	��    ��:	��
*	�� �� ֵ: 	��
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
*	�� �� ��:	TCS34725_Disable
*	����˵��:	��TCS34725ʧ��
*	��    ��:	��
*	�� �� ֵ: 	��
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
*	�� �� ��:	TCS34725_Read_ID
*	����˵��:	��ȡTCS34725��ID
*	��    ��:	��
*	�� �� ֵ: 	ID����
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
*	�� �� ��:	TCS34725_GetChannelData
*	����˵��:	��ȡTCS34725��ɫ��ͨ������
*	��    ��:	RGB_Channel_addr��ɫ��ͨ���Ĵ�����ַ
*	�� �� ֵ: 	ɫ������
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
*	�� �� ��:	TCS34725_GetRawData
*	����˵��:	��ȡTCS34725������ɫ��ͨ������
*	��    ��:	��
*	�� �� ֵ: 	1����ȡ�ɹ���0����ȡʧ��
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
*	�� �� ��:	TCS34725_RGBtoHSL
*	����˵��:	��RGBֵת��λHSLֵ
*	��    ��:	��
*	�� �� ֵ: 	��
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
*	�� �� ��:	TCS34725_Init
*	����˵��:	��ʼ��TCS34725
*	��    ��:	��
*	�� �� ֵ: 	��
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
*����˵����	TCS34725���Գ���
*�����βΣ�	��
*������ݣ�	��
*/
void TCS34725_Test(void)
{
	TCS34725_GetRawData();
	TCS34725_RGBtoHSL();

	printf("��ȡID��%X	\r\n",TCS34725_Read_ID());

	printf("��ȡRGB���ݣ�R=%d G=%d B=%d C=%d\r\n",TCS34725_rgbc.r,TCS34725_rgbc.g,TCS34725_rgbc.b,TCS34725_rgbc.c);
	printf("��ȡHSL���ݣ�H=%d S=%d L=%d\r\n",TCS34725_hsl.h,TCS34725_hsl.s,TCS34725_hsl.l);
}


