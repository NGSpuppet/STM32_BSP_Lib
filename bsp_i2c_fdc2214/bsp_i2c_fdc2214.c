
#include "bsp.h"
/*
    接线配置
传感器      主控
GND         GND
3V3         3V3
NC          悬空
NC          悬空
SDA         PB7     // 软件IIC 
SCL         PB6     // 软件IIC 
SD          GND
ADR         GND
INTB        GND
CLKIN       GND
*/
/*******************************************************************************
* 函数名	: FDC2214_Write_2Byte 
* 描述	    : 写寄存器数据
* 输入参数  : REG_add，value
* 返回参数  : 无
*******************************************************************************/
void FDC2214_Write_2Byte(uint8_t REG_add,uint16_t value)
{
    i2c_Start();  
    i2c_SendByte(FDC2214_Write);	        //ADDR=0时，地址0X2A<<1+0=0X54  
    i2c_WaitAck();              //等应答
    i2c_SendByte(REG_add);      //写地址
    i2c_WaitAck();              //等应答

    i2c_SendByte(value>>8);     //写高8位
    i2c_WaitAck();
    i2c_SendByte(value&0xff);   //写低8位
    i2c_WaitAck();
    i2c_Stop();                 //产生一个停止条件 
}
/*******************************************************************************
* 函数名	: FDC2214_Read_2Byte 
* 描述	    : 读寄存器数据
* 输入参数  : REG_add地址
* 返回参数  : 无
*******************************************************************************/
uint16_t FDC2214_Read_2Byte(uint8_t REG_add)
{
    uint8_t a,b;
    i2c_Start(); 
    i2c_SendByte(FDC2214_Write);	   //写命令ADDR=0
    i2c_WaitAck();
    i2c_SendByte(REG_add);     //地址
    i2c_WaitAck();
    i2c_Stop();

    i2c_Start();            //重新开始
    i2c_SendByte(FDC2214_Read);	   //发送读命令ADDR=0
    i2c_WaitAck(); 
    a=i2c_ReadByte();       //读高位
    i2c_Ack();
    b=i2c_ReadByte();		 //读低位
    i2c_NAck();
    i2c_Stop();
    return (uint16_t)((a<<8)+b);
}

/*
*函数说明： 读取FDC2214双寄存器数据
*输入形参： firstAddress：第一寄存器地址
*           secondAddress：第二寄存器地址
*输出数据： 双寄存器地址数据组合
*/
static uint32_t FDC2214_Read_2RegData(uint8_t firstAddress,uint8_t secondAddress)
{    
    uint16_t Register_data[2] = {0};
    
    Register_data[0] = FDC2214_Read_2Byte(firstAddress);
    Register_data[1] = FDC2214_Read_2Byte(secondAddress);

    return (uint32_t)((Register_data[0] << 16) + Register_data[1]); 
}

/*
*函数说明： 读取通道数据
*输入形参： index：通道索引（1~4）
*输出数据： 通道数据
*/
int FDC2214_Read_Channl_Data(uint8_t index)
{
	int result;
	switch(index)
	{
		case 0x01:
			result = FDC2214_Read_2RegData(FDC2214_DATA_CH0,FDC2214_DATA_LSB_CH0);
			break;
		case 0x02:
			result = FDC2214_Read_2RegData(FDC2214_DATA_CH1,FDC2214_DATA_LSB_CH1);
			break;
		case 0x03:
			result = FDC2214_Read_2RegData(FDC2214_DATA_CH2,FDC2214_DATA_LSB_CH2);
			break;
		case 0x04:
			result = FDC2214_Read_2RegData(FDC2214_DATA_CH3,FDC2214_DATA_LSB_CH3);
			break;
	}
	return result;
}

/*******************************************************************************
* 函数名	: FDC2214_SingleChannl_Init 
* 描述	    : 单通道初始化
* 输入参数  : 无
* 返回参数  : 无
*******************************************************************************/
void FDC2214_SingleChannl_Init(void)
{
    FDC2214_Write_2Byte(0x08,0xFFFF);   //设置转换时间  设置达到最高精度

    FDC2214_Write_2Byte(0x10,0x0064);   //FDC2214_SETTLECOUNT_CH0 计数

    FDC2214_Write_2Byte(0x14,0x2001);   //分频洗系数
    FDC2214_Write_2Byte(0x0C,0x0F00);   //调0值     CH0
    FDC2214_Write_2Byte(0x19,0x0000);   //ERROE_CONFIG
    
    FDC2214_Write_2Byte(0x1B,0x020D);   //通道配置
    FDC2214_Write_2Byte(0x1E,0xF800);   //配置驱动电流
    FDC2214_Write_2Byte(0x1A,0x1C81);   //参数配置
}
/*******************************************************************************
* 函数名	: FDC2214_MultiChannl_Init 
* 描述	    : 多通道初始化
* 输入参数  : 无
* 返回参数  : 无
*******************************************************************************/
void FDC2214_MultiChannl_Init(void)//双通道
{
    FDC2214_Write_2Byte(0x08,0x04D6);   //转换时间
  	FDC2214_Write_2Byte(0x09,0x04D6);
	FDC2214_Write_2Byte(0x0A,0x04D6);   //转换时间
  	FDC2214_Write_2Byte(0x0B,0x04D6);
	
	FDC2214_Write_2Byte(0x0C,0x0F00);   //调0值     CH0
	FDC2214_Write_2Byte(0x0D,0x0F00);   //调0值     CH1
	FDC2214_Write_2Byte(0x0E,0x0F00);   //调0值     CH2
	FDC2214_Write_2Byte(0x0F,0x0F00);   //调0值     CH3
  	
	FDC2214_Write_2Byte(0x10,0x000A);   //作用时间 CH1 
  	FDC2214_Write_2Byte(0x11,0x000A);   //CH2
	FDC2214_Write_2Byte(0x12,0x000A);   //CH3
	FDC2214_Write_2Byte(0x13,0x000A);   //CH4
    
	FDC2214_Write_2Byte(0x14,0x2002);   //分频
  	FDC2214_Write_2Byte(0x15,0x2002);   
    FDC2214_Write_2Byte(0x16,0x2002);   //分频
  	FDC2214_Write_2Byte(0x17,0x2002);   
	
  	FDC2214_Write_2Byte(0x19,0x0000);
  	FDC2214_Write_2Byte(0x1B,0xC20D);   //配置多通道   2通道--0x820D
	
  	FDC2214_Write_2Byte(0x1E,0x9000);   //驱动电流 CH0
  	FDC2214_Write_2Byte(0x1F,0x9000);   //CH1
	FDC2214_Write_2Byte(0x20,0x9000);   //CH2
  	FDC2214_Write_2Byte(0x21,0x9000);   //CH3
	FDC2214_Write_2Byte(0x1A,0x1C81);   //配置寄存器
}
	//Configuration register
	//	Active channel Select: b00 = ch0; b01 = ch1; b10 = ch2; b11 = ch3;
	//  |Sleep Mode: 0 - device active; 1 - device in sleep;
	//  ||Reserved, reserved, set to 1
	//  |||Sensor Activation Mode: 0 - drive sensor with full current. 1 - drive sensor with current set by DRIVE_CURRENT_CHn 
	//  ||||Reserved, set to 1
	//  |||||Reference clock: 0 - use internal; 1 - use external clock
	//  ||||||Reserved, set to 0
	//  |||||||Disable interrupt. 0 - interrupt output on INTB pin; 1 - no interrupt output
	//  ||||||||High current sensor mode: 0 - 1.5mA max. 1 - > 1.5mA, not available if Autoscan is enabled
	//  |||||||||     Reserved, set to 000001
	//  |||||||||     |
	// CCS1A1R0IH000000 -> 0001 1100 1000 0001 -> 0x1E81      0001 1100 1000 0001 -> 0x1C81

/*
*函数说明： FDC2214测试程序
*输入形参： 无
*输出数据： 无
*/
void FDC2214_Test(void)
{
    printf("CH0=%d\r\n",FDC2214_Read_Channl_Data(1));
    printf("CH1=%d\r\n",FDC2214_Read_Channl_Data(2));
    printf("CH2=%d\r\n",FDC2214_Read_Channl_Data(3));
    printf("CH3=%d\r\n",FDC2214_Read_Channl_Data(4));
    printf("\r\n");
}
