
#include "bsp.h"

//调用外部延时函数
extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

//传感器配置寄存器变量
uint8_t Set_Register;

/*
*函数说明：	初始化VEML6070的INT引脚
*输入形参：	无
*输出数据：	无
*/
static void VEML6070_INT_GPIO_Config(void)
{
	GPIO_InitTypeDef	INT_GPIO = {0};
	
	VEML6070_INT_CLK;

	INT_GPIO.Mode = GPIO_MODE_INPUT;
	INT_GPIO.Pull  = GPIO_NOPULL;
	INT_GPIO.Pin = VEML6070_PIN;

	HAL_GPIO_Init(VEML6070_PORT,&INT_GPIO);
}

/*
*函数说明：	向VEML6070的寄存器读取一个字节数据
*输入形参：	REG_Addr：寄存器地址
*输出数据：	寄存器数据
*/
static uint8_t VEML6070_Read_Byte(uint8_t REG_Addr)
{
	uint8_t byteData;
	
	i2c_Start();
	i2c_SendByte(REG_Addr | I2C_RD);
	i2c_WaitAck();
	byteData = i2c_ReadByte();
	i2c_Ack();
	i2c_Stop();
	return byteData;
}

/*
*函数说明：	向VEML6070写入一个字节数据
*输入形参：	data：要写入的数据
*输出数据：	无
*/
static void VEML6070_Write_Byte(uint8_t data)
{
	i2c_Start();
	i2c_SendByte(VEML6070_Set_ADDR);
	i2c_WaitAck();
	i2c_SendByte(data);
	i2c_WaitAck();
	i2c_Stop();
}

/*
*函数说明：	向VEML6070读取紫外光强度数据
*输入形参：	无
*输出数据：	16位的紫外光强度数据
*/
uint16_t VEML6070_Read_Light(void)
{
	uint16_t H,L;
	H = VEML6070_Read_Byte(VEML6070_Hlight_ADDR);
	L = VEML6070_Read_Byte(VEML6070_Llight_ADDR);
	return ((H <<= 8) | L);
}


/*
*函数说明：	读取VEML6070的中断状态
*输入形参：	无
*输出数据：	VEML6070的中断状态
*/
uint8_t VEML6070_Read_INT_State(void)
{
	return VEML6070_Read_Byte(VEML6070_ARA_ADDR);
}


/*
*函数说明：	设置VEML6070的ACK位是否使能位
*输入形参：	State：使能枚举
*输出数据：	无
*/
void VEML6070_Set_ACK_Bit(FunctionalState State)
{
	if(State == ENABLE)
		Set_Register |= 1<<5;
	else
		Set_Register &= ~(1<<5);
	VEML6070_Write_Byte(Set_Register);
}

/*
*函数说明：	设置VEML6070的ACK阈值位
*输入形参：	Steps：阈值枚举
*输出数据：	无
*/
void Set_VEML6070_ACK_THD_Bit(ACK_THD_Steps Steps)
{
	if(Steps == ACK_THD_145_Steps)
		Set_Register |= 1<<4;
	else
		Set_Register &= ~(1<<4);
	
	VEML6070_Write_Byte(Set_Register);
}

/*
*函数说明：	设置VEML6070的积分时间位
*输入形参：	Time：积分时间枚举
*输出数据：	无
*/
void VEML6070_Set_IT_Bit(IT_Sampling_Period Time)
{
	switch(Time)
	{
		case Time_4_T:
			Set_Register |= 3<<2;
			break;
		case Time_2_T:
			Set_Register &= ~(3<<2);
			Set_Register |= 1<<3;
			break;
		case Time_1_T:
			Set_Register &= ~(3<<2);
			Set_Register |= 1<<2;
			break;
		default:
			Set_Register &= ~(3<<2);
	}
	VEML6070_Write_Byte(Set_Register);
}

/*
*函数说明：	设置VEML6070的是否关机位
*输入形参：	State：使能枚举
*输出数据：	无
*/
void Set_VEML6070_SD_Bit(FunctionalState State)
{
	if(State == ENABLE)
		Set_Register |= 1;
	else
		Set_Register &= 0;
	
	VEML6070_Write_Byte(Set_Register);
}

/*
*函数说明：	初始化VEML6070
*输入数据：	无
*输出数据：	无
*/
void VEML6070_Config(void)
{
	Set_Register = 0;

#if VEML6070_INT_ENABLE
	VEML6070_INT_GPIO_Config();
#endif
	VEML6070_Write_Byte(Set_Register);

	VEML6070_Set_IT_Bit(Time_1_T);
}

/*
*函数说明：	VEML6070测序程序
*输入形参：	无
*输出数据：	无
*/
void VEML6070_Test(void)
{
	printf("紫外线亮度数据%d \r\n",VEML6070_Read_Light());
}


