/*
*程序说明：该程序是模拟I2C传输程序
*
*应用说明：在访问I2C设备前，请先调用 i2c_CheckDevice() 检测I2C设备是否正常
*	
*/

#include "bsp.h"

/*
*********************************************************************************************************
*	函 数 名: bsp_InitI2C
*	功能说明: 配置I2C总线的GPIO，采用模拟IO的方式实现
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_I2C_SW_Init(void)
{
	GPIO_InitTypeDef gpio_init;

	/* 第1步：打开GPIO时钟 */
	ALL_I2C_GPIO_CLK_ENABLE;
	
	gpio_init.Mode = GPIO_MODE_OUTPUT_OD;			/* 设置开漏输出 */
	gpio_init.Pull = GPIO_PULLUP;					/* 使能上拉电阻 */
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;	/* GPIO速度等级 */
	
	/* IIC时钟引脚 */
	gpio_init.Pin = I2C_SCL_PIN;	
	HAL_GPIO_Init(I2C_SCL_GPIO, &gpio_init);	
	
	/* IIC数据引脚 */
	gpio_init.Pin = I2C_SDA_PIN;	
	HAL_GPIO_Init(I2C_SDA_GPIO, &gpio_init);	

	i2c_Stop();		/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Delay
*	功能说明: I2C总线位延迟，最快400KHz
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	/*　
		CPU主频168MHz时，在内部Flash运行, MDK工程不优化。用台式示波器观测波形。
		循环次数为5时，SCL频率 = 1.78MHz (读耗时: 92ms, 读写正常，但是用示波器探头碰上就读写失败。时序接近临界)
		循环次数为10时，SCL频率 = 1.1MHz (读耗时: 138ms, 读速度: 118724B/s)
		循环次数为30时，SCL频率 = 440KHz， SCL高电平时间1.0us，SCL低电平时间1.2us

		上拉电阻选择2.2K欧时，SCL上升沿时间约0.5us，如果选4.7K欧，则上升沿约1us

		实际应用选择400KHz左右的速率即可
	*/
	//for (i = 0; i < 30; i++);
	//for (i = 0; i < 60; i++);
	//bsp_DelayUS(2); 229.57KHz时钟
	bsp_DelayUS(2);
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线启动信号
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	I2C_SDA_1();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_0();
	i2c_Delay();
	
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Start
*	功能说明: CPU发起I2C总线停止信号
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参:  _ucByte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* 先发送字节的高位bit7 */
	for (i = 0; i < 8; i++)
	{
		if (_ucByte & 0x80)
		{
			I2C_SDA_1();
		}
		else
		{
			I2C_SDA_0();
		}
		i2c_Delay();
		I2C_SCL_1();
		i2c_Delay();
		I2C_SCL_0();
		I2C_SCL_0();	/* 2019-03-14 针对GT811电容触摸，添加一行，相当于延迟几十ns */
		if (i == 7)
		{
			 I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */	
	}
}

/*
*********************************************************************************************************
*	函 数 名: i2c_ReadByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参:  无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		I2C_SCL_1();
		i2c_Delay();
		if (I2C_SDA_READ())
		{
			value++;
		}
		I2C_SCL_0();
		i2c_Delay();
	}
	return value;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参:  无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU释放SDA总线 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
	{
		re = 1;
	}
	else
	{
		re = 0;
	}
	I2C_SCL_0();
	i2c_Delay();
	return re;
}

/*
*********************************************************************************************************
*	函 数 名: i2c_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU驱动SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU释放SDA总线 */
	
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU驱动SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU产生1个时钟 */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	函 数 名: i2c_CheckDevice
*	功能说明: 检测I2C总线设备，CPU向发送设备地址，然后读取设备应答来判断该设备是否存在
*	形    参:  _Address：设备的I2C总线地址(经过偏移)
*	返 回 值: 返回值 0 表示正确， 返回1表示未探测到
*********************************************************************************************************
*/
I2C_SW_STATE bsp_I2C_SW_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ())
	{
		i2c_Start();		/* 发送启动信号 */

		/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
		i2c_SendByte(_Address & 0xFE);
		ucAck = i2c_WaitAck();	/* 检测设备的ACK应答 */

		i2c_Stop();			/* 发送停止信号 */
		if(ucAck == 0)
		{
			return I2C_SW_OK;
		}
		else
			return I2C_SW_ERROR;
	}
	return I2C_SW_ERROR;	/* I2C总线异常 */
}


/*
*函数说明：	使用I2C向从地址寄存器写入一个字节数据
*输入形参：	slave_addr：从地址（经过左移）
*			reg_addr：寄存器地址
*			write_data：要写入的数据
*输出数据：	I2C_SW_OK：成功 I2C_SW_ERROR：失败
*/
I2C_SW_STATE bsp_I2C_SW_Write_Byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t write_data)
{
	i2c_Start();
	i2c_SendByte(slave_addr & 0xFE);
	if(i2c_WaitAck() != 0)	goto error;
	i2c_SendByte(reg_addr);
	if(i2c_WaitAck() != 0)	goto error;

	i2c_SendByte(write_data);
	if(i2c_WaitAck() != 0)	goto error;
	i2c_Stop();

	return I2C_SW_OK;

error: return I2C_SW_ERROR;
}

/*
*函数说明：	使用I2C向从地址寄存器读取一个字节数据
*输入形参：	slave_addr：从地址（经过左移）
*			reg_addr：寄存器地址
*			read_data：读取数据缓冲地址
*输出数据：	I2C_SW_OK：成功 I2C_SW_ERROR：失败
*/
I2C_SW_STATE bsp_I2C_SW_Read_Byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *read_data)
{
	i2c_Start();
	i2c_SendByte(slave_addr & 0xFE);
	if(i2c_WaitAck() != 0)	goto error;
	i2c_SendByte(reg_addr);
	if(i2c_WaitAck() != 0)	goto error;	

	i2c_Start();
	i2c_SendByte(slave_addr | 0x01);
	if(i2c_WaitAck() != 0)	goto error;
	*read_data = i2c_ReadByte();
	i2c_NAck();
	i2c_Stop();

	return I2C_SW_OK;

error: return I2C_SW_ERROR;
}

/*
*函数说明：	使用I2C向从地址寄存器写入n个字节数据
*补充说明：	使用连续写时，寄存器的地址会自动增加（I2C设备特性）
*输入形参：	slave_addr：从地址（经过左移）
*			reg_addr：寄存器地址
*			write_data：要写入的数据
*			Length：写入数量（1~65535）
*输出数据：	I2C_SW_OK：成功 I2C_SW_ERROR：失败
*/
I2C_SW_STATE bsp_I2C_SW_Write_nByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *write_data, uint16_t Length)
{
	i2c_Start();
	i2c_SendByte(slave_addr & 0xFE);
	if(i2c_WaitAck() != 0)	goto error;
	i2c_SendByte(reg_addr);
	if(i2c_WaitAck() != 0)	goto error;

	while(Length--)
	{
		i2c_SendByte(*write_data);
		if(i2c_WaitAck() != 0)	goto error;

		if(Length == 0)
		{
			i2c_Stop();
		}
		else
			write_data++;
		
	}
	return I2C_SW_OK;

error: return I2C_SW_ERROR;
}

/*
*函数说明：	使用I2C向从地址寄存器读取n个字节数据
*补充说明：	使用连续读时，寄存器的地址会自动增加（I2C设备特性）
*输入形参：	slave_addr：从地址（经过左移）
*			reg_addr：寄存器地址
*			read_data：读取数据缓冲地址
*			Length：读取数量（1~65535）
*输出数据：	I2C_SW_OK：成功 I2C_SW_ERROR：失败
*/
I2C_SW_STATE bsp_I2C_SW_Read_nByte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t Length)
{
	i2c_Start();
	i2c_SendByte(slave_addr & 0xFE);
	if(i2c_WaitAck() != 0)	goto error;
	i2c_SendByte(reg_addr);
	if(i2c_WaitAck() != 0)	goto error;

	i2c_Start();
	i2c_SendByte(slave_addr | 0x01);
	if(i2c_WaitAck() != 0)	goto error;
	while(Length--)
	{
		*read_data = i2c_ReadByte();
		if(Length == 0)
		{
			i2c_NAck();
			i2c_Stop();
		}
		else
		{
			read_data++;
			i2c_Ack();
		}
	}
	return I2C_SW_OK;

error: return I2C_SW_ERROR;
}

