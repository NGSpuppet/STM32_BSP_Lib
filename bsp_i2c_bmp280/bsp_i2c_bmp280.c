/*
*程序说明：SD0引脚的电平会影响I2C地址，根据连接的电平更改“bsp_i2c_bmp280.h”
			中的BMP_SDO_Level电平宏来更改使用的I2C地址(默认为低电平)

*/


#include "bsp.h"
#include "math.h"		/* 计算海拔高度需要调用 */

BMP280_Typedef bmp280 = {0};		//这个全局结构体变量用来保存存在芯片内ROM补偿参数

/*
*函数说明： 向BMP280的寄存器写入数据
*输入形参： REG_addr：寄存器地址
*          data：要写入的数据
*输出数据： 无
*/
void BMP280_Write_Byte(uint8_t REG_addr,uint8_t data)
{
	// i2c_Start();
	// i2c_SendByte(BMP280_Write_ADDRESS);
	// i2c_WaitAck();
	// i2c_SendByte(REG_addr);
	// i2c_WaitAck();
	
	// i2c_SendByte(data);
	// i2c_WaitAck();
	// i2c_Stop();

	bsp_I2C_SW_Write_Byte(BMP280_ADDRESS, REG_addr, data);
}

/*
*函数说明： 向BMP280的寄存器读出数据
*输入形参： REG_addr：寄存器地址
*输出数据： 读取的数据
*/
uint8_t BMP280_Read_Byte(uint8_t REG_addr)
{
	uint8_t rec_data = 0;
	// i2c_Start();
	// i2c_SendByte(BMP280_Write_ADDRESS);
	// i2c_WaitAck();
	// i2c_SendByte(REG_addr);
	// i2c_WaitAck();
	
	// i2c_Start();
	// i2c_SendByte(BMP280_Read_ADDRESS);
	// i2c_WaitAck();
	// rec_data = i2c_ReadByte();
    // i2c_NAck();	//不应答
	// i2c_Stop();

	bsp_I2C_SW_Read_Byte(BMP280_ADDRESS, REG_addr, &rec_data);
	return rec_data;
}
/*
*函数说明： 向BMP280读取传感器ID
*输入形参： 无
*输出数据： 传感器ID
*/
uint8_t BMP280_Read_ID(void)
{
	return BMP280_Read_Byte(BMP280_CHIPID_REG);
}

/*
*函数说明： 获取BMP当前状态
*输入形参： status_flag：要检测的标志（BMP280_MEASURING：测量完成）（BMP280_IM_UPDATE：更新完成）
*输出数据： SET or RESET
*/
uint8_t BMP280_GetStatus(uint8_t status_flag)
{
	uint8_t flag;

	flag = BMP280_Read_Byte(BMP280_STATUS_REG);
	if(flag&status_flag)
    {
        return SET;
    }
	else
        return RESET;
}

/*
*函数说明： 设置BMP过采样因子（MODE）
*输入形参： Oversample_Mode：过采样因子结构体
*输出数据： 无
*/
void BMP280_Set_TemOversamp(BMP_OVERSAMPLE_MODE* Oversample_Mode)
{
	uint8_t Regtmp;

	Regtmp = ((Oversample_Mode->T_Osample)<<5)|
			 ((Oversample_Mode->P_Osample)<<2)|
			 ((Oversample_Mode)->WORKMODE);
	
	BMP280_Write_Byte(BMP280_CTRLMEAS_REG,Regtmp);
}

/*
*函数说明： 设置BMP保持时间和滤波器分频因子
*输入形参： BMP_Config：保持时间和滤波器分频因子结构体
*输出数据： 无
*/
void BMP280_Set_Standby_FILTER(BMP_CONFIG* BMP_Config)
{
	uint8_t Regtmp;

	Regtmp = ((BMP_Config->T_SB)<<5)|
			 ((BMP_Config->FILTER_COEFFICIENT)<<2)|
			 ((BMP_Config->SPI_EN));
	
	BMP280_Write_Byte(BMP280_CONFIG_REG,Regtmp);
}

/**************************传感器值转定点值*************************************/
BMP280_S32_t t_fine;			//用于计算补偿
//我用浮点补偿
#ifdef USE_FIXED_POINT_COMPENSATE
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
// t_fine carries fine temperature as global value
static BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
	BMP280_S32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig_T1))) >> 12) * 
	((BMP280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
{
	BMP280_S64_t var1, var2, p;
	var1 = ((BMP280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
	var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
	if (var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
	return (BMP280_U32_t)p;
}


// /***********************************CUT*************************************/
#else
/**************************传感器值转定点值*************************************/
//温度返回为摄氏度，双精度。“51.23”的产值等于51.23℃。
// t_fine carries fine temperature as global value
static double bmp280_compensate_T_double(BMP280_S32_t adc_T)
{
	double var1, var2, T;

	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

// 返回Pa的压力为双精度。“96386.2”的输出值等于96386.2 Pa = 963.862 hPa
static double bmp280_compensate_P_double(BMP280_S32_t adc_P)
{
	double var1, var2, p;

	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}
#endif

/*******************主要部分*********************/
/****************获取传感器精确值****************/

/*
*函数说明： 获取大气压值
*输入形参： 无
*输出数据： 大气压值（Pa）
*/
double BMP280_Get_Pressure(void)
{
	uint8_t XLsb,Lsb, Msb;
	long signed Bit32;
	double pressure;

	XLsb = BMP280_Read_Byte(BMP280_PRESSURE_XLSB_REG);
	Lsb	 = BMP280_Read_Byte(BMP280_PRESSURE_LSB_REG);
	Msb	 = BMP280_Read_Byte(BMP280_PRESSURE_MSB_REG);
	Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);	//寄存器的值,组成一个浮点数
	pressure = bmp280_compensate_P_double(Bit32);
	return pressure;
}

/*
*函数说明：	获取标准大气压值
*输入形参：	无
*输出数据：	标准大气压
*/
double BMP280_Get_StandardPressure(void)
{
	return (BMP280_Get_Pressure()/101325);
}

/*
*函数说明：	通过气压值计算海拔高度
*输入形参：	无
*输出数据：	海拔的高度
*/
double BMP280_Get_Altitude(void)
{
	return 44300*(1-(pow(BMP280_Get_StandardPressure(),(1/5.256))));
}

/*
*函数说明： 获取BMP280温度
*输入形参： 无
*输出数据： 温度值（℃）
*/
double BMP280_Get_Temperature(void)
{
	uint8_t XLsb,Lsb, Msb;
	long signed Bit32;
	double temperature;

	XLsb = BMP280_Read_Byte(BMP280_TEMPERATURE_XLSB_REG);
	Lsb	 = BMP280_Read_Byte(BMP280_TEMPERATURE_LSB_REG);
	Msb	 = BMP280_Read_Byte(BMP280_TEMPERATURE_MSB_REG);
	Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);	//寄存器的值,组成一个浮点数
	temperature = bmp280_compensate_T_double(Bit32);
	return temperature;
}

/*
*函数说明： 对BMP进行初始化
*输入数据： 无
*输出数据： 无
*/
void BMP280_Init(void)
{
	uint8_t Lsb,Msb;
	
	/********************接下来读出矫正参数*********************/
	//温度传感器的矫正值
	Lsb = BMP280_Read_Byte(BMP280_DIG_T1_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_T1_MSB_REG);
	bmp280.T1 = (((uint16_t)Msb)<<8) + Lsb;			//高位加低位
	Lsb = BMP280_Read_Byte(BMP280_DIG_T2_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_T2_MSB_REG);
	bmp280.T2 = (((uint16_t)Msb)<<8) + Lsb;		
	Lsb = BMP280_Read_Byte(BMP280_DIG_T3_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_T3_MSB_REG);
	bmp280.T3 = (((uint16_t)Msb)<<8) + Lsb;		
	
	//大气压传感器的矫正值
	Lsb = BMP280_Read_Byte(BMP280_DIG_P1_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P1_MSB_REG);
	bmp280.P1 = (((uint16_t)Msb)<<8) + Lsb;		
	Lsb = BMP280_Read_Byte(BMP280_DIG_P2_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P2_MSB_REG);
	bmp280.P2 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P3_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P3_MSB_REG);
	bmp280.P3 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P4_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P4_MSB_REG);
	bmp280.P4 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P5_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P5_MSB_REG);
	bmp280.P5 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P6_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P6_MSB_REG);
	bmp280.P6 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P7_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P7_MSB_REG);
	bmp280.P7 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P8_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P8_MSB_REG);
	bmp280.P8 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = BMP280_Read_Byte(BMP280_DIG_P9_LSB_REG);
	Msb = BMP280_Read_Byte(BMP280_DIG_P9_MSB_REG);
	bmp280.P9 = (((uint16_t)Msb)<<8) + Lsb;	
	/******************************************************/
	BMP280_Write_Byte(BMP280_RESET_REG,BMP280_RESET_VALUE);	//往复位寄存器写入给定值
	
	BMP_OVERSAMPLE_MODE			BMP_OVERSAMPLE_MODEStructure;
	BMP_OVERSAMPLE_MODEStructure.P_Osample = BMP280_P_MODE_3;
	BMP_OVERSAMPLE_MODEStructure.T_Osample = BMP280_T_MODE_1;
	BMP_OVERSAMPLE_MODEStructure.WORKMODE  = BMP280_NORMAL_MODE;
	BMP280_Set_TemOversamp(&BMP_OVERSAMPLE_MODEStructure);
	
	BMP_CONFIG					BMP_CONFIGStructure;
	BMP_CONFIGStructure.T_SB = BMP280_T_SB1;
	BMP_CONFIGStructure.FILTER_COEFFICIENT = BMP280_FILTER_MODE_4;
	BMP_CONFIGStructure.SPI_EN = DISABLE;
	BMP280_Set_Standby_FILTER(&BMP_CONFIGStructure);
}

/*
*函数说明： BMP280测试函数
*输入数据： 无
*输出数据： 无
*/
void BMP280_Test(void)
{		
	while(BMP280_GetStatus(BMP280_MEASURING) != RESET);		/* 等待转换完成 */
	while(BMP280_GetStatus(BMP280_IM_UPDATE) != RESET);		/* 等待更新完成 */
	
	printf("读取ID：%X  \r\n",BMP280_Read_ID());					/* 读取ID */
	printf("Temperature %f C\r\n",BMP280_Get_Temperature());		/* 读取温度 */
	printf("海拔高度：%f m\r\n",BMP280_Get_Altitude());
	// printf("StandardPressure %f Pa\r\n",BMP280_Get_StandardPressure());	/* 读取标准大气压 */
	// printf("Pressure %f Pa\r\n",BMP280_Get_Pressure());		/* 读取大气压 */

	printf("\r\n");
}


