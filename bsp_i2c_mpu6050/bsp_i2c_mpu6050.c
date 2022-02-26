/*
*程序说明：程序加入了DMP库和原始库数据的选择，在“bsp_i2c_mpu6050.h”
			中修改mpu6050_NODMP的宏定义来选择使用DMP库还是原始库
			程序加入了硬件I2C和软件I2C的选择，在“bsp_i2c_mpu6050.h”
			中修改MPU6050_MPU6050_I2C_SW_ENABLE的的宏定义来选择使用硬件I2C还是软件I2C
*/


#include "bsp.h"

#ifdef MPU6050_I2C_SW_ENABLE
#include "bsp_i2c_sw.h"		/* 调用软件I2C */
#else
#include "bsp_i2c_hw.h"		/* 调用硬件I2C */
#endif
/*************************** 头文件调用 ***************************/
#ifdef mpu6050_NODMP
/* 原始库调用 */
#include "bsp_i2c_mpu6050.h"
#else
/* DMP库调用 */
#include "inv_mpu.h"	/* 使用时，要在该头文件修改工作频率 */
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#endif

/*************************** 结构体声明 ***************************/
#ifdef mpu6050_NODMP
MPU6050_TypeDef g_tMPU6050 = {0};		/* MPU6050原始数据结构体 */
#else
Posture_TypeDef	Posture_Struct = {0};	/* MPU6050姿态数据结构体 */
#endif

#ifdef mpu6050_NODMP
/*
*********************************************************************************************************
*	函 数 名: bsp_InitMPU6050
*	功能说明: 初始化MPU-6050
*	形    参:  无
*	返 回 值: 1 表示正常， 0 表示不正常
*********************************************************************************************************
*/
void bsp_MPU6050_Init(void)
{
	MPU6050_Write_Byte(MPU6050_PWR_MGMT_1, 0x00);	//解除休眠状态
	MPU6050_Write_Byte(MPU6050_SMPLRT_DIV, 0x07);	//配置陀螺仪采样率
	MPU6050_Write_Byte(MPU6050_CONFIG, 0x06);		//配置低通滤波频率
	MPU6050_Write_Byte(MPU6050_GYRO_CONFIG, 0xE8);
	MPU6050_Write_Byte(MPU6050_ACCEL_CONFIG, 0x01);
}

/*
*********************************************************************************************************
*	函 数 名: MPU6050_WriteByte
*	功能说明: 向 MPU-6050 寄存器写入一个数据
*	形    参: _ucRegAddr : 寄存器地址
*			  _ucRegData : 寄存器数据
*	返 回 值: 无
*********************************************************************************************************
*/
void MPU6050_Write_Byte(uint8_t _ucRegAddr, uint8_t _ucRegData)
{
#ifdef MPU6050_I2C_SW_ENABLE
	/* 软件I2C写数据 */
	bsp_I2C_SW_Write_Byte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, _ucRegData);
#else
	/* 硬件I2C写数据 */
	bsp_I2C_HW_Write_Byte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, _ucRegData);
#endif
}

/*
*********************************************************************************************************
*	函 数 名: MPU6050_Read_Byte
*	功能说明: 读取 MPU-6050 寄存器的数据
*	形    参: _ucRegAddr : 寄存器地址
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t MPU6050_Read_Byte(uint8_t _ucRegAddr)
{
	uint8_t ucData;
#ifdef MPU6050_I2C_SW_ENABLE
	/* 软件I2C读数据 */
	bsp_I2C_SW_Read_Byte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, &ucData);
#else
	/* 硬件I2C读数据 */
	bsp_I2C_HW_Read_Byte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, &ucData);
#endif
	return ucData;
}

/*
*函数说明：	向MPU6050写入n个字节数据
*输入形参：	_ucRegAddr：寄存器地址
*			write_data：要写入字节的地址
*			length：写入数量（1~65535）
*输出数据：	无
*/
void MPU6050_Write_nByte(uint8_t _ucRegAddr, uint8_t *write_data, uint16_t length)
{
#ifdef MPU6050_I2C_SW_ENABLE
	/* 软件I2C写入数据 */
	bsp_I2C_SW_Write_nByte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, write_data, length);
#else
	/* 硬件I2C写入数据 */
	bsp_I2C_HW_Write_nByte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, write_data, length);
#endif
}

/*
*函数说明： 向MPU6050读取n个字节的数据
*输入形参：	_ucRegAddr：寄存器地址
*			read_data：读取字节的保存地址
*			length：写入数量（1~65535）	
*/
void MPU6050_Read_nByte(uint8_t _ucRegAddr, uint8_t *read_data, uint16_t length)
{
#ifdef	MPU6050_I2C_SW_ENABLE
	/* 软件I2C读取数据 */
	bsp_I2C_SW_Read_nByte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, read_data, length);
#else
	/* 硬件I2C读取数据 */
	bsp_I2C_HW_Read_nByte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, read_data, length);
#endif
}

/*
*函数说明：	读取MPU6050传感器的ID
*输入形参：	无
*输出数据：	传感器ID
*/
uint8_t MPU6050_Read_ID(void)
{
	/* 返回读取的ID值 */
	return MPU6050_Read_Byte(MPU6050_WHO_AM_I);
}

/*
*函数说明：	读取MPU6050传感器的温度
*输入形参：	无
*输出数据：	传感器的温度
*/
float MPU6050_Read_Temp(void)
{
	int16_t temp_original = 0;

	/* 获取温度原始数据 */
	temp_original = (MPU6050_Read_Byte(MPU6050_TEMP_OUT_H)<<8) | MPU6050_Read_Byte(MPU6050_TEMP_OUT_L);

	/* 计算并返回实际温度 */
	return (36.53f+(float)temp_original/340);
}

/*
*函数说明：	向MPU6050读取加速度数据
*输入形参：	axis_char：要读取的轴（'X'or'x'、'Y'or'y'、'Z'or'z'）
*输出数据：	当前轴的加速度值
*/
int16_t MPU6050_Read_Accel(char axis_char)
{
	int16_t accel_data = 0;

	switch(axis_char)
	{
		case 'X':
		case 'x':
			accel_data = (MPU6050_Read_Byte(MPU6050_ACCEL_XOUT_H) << 8) | MPU6050_Read_Byte(MPU6050_ACCEL_XOUT_L);
			break;

		case 'Y':
		case 'y':
			accel_data = (MPU6050_Read_Byte(MPU6050_ACCEL_YOUT_H) << 8) | MPU6050_Read_Byte(MPU6050_ACCEL_YOUT_L);
			break;

		case 'Z':
		case 'z':
			accel_data = (MPU6050_Read_Byte(MPU6050_ACCEL_ZOUT_H) << 8) | MPU6050_Read_Byte(MPU6050_ACCEL_ZOUT_L);
			break;

		default:
			Error_Handler(__FILE__, __LINE__);		/* 不在范围内 */
			break;
	}
	return accel_data;
}

/*
*函数说明：	向MPU6050读取陀螺仪数据
*输入形参：	axis_char：要读取的轴（'X'or'x'、'Y'or'y'、'Z'or'z'）
*输出数据：	当前轴的陀螺仪值
*/
int16_t MPU6050_Read_Gyro(char axis_char)
{
	int16_t accel_data = 0;

	switch(axis_char)
	{
		case 'X':
		case 'x':
			accel_data = (MPU6050_Read_Byte(MPU6050_GYRO_XOUT_H) << 8) | MPU6050_Read_Byte(MPU6050_GYRO_XOUT_L);
			break;

		case 'Y':
		case 'y':
			accel_data = (MPU6050_Read_Byte(MPU6050_GYRO_YOUT_H) << 8) | MPU6050_Read_Byte(MPU6050_GYRO_YOUT_L);
			break;

		case 'Z':
		case 'z':
			accel_data = (MPU6050_Read_Byte(MPU6050_GYRO_ZOUT_H) << 8) | MPU6050_Read_Byte(MPU6050_GYRO_ZOUT_L);
			break;

		default:
			Error_Handler(__FILE__, __LINE__);		/* 不在范围内 */
			break;
	}
	return accel_data;
}


/*
*函数说明：	MPU6050读取所有数据（加数据、陀螺仪、温度）
*输入形参：	无
*输出数据：	无
*/
void MPU6050_Read_AllData(void)
{
	/* 将读出的数据保存到全局结构体变量 */
	g_tMPU6050.Accel_X = MPU6050_Read_Accel('X');
	g_tMPU6050.Accel_Y = MPU6050_Read_Accel('Y');
	g_tMPU6050.Accel_Z = MPU6050_Read_Accel('Z');

	g_tMPU6050.Temp = MPU6050_Read_Temp();

	g_tMPU6050.GYRO_X = MPU6050_Read_Gyro('X');
	g_tMPU6050.GYRO_Y = MPU6050_Read_Gyro('Y');
	g_tMPU6050.GYRO_Z = MPU6050_Read_Gyro('Z');
}

/*
*函数说明：	MPU6050测试函数
*输入形参：	无
*输出数据：	无
*/
void MPU6050_Test(void)
{
	MPU6050_Read_AllData();
	printf("MPU6050 X轴加速度：%d \r\n",g_tMPU6050.Accel_X);
	printf("MPU6050 Y轴加速度：%d \r\n",g_tMPU6050.Accel_Y);
	printf("MPU6050 Z轴加速度：%d \r\n",g_tMPU6050.Accel_Z);
	printf("MPU6050 温度：%f \r\n",g_tMPU6050.Temp);
	printf("MPU6050 X轴陀螺仪：%d \r\n",g_tMPU6050.GYRO_X);
	printf("MPU6050 Y轴陀螺仪：%d \r\n",g_tMPU6050.GYRO_Y);
	printf("MPU6050 Z轴陀螺仪：%d \r\n",g_tMPU6050.GYRO_Z);
	printf("\r\n");
}
#else
/******************************* DMP库移植函数创建 *******************************/
/*
*函数说明：	向MPU6050中写入多个数据
*输入形参：	slave_addr：I2C地址(未偏移地址)
*			reg_addr：寄存器地址
*			length：写入的数据个数
*			data：要写入的数据地址
*输出数据：	0：成功 1：失败
*/
uint8_t MPU6050_WriteReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
#ifdef MPU6050_I2C_SW_ENABLE
	/* 软件I2C */
	if(bsp_I2C_SW_Write_nByte(slave_addr<<1, reg_addr, (uint8_t*)data, length) != I2C_SW_OK)
		Error_Handler(__FILE__, __LINE__);
#else
	/* 硬件I2C */	
	if(bsp_I2C_HW_Write_nByte(slave_addr<<1, reg_addr, (uint8_t*)data, (uint16_t)length) != HAL_OK)
		Error_Handler(__FILE__, __LINE__);
#endif
	return 0;
}

/*
*函数说明：	从MPU6050中读取多个数据
*输入形参：	slave_addr：I2C地址(未偏移地址)
*			reg_addr：寄存器地址
*			length：读取的数据个数
*			data：读取数据的保存地址
*输出数据：	0：成功 1：失败
*/
uint8_t MPU6050_ReadData(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data)
{
#ifdef MPU6050_I2C_SW_ENABLE
	/* 软件I2C */
	if(bsp_I2C_SW_Read_nByte(slave_addr<<1, reg_addr, (uint8_t*)data, length) != I2C_SW_OK)
		Error_Handler(__FILE__, __LINE__);
#else
	/* 硬件I2C */
	if(bsp_I2C_HW_Read_nByte(slave_addr<<1, reg_addr, data, (uint16_t)length) != HAL_OK)
		Error_Handler(__FILE__, __LINE__);
#endif
	return 0;
}

/*
*函数说明：	获取时间戳
*输入数据：	数据保存地址
*输出数据：	无
*/
void stm32_get_ms(unsigned long *count)
{
	*count = HAL_GetTick();
}
/******************************* DMP库移植函数创建end *******************************/

//q30格式,long转float时的除数.
#define q30  1073741824.0f

//陀螺仪方向设置
static int8_t gyro_orientation[9] = { 1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1};

/*
*函数说明：	MPU6050自检程序
*输入形参：	无
*输出数据：	无
*/
uint8_t run_self_test(void)
{
	int result;
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x3) 
	{
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		return 0;
	}else return 1;
}
//陀螺仪方向控制
uint16_t inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    uint16_t scalar; 
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
//方向转换
uint8_t inv_row_2_scale(const signed char *row)
{
    uint8_t b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

/*
*函数说明： DMP初始化
*输入形参： 无
*输出数据： 错误代码
*/
uint8_t MPU6050_DMP_Init(void)
{
	uint8_t res = 0;
    uint8_t RES = 0;

    RES = mpu_init();   //初始化MPU6050
	if(RES == 0)	
	{	 
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		if(res)return 1; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//设置FIFO
		if(res)return 2; 
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	//设置采样率
		if(res)return 3; 
		res=dmp_load_motion_driver_firmware();		//加载DMP固件
		if(res)return 4; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res)return 5; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//????dmp????
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
		if(res)return 7;   
		res=run_self_test();		//自检
		if(res)return 8;    
		res=mpu_set_dmp_state(1);	//使能DMP
		if(res)return 9;     
	}
    else
	    return RES;

	return 0;
}

/*
*函数说明：	MPU6050通过DMP库获得姿态数据
*输入形参：	pitch：俯仰角数据地址（精度:0.1°，范围:-90.0°到+90.0°）
*			roll：横滚角数据地址（精度:0.1°，范围:-180.0°到+180.0°）
*			yaw：偏航角数据地址（精度:0.1°，范围:-180.0°到+180.0°）
*输出数据：	0：成功 其他：失败
*/
uint8_t MPU6050_DMP_Get_Data(float *pitch,float *roll,float *yaw)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 

	/* 从FIFO中获得数据 */
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))
        return 1;	 

	/* 对数据处理 */
	if(sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / q30;	//q30格式转换为浮点数
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;

		//计算得到俯仰角/横滚角/航向角
		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	}else
		return 2;

	return 0;
}

/*
*函数说明：	MPU6050使用DMP库获得姿态数据的测试
*输入形参：	无
*输出数据：	无
*/
void MPU6050_DMP_Test(void)
{
	MPU6050_DMP_Get_Data(&Posture_Struct.pitch,&Posture_Struct.roll,&Posture_Struct.yaw);

	printf("pitch = %f \r\n",Posture_Struct.pitch);
	printf("roll = %f \r\n",Posture_Struct.roll);
	printf("yaw = %f \r\n",Posture_Struct.yaw);
	printf("\r\n");
}
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
