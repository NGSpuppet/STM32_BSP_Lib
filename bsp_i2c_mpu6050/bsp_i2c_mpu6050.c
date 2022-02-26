/*
*����˵�������������DMP���ԭʼ�����ݵ�ѡ���ڡ�bsp_i2c_mpu6050.h��
			���޸�mpu6050_NODMP�ĺ궨����ѡ��ʹ��DMP�⻹��ԭʼ��
			���������Ӳ��I2C�����I2C��ѡ���ڡ�bsp_i2c_mpu6050.h��
			���޸�MPU6050_MPU6050_I2C_SW_ENABLE�ĵĺ궨����ѡ��ʹ��Ӳ��I2C�������I2C
*/


#include "bsp.h"

#ifdef MPU6050_I2C_SW_ENABLE
#include "bsp_i2c_sw.h"		/* �������I2C */
#else
#include "bsp_i2c_hw.h"		/* ����Ӳ��I2C */
#endif
/*************************** ͷ�ļ����� ***************************/
#ifdef mpu6050_NODMP
/* ԭʼ����� */
#include "bsp_i2c_mpu6050.h"
#else
/* DMP����� */
#include "inv_mpu.h"	/* ʹ��ʱ��Ҫ�ڸ�ͷ�ļ��޸Ĺ���Ƶ�� */
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#endif

/*************************** �ṹ������ ***************************/
#ifdef mpu6050_NODMP
MPU6050_TypeDef g_tMPU6050 = {0};		/* MPU6050ԭʼ���ݽṹ�� */
#else
Posture_TypeDef	Posture_Struct = {0};	/* MPU6050��̬���ݽṹ�� */
#endif

#ifdef mpu6050_NODMP
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitMPU6050
*	����˵��: ��ʼ��MPU-6050
*	��    ��:  ��
*	�� �� ֵ: 1 ��ʾ������ 0 ��ʾ������
*********************************************************************************************************
*/
void bsp_MPU6050_Init(void)
{
	MPU6050_Write_Byte(MPU6050_PWR_MGMT_1, 0x00);	//�������״̬
	MPU6050_Write_Byte(MPU6050_SMPLRT_DIV, 0x07);	//���������ǲ�����
	MPU6050_Write_Byte(MPU6050_CONFIG, 0x06);		//���õ�ͨ�˲�Ƶ��
	MPU6050_Write_Byte(MPU6050_GYRO_CONFIG, 0xE8);
	MPU6050_Write_Byte(MPU6050_ACCEL_CONFIG, 0x01);
}

/*
*********************************************************************************************************
*	�� �� ��: MPU6050_WriteByte
*	����˵��: �� MPU-6050 �Ĵ���д��һ������
*	��    ��: _ucRegAddr : �Ĵ�����ַ
*			  _ucRegData : �Ĵ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MPU6050_Write_Byte(uint8_t _ucRegAddr, uint8_t _ucRegData)
{
#ifdef MPU6050_I2C_SW_ENABLE
	/* ���I2Cд���� */
	bsp_I2C_SW_Write_Byte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, _ucRegData);
#else
	/* Ӳ��I2Cд���� */
	bsp_I2C_HW_Write_Byte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, _ucRegData);
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: MPU6050_Read_Byte
*	����˵��: ��ȡ MPU-6050 �Ĵ���������
*	��    ��: _ucRegAddr : �Ĵ�����ַ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t MPU6050_Read_Byte(uint8_t _ucRegAddr)
{
	uint8_t ucData;
#ifdef MPU6050_I2C_SW_ENABLE
	/* ���I2C������ */
	bsp_I2C_SW_Read_Byte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, &ucData);
#else
	/* Ӳ��I2C������ */
	bsp_I2C_HW_Read_Byte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, &ucData);
#endif
	return ucData;
}

/*
*����˵����	��MPU6050д��n���ֽ�����
*�����βΣ�	_ucRegAddr���Ĵ�����ַ
*			write_data��Ҫд���ֽڵĵ�ַ
*			length��д��������1~65535��
*������ݣ�	��
*/
void MPU6050_Write_nByte(uint8_t _ucRegAddr, uint8_t *write_data, uint16_t length)
{
#ifdef MPU6050_I2C_SW_ENABLE
	/* ���I2Cд������ */
	bsp_I2C_SW_Write_nByte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, write_data, length);
#else
	/* Ӳ��I2Cд������ */
	bsp_I2C_HW_Write_nByte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, write_data, length);
#endif
}

/*
*����˵���� ��MPU6050��ȡn���ֽڵ�����
*�����βΣ�	_ucRegAddr���Ĵ�����ַ
*			read_data����ȡ�ֽڵı����ַ
*			length��д��������1~65535��	
*/
void MPU6050_Read_nByte(uint8_t _ucRegAddr, uint8_t *read_data, uint16_t length)
{
#ifdef	MPU6050_I2C_SW_ENABLE
	/* ���I2C��ȡ���� */
	bsp_I2C_SW_Read_nByte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, read_data, length);
#else
	/* Ӳ��I2C��ȡ���� */
	bsp_I2C_HW_Read_nByte(MPU6050_SLAVE_ADDRESS, _ucRegAddr, read_data, length);
#endif
}

/*
*����˵����	��ȡMPU6050��������ID
*�����βΣ�	��
*������ݣ�	������ID
*/
uint8_t MPU6050_Read_ID(void)
{
	/* ���ض�ȡ��IDֵ */
	return MPU6050_Read_Byte(MPU6050_WHO_AM_I);
}

/*
*����˵����	��ȡMPU6050���������¶�
*�����βΣ�	��
*������ݣ�	���������¶�
*/
float MPU6050_Read_Temp(void)
{
	int16_t temp_original = 0;

	/* ��ȡ�¶�ԭʼ���� */
	temp_original = (MPU6050_Read_Byte(MPU6050_TEMP_OUT_H)<<8) | MPU6050_Read_Byte(MPU6050_TEMP_OUT_L);

	/* ���㲢����ʵ���¶� */
	return (36.53f+(float)temp_original/340);
}

/*
*����˵����	��MPU6050��ȡ���ٶ�����
*�����βΣ�	axis_char��Ҫ��ȡ���ᣨ'X'or'x'��'Y'or'y'��'Z'or'z'��
*������ݣ�	��ǰ��ļ��ٶ�ֵ
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
			Error_Handler(__FILE__, __LINE__);		/* ���ڷ�Χ�� */
			break;
	}
	return accel_data;
}

/*
*����˵����	��MPU6050��ȡ����������
*�����βΣ�	axis_char��Ҫ��ȡ���ᣨ'X'or'x'��'Y'or'y'��'Z'or'z'��
*������ݣ�	��ǰ���������ֵ
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
			Error_Handler(__FILE__, __LINE__);		/* ���ڷ�Χ�� */
			break;
	}
	return accel_data;
}


/*
*����˵����	MPU6050��ȡ�������ݣ������ݡ������ǡ��¶ȣ�
*�����βΣ�	��
*������ݣ�	��
*/
void MPU6050_Read_AllData(void)
{
	/* �����������ݱ��浽ȫ�ֽṹ����� */
	g_tMPU6050.Accel_X = MPU6050_Read_Accel('X');
	g_tMPU6050.Accel_Y = MPU6050_Read_Accel('Y');
	g_tMPU6050.Accel_Z = MPU6050_Read_Accel('Z');

	g_tMPU6050.Temp = MPU6050_Read_Temp();

	g_tMPU6050.GYRO_X = MPU6050_Read_Gyro('X');
	g_tMPU6050.GYRO_Y = MPU6050_Read_Gyro('Y');
	g_tMPU6050.GYRO_Z = MPU6050_Read_Gyro('Z');
}

/*
*����˵����	MPU6050���Ժ���
*�����βΣ�	��
*������ݣ�	��
*/
void MPU6050_Test(void)
{
	MPU6050_Read_AllData();
	printf("MPU6050 X����ٶȣ�%d \r\n",g_tMPU6050.Accel_X);
	printf("MPU6050 Y����ٶȣ�%d \r\n",g_tMPU6050.Accel_Y);
	printf("MPU6050 Z����ٶȣ�%d \r\n",g_tMPU6050.Accel_Z);
	printf("MPU6050 �¶ȣ�%f \r\n",g_tMPU6050.Temp);
	printf("MPU6050 X�������ǣ�%d \r\n",g_tMPU6050.GYRO_X);
	printf("MPU6050 Y�������ǣ�%d \r\n",g_tMPU6050.GYRO_Y);
	printf("MPU6050 Z�������ǣ�%d \r\n",g_tMPU6050.GYRO_Z);
	printf("\r\n");
}
#else
/******************************* DMP����ֲ�������� *******************************/
/*
*����˵����	��MPU6050��д��������
*�����βΣ�	slave_addr��I2C��ַ(δƫ�Ƶ�ַ)
*			reg_addr���Ĵ�����ַ
*			length��д������ݸ���
*			data��Ҫд������ݵ�ַ
*������ݣ�	0���ɹ� 1��ʧ��
*/
uint8_t MPU6050_WriteReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
#ifdef MPU6050_I2C_SW_ENABLE
	/* ���I2C */
	if(bsp_I2C_SW_Write_nByte(slave_addr<<1, reg_addr, (uint8_t*)data, length) != I2C_SW_OK)
		Error_Handler(__FILE__, __LINE__);
#else
	/* Ӳ��I2C */	
	if(bsp_I2C_HW_Write_nByte(slave_addr<<1, reg_addr, (uint8_t*)data, (uint16_t)length) != HAL_OK)
		Error_Handler(__FILE__, __LINE__);
#endif
	return 0;
}

/*
*����˵����	��MPU6050�ж�ȡ�������
*�����βΣ�	slave_addr��I2C��ַ(δƫ�Ƶ�ַ)
*			reg_addr���Ĵ�����ַ
*			length����ȡ�����ݸ���
*			data����ȡ���ݵı����ַ
*������ݣ�	0���ɹ� 1��ʧ��
*/
uint8_t MPU6050_ReadData(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data)
{
#ifdef MPU6050_I2C_SW_ENABLE
	/* ���I2C */
	if(bsp_I2C_SW_Read_nByte(slave_addr<<1, reg_addr, (uint8_t*)data, length) != I2C_SW_OK)
		Error_Handler(__FILE__, __LINE__);
#else
	/* Ӳ��I2C */
	if(bsp_I2C_HW_Read_nByte(slave_addr<<1, reg_addr, data, (uint16_t)length) != HAL_OK)
		Error_Handler(__FILE__, __LINE__);
#endif
	return 0;
}

/*
*����˵����	��ȡʱ���
*�������ݣ�	���ݱ����ַ
*������ݣ�	��
*/
void stm32_get_ms(unsigned long *count)
{
	*count = HAL_GetTick();
}
/******************************* DMP����ֲ��������end *******************************/

//q30��ʽ,longתfloatʱ�ĳ���.
#define q30  1073741824.0f

//�����Ƿ�������
static int8_t gyro_orientation[9] = { 1, 0, 0,
                                      0, 1, 0,
                                      0, 0, 1};

/*
*����˵����	MPU6050�Լ����
*�����βΣ�	��
*������ݣ�	��
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
//�����Ƿ������
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
//����ת��
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
*����˵���� DMP��ʼ��
*�����βΣ� ��
*������ݣ� �������
*/
uint8_t MPU6050_DMP_Init(void)
{
	uint8_t res = 0;
    uint8_t RES = 0;

    RES = mpu_init();   //��ʼ��MPU6050
	if(RES == 0)	
	{	 
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//��������Ҫ�Ĵ�����
		if(res)return 1; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//����FIFO
		if(res)return 2; 
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	//���ò�����
		if(res)return 3; 
		res=dmp_load_motion_driver_firmware();		//����DMP�̼�
		if(res)return 4; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//���������Ƿ���
		if(res)return 5; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//????dmp????
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//����DMP�������(��󲻳���200Hz)
		if(res)return 7;   
		res=run_self_test();		//�Լ�
		if(res)return 8;    
		res=mpu_set_dmp_state(1);	//ʹ��DMP
		if(res)return 9;     
	}
    else
	    return RES;

	return 0;
}

/*
*����˵����	MPU6050ͨ��DMP������̬����
*�����βΣ�	pitch�����������ݵ�ַ������:0.1�㣬��Χ:-90.0�㵽+90.0�㣩
*			roll����������ݵ�ַ������:0.1�㣬��Χ:-180.0�㵽+180.0�㣩
*			yaw��ƫ�������ݵ�ַ������:0.1�㣬��Χ:-180.0�㵽+180.0�㣩
*������ݣ�	0���ɹ� ������ʧ��
*/
uint8_t MPU6050_DMP_Get_Data(float *pitch,float *roll,float *yaw)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 

	/* ��FIFO�л������ */
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))
        return 1;	 

	/* �����ݴ��� */
	if(sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / q30;	//q30��ʽת��Ϊ������
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;

		//����õ�������/�����/�����
		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	}else
		return 2;

	return 0;
}

/*
*����˵����	MPU6050ʹ��DMP������̬���ݵĲ���
*�����βΣ�	��
*������ݣ�	��
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
