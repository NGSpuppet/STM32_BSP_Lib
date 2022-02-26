
#ifndef _BSP_MPU6050_H
#define _BSP_MPU6050_H

#include "stdint.h"

/* �汾�� */
#define BSP_MPU6050_V	"BSP_MPU6050�汾��21.09.12"

#define mpu6050_NODMP			/* ʹ��ԭʼ�⣬ע��ʹ��DMP�� */

#define MPU6050_I2C_SW_ENABLE			/* ʹ�����I2C��Ĭ�ϣ���ע��ʹ��Ӳ��I2C */

#ifdef mpu6050_NODMP	/* ��ʹ��DMP�� */

/* I2C���ߵ�ַ�궨�� */
#define MPU6050_SLAVE_ADDRESS	(0x68<<1)		/* I2C�ӻ���ַ */
#define MPU6050_WRITE_ADDR		(MPU6050_SLAVE_ADDRESS & 0xFE)	/* д��ַ */
#define	MPU6050_READ_ADDR		(MPU6050_SLAVE_ADDRESS | 0x01)	/* ����ַ */

//****************************************
// ����MPU6050�ڲ���ַ
//****************************************
#define MPU6050_XG_OFFS_TC       	0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_YG_OFFS_TC       	0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_ZG_OFFS_TC       	0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_X_FINE_GAIN      	0x03 //[7:0] X_FINE_GAIN
#define MPU6050_Y_FINE_GAIN      	0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_Z_FINE_GAIN      	0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_XA_OFFS_H        	0x06 //[15:0] XA_OFFS
#define MPU6050_XA_OFFS_L_TC     	0x07
#define MPU6050_YA_OFFS_H        	0x08 //[15:0] YA_OFFS
#define MPU6050_YA_OFFS_L_TC     	0x09
#define MPU6050_ZA_OFFS_H        	0x0A //[15:0] ZA_OFFS
#define MPU6050_ZA_OFFS_L_TC     	0x0B
#define	MPU6050_SELF_TEST_X_REG		0x0D //�Լ�Ĵ���X
#define	MPU6050_SELF_TEST_Y_REG		0x0E //�Լ�Ĵ���Y
#define	MPU6050_SELF_TEST_Z_REG		0x0F //�Լ�Ĵ���Z
#define	MPU6050_SELF_TEST_A_REG		0x10 //�Լ�Ĵ���A
#define MPU6050_XG_OFFS_USRH     	0x13 //[15:0] XG_OFFS_USR
#define MPU6050_XG_OFFS_USRL     	0x14
#define MPU6050_YG_OFFS_USRH     	0x15 //[15:0] YG_OFFS_USR
#define MPU6050_YG_OFFS_USRL     	0x16
#define MPU6050_ZG_OFFS_USRH     	0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_ZG_OFFS_USRL     	0x18

#define	MPU6050_SMPLRT_DIV			0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	MPU6050_CONFIG				0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	MPU6050_GYRO_CONFIG			0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	MPU6050_ACCEL_CONFIG		0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)

#define MPU6050_FF_THR           	0x1D
#define MPU6050_FF_DUR           	0x1E
#define MPU6050_MOT_THR          	0x1F
#define MPU6050_MOT_DUR          	0x20
#define MPU6050_ZRMOT_THR        	0x21
#define MPU6050_ZRMOT_DUR        	0x22
#define MPU6050_FIFO_EN          	0x23
#define MPU6050_I2C_MST_CTRL     	0x24
#define MPU6050_I2C_SLV0_ADDR    	0x25
#define MPU6050_I2C_SLV0_REG     	0x26
#define MPU6050_I2C_SLV0_CTRL    	0x27
#define MPU6050_I2C_SLV1_ADDR    	0x28
#define MPU6050_I2C_SLV1_REG     	0x29
#define MPU6050_I2C_SLV1_CTRL    	0x2A
#define MPU6050_I2C_SLV2_ADDR    	0x2B
#define MPU6050_I2C_SLV2_REG     	0x2C
#define MPU6050_I2C_SLV2_CTRL    	0x2D
#define MPU6050_I2C_SLV3_ADDR    	0x2E
#define MPU6050_I2C_SLV3_REG     	0x2F
#define MPU6050_I2C_SLV3_CTRL    	0x30
#define MPU6050_I2C_SLV4_ADDR    	0x31
#define MPU6050_I2C_SLV4_REG     	0x32
#define MPU6050_I2C_SLV4_DO      	0x33
#define MPU6050_I2C_SLV4_CTRL    	0x34
#define MPU6050_I2C_SLV4_DI      	0x35
#define MPU6050_I2C_MST_STATUS   	0x36
#define MPU6050_INT_PIN_CFG      	0x37
#define MPU6050_INT_ENABLE       	0x38
#define MPU6050_DMP_INT_STATUS   	0x39
#define MPU6050_INT_STATUS       	0x3A
/* ���ٶ����ݵ�ַ */
#define	MPU6050_ACCEL_XOUT_H		0x3B	//X����ٶ����ݵĸ�λ�Ĵ�����ַ
#define	MPU6050_ACCEL_XOUT_L		0x3C	//X����ٶ����ݵĵ�λ�Ĵ�����ַ
#define	MPU6050_ACCEL_YOUT_H		0x3D	//Y����ٶ����ݵĸ�λ�Ĵ�����ַ
#define	MPU6050_ACCEL_YOUT_L		0x3E	//Y����ٶ����ݵĵ�λ�Ĵ�����ַ
#define	MPU6050_ACCEL_ZOUT_H		0x3F	//Z����ٶ����ݵĸ�λ�Ĵ�����ַ
#define	MPU6050_ACCEL_ZOUT_L		0x40	//Z����ٶ����ݵĵ�λ�Ĵ�����ַ
/* �¶����ݵ�ַ */	
#define	MPU6050_TEMP_OUT_H			0x41	//�¶����ݵĸ�λ�Ĵ�����ַ
#define	MPU6050_TEMP_OUT_L			0x42	//�¶����ݵĵ�λ�Ĵ�����ַ
/* ���������ݵ�ַ */	
#define	MPU6050_GYRO_XOUT_H			0x43	//X�����������ݵĸ�λ�Ĵ�����ַ
#define	MPU6050_GYRO_XOUT_L			0x44	//X�����������ݵĵ�λ�Ĵ�����ַ
#define	MPU6050_GYRO_YOUT_H			0x45	//Y�����������ݵĸ�λ�Ĵ�����ַ
#define	MPU6050_GYRO_YOUT_L			0x46	//Y�����������ݵĵ�λ�Ĵ�����ַ
#define	MPU6050_GYRO_ZOUT_H			0x47	//Z�����������ݵĸ�λ�Ĵ�����ַ
#define	MPU6050_GYRO_ZOUT_L			0x48	//Z�����������ݵĵ�λ�Ĵ�����ַ

#define MPU6050_EXT_SENS_DATA_00 	0x49
#define MPU6050_EXT_SENS_DATA_01 	0x4A
#define MPU6050_EXT_SENS_DATA_02 	0x4B
#define MPU6050_EXT_SENS_DATA_03 	0x4C
#define MPU6050_EXT_SENS_DATA_04 	0x4D
#define MPU6050_EXT_SENS_DATA_05 	0x4E
#define MPU6050_EXT_SENS_DATA_06 	0x4F
#define MPU6050_EXT_SENS_DATA_07 	0x50
#define MPU6050_EXT_SENS_DATA_08 	0x51
#define MPU6050_EXT_SENS_DATA_09 	0x52
#define MPU6050_EXT_SENS_DATA_10 	0x53
#define MPU6050_EXT_SENS_DATA_11 	0x54
#define MPU6050_EXT_SENS_DATA_12 	0x55
#define MPU6050_EXT_SENS_DATA_13 	0x56
#define MPU6050_EXT_SENS_DATA_14 	0x57
#define MPU6050_EXT_SENS_DATA_15 	0x58
#define MPU6050_EXT_SENS_DATA_16 	0x59
#define MPU6050_EXT_SENS_DATA_17 	0x5A
#define MPU6050_EXT_SENS_DATA_18 	0x5B
#define MPU6050_EXT_SENS_DATA_19 	0x5C
#define MPU6050_EXT_SENS_DATA_20 	0x5D
#define MPU6050_EXT_SENS_DATA_21 	0x5E
#define MPU6050_EXT_SENS_DATA_22 	0x5F
#define MPU6050_EXT_SENS_DATA_23 	0x60
#define MPU6050_MOT_DETECT_STATUS   0x61
#define MPU6050_I2C_SLV0_DO     	0x63
#define MPU6050_I2C_SLV1_DO     	0x64
#define MPU6050_I2C_SLV2_DO     	0x65
#define MPU6050_I2C_SLV3_DO     	0x66
#define MPU6050_I2C_MST_DELAY_CTRL  0x67
#define MPU6050_SIGNAL_PATH_RESET   0x68
#define MPU6050_MOT_DETECT_CTRL     0x69
#define MPU6050_USER_CTRL        	0x6A

#define	MPU6050_PWR_MGMT_1			0x6B	//��Դ��������ֵ��0x00(��������)
#define MPU6050_PWR_MGMT_2       	0x6C
#define MPU6050_BANK_SEL         	0x6D
#define MPU6050_MEM_START_ADDR   	0x6E
#define MPU6050_MEM_R_W          	0x6F
#define MPU6050_DMP_CFG_1        	0x70
#define MPU6050_DMP_CFG_2        	0x71
#define MPU6050_FIFO_COUNTH      	0x72
#define MPU6050_FIFO_COUNTL      	0x73
#define MPU6050_FIFO_R_W         	0x74
#define	MPU6050_WHO_AM_I			0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)

/* MPU6050���ݽṹ�� */
typedef struct
{
	/* ���ٶ� */
	int16_t Accel_X;	
	int16_t Accel_Y;
	int16_t Accel_Z;

	/* �¶� */
	float Temp;

	/* ������ */
	int16_t GYRO_X;
	int16_t GYRO_Y;
	int16_t GYRO_Z;
}MPU6050_TypeDef;

#else		/* ʹ��DMP�� */

typedef struct{
	float pitch;	/* ������ */
	float roll;		/* ������ */
	float yaw;		/* ƫ���� */
}Posture_TypeDef;	/* MPU6050��̬���� */

#endif

#ifdef mpu6050_NODMP	/* ��ʹ��DMP�� */

/* �������� */
void bsp_MPU6050_Init(void);
void MPU6050_Write_Byte(uint8_t _ucRegAddr, uint8_t _ucRegData);
uint8_t MPU6050_Read_Byte(uint8_t _ucRegAddr);
void MPU6050_Write_nByte(uint8_t _ucRegAddr, uint8_t *write_data, uint16_t length);
void MPU6050_Read_nByte(uint8_t _ucRegAddr, uint8_t *read_data, uint16_t length);
uint8_t MPU6050_Read_ID(void);
float MPU6050_Read_Temp(void);
int16_t MPU6050_Read_Accel(char axis_char);
int16_t MPU6050_Read_Gyro(char axis_char);
void MPU6050_Read_AllData(void);

/* ���Ժ��� */
void MPU6050_Test(void);

#else	/* ʹ��DMP�� */

/* ������DMP�ĺ��� */
uint8_t MPU6050_WriteReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
uint8_t MPU6050_ReadData(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
void stm32_get_ms(unsigned long *count);

//����DMP��ĺ���
uint8_t inv_row_2_scale(const signed char *row);
uint16_t inv_orientation_matrix_to_scalar(const signed char *mtx);
uint8_t run_self_test(void);
uint8_t MPU6050_DMP_Init(void);
uint8_t MPU6050_DMP_Get_Data(float *pitch,float *roll,float *yaw);

/* ���Ժ��� */
void MPU6050_DMP_Test(void);

#endif

#endif

