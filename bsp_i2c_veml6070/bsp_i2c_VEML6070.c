
#include "bsp.h"

//�����ⲿ��ʱ����
extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

//���������üĴ�������
uint8_t Set_Register;

/*
*����˵����	��ʼ��VEML6070��INT����
*�����βΣ�	��
*������ݣ�	��
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
*����˵����	��VEML6070�ļĴ�����ȡһ���ֽ�����
*�����βΣ�	REG_Addr���Ĵ�����ַ
*������ݣ�	�Ĵ�������
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
*����˵����	��VEML6070д��һ���ֽ�����
*�����βΣ�	data��Ҫд�������
*������ݣ�	��
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
*����˵����	��VEML6070��ȡ�����ǿ������
*�����βΣ�	��
*������ݣ�	16λ�������ǿ������
*/
uint16_t VEML6070_Read_Light(void)
{
	uint16_t H,L;
	H = VEML6070_Read_Byte(VEML6070_Hlight_ADDR);
	L = VEML6070_Read_Byte(VEML6070_Llight_ADDR);
	return ((H <<= 8) | L);
}


/*
*����˵����	��ȡVEML6070���ж�״̬
*�����βΣ�	��
*������ݣ�	VEML6070���ж�״̬
*/
uint8_t VEML6070_Read_INT_State(void)
{
	return VEML6070_Read_Byte(VEML6070_ARA_ADDR);
}


/*
*����˵����	����VEML6070��ACKλ�Ƿ�ʹ��λ
*�����βΣ�	State��ʹ��ö��
*������ݣ�	��
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
*����˵����	����VEML6070��ACK��ֵλ
*�����βΣ�	Steps����ֵö��
*������ݣ�	��
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
*����˵����	����VEML6070�Ļ���ʱ��λ
*�����βΣ�	Time������ʱ��ö��
*������ݣ�	��
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
*����˵����	����VEML6070���Ƿ�ػ�λ
*�����βΣ�	State��ʹ��ö��
*������ݣ�	��
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
*����˵����	��ʼ��VEML6070
*�������ݣ�	��
*������ݣ�	��
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
*����˵����	VEML6070�������
*�����βΣ�	��
*������ݣ�	��
*/
void VEML6070_Test(void)
{
	printf("��������������%d \r\n",VEML6070_Read_Light());
}


