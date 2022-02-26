/*
*����˵�����ó�����ģ��I2C�������
*
*Ӧ��˵�����ڷ���I2C�豸ǰ�����ȵ��� i2c_CheckDevice() ���I2C�豸�Ƿ�����
*	
*/

#include "bsp.h"

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitI2C
*	����˵��: ����I2C���ߵ�GPIO������ģ��IO�ķ�ʽʵ��
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_I2C_SW_Init(void)
{
	GPIO_InitTypeDef gpio_init;

	/* ��1������GPIOʱ�� */
	ALL_I2C_GPIO_CLK_ENABLE;
	
	gpio_init.Mode = GPIO_MODE_OUTPUT_OD;			/* ���ÿ�©��� */
	gpio_init.Pull = GPIO_PULLUP;					/* ʹ���������� */
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;	/* GPIO�ٶȵȼ� */
	
	/* IICʱ������ */
	gpio_init.Pin = I2C_SCL_PIN;	
	HAL_GPIO_Init(I2C_SCL_GPIO, &gpio_init);	
	
	/* IIC�������� */
	gpio_init.Pin = I2C_SDA_PIN;	
	HAL_GPIO_Init(I2C_SDA_GPIO, &gpio_init);	

	i2c_Stop();		/* ��һ��ֹͣ�ź�, ��λI2C�����ϵ������豸������ģʽ */
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_Delay
*	����˵��: I2C����λ�ӳ٣����400KHz
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void i2c_Delay(void)
{
	/*��
		CPU��Ƶ168MHzʱ�����ڲ�Flash����, MDK���̲��Ż�����̨ʽʾ�����۲Ⲩ�Ρ�
		ѭ������Ϊ5ʱ��SCLƵ�� = 1.78MHz (����ʱ: 92ms, ��д������������ʾ����̽ͷ���ϾͶ�дʧ�ܡ�ʱ��ӽ��ٽ�)
		ѭ������Ϊ10ʱ��SCLƵ�� = 1.1MHz (����ʱ: 138ms, ���ٶ�: 118724B/s)
		ѭ������Ϊ30ʱ��SCLƵ�� = 440KHz�� SCL�ߵ�ƽʱ��1.0us��SCL�͵�ƽʱ��1.2us

		��������ѡ��2.2Kŷʱ��SCL������ʱ��Լ0.5us�����ѡ4.7Kŷ����������Լ1us

		ʵ��Ӧ��ѡ��400KHz���ҵ����ʼ���
	*/
	//for (i = 0; i < 30; i++);
	//for (i = 0; i < 60; i++);
	//bsp_DelayUS(2); 229.57KHzʱ��
	bsp_DelayUS(2);
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_Start
*	����˵��: CPU����I2C���������ź�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Start(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C���������ź� */
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
*	�� �� ��: i2c_Start
*	����˵��: CPU����I2C����ֹͣ�ź�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Stop(void)
{
	/* ��SCL�ߵ�ƽʱ��SDA����һ�������ر�ʾI2C����ֹͣ�ź� */
	I2C_SDA_0();
	i2c_Delay();
	I2C_SCL_1();
	i2c_Delay();
	I2C_SDA_1();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_SendByte
*	����˵��: CPU��I2C�����豸����8bit����
*	��    ��:  _ucByte �� �ȴ����͵��ֽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_SendByte(uint8_t _ucByte)
{
	uint8_t i;

	/* �ȷ����ֽڵĸ�λbit7 */
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
		I2C_SCL_0();	/* 2019-03-14 ���GT811���ݴ��������һ�У��൱���ӳټ�ʮns */
		if (i == 7)
		{
			 I2C_SDA_1(); // �ͷ�����
		}
		_ucByte <<= 1;	/* ����һ��bit */	
	}
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_ReadByte
*	����˵��: CPU��I2C�����豸��ȡ8bit����
*	��    ��:  ��
*	�� �� ֵ: ����������
*********************************************************************************************************
*/
uint8_t i2c_ReadByte(void)
{
	uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
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
*	�� �� ��: i2c_WaitAck
*	����˵��: CPU����һ��ʱ�ӣ�����ȡ������ACKӦ���ź�
*	��    ��:  ��
*	�� �� ֵ: ����0��ʾ��ȷӦ��1��ʾ��������Ӧ
*********************************************************************************************************
*/
uint8_t i2c_WaitAck(void)
{
	uint8_t re;

	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
	i2c_Delay();
	if (I2C_SDA_READ())	/* CPU��ȡSDA����״̬ */
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
*	�� �� ��: i2c_Ack
*	����˵��: CPU����һ��ACK�ź�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_Ack(void)
{
	I2C_SDA_0();	/* CPU����SDA = 0 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
	I2C_SDA_1();	/* CPU�ͷ�SDA���� */
	
	i2c_Delay();
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_NAck
*	����˵��: CPU����1��NACK�ź�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void i2c_NAck(void)
{
	I2C_SDA_1();	/* CPU����SDA = 1 */
	i2c_Delay();
	I2C_SCL_1();	/* CPU����1��ʱ�� */
	i2c_Delay();
	I2C_SCL_0();
	i2c_Delay();
}

/*
*********************************************************************************************************
*	�� �� ��: i2c_CheckDevice
*	����˵��: ���I2C�����豸��CPU�����豸��ַ��Ȼ���ȡ�豸Ӧ�����жϸ��豸�Ƿ����
*	��    ��:  _Address���豸��I2C���ߵ�ַ(����ƫ��)
*	�� �� ֵ: ����ֵ 0 ��ʾ��ȷ�� ����1��ʾδ̽�⵽
*********************************************************************************************************
*/
I2C_SW_STATE bsp_I2C_SW_CheckDevice(uint8_t _Address)
{
	uint8_t ucAck;

	if (I2C_SDA_READ() && I2C_SCL_READ())
	{
		i2c_Start();		/* ���������ź� */

		/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
		i2c_SendByte(_Address & 0xFE);
		ucAck = i2c_WaitAck();	/* ����豸��ACKӦ�� */

		i2c_Stop();			/* ����ֹͣ�ź� */
		if(ucAck == 0)
		{
			return I2C_SW_OK;
		}
		else
			return I2C_SW_ERROR;
	}
	return I2C_SW_ERROR;	/* I2C�����쳣 */
}


/*
*����˵����	ʹ��I2C��ӵ�ַ�Ĵ���д��һ���ֽ�����
*�����βΣ�	slave_addr���ӵ�ַ���������ƣ�
*			reg_addr���Ĵ�����ַ
*			write_data��Ҫд�������
*������ݣ�	I2C_SW_OK���ɹ� I2C_SW_ERROR��ʧ��
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
*����˵����	ʹ��I2C��ӵ�ַ�Ĵ�����ȡһ���ֽ�����
*�����βΣ�	slave_addr���ӵ�ַ���������ƣ�
*			reg_addr���Ĵ�����ַ
*			read_data����ȡ���ݻ����ַ
*������ݣ�	I2C_SW_OK���ɹ� I2C_SW_ERROR��ʧ��
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
*����˵����	ʹ��I2C��ӵ�ַ�Ĵ���д��n���ֽ�����
*����˵����	ʹ������дʱ���Ĵ����ĵ�ַ���Զ����ӣ�I2C�豸���ԣ�
*�����βΣ�	slave_addr���ӵ�ַ���������ƣ�
*			reg_addr���Ĵ�����ַ
*			write_data��Ҫд�������
*			Length��д��������1~65535��
*������ݣ�	I2C_SW_OK���ɹ� I2C_SW_ERROR��ʧ��
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
*����˵����	ʹ��I2C��ӵ�ַ�Ĵ�����ȡn���ֽ�����
*����˵����	ʹ��������ʱ���Ĵ����ĵ�ַ���Զ����ӣ�I2C�豸���ԣ�
*�����βΣ�	slave_addr���ӵ�ַ���������ƣ�
*			reg_addr���Ĵ�����ַ
*			read_data����ȡ���ݻ����ַ
*			Length����ȡ������1~65535��
*������ݣ�	I2C_SW_OK���ɹ� I2C_SW_ERROR��ʧ��
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

