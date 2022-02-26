
/*
*����˵����	����޸���21.11.02
*			��������Ҫ��������GPIO��EXTI�ж���ʹ��
*			���ڳ�������10ms�����Ե���
*			������������˳���������
*��Ҫ���ã�	KeyBoard_Scan_10ms()��ɨ�谴������
*			KeyBoard_Read_data()����ȡ����
*/


#include "bsp.h"
#include "bsp_keyboard4x4.h"

//�����ⲿ��ʱ����
extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

//���󰴼����ݽṹ��
KeyBoard_Typedef 	KeyBoard_Struct = {0};

//����˵�������󰴼�������ģʽ��ʼ������
//�����βΣ���
//������ݣ���
static void Keyboard_GPIO_Config_Mode1(void)
{	
	GPIO_InitTypeDef	GPIO_KeyBoarStruct = {0};
	//��4λ����Ϊ������©����͵�ƽ
	GPIO_KeyBoarStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_KeyBoarStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_KeyBoarStruct.Pull = GPIO_PULLUP;
	GPIO_KeyBoarStruct.Pin = KEYBOARD4X4_LOW_4PIN;
	HAL_GPIO_Init(KEYBOARD4X4_GPIO_LPORT,&GPIO_KeyBoarStruct);
	
	//��4λ����Ϊ�½����ж�����ģʽ
	GPIO_KeyBoarStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_KeyBoarStruct.Pin = KEYBOARD4X4_HIGH_4PIN;
	HAL_GPIO_Init(KEYBOARD4X4_GPIO_HPORT,&GPIO_KeyBoarStruct);
	
	KEYBOARD4X4_GPIO_HPORT->ODR = (KEYBOARD4X4_GPIO_HPORT->ODR & ~KEYBOARD4X4_HIGH_4PIN);//�˿�����͵�ƽ
	KEYBOARD4X4_GPIO_LPORT->ODR = (KEYBOARD4X4_GPIO_LPORT->ODR & ~KEYBOARD4X4_LOW_4PIN);
}

//����˵�������󰴼�������ģʽ��ʼ������
//�����βΣ���
//������ݣ���
static void Keyboard_GPIO_Config_Mode2(void)
{	
	GPIO_InitTypeDef	GPIO_KeyBoarStruct = {0};
	//��4λ����Ϊ������©����͵�ƽ
	GPIO_KeyBoarStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_KeyBoarStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_KeyBoarStruct.Pull = GPIO_PULLUP;
	GPIO_KeyBoarStruct.Pin = KEYBOARD4X4_HIGH_4PIN;
	HAL_GPIO_Init(KEYBOARD4X4_GPIO_HPORT,&GPIO_KeyBoarStruct);
	
	//��4λ����Ϊ�½����ж�����ģʽ
	GPIO_KeyBoarStruct.Mode = GPIO_MODE_INPUT;
	GPIO_KeyBoarStruct.Pin = KEYBOARD4X4_LOW_4PIN;
	HAL_GPIO_Init(KEYBOARD4X4_GPIO_LPORT,&GPIO_KeyBoarStruct);
	
	KEYBOARD4X4_GPIO_HPORT->ODR = (KEYBOARD4X4_GPIO_HPORT->ODR & ~KEYBOARD4X4_HIGH_4PIN);//�˿�����͵�ƽ
	KEYBOARD4X4_GPIO_LPORT->ODR = (KEYBOARD4X4_GPIO_LPORT->ODR & ~KEYBOARD4X4_LOW_4PIN);
	
}

//����˵�������󰴼��˿�10ms����ɨ�躯��
//�����βΣ���
//������ݣ���
void KeyBoard_Scan_10ms(void)
{
	static __IO uint16_t column = 0;		//��
	static __IO uint16_t row = 0;			//��
	
	if(KeyBoard_Struct.keyboard_updata_flag == 0)
	{
		if((KeyBoard_Struct.keyboard_mode_falg == 0))
		{
			if((KEYBOARD4X4_GPIO_HPORT->IDR & KEYBOARD4X4_HIGH_4PIN) != KEYBOARD4X4_HIGH_4PIN)//ȷ�ϰ�������
			{
				column = KEYBOARD4X4_GPIO_HPORT->IDR & KEYBOARD4X4_HIGH_4PIN;//��ȡ�к�
		
				KeyBoard_Struct.keyboard_mode_falg = 1;
				Keyboard_GPIO_Config_Mode2();				
				return;
			}
		}
		else if((KeyBoard_Struct.keyboard_mode_falg == 1))
		{
			if(((KEYBOARD4X4_GPIO_LPORT->IDR & KEYBOARD4X4_LOW_4PIN) != KEYBOARD4X4_LOW_4PIN))
			{
				row = KEYBOARD4X4_GPIO_LPORT->IDR & KEYBOARD4X4_LOW_4PIN;//��ȡ�к�
				row = row >> 1;
				switch ( column | row )//column|rowΪ�������º��Ӧ�˿ڵı���
				{
					//������Ӧ����� �ɸ��������������ֵ 
					case 0XEE00:
						KeyBoard_Struct.keyboard_data = '1'; 	//keyboard 1
						break;
					case 0XDE00: 
						KeyBoard_Struct.keyboard_data = '2'; 	//keyboard 2
						break;	
					case 0XBE00:
						KeyBoard_Struct.keyboard_data = '3';	//keyboard 3
						break;                
					case 0X7E00:
						KeyBoard_Struct.keyboard_data = 'A'; 	//keyboard A
						break;                                              
					case 0XED00:
						KeyBoard_Struct.keyboard_data = '4';	//keyboard 4
						break;
					case 0XDD00: 
						KeyBoard_Struct.keyboard_data = '5'; 	//keyboard 5 
						break;
					case 0XBD00:
						KeyBoard_Struct.keyboard_data = '6'; 	//keyboard 6
						break;
					case 0X7D00:
						KeyBoard_Struct.keyboard_data = 'B'; 	//keyboard B
						break;                                  
					case 0XEB00: 
						KeyBoard_Struct.keyboard_data = '7'; 	//keyboard 7
						break;
					case 0XDB00:
						KeyBoard_Struct.keyboard_data = '8';	//keyboard 8
						break;
					case 0XBB00:
						KeyBoard_Struct.keyboard_data = '9';	//keyboard 9
						break;
					case 0X7B00:
						KeyBoard_Struct.keyboard_data = 'C';	//keyboard C
						break;                                    
					case 0XE700:
						KeyBoard_Struct.keyboard_data = '*';	//keyboard *
						break;
					case 0XD700:
						KeyBoard_Struct.keyboard_data = '0'; 	//keyboard 0
						break;
					case 0XB700:
						KeyBoard_Struct.keyboard_data = '#';	//keyboard #
						break;
					case 0X7700:
						KeyBoard_Struct.keyboard_data = 'D'; 	//keyboard D
						break;
					
					default: 
						KeyBoard_Struct.keyboard_data	= 0XFF;	//error
						break;     
				}
				if(KeyBoard_Struct.keyboard_data != 0XFF)
					KeyBoard_Struct.keyboard_updata_flag = 1;//���°������±�־	
			}

			KeyBoard_Struct.keyboard_mode_falg = 0;
			Keyboard_GPIO_Config_Mode1();
		}
	}
}

/*
*����˵������ȡ���󰴼��İ�������
*�����βΣ���
*������ݣ��������ݣ���Ч���ݣ�0xFF��
*/
char KeyBoard_Read_data(void)
{
	char keyboard_data;
	if(KeyBoard_Struct.keyboard_updata_flag == 1)
	{
		keyboard_data = KeyBoard_Struct.keyboard_data;
		KeyBoard_Struct.keyboard_updata_flag= 0;
		return keyboard_data;
	}
	else
		return 0xff;
}
/*
*����˵�������󰴼���ʼ��
*�����βΣ���
*������ݣ���
*/
void bsp_KeyBoard_Init(void)
{
	KEYBOARD4X4_GPIO_HCLK;
	KEYBOARD4X4_GPIO_LCLK;

	Keyboard_GPIO_Config_Mode1();
}
