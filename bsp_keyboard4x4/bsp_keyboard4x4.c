
/*
*程序说明：	最后修改于21.11.02
*			程序不再需要在阻塞和GPIO的EXTI中断中使用
*			现在程序用于10ms周期性调用
*			这样大大增加了程序的灵活性
*主要调用：	KeyBoard_Scan_10ms()来扫描按键矩阵
*			KeyBoard_Read_data()来获取按键
*/


#include "bsp.h"
#include "bsp_keyboard4x4.h"

//调用外部延时函数
extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

//矩阵按键数据结构体
KeyBoard_Typedef 	KeyBoard_Struct = {0};

//函数说明：矩阵按键列输入模式初始化配置
//输入形参：无
//输出数据：无
static void Keyboard_GPIO_Config_Mode1(void)
{	
	GPIO_InitTypeDef	GPIO_KeyBoarStruct = {0};
	//低4位配置为上拉开漏输出低电平
	GPIO_KeyBoarStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_KeyBoarStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_KeyBoarStruct.Pull = GPIO_PULLUP;
	GPIO_KeyBoarStruct.Pin = KEYBOARD4X4_LOW_4PIN;
	HAL_GPIO_Init(KEYBOARD4X4_GPIO_LPORT,&GPIO_KeyBoarStruct);
	
	//高4位配置为下降沿中断上拉模式
	GPIO_KeyBoarStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_KeyBoarStruct.Pin = KEYBOARD4X4_HIGH_4PIN;
	HAL_GPIO_Init(KEYBOARD4X4_GPIO_HPORT,&GPIO_KeyBoarStruct);
	
	KEYBOARD4X4_GPIO_HPORT->ODR = (KEYBOARD4X4_GPIO_HPORT->ODR & ~KEYBOARD4X4_HIGH_4PIN);//端口输出低电平
	KEYBOARD4X4_GPIO_LPORT->ODR = (KEYBOARD4X4_GPIO_LPORT->ODR & ~KEYBOARD4X4_LOW_4PIN);
}

//函数说明：矩阵按键行输入模式初始化配置
//输入形参：无
//输出数据：无
static void Keyboard_GPIO_Config_Mode2(void)
{	
	GPIO_InitTypeDef	GPIO_KeyBoarStruct = {0};
	//低4位配置为上拉开漏输出低电平
	GPIO_KeyBoarStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_KeyBoarStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_KeyBoarStruct.Pull = GPIO_PULLUP;
	GPIO_KeyBoarStruct.Pin = KEYBOARD4X4_HIGH_4PIN;
	HAL_GPIO_Init(KEYBOARD4X4_GPIO_HPORT,&GPIO_KeyBoarStruct);
	
	//高4位配置为下降沿中断上拉模式
	GPIO_KeyBoarStruct.Mode = GPIO_MODE_INPUT;
	GPIO_KeyBoarStruct.Pin = KEYBOARD4X4_LOW_4PIN;
	HAL_GPIO_Init(KEYBOARD4X4_GPIO_LPORT,&GPIO_KeyBoarStruct);
	
	KEYBOARD4X4_GPIO_HPORT->ODR = (KEYBOARD4X4_GPIO_HPORT->ODR & ~KEYBOARD4X4_HIGH_4PIN);//端口输出低电平
	KEYBOARD4X4_GPIO_LPORT->ODR = (KEYBOARD4X4_GPIO_LPORT->ODR & ~KEYBOARD4X4_LOW_4PIN);
	
}

//函数说明：矩阵按键端口10ms周期扫描函数
//输入形参：无
//输出数据：无
void KeyBoard_Scan_10ms(void)
{
	static __IO uint16_t column = 0;		//列
	static __IO uint16_t row = 0;			//行
	
	if(KeyBoard_Struct.keyboard_updata_flag == 0)
	{
		if((KeyBoard_Struct.keyboard_mode_falg == 0))
		{
			if((KEYBOARD4X4_GPIO_HPORT->IDR & KEYBOARD4X4_HIGH_4PIN) != KEYBOARD4X4_HIGH_4PIN)//确认按键按下
			{
				column = KEYBOARD4X4_GPIO_HPORT->IDR & KEYBOARD4X4_HIGH_4PIN;//获取列号
		
				KeyBoard_Struct.keyboard_mode_falg = 1;
				Keyboard_GPIO_Config_Mode2();				
				return;
			}
		}
		else if((KeyBoard_Struct.keyboard_mode_falg == 1))
		{
			if(((KEYBOARD4X4_GPIO_LPORT->IDR & KEYBOARD4X4_LOW_4PIN) != KEYBOARD4X4_LOW_4PIN))
			{
				row = KEYBOARD4X4_GPIO_LPORT->IDR & KEYBOARD4X4_LOW_4PIN;//获取行号
				row = row >> 1;
				switch ( column | row )//column|row为按键按下后对应端口的编码
				{
					//按键对应的码表 可根据需求调整返回值 
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
					KeyBoard_Struct.keyboard_updata_flag = 1;//更新按键更新标志	
			}

			KeyBoard_Struct.keyboard_mode_falg = 0;
			Keyboard_GPIO_Config_Mode1();
		}
	}
}

/*
*函数说明：读取矩阵按键的按下数据
*输入形参：无
*输出数据：按下数据（无效数据：0xFF）
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
*函数说明：矩阵按键初始化
*输入形参：无
*输出数据：无
*/
void bsp_KeyBoard_Init(void)
{
	KEYBOARD4X4_GPIO_HCLK;
	KEYBOARD4X4_GPIO_LCLK;

	Keyboard_GPIO_Config_Mode1();
}
