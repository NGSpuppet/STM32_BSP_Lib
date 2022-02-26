
#include "bsp.h"
#include "bsp_HC_RS04.h"

/* 调用外部延时函数 */
extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

/* 结构体声明 */
TIM_HandleTypeDef   HC_RS04_TIM;
HC_RS04_TypeDef  HC_RS04_Struct;

/*
*函数说明： HC-RS04的GPIO初始化
*输入形参： 无
*输出数据： 无
*/
static void HC_RS04_GPIO_Config(void)
{
    GPIO_InitTypeDef    HC_RS04_GPIO = {0};

    HC_RS04_Trig_CLK;   /* 开启Trig对应IO口的时钟 */
    HC_RS04_Echo_CLK;   /* 开启Echo对应IO口的时钟 */

    /* Trig初始化 */
    HC_RS04_GPIO.Pull = GPIO_NOPULL;
    HC_RS04_GPIO.Mode = GPIO_MODE_OUTPUT_PP;
    HC_RS04_GPIO.Speed = GPIO_SPEED_FREQ_LOW;
    HC_RS04_GPIO.Pin = HC_RS04_Trig_PIN;
    HAL_GPIO_Init(HC_RS04_Trig_PORT,&HC_RS04_GPIO);

    /* Echo初始化 */
    HC_RS04_GPIO.Mode = GPIO_MODE_INPUT;
    HC_RS04_GPIO.Pin = HC_RS04_Echo_PIN;
    HAL_GPIO_Init(HC_RS04_Echo_PORT,&HC_RS04_GPIO);

    HC_RS04_Trig_LOW;   /* Trig引脚拉低 */
}

/*
*函数说明： 配置用于反馈电平计时的定时器（TIM7）
*输入形参： 无
*输出数据： 无
*/
static void HC_RS04_TIM_Config(void)
{
    if((HC_RS04_TIMx->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN)
    {
        /* TIM已经被使用 */
        Error_Handler(__FILE__, __LINE__);
    }

    HC_RS04_TIMx_CLK;   /* 开启对应定时器的时钟 */

    /* 配置对应定时器 */
    HC_RS04_TIM.Instance = HC_RS04_TIMx;                                    /* 对应基本定时器7 */
    HC_RS04_TIM.Init.Prescaler = (200-1);                                   /* 200预分频 */
    HC_RS04_TIM.Init.Period = 0xFFFF;                                       /* 设置为最大计数值 */
    HC_RS04_TIM.Init.CounterMode = TIM_COUNTERMODE_UP;                      /* 向上递增计数 */
    HC_RS04_TIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;                /* 不分频 */
    HC_RS04_TIM.Init.RepetitionCounter = 0x00;                              /* 不用重复计数 */
    HC_RS04_TIM.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;    /* 取消自动重装载预加载值 */

    /* 定时器初始化 */
    if(HAL_TIM_Base_Init(&HC_RS04_TIM) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

/*
*函数说明： 开始测量距离
*输入形参： 无
*输出数据： 无
*/
void HC_RS04_Start(void)
{
    HC_RS04_TIMx->CNT = 0;  /* 清除计数值 */
    __HAL_TIM_CLEAR_IT(&HC_RS04_TIM,TIM_IT_UPDATE);     /* 清除对应定时器中断标志 */

    /* 触发 */
    HC_RS04_Trig_HIGH;      /* Trig拉高 */
    delay_us(10);           /* 延时10uS */
    HC_RS04_Trig_LOW;       /* Trig拉低 */

    while((HC_RS04_Echo_PORT->IDR & HC_RS04_Echo_PIN) != HC_RS04_Echo_PIN);     /* 等待Echo引脚拉高 */

    __HAL_TIM_ENABLE(&HC_RS04_TIM);     /* 使能定时器开始计数 */

    while((HC_RS04_Echo_PORT->IDR & HC_RS04_Echo_PIN) == HC_RS04_Echo_PIN);     /* 等待Echo引脚拉低 */

    __HAL_TIM_DISABLE(&HC_RS04_TIM);    /* 失能定时器停止计数 */

    HC_RS04_Struct.distance_count = HC_RS04_TIMx->CNT;      /* 获得计数值（对应Echo高电平时间） */
    HC_RS04_Struct.achieve_flag = 1;                        /* 更新完成标志 */

}

/*
*函数说明： 超声波距离转换（厘米）
*调用条件： 先用“HC_RS04_Start()”完成反馈电平的计时
*输入形参： 无
*输出数据： 距离（单位：厘米）
*/
float HC_RS04_Distance_Calculate_cm(void)
{
    /* 判断是否完成 */
    if(HC_RS04_Struct.achieve_flag == 1)
    {
        HC_RS04_Struct.achieve_flag = 0;    /* 清除完成标志 */
        return ((float)HC_RS04_Struct.distance_count/58);   /* 计算距离 */
    }
    return 0;
}

/*
*函数说明： 超声波距离转换（英寸）
*输入形参： 无
*输出数据： 距离（单位：英寸）
*/
float HC_RS04_Distance_Calculate_in(void)
{

    /* 判断是否完成 */
    if(HC_RS04_Struct.achieve_flag == 1)
    {
        HC_RS04_Struct.achieve_flag = 0;    /* 清除完成标志 */
        return ((float)HC_RS04_Struct.distance_count)/148;  /* 计算距离 */
    }
    return 0;
}

/*
*函数说明： HC-RS40超声波模块初始化
*输入形参： 无；
*输出数据： 无
*/
void HC_RS04_Init(void)
{
    HC_RS04_GPIO_Config();      /* 初始化对应IO */
    HC_RS04_TIM_Config();       /* 初始化对应TIM */
}

/*
*函数说明： 定时器中断服务函数（TIM7）
*输入形参： 无
*输出数据： 无
*/
void HC_RS04_TIMx_IRQHandler(void)
{
    /* 用不到，没有开启定时器中断 */
}

/*
*函数说明： HC_RS04测试程序
*输入形参： 无
*输出数据： 无
*/
void HC_RS04_Test(void)
{
/* 宏定义是使用cm还是in */
#define cm_flag     1       /* 1：单位为cm，0：单位为in */

    HC_RS04_Start();    /* 开始测量 */
    while(HC_RS04_Struct.achieve_flag != 1);    /* 等待完成 */

    if(cm_flag)
    {
        printf("测量距离：%f cm\r\n",HC_RS04_Distance_Calculate_cm());
    }
    else
        printf("测量距离：%f in\r\n",HC_RS04_Distance_Calculate_in());
}
