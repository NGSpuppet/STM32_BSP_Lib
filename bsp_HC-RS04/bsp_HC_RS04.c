
#include "bsp.h"
#include "bsp_HC_RS04.h"

/* �����ⲿ��ʱ���� */
extern void delay_us(uint32_t i);
extern void delay_ms(uint32_t i);

/* �ṹ������ */
TIM_HandleTypeDef   HC_RS04_TIM;
HC_RS04_TypeDef  HC_RS04_Struct;

/*
*����˵���� HC-RS04��GPIO��ʼ��
*�����βΣ� ��
*������ݣ� ��
*/
static void HC_RS04_GPIO_Config(void)
{
    GPIO_InitTypeDef    HC_RS04_GPIO = {0};

    HC_RS04_Trig_CLK;   /* ����Trig��ӦIO�ڵ�ʱ�� */
    HC_RS04_Echo_CLK;   /* ����Echo��ӦIO�ڵ�ʱ�� */

    /* Trig��ʼ�� */
    HC_RS04_GPIO.Pull = GPIO_NOPULL;
    HC_RS04_GPIO.Mode = GPIO_MODE_OUTPUT_PP;
    HC_RS04_GPIO.Speed = GPIO_SPEED_FREQ_LOW;
    HC_RS04_GPIO.Pin = HC_RS04_Trig_PIN;
    HAL_GPIO_Init(HC_RS04_Trig_PORT,&HC_RS04_GPIO);

    /* Echo��ʼ�� */
    HC_RS04_GPIO.Mode = GPIO_MODE_INPUT;
    HC_RS04_GPIO.Pin = HC_RS04_Echo_PIN;
    HAL_GPIO_Init(HC_RS04_Echo_PORT,&HC_RS04_GPIO);

    HC_RS04_Trig_LOW;   /* Trig�������� */
}

/*
*����˵���� �������ڷ�����ƽ��ʱ�Ķ�ʱ����TIM7��
*�����βΣ� ��
*������ݣ� ��
*/
static void HC_RS04_TIM_Config(void)
{
    if((HC_RS04_TIMx->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN)
    {
        /* TIM�Ѿ���ʹ�� */
        Error_Handler(__FILE__, __LINE__);
    }

    HC_RS04_TIMx_CLK;   /* ������Ӧ��ʱ����ʱ�� */

    /* ���ö�Ӧ��ʱ�� */
    HC_RS04_TIM.Instance = HC_RS04_TIMx;                                    /* ��Ӧ������ʱ��7 */
    HC_RS04_TIM.Init.Prescaler = (200-1);                                   /* 200Ԥ��Ƶ */
    HC_RS04_TIM.Init.Period = 0xFFFF;                                       /* ����Ϊ������ֵ */
    HC_RS04_TIM.Init.CounterMode = TIM_COUNTERMODE_UP;                      /* ���ϵ������� */
    HC_RS04_TIM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;                /* ����Ƶ */
    HC_RS04_TIM.Init.RepetitionCounter = 0x00;                              /* �����ظ����� */
    HC_RS04_TIM.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;    /* ȡ���Զ���װ��Ԥ����ֵ */

    /* ��ʱ����ʼ�� */
    if(HAL_TIM_Base_Init(&HC_RS04_TIM) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);
    }
}

/*
*����˵���� ��ʼ��������
*�����βΣ� ��
*������ݣ� ��
*/
void HC_RS04_Start(void)
{
    HC_RS04_TIMx->CNT = 0;  /* �������ֵ */
    __HAL_TIM_CLEAR_IT(&HC_RS04_TIM,TIM_IT_UPDATE);     /* �����Ӧ��ʱ���жϱ�־ */

    /* ���� */
    HC_RS04_Trig_HIGH;      /* Trig���� */
    delay_us(10);           /* ��ʱ10uS */
    HC_RS04_Trig_LOW;       /* Trig���� */

    while((HC_RS04_Echo_PORT->IDR & HC_RS04_Echo_PIN) != HC_RS04_Echo_PIN);     /* �ȴ�Echo�������� */

    __HAL_TIM_ENABLE(&HC_RS04_TIM);     /* ʹ�ܶ�ʱ����ʼ���� */

    while((HC_RS04_Echo_PORT->IDR & HC_RS04_Echo_PIN) == HC_RS04_Echo_PIN);     /* �ȴ�Echo�������� */

    __HAL_TIM_DISABLE(&HC_RS04_TIM);    /* ʧ�ܶ�ʱ��ֹͣ���� */

    HC_RS04_Struct.distance_count = HC_RS04_TIMx->CNT;      /* ��ü���ֵ����ӦEcho�ߵ�ƽʱ�䣩 */
    HC_RS04_Struct.achieve_flag = 1;                        /* ������ɱ�־ */

}

/*
*����˵���� ����������ת�������ף�
*���������� ���á�HC_RS04_Start()����ɷ�����ƽ�ļ�ʱ
*�����βΣ� ��
*������ݣ� ���루��λ�����ף�
*/
float HC_RS04_Distance_Calculate_cm(void)
{
    /* �ж��Ƿ���� */
    if(HC_RS04_Struct.achieve_flag == 1)
    {
        HC_RS04_Struct.achieve_flag = 0;    /* �����ɱ�־ */
        return ((float)HC_RS04_Struct.distance_count/58);   /* ������� */
    }
    return 0;
}

/*
*����˵���� ����������ת����Ӣ�磩
*�����βΣ� ��
*������ݣ� ���루��λ��Ӣ�磩
*/
float HC_RS04_Distance_Calculate_in(void)
{

    /* �ж��Ƿ���� */
    if(HC_RS04_Struct.achieve_flag == 1)
    {
        HC_RS04_Struct.achieve_flag = 0;    /* �����ɱ�־ */
        return ((float)HC_RS04_Struct.distance_count)/148;  /* ������� */
    }
    return 0;
}

/*
*����˵���� HC-RS40������ģ���ʼ��
*�����βΣ� �ޣ�
*������ݣ� ��
*/
void HC_RS04_Init(void)
{
    HC_RS04_GPIO_Config();      /* ��ʼ����ӦIO */
    HC_RS04_TIM_Config();       /* ��ʼ����ӦTIM */
}

/*
*����˵���� ��ʱ���жϷ�������TIM7��
*�����βΣ� ��
*������ݣ� ��
*/
void HC_RS04_TIMx_IRQHandler(void)
{
    /* �ò�����û�п�����ʱ���ж� */
}

/*
*����˵���� HC_RS04���Գ���
*�����βΣ� ��
*������ݣ� ��
*/
void HC_RS04_Test(void)
{
/* �궨����ʹ��cm����in */
#define cm_flag     1       /* 1����λΪcm��0����λΪin */

    HC_RS04_Start();    /* ��ʼ���� */
    while(HC_RS04_Struct.achieve_flag != 1);    /* �ȴ���� */

    if(cm_flag)
    {
        printf("�������룺%f cm\r\n",HC_RS04_Distance_Calculate_cm());
    }
    else
        printf("�������룺%f in\r\n",HC_RS04_Distance_Calculate_in());
}
