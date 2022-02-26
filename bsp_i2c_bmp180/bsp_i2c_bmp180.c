
#include "bsp.h"

//�洢BMP180���ݵĽṹ
_bmp180 bmp180;

extern void delay_us (uint32_t i);
extern void delay_ms (uint32_t i);

/*
*����˵���� ��BMP180���г�ʼ��
*�����βΣ� ��
*������ݣ� ��
*/
void BMP180_Init(void)
{
    bmp180.AC1 = BMP180_Read_2Byte(0xAA);
    bmp180.AC2 = BMP180_Read_2Byte(0xAC);
    bmp180.AC3 = BMP180_Read_2Byte(0xAE);
    bmp180.AC4 = BMP180_Read_2Byte(0xB0);
    bmp180.AC5 = BMP180_Read_2Byte(0xB2);
    bmp180.AC6 = BMP180_Read_2Byte(0xB4);
    bmp180.B1  = BMP180_Read_2Byte(0xB6);
    bmp180.B2  = BMP180_Read_2Byte(0xB8);
    bmp180.MB  = BMP180_Read_2Byte(0xBA);
    bmp180.MC  = BMP180_Read_2Byte(0xBC);
    bmp180.MD  = BMP180_Read_2Byte(0xBE);
}

/*
*����˵���� ��BMP180�ļĴ���д��һ���ֽ�����
*�����βΣ� REG_Addr���Ĵ�����ַ
*          data��Ҫд�������
*������ݣ� ��
*/
void BMP180_Write_Byte(uint8_t REG_Addr,uint8_t data)
{
    bsp_I2C_SW_Write_Byte(BMP180_ADDRESS, REG_Addr, data);
}

/*
*����˵���� ��BMP180�ļĴ�����һ���ֽ�����
*�����βΣ� REG_Addr���Ĵ�����ַ
*������ݣ� �Ĵ����ڵ�����
*/
uint8_t BMP180_Read_Byte(uint8_t REG_Addr)
{
    uint8_t data;

    bsp_I2C_SW_Read_Byte(BMP180_ADDRESS, REG_Addr, &data);

    return data;
}

/*
*����˵���� ��BMP180�ļĴ����������ֽ�����
*�����βΣ� REG_Addr���Ĵ�����ַ
*������ݣ� �Ĵ����ڵ�����
*/
int16_t BMP180_Read_2Byte(uint8_t ReadAddr)
{
    uint8_t data[2];

    bsp_I2C_SW_Read_nByte(BMP180_ADDRESS, ReadAddr, data, 2);

    return ((uint16_t)(data[0]<<8) | data[1]);
}

uint8_t BMP180_Read_ID(void)
{
    return BMP180_Read_Byte(BMP180_ID_REG);
}

/*
*����˵���� ��BMP180��ȡδ�������¶�
*�����βΣ� ��
*������ݣ� ��
*/
int32_t BMP180_Read_UT(void)
{
    long temp = 0;
    BMP180_Write_Byte(0xF4,0x2E);

    delay_ms(5);
    temp = (long)BMP180_Read_2Byte(0xF6);
    return temp;
}

/*
*����˵���� ��BMP180��ȡδ�����Ĵ���ѹ
*�����βΣ� ��
*������ݣ� ��
*/
int32_t BMP180_Read_UP(void)
{
    long pressure = 0;

    BMP180_Write_Byte(0xF4,0x34);
    delay_ms(5);

    pressure = (long)BMP180_Read_2Byte(0xF6);
    //pressure = pressure + BMP_ReadOneByte(0xf8);
    pressure &= 0x0000FFFF;

    return pressure;
}

/*
*����˵���� �û�ȡ�Ĳ������¶Ⱥʹ���ѹ���������������㺣��
*�����βΣ� ��
*������ݣ� ��
*/
void BMP180_UncompemstatedToTrue(void)
{
    bmp180.UT = BMP180_Read_UT();//��һ�ζ�ȡ����
    bmp180.UT = BMP180_Read_UT();//���еڶ��ζ�ȡ��������
    bmp180.UP = BMP180_Read_UP();

    bmp180.X1 = ((bmp180.UT - bmp180.AC6) * bmp180.AC5) >> 15;
    bmp180.X2 = (((long)bmp180.MC) << 11) / (bmp180.X1 + bmp180.MD);
    bmp180.B5 = bmp180.X1 + bmp180.X2;
    bmp180.Temp  = (bmp180.B5 + 8) >> 4;

    bmp180.B6 = bmp180.B5 - 4000;
    bmp180.X1 = ((long)bmp180.B2 * (bmp180.B6 * bmp180.B6 >> 12)) >> 11;
    bmp180.X2 = ((long)bmp180.AC2) * bmp180.B6 >> 11;
    bmp180.X3 = bmp180.X1 + bmp180.X2;

    bmp180.B3 = ((((long)bmp180.AC1) * 4 + bmp180.X3) + 2) /4;
    bmp180.X1 = ((long)bmp180.AC3) * bmp180.B6 >> 13;
    bmp180.X2 = (((long)bmp180.B1) *(bmp180.B6*bmp180.B6 >> 12)) >>16;
    bmp180.X3 = ((bmp180.X1 + bmp180.X2) + 2) >> 2;
    bmp180.B4 = ((long)bmp180.AC4) * (unsigned long)(bmp180.X3 + 32768) >> 15;
    bmp180.B7 = ((unsigned long)bmp180.UP - bmp180.B3) * 50000;

    if(bmp180.B7 < 0x80000000)
    {
        bmp180.p = (bmp180.B7 * 2) / bmp180.B4;     
    }
    else
    {
        bmp180.p = (bmp180.B7 / bmp180.B4) * 2;
    }

    bmp180.X1 = (bmp180.p >> 8) * (bmp180.p >>8);
    bmp180.X1 = (((long)bmp180.X1) * 3038) >> 16;
    bmp180.X2 = (-7357 * bmp180.p) >> 16;

    bmp180.p = bmp180.p + ((bmp180.X1 + bmp180.X2 + 3791) >> 4);

    bmp180.altitude = 44330 * (1-pow(((bmp180.p) / 101325.0),(1.0/5.255)));  
}

/*
*����˵���� ����BMP180����
*�����βΣ� ��
*������ݣ� ��
*/
void BMP180_Test(void)
{
    BMP180_UncompemstatedToTrue();
    printf("��ȡID��%X \r\n",BMP180_Read_ID());
    printf("��ȡ�¶ȣ�%d.%d C\r\n",bmp180.Temp/10,bmp180.Temp%10);
    printf("��ȡ��ѹ��%d Pa\r\n",bmp180.p);
    printf("��ȡ���θ߶ȣ�%.5f m\r\n",bmp180.altitude);
}
