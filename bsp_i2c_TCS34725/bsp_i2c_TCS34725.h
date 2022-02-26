
#ifndef __TCS34725_H
#define __TCS34725_H

//变量重定义
#define uint8_t			unsigned char
#define uint16_t		unsigned short
#define uint32_t		unsigned int

//从地址宏定义
#define TCS34725_ADDRESS			0x52
#define TCS34725_WRITE_ADDRESS		0X52			/* 从设备写地址 */
#define TCS34725_READ_ADDRESS		(0X52|0X01)		/* 从设备读地址 */
//命令位
#define TCS34725_COMMAND_BIT      (0x80)
//寄存器地址宏
#define TCS34725_ENABLE           (0x00)
#define TCS34725_ENABLE_AIEN      (0x10)    /* RGBC Interrupt Enable */
#define TCS34725_ENABLE_WEN       (0x08)    /* Wait enable - Writing 1 activates the wait timer */
#define TCS34725_ENABLE_AEN       (0x02)    /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define TCS34725_ENABLE_PON       (0x01)    /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
#define TCS34725_ATIME            (0x01)    /* Integration time */
#define TCS34725_WTIME            (0x03)    /* Wait time (if TCS34725_ENABLE_WEN is asserted) */
#define TCS34725_WTIME_2_4MS      (0xFF)    /* WLONG0 = 2.4ms   WLONG1 = 0.029s */
#define TCS34725_WTIME_204MS      (0xAB)    /* WLONG0 = 204ms   WLONG1 = 2.45s  */
#define TCS34725_WTIME_614MS      (0x00)    /* WLONG0 = 614ms   WLONG1 = 7.4s   */
#define TCS34725_AILTL            (0x04)    /* Clear channel lower interrupt threshold */
#define TCS34725_AILTH            (0x05)
#define TCS34725_AIHTL            (0x06)    /* Clear channel upper interrupt threshold */
#define TCS34725_AIHTH            (0x07)
#define TCS34725_PERS             (0x0C)    /* Persistence register - basic SW filtering mechanism for interrupts */
#define TCS34725_PERS_NONE        (0b0000)  /* Every RGBC cycle generates an interrupt                                */
#define TCS34725_PERS_1_CYCLE     (0b0001)  /* 1 clean channel value outside threshold range generates an interrupt   */
#define TCS34725_PERS_2_CYCLE     (0b0010)  /* 2 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_3_CYCLE     (0b0011)  /* 3 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_5_CYCLE     (0b0100)  /* 5 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_10_CYCLE    (0b0101)  /* 10 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_15_CYCLE    (0b0110)  /* 15 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_20_CYCLE    (0b0111)  /* 20 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_25_CYCLE    (0b1000)  /* 25 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_30_CYCLE    (0b1001)  /* 30 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_35_CYCLE    (0b1010)  /* 35 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_40_CYCLE    (0b1011)  /* 40 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_45_CYCLE    (0b1100)  /* 45 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_50_CYCLE    (0b1101)  /* 50 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_55_CYCLE    (0b1110)  /* 55 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_60_CYCLE    (0b1111)  /* 60 clean channel values outside threshold range generates an interrupt */
#define TCS34725_CONFIG           (0x0D)
#define TCS34725_CONFIG_WLONG     (0x02)    /* Choose between short and long (12x) wait times via TCS34725_WTIME */
#define TCS34725_CONTROL          (0x0F)    /* Set the gain level for the sensor */
#define TCS34725_ID               (0x12)    /* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_STATUS           (0x13)
#define TCS34725_STATUS_AINT      (0x10)    /* RGBC Clean channel interrupt */
#define TCS34725_STATUS_AVALID    (0x01)    /* Indicates that the RGBC channels have completed an integration cycle */
#define TCS34725_CDATAL           (0x14)    /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)    /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)    /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)    /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)


#define max3v(v1, v2, v3)   ((v1)<(v2)? ((v2)<(v3)?(v3):(v2)):((v1)<(v3)?(v3):(v1)))
#define min3v(v1, v2, v3)   ((v1)>(v2)? ((v2)>(v3)?(v3):(v2)):((v1)>(v3)?(v3):(v1)))

/* RGB色彩结构体 */
typedef struct{
	uint16_t c;      //[0-65536]
	uint16_t r;
	uint16_t g;
	uint16_t b;
}TCS34725_RGBC;

/* HSL色彩结构体 */
typedef struct{
	uint16_t h;       //[0,360]
	uint8_t  s;       //[0,100]
	uint8_t  l;       //[0,100]
}TCS34725_HSL;

/* 积分时间枚举 */
typedef enum {
	INTEGRATION_2_4MS = 0xFF,   /**<  2.4ms - 1 cycle    - Max Count: 1024  */
	INTEGRATION_24MS  = 0xF6,   /**<  24ms  - 10 cycles  - Max Count: 10240 */
	INTEGRATION_50MS  = 0xEB,   /**<  50ms  - 20 cycles  - Max Count: 20480 */
	INTEGRATION_101MS = 0xD5,   /**<  101ms - 42 cycles  - Max Count: 43008 */
	INTEGRATION_154MS = 0xC0,   /**<  154ms - 64 cycles  - Max Count: 65535 */
	INTEGRATION_240MS = 0x9C,   /**<  240ms - 100 cycles - Max Count: 65535 */
	INTEGRATION_700MS = 0x00,   /**<  700ms - 256 cycles - Max Count: 65535 */
}TCS34725_INTEGRATIONTIME;

/* 增益枚举 */
typedef enum{
	GAIN_1X  = 0x00,   /**<  No gain  */
	GAIN_4X  = 0x01,   /**<  4x gain  */
	GAIN_16X = 0x02,   /**<  16x gain */
	GAIN_60X = 0x03,   /**<  60x gain */
}TCS34725_GAIN;

extern TCS34725_RGBC TCS34725_rgbc;
extern TCS34725_HSL  TCS34725_hsl;

//函数声明
void TCS34725_Write_Byte(uint8_t REG_Address, uint8_t write_byte);
void TCS34725_Read_Byte(uint8_t REG_Address, uint8_t *read_addr);
void TCS34725_Write_nByte(uint8_t REG_Address, uint8_t *write_addr, uint8_t number);
void TCS34725_Read_nByte(uint8_t REG_Address, uint8_t *read_addr, uint8_t number);
void TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME time);
void TCS34725_SetGain(TCS34725_GAIN gain);
void TCS34725_Enable(void);
void TCS34725_Disable(void);
uint8_t TCS34725_Read_ID(void);
uint8_t TCS34725_GetChannelData(uint8_t RGB_Channel_addr);
uint8_t TCS34725_GetRawData(void);
void TCS34725_RGBtoHSL(void);
void TCS34725_Init(void);

void TCS34725_Test(void);
#endif
