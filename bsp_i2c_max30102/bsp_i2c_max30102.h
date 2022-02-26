#ifndef __BSP_I2C_MAX30102_H
#define __BSP_I2C_MAX30102_H

#include "stm32h7xx_hal.h"

#define MAX30102_INT_PROT       GPIOE
#define MAX30102_INT_PIN        GPIO_PIN_2
#define MAX30102_INT_CLK        __HAL_RCC_GPIOE_CLK_ENABLE()

#define MAX30102_INT_LEVEL      ((MAX30102_INT_PROT->IDR & MAX30102_INT_PIN) == MAX30102_INT_PIN) ? 1 : 0

/* I2C从地址 */
#define MAX30102_ADDR           (0x57<<1)
#define MAX30102_WRITE_ADDR     (MAX30102_ADDR)
#define MAX30102_READ_ADDR      (MAX30102_ADDR | 1)

/* 寄存器地址 */
#define MAX30102_INTR_STATUS_1 0x00
#define MAX30102_INTR_STATUS_2 0x01
#define MAX30102_INTR_ENABLE_1 0x02
#define MAX30102_INTR_ENABLE_2 0x03
#define MAX30102_FIFO_WR_PTR 0x04
#define MAX30102_OVF_COUNTER 0x05
#define MAX30102_FIFO_RD_PTR 0x06
#define MAX30102_FIFO_DATA 0x07
#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_SPO2_CONFIG 0x0A
#define MAX30102_LED1_PA 0x0C
#define MAX30102_LED2_PA 0x0D
#define MAX30102_PILOT_PA 0x10
#define MAX30102_MULTI_LED_CTRL1 0x11
#define MAX30102_MULTI_LED_CTRL2 0x12
#define MAX30102_TEMP_INTR 0x1F
#define MAX30102_TEMP_FRAC 0x20
#define MAX30102_TEMP_CONFIG 0x21
#define MAX30102_PROX_INT_THRESH 0x30
#define MAX30102_REV_ID 0xFE
#define MAX30102_PART_ID 0xFF

/* 函数声明 */
void bsp_Max30102_Write_Byte(uint8_t REG_addr, uint8_t write_data);
void bsp_Max30102_Read_Byte(uint8_t REG_addr, uint8_t *read_data);
void bsp_Max30102_Write_nByte(uint8_t REG_addr, uint8_t *read_data, uint16_t Length);
void bsp_Max30102_Read_nByte(uint8_t REG_addr, uint8_t *write_data, uint16_t Length);
void bsp_Max30102_Read_FIFO(uint32_t *red_led, uint32_t *ir_led);
void bsp_Max30102_Init(void);

void bsp_Max30102_Test(void);
/*************************** 用于MAX30102数据处理 ***************************/
#define true 1
#define false 0
#define FS 100
#define BUFFER_SIZE  (FS* 5) 
#define HR_FIFO_SIZE 7
#define MA4_SIZE  4 // DO NOT CHANGE
#define HAMMING_SIZE  5// DO NOT CHANGE
#define min(x,y) ((x) < (y) ? (x) : (y))

static  int32_t an_dx[BUFFER_SIZE-MA4_SIZE]; // delta
static  int32_t an_x[BUFFER_SIZE]; //ir
static  int32_t an_y[BUFFER_SIZE]; //red

/* 函数声明 */
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer ,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer ,   int32_t *pn_spo2, int8_t *pch_spo2_valid ,  int32_t *pn_heart_rate , int8_t  *pch_hr_valid);
void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks,  int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks,  int32_t *pn_x, int32_t n_size, int32_t n_min_height);
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks,   int32_t  *pn_x, int32_t n_min_distance);
void maxim_sort_ascend(int32_t *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);

														
/************************** 用于MAX30102数据处理end **************************/


#endif
