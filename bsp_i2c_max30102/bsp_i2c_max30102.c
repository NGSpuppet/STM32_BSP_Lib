

#include "bsp.h"
#include "bsp_i2c_max30102.h"

/*
*����˵���� ��MAX30102д��һ���ֽ�
*�����βΣ� REG_addr���Ĵ�����ַ
*           write_data��Ҫд�������
*������ݣ� ��
*/
void bsp_Max30102_Write_Byte(uint8_t REG_addr, uint8_t write_data)
{
    bsp_I2C_SW_Write_Byte(MAX30102_ADDR, REG_addr, write_data);
}

/*
*����˵���� ��MAX30102��ȡһ���ֽ�
*�����βΣ� REG_addr���Ĵ�����ַ
*           read_data����ȡ���ݻ����ַ
*������ݣ� ��
*/
void bsp_Max30102_Read_Byte(uint8_t REG_addr, uint8_t *read_data)
{
    bsp_I2C_SW_Read_Byte(MAX30102_ADDR, REG_addr, read_data);
}

/*
*����˵���� ��MAX30102д��n���ֽ�
*�����βΣ� REG_addr���Ĵ�����ַ
*           write_data��Ҫд�������
*           Length��д��������1~65535��
*������ݣ� ��
*/
void bsp_Max30102_Write_nByte(uint8_t REG_addr, uint8_t *read_data, uint16_t Length)
{
    bsp_I2C_SW_Write_nByte(MAX30102_ADDR, REG_addr, read_data, Length);
}

/*
*����˵���� ��MAX30102��ȡn���ֽ�
*�����βΣ� REG_addr���Ĵ�����ַ
*           read_data����ȡ���ݻ����ַ
*           Length��д��������1~65535��
*������ݣ� ��
*/
void bsp_Max30102_Read_nByte(uint8_t REG_addr, uint8_t *write_data, uint16_t Length)
{
    bsp_I2C_SW_Read_nByte(MAX30102_ADDR, REG_addr, write_data, Length);
}

/*
*����˵���� MAX30102��INT�ܽų�ʼ��
*�����βΣ� ��
*������ݣ� ��
*/
static void bsp_Max30102_GPIO_INT_Config(void)
{
    GPIO_InitTypeDef    Max30102_GPIO_INT = {0};

    MAX30102_INT_CLK;   /* ����INT��Ӧ��IOʱ�� */

    Max30102_GPIO_INT.Mode = GPIO_MODE_INPUT;       /* ����ģʽ */
    Max30102_GPIO_INT.Pull = GPIO_NOPULL;           /* ������ */
    Max30102_GPIO_INT.Pin = MAX30102_INT_PIN;
    HAL_GPIO_Init(MAX30102_INT_PROT, &Max30102_GPIO_INT);
}

/*
*����˵���� ��ȡ�����״̬�Ĵ���
*�����βΣ� status_data��״̬�Ĵ������ݻ����ַ
*������ݣ� ��
*/
static void bsp_Max30102_Read_Clear_Status(void)
{
    uint8_t status_data;
    /* ��ȡ�����״̬�Ĵ��� */
    bsp_Max30102_Read_Byte(MAX30102_INTR_STATUS_1, &status_data);
    bsp_Max30102_Read_Byte(MAX30102_INTR_STATUS_2, &status_data);
}

/*
*����˵���� ��λMAX30102
*�����βΣ� ��
*�������;  ��
*/
static void bsp_Max30102_Reset_Max30102(void)
{
    bsp_Max30102_Write_Byte(MAX30102_MODE_CONFIG,0x40);
}

/*
*����˵���� ��MAX30102��ȡFIFO����
*�����βΣ� red_led����ɫLEDԭʼ����
*           ir_led������ԭʼ����
*������ݣ� ��
*/
void bsp_Max30102_Read_FIFO(uint32_t *red_led, uint32_t *ir_led)
{
    *red_led = 0;
    *ir_led = 0;
    uint32_t un_temp;
    uint8_t ach_i2c_data[6];

    /* ��ȡ�����״̬�Ĵ��� */
    bsp_Max30102_Read_Clear_Status();

    bsp_Max30102_Read_nByte(MAX30102_FIFO_DATA, ach_i2c_data, 6);

    un_temp = (uint8_t)ach_i2c_data[0];
    un_temp <<= 16;
    *red_led += un_temp;
    un_temp = (uint8_t)ach_i2c_data[1];
    un_temp <<= 8;
    *red_led += un_temp;
    un_temp = (uint8_t)ach_i2c_data[2];
    *red_led += un_temp;

    un_temp = (unsigned char) ach_i2c_data[3];
    un_temp <<= 16;
    *ir_led += un_temp;
    un_temp = (unsigned char) ach_i2c_data[4];
    un_temp <<= 8;
    *ir_led += un_temp;
    un_temp = (unsigned char) ach_i2c_data[5];
    *ir_led += un_temp;

    *red_led &= 0x03FFFF;  //Mask MSB [23:18]
    *ir_led &= 0x03FFFF;  //Mask MSB [23:18]

}

/*
*����˵���� ��ʼ��MAX30102
*�����βΣ� ��
*������ݣ� ��
*/
void bsp_Max30102_Init(void)
{
	bsp_Max30102_GPIO_INT_Config();
	
    bsp_Max30102_Reset_Max30102();
    bsp_Max30102_Read_Clear_Status();

    bsp_Max30102_Write_Byte(MAX30102_INTR_ENABLE_1,0xc0);    /* �жϼĴ������� */
    bsp_Max30102_Write_Byte(MAX30102_INTR_ENABLE_2,0x00);
    bsp_Max30102_Write_Byte(MAX30102_FIFO_WR_PTR,0x00);     /* FIFO_WR_PTR[4:0] */
    bsp_Max30102_Write_Byte(MAX30102_OVF_COUNTER,0x00);     /* OVF_COUNTER[4:0] */
    bsp_Max30102_Write_Byte(MAX30102_FIFO_RD_PTR,0x00);     /* FIFO_RD_PTR[4:0] */
    bsp_Max30102_Write_Byte(MAX30102_FIFO_CONFIG,0x0f);
    bsp_Max30102_Write_Byte(MAX30102_MODE_CONFIG,0x03);      /* 0x02�������ں�ɫ��0x03������SpO2ģʽ��0x07����ģLED */
    bsp_Max30102_Write_Byte(MAX30102_SPO2_CONFIG,0x27);      /* SPO2_ADC��Χ��4096nA��Sp02�����ʣ�100Hz��LED����400nS */
    bsp_Max30102_Write_Byte(MAX30102_LED1_PA,0x24);          /* ΪLED1ѡ��7mA��ֵ */
    bsp_Max30102_Write_Byte(MAX30102_LED2_PA,0x24);          /* ΪLED2ѡ��7mA��ֵ */
    bsp_Max30102_Write_Byte(MAX30102_PILOT_PA,0x7f);         /*  ΪPILOT LEDѡ��25mA��ֵ */
}

#define MAX_BRIGHTNESS 255
/*
*����˵���� Max30102���Գ���
*�����βΣ� ��
*������ݣ� ��
*/
void bsp_Max30102_Test(void)
{
    uint32_t aun_ir_buffer[500]; //���⴫����ֵ
    int32_t n_ir_buffer_length;    //���ݳ���
    uint32_t aun_red_buffer[500];    //��ɫLED������ֵ
    int32_t n_sp02; //Ѫ��ֵ
    int8_t ch_spo2_valid;   //��ʾSPO2�����Ƿ���Ч
    int32_t n_heart_rate;   //����ֵ
    int8_t  ch_hr_valid;    //��ʾ���ʼ����Ƿ���Ч

    uint32_t un_min, un_max, un_prev_data;  //���ڼ��㷴ӳ�����İ��� LED ���ȵı���
    int i;
    int32_t n_brightness;
    float f_temp;

    n_brightness=0;
    un_min=0x3FFFF;
    un_max=0;

    n_ir_buffer_length=500;

    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(MAX30102_INT_LEVEL);      //�ȴ��жϹܽŴ���
        
        bsp_Max30102_Read_FIFO((aun_red_buffer+i), (aun_ir_buffer+i));  //�� MAX30102 FIFO ��ȡ
            
        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //�����ź���Сֵ
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //�����ź����ֵ
        printf("red = %d,    ", aun_red_buffer[i]);
        printf("ir = %d\r\n", aun_ir_buffer[i]);
    }
    un_prev_data=aun_red_buffer[i];


    //��ǰ 500 ��������������ǰ 5 �룩��������ʺ� SpO2
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    
    //���ϴ� MAX30102 ������ÿ 1 �����һ�����ʺ� SpO2
    while(1)
    {
        delay_ms(200);
        i=0;
        un_min=0x3FFFF;
        un_max=0;
        
        //ת���ڴ��е�ǰ 100 ������������� 400 ��������������
        for(i=100;i<500;i++)
        {
            aun_red_buffer[i-100]=aun_red_buffer[i];
            aun_ir_buffer[i-100]=aun_ir_buffer[i];
            
            //�����ź���Сֵ�����ֵ
            if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];
            if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];
        }
        
        //��ȡ100���������м���
        for(i=400;i<500;i++)
        {
            un_prev_data=aun_red_buffer[i-1];
            while(MAX30102_INT_LEVEL);
            bsp_Max30102_Read_FIFO((aun_red_buffer+i), (aun_ir_buffer+i));
        
            if(aun_red_buffer[i]>un_prev_data)//ֻ�Ǹ�����������AD���ݵ�ƫ����ȷ��LED������
            {
                f_temp=aun_red_buffer[i]-un_prev_data;
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness-=(int)f_temp;
                if(n_brightness<0)
                    n_brightness=0;
            }
            else
            {
                f_temp=un_prev_data-aun_red_buffer[i];
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness+=(int)f_temp;
                if(n_brightness>MAX_BRIGHTNESS)
                    n_brightness=MAX_BRIGHTNESS;
            }

            // /* ��ÿ��������ӡ */
            // if((ch_hr_valid == 1) && (ch_spo2_valid == 1))  //�ж�������Ч��
            // printf("red = %d, ir = %d, HR = %d, SpO2 = %d\r\n", aun_red_buffer[i], aun_ir_buffer[i], n_heart_rate, n_sp02);
        }
        //һ��������������˶���Ѫ������Ϊ150-230����ÿ��
        //��������ÿ��������������60��100��
         if((ch_hr_valid == 1) && (ch_spo2_valid == 1))
         printf("HR = %d, SpO2 = %d\r\n", n_heart_rate, n_sp02);

        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    }
}

/*************************** ����MAX30102���ݴ��� ***************************/

const uint16_t auw_hamm[31]={41,276,512,276,41}; //Hamm=  long16(512* hamming(5)');
//uch_spo2_table ����Ϊ -45,060 *ratioAverage *ratioAverage + 30,354 *ratioAverage + 94,845;
const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
                            99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
                            100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
                            97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
                            90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
                            80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
                            66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
                            49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
                            28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
                            3, 2, 1 } ;





/*
* \brief        �������ʺ� SpO2 ˮƽ
* \par          ϸ��
*               ͨ����� PPG ���ڵķ�ֵ����Ӧ�ĺ�/�����źŵ� AC/DC������ SPO2 �ı��ʡ�
*               ���ڸ��㷨��Ե��� Arm M0/M3�����ڼĴ��������SPO2 �Ĺ�ʽû�дﵽ׼ȷ�ȡ�
*               ��ˣ�׼ȷ�� SPO2 ��Ԥ�ȼ���ģ���Ϊÿ�����ʱ��� longo uch_spo2_table[]��
*
* \param[in]    *pun_ir_buffer           - ���⴫�������ݻ�����
* \param[in]    n_ir_buffer_length       - ���⴫�������ݻ���������
* \param[in]    *pun_red_buffer          - ��ɫ���������ݻ�����
* \param[out]    *pn_spo2                - ����� SpO2 ֵ
* \param[out]    *pch_spo2_valid         - 1 �������� SpO2 ֵ��Ч
* \param[out]    *pn_heart_rate          - ���������ֵ
* \param[out]    *pch_hr_valid           - 1 ������������ֵ��Ч
*
* \retval       None
*/
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, 
                              int32_t *pn_heart_rate, int8_t  *pch_hr_valid)
{
    uint32_t un_ir_mean ,un_only_once ;
    int32_t k ,n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count ,n_middle_idx;
    int32_t n_th1, n_npks,n_c_min;      
    int32_t an_ir_valley_locs[15] ;
    int32_t an_exact_ir_valley_locs[15] ;
    int32_t an_dx_peak_locs[15] ;
    int32_t n_peak_interval_sum;
    
    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc; 
    int32_t n_y_dc_max, n_x_dc_max; 
    int32_t n_y_dc_max_idx, n_x_dc_max_idx; 
    int32_t an_ratio[5],n_ratio_average; 
    int32_t n_nume,  n_denom ;
    //ȥ�� ir �źŵ� DC 
    un_ir_mean =0; 
    for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
    un_ir_mean =un_ir_mean/n_ir_buffer_length ;
    for (k=0 ; k<n_ir_buffer_length ; k++ )  an_x[k] =  pun_ir_buffer[k] - un_ir_mean ; 
    
    //4 ���ƶ�ƽ����
    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
        n_denom= ( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3]);
        an_x[k]=  n_denom/(int32_t)4; 
    }

    //�õ�ƽ�������źŵĲ���
    
    for( k=0; k<BUFFER_SIZE-MA4_SIZE-1;  k++)
        an_dx[k]= (an_x[k+1]- an_x[k]);

    //2 ���ƶ�ƽ���ߵ�ǰ
    for(k=0; k< BUFFER_SIZE-MA4_SIZE-2; k++){
        an_dx[k] =  ( an_dx[k]+an_dx[k+1])/2 ;
    }
    
    // ������
    // ��ת���Σ��Ա����ǿ����÷�ֵ���������ֵ
    for ( i=0 ; i<BUFFER_SIZE-HAMMING_SIZE-MA4_SIZE-2 ;i++){
        s= 0;
        for( k=i; k<i+ HAMMING_SIZE ;k++){
            s -= an_dx[k] *auw_hamm[k-i] ; 
                     }
        an_dx[i]= s/ (int32_t)1146; // ���� auw_hamm ���ܺ�
    }

 
    n_th1=0; // ��ֵ����
    for ( k=0 ; k<BUFFER_SIZE-HAMMING_SIZE ;k++){
        n_th1 += ((an_dx[k]>0)? an_dx[k] : ((int32_t)0-an_dx[k])) ;
    }
    n_th1= n_th1/ ( BUFFER_SIZE-HAMMING_SIZE);
    // ��ֵλ��ʵ������ԭʼ�ź�������λ�õ���������Ϊ���Ƿ�ת���ź�         
    maxim_find_peaks( an_dx_peak_locs, &n_npks, an_dx, BUFFER_SIZE-HAMMING_SIZE, n_th1, 8, 5 );//peak_height, peak_distance, max_num_peaks 

    n_peak_interval_sum =0;
    if (n_npks>=2){
        for (k=1; k<n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k]-an_dx_peak_locs[k -1]);
        n_peak_interval_sum=n_peak_interval_sum/(n_npks-1);
        *pn_heart_rate=(int32_t)(6000/n_peak_interval_sum);// ÿ���ӽ���
        *pch_hr_valid  = 1;
    }
    else  {
        *pn_heart_rate = -999;
        *pch_hr_valid  = 0;
    }
            
    for ( k=0 ; k<n_npks ;k++)
        an_ir_valley_locs[k]=an_dx_peak_locs[k]+HAMMING_SIZE/2; 


    // raw value : RED(=y) and IR(=X)
    // we need to assess DC and AC value of ir and red PPG. 
    for (k=0 ; k<n_ir_buffer_length ; k++ )  {
        an_x[k] =  pun_ir_buffer[k] ; 
        an_y[k] =  pun_red_buffer[k] ; 
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count =0; 
    for(k=0 ; k<n_npks ;k++){
        un_only_once =1;
        m=an_ir_valley_locs[k];
        n_c_min= 16777216;//2^24;
        if (m+5 <  BUFFER_SIZE-HAMMING_SIZE  && m-5 >0){
            for(i= m-5;i<m+5; i++)
                if (an_x[i]<n_c_min){
                    if (un_only_once >0){
                       un_only_once =0;
                   } 
                   n_c_min= an_x[i] ;
                   an_exact_ir_valley_locs[k]=i;
                }
            if (un_only_once ==0)
                n_exact_ir_valley_locs_count ++ ;
        }
    }
    if (n_exact_ir_valley_locs_count <2 ){
       *pn_spo2 =  -999 ; // do not use SPO2 since signal ratio is out of range
       *pch_spo2_valid  = 0; 
       return;
    }
    // 4 pt MA
    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
        an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int32_t)4;
        an_y[k]=( an_y[k]+an_y[k+1]+ an_y[k+2]+ an_y[k+3])/(int32_t)4;
    }

    //using an_exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration ratio
    //finding AC/DC maximum of raw ir * red between two valley locations
    n_ratio_average =0; 
    n_i_ratio_count =0; 
    
    for(k=0; k< 5; k++) an_ratio[k]=0;
    for (k=0; k< n_exact_ir_valley_locs_count; k++){
        if (an_exact_ir_valley_locs[k] > BUFFER_SIZE ){             
            *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid  = 0; 
            return;
        }
    }
    // find max between two valley locations 
    // and use ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2 

    for (k=0; k< n_exact_ir_valley_locs_count-1; k++){
        n_y_dc_max= -16777216 ; 
        n_x_dc_max= - 16777216; 
        if (an_exact_ir_valley_locs[k+1]-an_exact_ir_valley_locs[k] >10){
            for (i=an_exact_ir_valley_locs[k]; i< an_exact_ir_valley_locs[k+1]; i++){
                if (an_x[i]> n_x_dc_max) {n_x_dc_max =an_x[i];n_x_dc_max_idx =i; }
                if (an_y[i]> n_y_dc_max) {n_y_dc_max =an_y[i];n_y_dc_max_idx=i;}
            }
            n_y_ac= (an_y[an_exact_ir_valley_locs[k+1]] - an_y[an_exact_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_exact_ir_valley_locs[k]); //red
            n_y_ac=  an_y[an_exact_ir_valley_locs[k]] + n_y_ac/ (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k])  ; 
        
        
            n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw 
            n_x_ac= (an_x[an_exact_ir_valley_locs[k+1]] - an_x[an_exact_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_exact_ir_valley_locs[k]); // ir
            n_x_ac=  an_x[an_exact_ir_valley_locs[k]] + n_x_ac/ (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k]); 
            n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw 
            n_nume=( n_y_ac *n_x_dc_max)>>7 ; //prepare X100 to preserve floating value
            n_denom= ( n_x_ac *n_y_dc_max)>>7;
            if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
            {   
                an_ratio[n_i_ratio_count]= (n_nume*100)/n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
                n_i_ratio_count++;
            }
        }
    }

    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx= n_i_ratio_count/2;

    if (n_middle_idx >1)
        n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx ];

    if( n_ratio_average>2 && n_ratio_average <184){
        n_spo2_calc= uch_spo2_table[n_ratio_average] ;
        *pn_spo2 = n_spo2_calc ;
        *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
    }
    else{
        *pn_spo2 =  -999 ; // do not use SPO2 since signal ratio is out of range
        *pch_spo2_valid  = 0; 
    }
}

/*
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
{
    maxim_peaks_above_min_height( pn_locs, pn_npks, pn_x, n_size, n_min_height );
    maxim_remove_close_peaks( pn_locs, pn_npks, pn_x, n_min_distance );
    *pn_npks = min( *pn_npks, n_max_num );
}

/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t  *pn_x, int32_t n_size, int32_t n_min_height)
{
    int32_t i = 1, n_width;
    *pn_npks = 0;
    
    while (i < n_size-1){
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){            // find left edge of potential peaks
            n_width = 1;
            while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])    // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i+n_width] && (*pn_npks) < 15 ){                            // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;        
                // for flat peaks, peak location is left edge
                i += n_width+1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}

/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
{
    
    int32_t i, j, n_old_npks, n_dist;
    
    /* Order peaks from large to small */
    maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

    for ( i = -1; i < *pn_npks; i++ ){
        n_old_npks = *pn_npks;
        *pn_npks = i+1;
        for ( j = i+1; j < n_old_npks; j++ ){
            n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
            if ( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices longo ascending order
    maxim_sort_ascend( pn_locs, *pn_npks );
}

/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
void maxim_sort_ascend(int32_t *pn_x,int32_t n_size) 
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/ 
void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
            pn_indx[j] = pn_indx[j-1];
        pn_indx[j] = n_temp;
    }
}




