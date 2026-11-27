#ifndef _ADC_H
#define _ADC_H

#define Adc_Channel_Number  4
#define Mean_Filter_Times 10

/***************ADC通道宏定义*********/

#define Adc_LEFT1       (ADC_CH8_P00)       // 左横电感采集板1 (P0.0)
#define Adc_LEFT2       (ADC_CH9_P01)       // 左竖电感采集板2 (P0.1)
#define Adc_RIGHT1      (ADC_CH13_P05)      // 右竖电感采集板1 (P0.5)
#define Adc_RIGHT2      (ADC_CH14_P06)      // 右横电感采集板2 (P0.6)

void init_Adc(void);
void Adc_Read(void);
void Adc_Count (void);
void Adc_Limit(float* temp,float min,float max);
float filter_mid3(float val);
float Error_Count();
float sliding_window_filter(float* input, float newdata ,int length);
float one_filter_float(float now,float last,float a);

int Mean_Filter(int* Temp,int Times);
//全局变量
extern float Adc_LEFT1_Value,Adc_LEFT2_Value,Adc_RIGHT1_Value,Adc_RIGHT2_Value;
extern int Avg_Adc[4];
extern float Err;
extern float l, r, lm, rm;


#endif