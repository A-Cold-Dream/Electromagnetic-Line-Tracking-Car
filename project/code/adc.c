#include "system.h"

int Avg_Adc[4]={0};
// adc最大最小值，上跑道测
int Adc_Max[4]={160,1830,3460,150};//（510，510，510，510）1.06
int Adc_Min[4]={30,210,1130,30};    //（100，140，140，90）/2
float error_group[3]={0};
float Adc_LEFT1_Value,Adc_LEFT2_Value,Adc_RIGHT1_Value,Adc_RIGHT2_Value;
float Err;
// 差比和差算法系数（全局变量，可通过VOFA调整）
float err_A = 1.50f, err_B = 3.50f, err_C = 1.0f, err_P = 1.0f;
float l, r, lm, rm;
float numerator, denominator;

//adc初始化函数
void init_Adc(void)
{
    adc_init(Adc_LEFT1, ADC_12BIT);
    adc_init(Adc_LEFT2, ADC_12BIT);
    adc_init(Adc_RIGHT1, ADC_12BIT);
    adc_init(Adc_RIGHT2, ADC_12BIT);
}

//均值滤波函数，去除最大最小值
int Mean_Filter(int* Temp,int Times)
{
	int i=0;
	int Min=4096,Max=0,Sum = 0;
	for(i = 0;i<Times;i++)
	{
		if(Temp[i]>Max)
			Max = Temp[i];
		if(Min > Temp[i])
			Min = Temp[i];
		  Sum+=Temp[i];
	}
	return (Sum-Max-Min)/(Times-2);
}

//adc限幅函数
void Adc_Limit(float* temp,float min,float max)
{
	*temp=*temp>max?max:*temp;
	*temp=*temp<min?min:*temp;
}

//采集Adc各个通道数值，并去除最大最小值均值滤波
void Adc_Read(void)
{

    int i;
    int Adc_Value[4][10]={0};

//*******************************连续采集10次电感值*******************************//
    for(i=0;i<10;i++)
    {
        Adc_Value[0][i]=adc_convert(Adc_LEFT1);   //采集左横电感数据
        Adc_Value[1][i]=adc_convert(Adc_LEFT2);   //采集左竖电感数据
        Adc_Value[2][i]=adc_convert(Adc_RIGHT1);  //采集右竖电感数据
        Adc_Value[3][i]=adc_convert(Adc_RIGHT2);  //采集右横电感数据

    }
/*===========================超级滤波并去除个位数=================================*/
    for (i=0;i<Adc_Channel_Number;i++)
    {
        Avg_Adc[i]=Mean_Filter(Adc_Value[i],10);//均值滤波函数调用
    }
   for(i=0;i<Adc_Channel_Number ;i++)            //将数值中个位数除掉
   {
       Avg_Adc[i] = (int)(Avg_Adc[i]/10*10);
   }

}




void Adc_Count (void)//电感值归一化
{
    Adc_LEFT1_Value =  (float)(Avg_Adc[0]-Adc_Min[0]) /  (Adc_Max[0]- Adc_Min[0]) * 100.0;		// 左横电感归一化
    Adc_LEFT2_Value =  (float)(Avg_Adc[1]-Adc_Min[1]) /  (Adc_Max[1]- Adc_Min[1]) * 100.0;		// 左竖电感归一化
    Adc_RIGHT1_Value = (float)(Avg_Adc[2]-Adc_Min[2]) /  (Adc_Max[2]- Adc_Min[2]) * 100.0;		// 右竖电感归一化
    Adc_RIGHT2_Value = (float)(Avg_Adc[3]-Adc_Min[3]) /  (Adc_Max[3]- Adc_Min[3]) * 100.0;		// 右横电感归一化
		//电感值限幅0-100
    Adc_Limit(&Adc_LEFT1_Value,1,100);
    Adc_Limit(&Adc_LEFT2_Value,1,100);
    Adc_Limit(&Adc_RIGHT1_Value,1,100);
    Adc_Limit(&Adc_RIGHT2_Value,1,100);
}


float Error_Count(void)
{
    
    l = Adc_LEFT1_Value;
    r = Adc_RIGHT2_Value;
    lm = Adc_LEFT2_Value;
    rm = Adc_RIGHT1_Value;
    
    numerator = err_A * (l - r) + err_B * (lm - rm);
    denominator = err_A * (l + r) + err_C * abs(lm - rm) + 0.01f;   
    Err = (numerator / denominator) * err_P;
    Err = sliding_window_filter(error_group, Err, 3);
    
    return Err;
}

float sliding_window_filter(float* input, float newdata ,int length)
{
	int i=0;
	float sum=0;
	for(i=0;i<length-1;i++)
	{
		input[i]=input[i+1];
		sum+=input[i];
	}
	input[length-1]=newdata;
	sum+=input[length-1];
	return sum/length;
}

// 3点中值滤波（抗尖刺，不延迟）
float filter_mid3(float val)
{
    static float buf[3];
    static uint8 i = 0;
    float a, b, c;

    buf[i++] = val;
    i %= 3;

    a = buf[0];
    b = buf[1];
    c = buf[2];

    // 取中间值
    if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
    if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
    return c;
}
//一阶低通滤波函数
float one_filter_float(float now,float last,float a)
{
	float out;
	out=a*(now)+(1-a)*last;
	return out;
}


