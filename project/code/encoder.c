#include "system.h"


// 全局变量用于存储左右轮速度，有多处调用，send中使用；
float left_speed_global = 0.0f;
float right_speed_global = 0.0f;
float speed_avl = 0.0f;// 平均速度

// 滑窗滤波数组，存储历史数据供滤波使用
float count_L_group[3]={0};
float count_R_group[3]={0};

void encoder_init(void)
{
    // 初始化左侧编码器
    encoder_dir_init(ENCODER_DIR_L, ENCODER_DIR_DIR_L, ENCODER_DIR_PULSE_L);
    // 初始化右侧编码器
    encoder_dir_init(ENCODER_DIR_R, ENCODER_DIR_DIR_R, ENCODER_DIR_PULSE_R);
}



//读取编码器数值并滤波
float encoder_L_get(void)//100%  1570
{
    int16 encoder_L = 0;
    // 读取计数值（驱动层已根据方向引脚自动处理符号，无需重复判向）
    encoder_L = encoder_get_count(ENCODER_DIR_L);

    // 清除编码器计数值，为下次计数做准备
    encoder_clear_count(ENCODER_DIR_L);
	
		//滑动窗口滤波
		encoder_L=(int16)sliding_window_filter(count_L_group,(float)encoder_L,3);
	
    //返回编码器计算数值
    return (float)-encoder_L;
}

float encoder_R_get(void)//100%  1580
{
    int16 encoder_R = 0;
    // 读取计数值（驱动层已根据方向引脚自动处理符号，无需重复判向）
    encoder_R = encoder_get_count(ENCODER_DIR_R);

    // 清除编码器计数值，为下次计数做准备
    encoder_clear_count(ENCODER_DIR_R);
	
		//滑动窗口滤波
		encoder_R=(int16)sliding_window_filter(count_R_group,(float)encoder_R,3);
	
    //返回编码器计算数值
    return (float)encoder_R;
}

void encoder_update(void)
{
	
    left_speed_global = encoder_L_get();
    right_speed_global = encoder_R_get();

    speed_avl= (left_speed_global+right_speed_global)/2;    
}   
