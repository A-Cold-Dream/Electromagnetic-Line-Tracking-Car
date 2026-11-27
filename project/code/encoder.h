#ifndef _ENCODER_H
#define _ENCODER_H
#include "zf_common_typedef.h"


// 编码器引脚定义
#define ENCODER_DIR_L                   (TIM3_ENCOEDER)    //修改0--->3
#define ENCODER_DIR_DIR_L               (IO_P53)           //35-->53
#define ENCODER_DIR_PULSE_L             (TIM3_ENCOEDER_P04)//

#define ENCODER_DIR_R                   (TIM0_ENCOEDER)
#define ENCODER_DIR_DIR_R               (IO_P35)
#define ENCODER_DIR_PULSE_R             (TIM0_ENCOEDER_P34)


// 路程转换系数
#define ENCODER_DISTANCE                (0.03485)
#define encoder_pulses 1024 //编码器线数
#define wheel_d   4.0f  //轮子直径cm
#define time      0.002f//采样时间2ms
#define pi        3.1415926f

// 全局变量声明（外部可访问）
extern float left_speed_global;   // 左轮速度
extern float right_speed_global;   // 右轮速度
extern float speed_avl; // 平均速度

// 函数声明
void encoder_init(void);
float encoder_L_get(void);
float encoder_R_get(void);
void encoder_update(void);

#endif