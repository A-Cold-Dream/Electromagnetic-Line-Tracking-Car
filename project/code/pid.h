#ifndef _PID_H
#define _PID_H

#include "zf_common_typedef.h"

// PID结构体定义
typedef struct 
{
    float SetValue;     // 目标值
    float ActValue;     // 实际值
    float ek;           // 当前误差
    float ek_1;         // 上一次误差
    float ek_2;         // 上两次误差
    float ek_sum;       // 误差累积（积分项）
    float KP;           // 比例系数
    float KI;           // 积分系数
    float KD;           // 微分系数
    float PIDmax;       // 输出上限
    float PIDmin;       // 输出下限
    float PIDout;       // PID输出
} PID;

// 声明全局变量
extern PID Speed;      // 速度环参数配置（统一调参用）
extern PID Speed_L;    // 左轮速度环
extern PID Speed_R;    // 右轮速度环
extern PID Deviation;  // 偏差环
extern PID Angle;      // 角度环
extern float Speed_L_Out;  // 左轮速度PID输出
extern float Speed_R_Out;  // 右轮速度PID输出
extern float base_speed;   // 基础速度

// 函数声明
void PID_Init(void);
void Speed_PID_Sync(void);
float PID_Control_Inc(PID* pid);
float PID_Control_Pos(PID* pid);
void Deviation_PID_Count(void);
void Speed_PID_Count(void);
void Angle_PID_Count(void);

#endif