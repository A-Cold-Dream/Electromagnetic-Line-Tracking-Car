#ifndef __IMU660RA_H
#define __IMU660RA_H
#include "system.h"

//extern float null_drift_z;	//零漂
extern float imu_angle;         // 偏航角（检测左右）   
extern float imu_pitch;         // 俯仰角（检测前后翘起）
extern float avl_gyro_z;        // 原始Z轴角速度（未滤波）
extern float avl_gyro_x;        // 原始X轴角速度（未滤波）
extern float avl_gyro_y;        // 原始Y轴角速度（未滤波）      
extern float gyro_z_filtered;     // 过滤后的Z轴角速度（用于计算偏航角）
extern float gyro_x_filtered;     // 过滤后的X轴角速度（用于计算俯仰角）
extern float gyro_y_filtered;
extern uint8 car_tilt_flag;       // 翘起检测标志（0:正常, 1:翘起）
extern float tilt_threshold;      // 翘起角度阈值

void gyro_get(void);
void angle_get(void);
void angle_clear(void);
void tilt_detect(void);           // 翘起检测函数
//void null_drift_calculate(void);

#endif