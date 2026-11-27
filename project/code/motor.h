#ifndef _MOTOR_H
#define _MOTOR_H

// 电机占空比限幅（百分比：-100~100）
#define duty_max  6000    // 正转最大100%
#define duty_min  -3000   // 反转最大-100%
#define fan_max   800    // 风扇最大800

//函数声明
void init_PWM(void);
void Fan_Out(int fan_duty);
void Motor_Out(int left_duty,int right_duty);//

//电机引脚宏定义
#define DIR_R           (IO_P66)            // 方向_右
#define PWM_R           (PWMA_CH3P_P64)     // 驱动引脚_右
#define DIR_L           (IO_P62)            // 方向_左
#define PWM_L           (PWMA_CH1P_P60)     // 驱动引脚_左

#define PWM_1           (PWMB_CH3_P33)			// 无刷

#endif


