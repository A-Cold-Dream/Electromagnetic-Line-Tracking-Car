#ifndef _SYSTEM_H
#define _SYSTEM_H

//ȫ��ͷ�ļ����ϣ�ͳһ�����������ظ�����
#include "zf_common_headfile.h"
#include "zf_device_wireless_uart.h"
#include "zf_common_debug.h"
#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_device_ips114.h"
#include "send.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "adc.h"
#include "action.h"
#include "imu.h"
#include "menu.h"

void system_init(void);

//---- 中断标志位（ISR中只设置，主循环中检查并清除）----
extern volatile uint8 flag_5ms;    // 5ms周期标志
extern volatile uint8 flag_10ms;   // 10ms周期标志

//---- 任务函数声明（从ISR中抽离的逻辑，在主循环中调用）----
void task_5ms(void);   // 5ms任务：按键、编码器、陀螺仪、电机控制
void task_10ms(void);  // 10ms任务：ADC采集、误差计算、偏差PID

#endif