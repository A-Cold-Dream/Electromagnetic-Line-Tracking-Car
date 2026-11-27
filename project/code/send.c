#include "system.h"
#include <string.h>
#include <stdlib.h>

extern float base_speed;
extern float KP2;
extern PID Speed;
extern PID Deviation;
extern PID Angle;
extern float Err;
extern float left_speed_global;
extern float right_speed_global;
//extern float gyroz;
extern float a;
extern float k;
extern float diff_H;
extern float diff_S;
extern int fan_pwm;  // 风扇PWM输出值
extern float imu_angle;         // 偏航角（检测左右）   
extern float imu_pitch;         // 俯仰角（检测前后翘起）
extern float avl_gyro_z;        // 原始Z轴角速度（未滤波）
extern float avl_gyro_x;        // 原始X轴角速度（未滤波）
extern float avl_gyro_y;        // 原始Y轴角速度（未滤波）      
extern int8 element;            // 元素标志（0:正常循迹, 1:右环岛, 2:左环岛）      


// 差比和差算法系数（外部声明）
extern float err_A, err_B, err_C, err_P;

// 小车运动标志位：0=停止，1=运动
uint8 car_running_flag = 0;
// 数据发送选择：1=速度和PID, 2=角度PID, 3=ADC归一化, 4=差值数据, 5=原始ADC
uint8 send_data_select = 1;

void send_init(void)
{
//    wireless_uart_init();
    debug_init();
}

void send_total_speed(void)
{
    uint8 buffer[128] = {0};

    
    // 根据选择发送不同数据
    switch(send_data_select)
    {
        case 1: // 速度和PID数据
//            printf("%.2f,%.2f,%.2f,%.2f\n", left_speed_global, right_speed_global, base_speed, Deviation.PIDout);
            break;
        case 2: // 角度PID数据
            printf("%.2f,%.2f,%.2f,%.2f\n", Angle.PIDout, avl_gyro_x, avl_gyro_y, avl_gyro_z);     
            break;
        case 3: // ADC归一化结果
            printf("%.2f,%.2f,%.2f,%.2f\n", Adc_LEFT1_Value, Adc_LEFT2_Value, Adc_RIGHT1_Value, Adc_RIGHT2_Value);
            break;
        case 4: // 差值数据
            printf("%.2f,%.2f,%.2f,%.2f\n", imu_angle, imu_pitch, gyro_z_filtered, gyro_x_filtered);
            break;
        case 5: // 原始ADC值
            printf("%d,%d,%d,%d\n", Avg_Adc[0], Avg_Adc[1], Avg_Adc[2], Avg_Adc[3]);
            break;
        case 6: // element状态数据
            printf("%d\n", element);
            break;
        default:
            printf("Invalid Data Select\n");
            break;
    }
}

/*
 * 解析来自VOFA上位机的调参指令
 * 指令格式：PK=%.2f!  IK=%.2f!  DK=%.2f!  BS=%.2f!  EP=%.2f!  ED=%.2f!  AC=%.2f!  SA=%.2f!  SB=%.2f!  SC=%.2f!
 */
void get_PIDdata(void)
{
    static uint8 buf[64];
    uint32 len;
    int i;
    int startIdx;
    int endIdx;
    int copyLen;
    char valueStr[16];
    float PIDpara;
	
    memset(buf, 0, 64);
    len = wireless_uart_read_buffer(buf, 64);

    startIdx = -1;
    endIdx   = -1;

    /* 查找 '!' */
    for (i = 0; i < (int)len; i++)
    {
        if (buf[i] == '!')
        {
            endIdx = i;
            break;
        }
    }
    if (endIdx < 0)
    {
        return;
    }

    /* 查找 '=' */
    for (i = 0; i < endIdx; i++)
    {
        if (buf[i] == '=')
        {
            startIdx = i + 1;
            break;
        }
    }
    if (startIdx < 0 || endIdx <= startIdx)
    {
        return;
    }

    /* 提取数值字符串 */
    copyLen = endIdx - startIdx;
    if (copyLen >= 16)
    {
        return;
    }
    memset(valueStr, 0, 16);
    memcpy(valueStr, (char *)&buf[startIdx], copyLen);
    valueStr[copyLen] = '\0';
    PIDpara = (float)atof(valueStr);

    /* PK=%.2f! */
    if (buf[0] == 'P' && buf[1] == 'K')
    {
        if (PIDpara >= 0.0 && PIDpara <= 50.0)
        {
            Speed.KP = PIDpara;
            Speed_PID_Sync();
            printf("KP = %.3f\n", PIDpara);
        }
    }
    /* IK=%.2f! */
    else if (buf[0] == 'I' && buf[1] == 'K')
    {
        if (PIDpara >= 0.0 && PIDpara <= 10.0)
        {
            Speed.KI = PIDpara;
            Speed_PID_Sync();
            printf("KI = %.3f\n", PIDpara);
        }
    }
    /* DK=%.2f! */
    else if (buf[0] == 'D' && buf[1] == 'K')
    {
        if (PIDpara >= 0.0 && PIDpara <= 10.0)
        {
            Speed.KD = PIDpara;
            Speed_PID_Sync();
            printf("KD = %.3f\n", PIDpara);
        }
    }
    /* BS=%.2f! */
    else if (buf[0] == 'B' && buf[1] == 'S')
    {
        if (PIDpara >= 0.0 && PIDpara <= 500.0)
        {
            base_speed = PIDpara;
            printf("Base Speed = %.3f\n", PIDpara);
        }
    }
    /* EP=%.2f! */
    else if (buf[0] == 'E' && buf[1] == 'P')
    {
        if (PIDpara >= 0.0 && PIDpara <= 500.0)
        {
            Deviation.KP = PIDpara;
            printf("Deviation KP = %.3f\n", PIDpara);
        }
    }
    /* EI=%.2f! */
    else if (buf[0] == 'E' && buf[1] == 'I')
    {
        if (PIDpara >= 0.0 && PIDpara <= 50.0)
        {
            Deviation.KI = PIDpara;
            printf("Deviation KI = %.3f\n", PIDpara);
        }
    }
    /* ED=%.2f! */
    else if (buf[0] == 'E' && buf[1] == 'D')
    {
        if (PIDpara >= 0.0 && PIDpara <= 1500.0)
        {
            Deviation.KD = PIDpara;
            printf("Deviation KD = %.3f\n", PIDpara);
        }
    }
    /* AC=%.2f! 小车运动控制 */
    else if (buf[0] == 'A' && buf[1] == 'C')
    {
        if (PIDpara == 1.0)
        {
            car_running_flag = 1;
            printf("Car Start!\n");
        }
        else if (PIDpara == 0.0)
        {
            car_running_flag = 0;
            printf("Car Stop!\n");
        }
    }
    /* SA=%.2f! 差比和差算法系数A */
    else if (buf[0] == 'S' && buf[1] == 'A')
    {
        if (PIDpara >= 0.0 && PIDpara <= 20.0)
        {
            err_A = PIDpara;
            printf("Error A = %.3f\n", PIDpara);
        }
    }
    /* SB=%.2f! 差比和差算法系数B */
    else if (buf[0] == 'S' && buf[1] == 'B')
    {
        if (PIDpara >= 0.0 && PIDpara <= 12.0)
        {
            err_B = PIDpara;
            printf("Error B = %.3f\n", PIDpara);
        }
    }
    /* SC=%.2f! 差比和差算法系数C */
    else if (buf[0] == 'S' && buf[1] == 'C')
    {
        if (PIDpara >= 0.0 && PIDpara <= 12.0)
        {
            err_C = PIDpara;
            printf("Error C = %.3f\n", PIDpara);
        }
    }
		/* SP=%.2f! 差比和差算法系数C */
    else if (buf[0] == 'S' && buf[1] == 'P')
    {
        if (PIDpara >= 0.0 && PIDpara <= 5.0)
        {
            err_P = PIDpara;
            printf("Error C = %.3f\n", PIDpara);
        }
    }
    /* AP=%.2f! 角度环KP */
    /*else if (buf[0] == 'A' && buf[1] == 'P')
    {
        if (PIDpara >= 0.0 && PIDpara <= 100.0)
        {
            Angle.KP = PIDpara;
            printf("Angle KP = %.3f\n", PIDpara);
        }
    }*/
    /* AI=%.2f! 角度环KI */
    /*else if (buf[0] == 'A' && buf[1] == 'I')
    {
        if (PIDpara >= 0.0 && PIDpara <= 10.0)
        {
            Angle.KI = PIDpara;
            printf("Angle KI = %.3f\n", PIDpara);
        }
    }*/
		/* AD=%.2f! 角度环KD */
		/*else if (buf[0] == 'A' && buf[1] == 'D')
    {
        if (PIDpara >= 0.0 && PIDpara <= 100.0)
        {
            Angle.KD = PIDpara;
            printf("Angle KD = %.3f\n", PIDpara);
        }
    }*/
		/* KP2=%.2f! 非线性系数 */
		/*else if (buf[0] == 'K' && buf[1] == 'P')
    {
        if (PIDpara >= 0.0 && PIDpara <= 100.0)
        {
            KP2 = PIDpara;
            printf("KP2 = %.3f\n", PIDpara);
        }
    }*/
    /* SD=%.2f! 数据选择命令 */
    else if (buf[0] == 'S' && buf[1] == 'D')
    {
        if (PIDpara >= 1.0 && PIDpara <= 6.0)
        {
            send_data_select = (uint8)PIDpara;
            printf("Send Data Select = %d\n", PIDpara);
        }
    }
    /* SF=%.2f! 风扇PWM控制命令 */
    /*else if (buf[0] == 'S' && buf[1] == 'F')
    {
        if (PIDpara >= 0.0 && PIDpara <= 6000.0)
        {
            fan_pwm = (int)PIDpara;
            printf("Fan PWM = %d\n", PIDpara);
        }
    }*/
}

