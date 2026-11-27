#include "system.h"

#define PIT_CH                          (TIM1_PIT ) 

void main()
{
    uint16 loop_count = 0;  // 主循环计数器，用于降频
    
    clock_init(SYSTEM_CLOCK_30M);
    debug_init();
    system_init();
    
    // 配置5ms定时器中断
    pit_ms_init(PIT_CH, 5);

    // 初始化测试LED
    gpio_init(IO_P52, GPO, 1, GPO_PUSH_PULL);  
    
    // 初始化屏幕
    menu_init();

    while(1)
    {
		
//		Motor_Out(10000,10000);
		
        // 5ms任务：按键、陀螺仪、电机控制（encoder_update在ISR中已执行）
        if(flag_5ms)
        {
            flag_5ms = 0;
            task_5ms();
        }

        // 10ms任务：ADC采集、误差计算、偏差PID
        if(flag_10ms)
        {
            flag_10ms = 0;
            task_10ms();
        }

        // 串口通信：每次循环都检查（非阻塞读取）
//        get_PIDdata();

        // 串口输出和屏幕显示降频：每4次循环输出一次（约20ms一次）
        // 避免频繁printf和屏幕刷新延长主循环周期，影响task_5ms时序
        loop_count++;
        if(loop_count >= 4)
        {
            loop_count = 0;
            send_total_speed();
            menu_show_adc_data();
        }
    }
}
