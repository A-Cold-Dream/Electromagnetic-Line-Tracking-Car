#include "system.h"



void init_PWM(void)
{
    //-----MOS模式-----------
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认高电平
    pwm_init(PWM_L, 17000, 0);                                                  // PWM 通道初始化 频率17KHz 占空比初始为0

    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认高电平
    pwm_init(PWM_R, 17000, 0);                                                  // PWM 通道初始化 频率17KHz 占空比初始为0

    pwm_init(PWM_1, 50, 500);                                                  // PWM 通道初始化 频率50Hz 占空比初始为0
}

// 电机输出函数
// 参数范围：-100~100（百分比）
// -100=全速反转，0=停止，100=全速正转
// PWM_DUTY_MAX=10000，百分比*100 = 实际PWM值
void Motor_Out(int left_duty, int right_duty)
{
    // 左侧限幅
    if(left_duty > duty_max) left_duty = duty_max;
    if(left_duty < duty_min) left_duty = duty_min;
		  
    // 右侧限幅
    if(right_duty > duty_max) right_duty = duty_max;
    if(right_duty < duty_min) right_duty = duty_min;

    // 左侧占空比设置
    if(left_duty > 0)           // 正转
    {
        gpio_set_level(DIR_L, GPIO_HIGH);
				pwm_set_duty(PWM_L, left_duty);  //输入left_duty:50,pwm输出5000,最大速度10000
    }
    else                        // 反转或停止
    {
        gpio_set_level(DIR_L, GPIO_LOW);
        pwm_set_duty(PWM_L, (-left_duty));
    }

    // 右侧占空比设置
    if(right_duty > 0)          // 正转
    {
        gpio_set_level(DIR_R, GPIO_HIGH);
        pwm_set_duty(PWM_R, right_duty);
    }
    else                        // 反转或停止
    {
        gpio_set_level(DIR_R, GPIO_LOW);
        pwm_set_duty(PWM_R, (-right_duty));
    }
}

// 风扇输出函数
// 参数范围：0~100（百分比），0=停止，100=全速
void Fan_Out(int fan_duty)
{
    if(fan_duty < 0) fan_duty = 0;
    if(fan_duty > fan_max) fan_duty = fan_max;
    pwm_set_duty(PWM_1, fan_duty);
}


