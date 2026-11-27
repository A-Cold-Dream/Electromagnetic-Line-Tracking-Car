#include "system.h"

//---- 中断标志位定义 ----
volatile uint8 flag_5ms = 0;
volatile uint8 flag_10ms = 0;



//---- 原ISR中的静态变量，移到此处 ----
static unsigned int fan_pwm_ramp = 0;
static unsigned short timer_start_count = 0;  // 启动计数器
static int stop_count = 0;  // 停车检测计数器



// 外部变量声明
extern uint8 car_running_flag;
extern int16 speed_straight;  //赋初始值，防止随机数
extern float l, r, lm, rm;
extern int8 element;
extern int16 ringR_flag_task;
extern int16 ringL_flag_task;
extern float Err;

void system_init(void)
{
	send_init();
	init_PWM();
	encoder_init();
	init_Adc();
	PID_Init();
	imu660ra_init();  // 初始化陀螺仪
}

//---- 5ms任务：按键、陀螺仪、停车检测、电机控制 ----
// 注意：encoder_update()已移至ISR中执行，保证固定5ms读取间隔
void task_5ms(void)
{
	static uint8 car_running_last = 0;  // 上次的运行状态，用于检测启动沿
	
	Adc_Read();
	Adc_Count();
	Error_Count();
	

	key_process();       // 按键处理	
	gyro_get();          // 获取陀螺仪值

	// 检测启动瞬间（0→1上升沿）：立即赋初值，跳过ramp等待
	if(car_running_flag == 1 && car_running_last == 0)
	{
		speed_straight = 400;  // 立即设置目标速度（编码器计数值）
		fan_pwm_ramp = 100;    // 跳过风扇ramp（负压未启用时不需要等待）
		timer_start_count = 100;  // 跳过启动计数等待
		stop_count = 0;        // 清除停车检测计数
	}
	car_running_last = car_running_flag;

	// 停车检测：电磁值过小持续一定时间则停车（仅在运行中检测）
	if(car_running_flag == 1)
	{
		if(l + r + lm + rm < 30)
		{
			stop_count++;
			if(stop_count >= 70)  // 100*5ms = 500ms
			{
				car_running_flag = 0;
			}
		}
		else
		{
			stop_count = 0;
		}
	}

	// 电机控制
	if(car_running_flag == 1)
	{
		// 风扇ramp（仅在启用负压时需要，当前注释了Fan_Out，直接跳过）
		if(fan_pwm_ramp < 100)
		{
			fan_pwm_ramp += 1;  // 每5ms加1%，500ms到100%
		}
//		Fan_Out(700);  // 想启用负压就打开这个

		// 直接启动循迹
		xunji();
	}
	else
	{
		Fan_Out(500);
		Motor_Out(0, 0);													//初始默认20
		timer_start_count = 0;
		fan_pwm_ramp = 0;
		speed_straight = 0;				
	}
}

//---- 10ms任务：ADC采集、误差计算、偏差PID ----
void task_10ms(void)
{
	

	// 环岛入环/出环误差补偿
	if(element == 1 && ringR_flag_task == 1)      // 右环岛入环
	{
		Err = Err - 0.3;
	}
	if(element == 2 && ringL_flag_task == 1)      // 左环岛入环
	{
		Err = Err + 0.3;
	}
	if(element == 1 && ringR_flag_task == 4)      // 右环岛出环
	{
		Err = Err + 0.45;
	}
	if(element == 2 && ringL_flag_task == 4)      // 左环岛出环
	{
		Err = Err - 0.45;
	}

	Deviation_PID_Count();
}


