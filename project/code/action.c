#include "system.h"

float abs_angle = 0;
int16 speed_straight = 0;//(直行速度)
int16 speed_ringR = 230;//(环岛速度)
int16 speed_ringL = 230;//(环岛速度)       

int8 element = 0;
int8 turn_flag = 0;
float distance_ringR = 0;    // 右环岛距离积分
float distance_ringL = 0;    // 左环岛距离积分

// 环岛状态标志
int16 ring_flag_ing = 0;      // 是否处于环岛任务的状态中，初始为0 
int16 ringR_flag_task = 0;    // 右环岛任务决策状态 (0:未遇到环岛, 1~5:环岛各阶段)
int16 ringR_flag_execute = 1; // 右环岛执行状态 (1:直线进岛, 4:返回原位)
int16 ringL_flag_task = 0;    // 左环岛任务决策状态 (0:未遇到环岛, 1~5:环岛各阶段)
int16 ringL_flag_execute = 1; // 左环岛执行状态 (1:直线进岛, 4:返回原位) 

// 外部声明
extern uint8 car_running_flag;
extern int Avg_Adc[4];

// 全局变量，风扇PWM输出值
int fan_pwm = 0;


/**
 * @brief 主动作调度函数
 * - 调用PID控制（被注释）
 * - 检测小车是否卡死（四路传感器值均很低）
 * - 根据 car_running_flag 控制电机与风扇
 */
void action(void)
{
    static int i = 0; // 卡死检测计数器

    // 原始代码中 l, r, lm, rm 应为传感器宏或变量，代表左右及中左右传感器值
    // 若四路传感器值总和过低（<30），认为小车可能脱线或卡死
    if (l + r + lm + rm < 30) {
        i++;
        if (i >= 70) { // 连续100次检测到脱线，停止小车
            car_running_flag = 0;
        }
    } else {
        i = 0; //条件不满足时重置计数器
    }

    // 根据运行标志控制输出
    if (car_running_flag == 1) {
//        Fan_Out(700); // 风扇全速
        xunji();      // 执行循迹逻辑
    } else {
        Fan_Out(500);           // 关闭风扇
        Motor_Out(0, 0);      // 停止电机
    }
}

/**
 * @brief 按键处理函数（带消抖）
 * - 使用状态机实现软件消抖
 * - 短按切换 car_running_flag（启动/停止小车）
 */
typedef enum {
    IDLE,      // 空闲状态
    DEBOUNCE,  // 消抖中
    PRESSED    // 已确认按下
} KEY_STATE;

static KEY_STATE key_state = IDLE;
static uint8 key_count = 0;

void key_process(void)
{
    uint8 key_val = gpio_get_level(IO_P70); // 读取按键IO（低电平有效）

    switch (key_state) {
        case IDLE:
            if (key_val == 0) key_state = DEBOUNCE; // 检测到下降沿
            break;

        case DEBOUNCE:
            if (key_val == 0) {
                if (++key_count >= 5) { // 连续5次低电平，确认按下
                    car_running_flag = !car_running_flag; // 切换运行状态
                    key_state = PRESSED;
                    key_count = 0;
                }
            } else {
                key_state = IDLE; // 消抖失败，返回空闲
                key_count = 0;
            }
            break;

        case PRESSED:
            if (key_val == 1 && ++key_count >= 5) {
                key_state = IDLE; // 按键释放后返回空闲
            }
            break;
    }
}

/**
 * @brief 主循迹逻辑 + 环岛识别入口
 * - 根据传感器和陀螺仪判断是否进入左右环岛
 * - 调用对应环岛处理函数
 */
void xunji(void)
{
    int L_PWM, R_PWM;

    // 判断是否进入右环岛（未在环岛处理中，左右传感器强信号，右中>左中，且无大角度倾斜）
    if (ring_flag_ing == 0 && r + l >= 190 && rm > lm && gyro_x_filtered < 10) {
        element = 1;                // 设置为右环岛模式
        ringR_flag_task = 1;        // 启动右环岛任务
        ring_flag_ing = 300;        // 设置防重入时间（约300ms）
        angle_clear();              // 清零IMU角度累计
    }
    // 判断是否进入左环岛
    else if (ring_flag_ing == 0 && r + l >= 190 && lm > rm && gyro_x_filtered < 10) {
        element = 2;                // 设置为左环岛模式
        ringL_flag_task = 1;        // 启动左环岛任务
        ring_flag_ing = 300;        // 防重入
        angle_clear();              // 清零IMU角度
    }
    // 若陀螺仪检测到大角度倾斜（如转弯完成），强制退出环岛模式
    else if (gyro_x_filtered > 15) {
        element = 0; // 回到普通循迹
    }

    // 根据当前元素类型执行不同逻辑
    switch (element) {
        case 0: // 普通循迹
            base_speed = speed_straight;
            Speed_PID_Count(); // 计算左右轮PID输出
            L_PWM = (int)(Speed_L_Out);
            R_PWM = (int)(Speed_R_Out);
            Motor_Out(L_PWM, R_PWM);

            if (ring_flag_ing > 0) {
                ring_flag_ing--; // 防重入计时递减
            }
            break;

        case 1: // 右环岛处理
            ringR_task();      // 更新任务状态
            ringR_execute();   // 执行电机控制
            break;

        case 2: // 左环岛处理
            ringL_task();      // 更新任务状态
            ringL_execute();   // 执行电机控制
            break;

        default:
            break;
    }
}

/**
 * @brief 右环岛任务状态机
 */
void ringR_task(void)
{
    switch (ringR_flag_task) {
        case 1: // 进入环岛阶段：等待IMU角度转过约280度（近一圈）
            angle_get(); // 更新IMU角度（imu_angle 应为全局变量）
            if (imu_angle > 280) { // 注意：此处假设角度为正向累加（顺时针）
                ringR_flag_task = 4;        // 切换到退出阶段
                ringR_flag_execute = 4;     // 执行直行退出
                distance_ringR = 0;         // 重置退出距离计数
                gpio_set_level(IO_P52, 0);  // 可选：点亮LED表示环岛完成
            }
            break;

        case 4: // 退出环岛阶段：直行一段距离确保完全驶出
            distance_ringR += speed_avl; // speed_avl 应为当前平均速度（单位需一致）
            if (distance_ringR > 20000) { // 阈值需根据实际单位校准（如mm或编码器脉冲）
                distance_ringR = 0;
                ringR_flag_task = 0;        // 任务结束
                ringR_flag_execute = 1;     // 恢复默认执行状态
                element = 0;                // 切回普通循迹
                gpio_set_level(IO_P52, 1);  // 关闭指示灯
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 右环岛电机执行
 */
void ringR_execute(void)
{
    int L_PWM, R_PWM;

    switch (ringR_flag_execute) {
        case 1: // 直行进入环岛（实际由循迹PID控制方向）
			base_speed = speed_ringR;        
            break;
        case 4: // 直行退出环岛
            base_speed = speed_ringR; // 设置环岛专用速度
            break;
        default:
            break;
    }

    // 只要处于环岛执行状态，就运行速度PID并输出电机
    if (ringR_flag_execute != 0) {
        Speed_PID_Count();
        L_PWM = (int)(Speed_L_Out);
        R_PWM = (int)(Speed_R_Out);
        Motor_Out(L_PWM, R_PWM);
    }
}

/**
 * @brief 左环岛任务状态机
 */
void ringL_task(void)
{
    switch (ringL_flag_task) {
        case 1: // 进入左环岛：等待IMU角度转过 -280 度（逆时针）
            angle_get();
            if (imu_angle < -280) {
                ringL_flag_task = 4;
                ringL_flag_execute = 4;
                distance_ringL = 0;
                gpio_set_level(IO_P52, 0); // 同一脚指示灯
            }
            break;

        case 4: // 退出左环岛：直行一段距离
            distance_ringL += speed_avl;
            if (distance_ringL > 20000) {
                distance_ringL = 0;
                ringL_flag_task = 0;
                ringL_flag_execute = 1;
                element = 0;
                gpio_set_level(IO_P52, 1);
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 左环岛电机执行
 */
void ringL_execute(void)
{
    int L_PWM, R_PWM;

    switch (ringL_flag_execute) {
        case 1:
			base_speed = speed_ringL;        
            break;
        case 4:
            base_speed = speed_ringL;
            break;
        default:
            break;
    }

    if (ringL_flag_execute != 0) {
		// 只有在环岛状态时，才执行环岛的速度分配
        Speed_PID_Count();
        L_PWM = (int)(Speed_L_Out);
        R_PWM = (int)(Speed_R_Out);
        Motor_Out(L_PWM, R_PWM);
    }
}

