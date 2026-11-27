#include "system.h"

/*外环应该使用位置式PID（与期望的误差），内环使用增量式PID（应该如何改进）*/
PID Speed={0};  							// 速度环参数配置（统一调参用）
PID Speed_L={0};  						// 左轮速度环，控制左轮速度              
PID Speed_R={0};  						// 右轮速度环，控制右轮速度
PID Deviation={0};            // 偏差环，计算转向差值，生成转向需求
PID Angle={0};  							// 角度环，执行偏差环的转向需求
float Speed_L_Out=0,Speed_R_Out=0; 	  // 速度PID输出
float KP2=0;
float base_speed; // 基础速度(BS80   22.5 1.4 0  BS100  13 1 0)
float k=0; // 输出系数
/*float KD2=0;（非线性控制系数） float Difference_Speed=0;（差速变量）这两个未使用*/


// PID初始化
void PID_Init(void)
{
	// 速度环pid初始化（统一配置参数）
	Speed.KP=5.00;//1.75
	Speed.KI=0.35;//0.35
	Speed.KD=0;
	
	// 输出限幅：-100~100（百分比，对应Motor_Out参数范围）
	Speed.PIDmax=5000;
	Speed.PIDmin=-5000;
	
	// 将统一参数直接复制给左右轮（结构体直接赋值）
	Speed_L = Speed;
	Speed_R = Speed;
	
	// 偏差环pid初始化
	Deviation.KP=150;//100	转弯灵敏度
	Deviation.KI=0;
	Deviation.KD=800;//600
	Deviation.PIDmax=100;
	Deviation.PIDmin=-100;
	
	// 角度环pid初始化
	Angle.KP=10;
	Angle.KI=0;
	Angle.KD=0;
	Angle.PIDmax=2000;
	Angle.PIDmin=-2000;
	
}
// 速度环参数同步函数（将Speed参数同步到左右轮）
void Speed_PID_Sync(void)
{
	Speed_L = Speed;
	Speed_R = Speed;
}

// 增量式和位置式pid不用修改，往后调用即可
// 增量式PID（计算相比上次应该增加/减少多少，需要累加得到最终输出）
float PID_Control_Inc(PID* pid) 
{
		// inc是pid最后的输出量
    float inc;
    pid->ek = pid->SetValue - pid->ActValue;   
    
    inc = pid->KP * (pid->ek - pid->ek_1)             
        + pid->KI * pid->ek                         
        + pid->KD * (pid->ek - 2*pid->ek_1 + pid->ek_2);  
    pid->ek_2 = pid->ek_1;  
    pid->ek_1 = pid->ek;
    return inc;  // 返回增量（变化量）
}
// 位置式PID（直接计算当前应该输出多少，包含误差累积（积分项））
float PID_Control_Pos(PID* pid)  
{
		// Pos同理inc
    float Pos;
    pid->ek = pid->SetValue - pid->ActValue;  					  // 计算当前误差
    pid->ek_sum += pid->ek;  												 			// 累加误差
    Pos = pid->KP * pid->ek             							    // 比例项：当前误差
        + pid->KI * pid->ek_sum        			 						  // 积分项：误差累积
        + pid->KD * (pid->ek - pid->ek_1);  						  // 微分项：误差变化
    pid->ek_1 = pid->ek;  														    // 更新历史误差
    return Pos;  // 返回绝对值
}
//偏差PID计算
void Deviation_PID_Count()  
{
   Deviation.ActValue = Err;															//调试模式时要注释该语句，会模拟转向
   Deviation.SetValue =  0;		                            //期望偏差为0
	 //偏差pid输出
   Deviation.PIDout= PID_Control_Pos(&Deviation);
	 //KP2*fabs(-Error)*(-Error) + PID_Control_Pos(&Deviation)
//    -0.15*imu660ra_gyro_z/16.4 //位置式PID增加陀螺仪中的Z轴角速度，抑制转向
    ;
	 //输出限幅
	 if(Deviation.PIDout > Deviation.PIDmax)
	 {
		 Deviation.PIDout =Deviation.PIDmax;
	 }
	 if(Deviation.PIDout <Deviation.PIDmin)
	 {
	   Deviation.PIDout =Deviation.PIDmin;
	 }
   k = Deviation.PIDout * 0.01;
   Adc_Limit(&k, -0.65, 0.65);
}
//速度pid - 左右轮分别控制
void Speed_PID_Count()
{
    float delta_pid_L, delta_pid_R;
    
    // 根据k值设置左右轮速度
    if(k >= 0)
    {
        // 右转：右轮减速，左轮稍微加速
        Speed_L.SetValue = base_speed * (1 + k * 0.4);
        Speed_R.SetValue = base_speed * (1 - k);
    }
    else
    {
        // 左转：左轮减速，右轮稍微加速
        Speed_L.SetValue = base_speed * (1 + k);
        Speed_R.SetValue = base_speed * (1 - k * 0.4);
    }
    
    // 左轮速度PID
    Speed_L.ActValue = left_speed_global; // 左轮实际速度
    delta_pid_L = PID_Control_Inc(&Speed_L);
    Speed_L_Out += delta_pid_L;
    
    // 左轮输出限幅
    if(Speed_L_Out < Speed_L.PIDmin)
    {
        Speed_L_Out = Speed_L.PIDmin;
    }
    if(Speed_L_Out > Speed_L.PIDmax)
    {
        Speed_L_Out = Speed_L.PIDmax;
    }
    
    // 右轮速度PID
    Speed_R.ActValue = right_speed_global; // 右轮实际速度
    delta_pid_R = PID_Control_Inc(&Speed_R);
    Speed_R_Out += delta_pid_R;
    
    // 右轮输出限幅
    if(Speed_R_Out < Speed_R.PIDmin)
    {
        Speed_R_Out = Speed_R.PIDmin;
    }
    if(Speed_R_Out > Speed_R.PIDmax)
    {
        Speed_R_Out = Speed_R.PIDmax;
    }
	
	printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",Speed_L_Out,Speed_R_Out,left_speed_global,right_speed_global,Speed_L.SetValue,Speed_R.SetValue,PID_Control_Pos(&Deviation));
}


// 角速度PID计算（位置式）
/*void Angle_PID_Count()  // 角速度-PWM
{
   
   Angle.SetValue = 0;  // 偏差环输出作为角度环的目标
   Angle.ActValue = 0;  // 实际角速度
   
   // 调用位置式PID函数
   Angle.PIDout = PID_Control_Pos(&Angle);
   
   // 输出限幅
   if(Angle.PIDout > Angle.PIDmax)
   {
      Angle.PIDout = Angle.PIDmax;
   }
   if(Angle.PIDout < Angle.PIDmin)
   {
      Angle.PIDout = Angle.PIDmin;
   }
}*/