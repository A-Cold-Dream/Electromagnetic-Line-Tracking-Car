#include "system.h"

//float null_drift_z;	//零漂
float gyro_x, gyro_y, gyro_z;
float imu_angle;
float imu_pitch;          // 俯仰角（检测前后翘起）
float avl_gyro_x, avl_gyro_y, avl_gyro_z;
float gyro_x_filtered, gyro_y_filtered, gyro_z_filtered;
float alpha = 0.9 ;

// 翘起检测标志和阈值
uint8 car_tilt_flag = 0;   // 0:正常, 1:翘起
float tilt_threshold = 30; // 翘起角度阈值（度）

void gyro_get()      //角速度
{
	imu660ra_get_gyro();    	//获取陀螺仪数据
	
	// 获取X轴角速度（用于计算俯仰角）
	gyro_x = imu660ra_gyro_x;
	gyro_x = imu660ra_gyro_transition(imu660ra_gyro_x);
	avl_gyro_x = gyro_x ;	
	gyro_x_filtered = alpha * gyro_x_filtered + (1 - alpha) * gyro_x;
	
	// 获取Y轴角速度（备用）
	gyro_y = imu660ra_gyro_y;
	gyro_y = imu660ra_gyro_transition(imu660ra_gyro_y);
	avl_gyro_y = gyro_y ;		
	gyro_y_filtered = alpha * gyro_y_filtered + (1 - alpha) * gyro_y;

	// 获取Z轴角速度（用于计算偏航角）
	gyro_z = imu660ra_gyro_z;
	gyro_z = imu660ra_gyro_transition(imu660ra_gyro_z);	
	avl_gyro_z = gyro_z ;
	gyro_z_filtered = alpha * gyro_z_filtered + (1 - alpha) * avl_gyro_z;
}

void angle_get()     //角度获取
{
	// 计算偏航角（Z轴）
	imu_angle += 0.005 * gyro_z_filtered;
	
	// 计算俯仰角（X轴）
	imu_pitch += 0.005 * gyro_x_filtered;
}

void angle_clear()   //角度清零
{
	imu_angle = 0;	
	imu_pitch = 0;   // 同时清零俯仰角
}

// 检测小车是否翘起
void tilt_detect()
{
	// 检测俯仰角是否超过阈值
	if (imu_pitch > tilt_threshold || imu_pitch < -tilt_threshold)
	{
		car_tilt_flag = 1;  // 翘起
	}
	else
	{
		car_tilt_flag = 0;  // 正常
	}
}
