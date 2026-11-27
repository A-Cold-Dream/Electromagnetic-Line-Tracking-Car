#ifndef _ACTION_H
#define _ACTION_H


// 全局元素类型标识：0=普通循迹，1=右环岛，2=左环岛
extern int8 element;
//右环岛
extern float distance_ringR;   	// 右环岛退出后需直行的距离累计值
extern int16 ringR_flag_task;		 // 右环岛任务状态（0:未触发，1:进入中,4:退出中）
extern int16 ringR_flag_execute;		// 右环岛执行状态（1:直行进入，4:直行退出）
    
//左环岛
extern float distance_ringL;
extern int16 ringL_flag_task;
extern int16 ringL_flag_execute;



void action(void);
void xunji(void);
void key_process(void);
void ringR_task(void);
void ringR_execute(void);
void ringL_task(void);
void ringL_execute(void);



#endif