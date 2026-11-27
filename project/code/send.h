#ifndef _SEND_H
#define _SEND_H

#include "zf_common_typedef.h"

void send_init(void);
void send_total_speed(void);
void get_PIDdata(void);

extern uint8 car_running_flag;

#endif