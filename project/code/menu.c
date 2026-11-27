#include "system.h"

extern int Avg_Adc[4];
extern float k;
void menu_init(void)
{
    ips114_init();
    ips114_clear(0xFFFF);
}

void menu_show_adc_data(void)
{
    ips114_show_string(0, 0*16, "L:");
    ips114_show_float(30, 0*16, (float)Avg_Adc[0], 4, 1);

    ips114_show_string(0, 1*16, "LM:");
    ips114_show_float(30, 1*16, (float)Avg_Adc[1], 4, 1);
    
    ips114_show_string(0, 2*16, "RM:");
    ips114_show_float(30, 2*16, (float)Avg_Adc[2], 4, 1);
    
    ips114_show_string(0, 3*16, "R:");
    ips114_show_float(30, 3*16, (float)Avg_Adc[3], 4, 1);
	
	
	ips114_show_string(0, 4*16, "l :");
    ips114_show_float(30, 4*16, (float)l, 3, 1);
	
	ips114_show_string(0, 5*16, "r :");
    ips114_show_float(30, 5*16, (float)r, 3, 1);
	
	ips114_show_string(0, 6*16, "lm:");
    ips114_show_float(30, 6*16, (float)lm, 3, 1);
	
	ips114_show_string(0, 7*16, "rm:");
    ips114_show_float(30, 7*16, (float)rm, 3, 1);
	
	
	ips114_show_string(100, 0*16, "K:");
    ips114_show_float(130, 0*16, (float)k, 4, 1);
}

