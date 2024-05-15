#ifndef __BSP_PID_H
#define	__BSP_PID_H
#include "stm32f1xx.h"
#include "bsp_debug_usart.h"
#include "bsp_motor_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>  // 包含stdbool.h头文件以使用bool类型

#define A 400
#define B 100

typedef struct
{
    float target_val;     //目标值
    float actual_val;     //实际值
    float err;            //定义当前偏差值
    float err_next;       //定义下一个偏差值
    float err_last;       //定义最后一个偏差值
    float Kp, Ki, Kd;     //定义比例、积分、微分系数
    float integral;          		//定义积分值
    float integral_max;
    float integral_min;
    _Bool pid_enable;
}_pid;

extern _pid pid3;
extern _pid pid4;

// 在文件的开头或全局范围内定义全局变量
extern _Bool Pflag4;   // Pflag是布尔型变量
extern _Bool Iflag4;   // Iflag是布尔型变量
extern _Bool Iflagz4;  // Iflagz是布尔型变量

extern _Bool Pflag3;   // Pflag是布尔型变量
extern _Bool Iflag3;   // Iflag是布尔型变量
extern _Bool Iflagz3;  // Iflagz是布尔型变量

extern _Bool pid3_enable;
extern _Bool pid4_enable;

extern void PID_param_init(void);
extern void set_pid_target3(_pid *pid, float temp_val);
extern void set_pid_target4(_pid *pid, float temp_val);
extern float get_pid_target(_pid *pid);
extern void set_p_i_d(_pid *pid, float p, float i, float d);
extern float PID_realize(_pid *pid, float actual_val, _Bool Pflag, _Bool Iflag, _Bool Iflagz);
extern void updateFlagsForPID(_pid *pid, _Bool Pflag, _Bool Iflag, _Bool Iflagz, float threshold1, float threshold2);

#endif
