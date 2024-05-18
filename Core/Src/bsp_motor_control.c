#include "bsp_motor_control.h"
#include "bsp_debug_usart.h"
#include "bsp_adc.h"
#include "bsp_led.h"
//#include "bsp_pid.h"
#include "bsp_basic_tim.h"
#include "protocol.h"
#include "bsp_tim.h"
#include <math.h>
#include <stdlib.h>
#include "bsp_hc05.h"
#include "bsp_usart_blt.h"
#include <string.h>



motor_dir_t MOTOR1_direction  = MOTOR_REV;       // 记录方向
motor_dir_t MOTOR2_direction  = MOTOR_FWD;       // 记录方向
motor_dir_t MOTOR3_direction  = MOTOR_REV;       // 记录方向
motor_dir_t MOTOR4_direction  = MOTOR_FWD;       // 记录方向
motor_dir_t MOTOR5_direction  = MOTOR_FWD;       // 记录方向

uint16_t    motor1_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机1占空比
uint16_t    motor2_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机2占空比
uint16_t    motor3_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机3占空比
uint16_t    motor4_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机4占空比
uint16_t    motor5_dutyfactor = T1_PEM_motor1_dutyfactor;            // 记录电机5占空比

uint8_t     is_motor1_en = 0;            			// 电机1使能
uint8_t     is_motor2_en = 0;            			// 电机2使能
uint8_t     is_motor3_en = 0;            			// 电机3使能
uint8_t     is_motor4_en = 0;            			// 电机4使能
uint8_t     is_motor5_en = 0;            			// 电机5使能

unsigned int Task_Delay[NumOfTask];
char linebuff[1024];

uint16_t randomcnt1 = 1;
uint16_t randomcnt2 = 1;
uint16_t randomcnt3 = 1;
uint16_t randomcnt5 = 1;

uint16_t sensor_triggered = 0;

typedef struct {
    float horizontal;//仰角pid4
    float vertical;//水平摆角pid3
    float M1speed;
    float M2speed;
} Position;

Position all_positions[25] =
        {
                {1100,2500,2000,2000},{1100,2250,2000,2000}, {1100,2100,2000,2000},{1100,2100,2000,2000},{1100,1920,2000,2000},//A1-E1

                {1100,1900,2000,2000},{1100, 1900,2000,2000},{1100,1750,2000,2000},{1100,1780,2000,2000},{1100,1720,2000,2000},//A2-E2

                {1100,1738,2000,2000},{1100, 1591,2000,2000},{1100, 1580,2000,2000},{1100, 1530,2000,2000}, {1100, 1560,2000,2000}, //A3-E3

                {1100, 1290,2000,2000}, {1100,745,2000,2000}, {1100, 1200,2000,2000}, {1100, 1300,2000,2000}, {1100, 1230,2000,2000},//A4-E4

                {1100,800,2000,2000},{1100,789,2000,2000},{1100,1000,2000,2000},{1100,1200,2000,2000},{1100, 800,2000,2000}//A5-E5

        };

Position left_positions[15] =
        {
                {1100,2500,2000,2000},{1100,2250,2000,2000}, {1100,2100,2000,2000},{1100,2100,2000,2000},{1100,1920,2000,2000},//A1-E1

                {1100,1900,2000,2000},{1100, 1900,2000,2000},{1100,1750,2000,2000},{1100,1780,2000,2000},{1100,1720,2000,2000},//A2-E2

                {1100,1738,2000,2000},{1100, 1591,2000,2000},{1100, 1580,2000,2000},{1100, 1530,2000,2000}, {1100, 1560,2000,2000}, //A3-E3
        };

Position right_positions[15] =
        {
                {1100,1738,2000,2000},{1100, 1591,2000,2000},{1100, 1580,2000,2000},{1100, 1530,2000,2000}, {1100, 1560,2000,2000}, //A3-E3

                {1100, 1290,2000,2000}, {1100,745,2000,2000}, {1100, 1200,2000,2000}, {1100, 1300,2000,2000}, {1100, 1230,2000,2000},//A4-E4

                {1100,800,2000,2000},{1100,789,2000,2000},{1100,1000,2000,2000},{1100,1200,2000,2000},{1100, 800,2000,2000}//A5-E5
        };

// 函数来生成指定范围内的随机数
int generate_random_all_position(void) {
    return rand() % 25; // 返回0到24之间的随机数
}

int generate_random_left_position(void) {
    return rand() % 15; // 返回0到24之间的随机数
}

int generate_random_right_position(void) {
    return rand() % 15; // 返回0到24之间的随机数
}

void wakeup_motor(void)
{
    MOTOR_ENABLE_nSLEEP();
}

void sleep_motor(void)
{
    MOTOR_DISABLE_nSLEEP();
}

/**
  * @brief  设置电机1速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor1_speed(uint16_t v)
{
    motor1_dutyfactor = v;

    if (MOTOR1_direction == MOTOR_FWD)
    {
        MOTOR1_SET_FWD_COMPAER(motor1_dutyfactor);     // 设置速度
        MOTOR1_REV_DISABLE();
        MOTOR1_FWD_ENABLE();
    }
    else
    {
        MOTOR1_SET_REV_COMPAER(motor1_dutyfactor);     // 设置速度
        MOTOR1_FWD_DISABLE();
        MOTOR1_REV_ENABLE();
    }
}

/**
  * @brief  设置电机1方向
  * @param  无
  * @retval 无
  */
void set_motor1_direction(motor_dir_t dir)
{
    MOTOR1_direction = dir;

    if (MOTOR1_direction == MOTOR_FWD)
    {
        MOTOR1_SET_FWD_COMPAER(motor1_dutyfactor);      // 设置正向速度
        MOTOR1_REV_DISABLE();                           // 设置反向速度
        MOTOR1_FWD_ENABLE();
    }
    else
    {
        MOTOR1_FWD_DISABLE();                           // 设置正向速度
        MOTOR1_SET_REV_COMPAER(motor1_dutyfactor);      // 设置反向速度
        MOTOR1_REV_ENABLE();
    }
}

/**
  * @brief  使能电机1
  * @param  方向
  * @retval 无
  */
void set_motor1_enable(void)
{
    is_motor1_en = 1;
    MOTOR1_FWD_ENABLE();
    MOTOR1_REV_ENABLE();
}

/**
  * @brief  禁用电机1
  * @param  无
  * @retval 无
  */
void set_motor1_disable(void)
{
    is_motor1_en = 0;
    MOTOR1_FWD_DISABLE();
    MOTOR1_REV_DISABLE();
}

/**
  * @brief  设置电机2速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor2_speed(uint16_t v)
{
    motor2_dutyfactor = v;

    if (MOTOR2_direction == MOTOR_FWD)
    {
        MOTOR2_SET_FWD_COMPAER(motor2_dutyfactor);     // 设置速度
        MOTOR2_REV_DISABLE();
        MOTOR2_FWD_ENABLE();
    }
    else
    {
        MOTOR2_SET_REV_COMPAER(motor2_dutyfactor);     // 设置速度
        MOTOR2_FWD_DISABLE();
        MOTOR2_REV_ENABLE();
    }
}

/**
  * @brief  设置电机2方向
  * @param  无
  * @retval 无
  */
void set_motor2_direction(motor_dir_t dir)
{
    MOTOR2_direction = dir;

    if (MOTOR2_direction == MOTOR_FWD)
    {
        MOTOR2_SET_FWD_COMPAER(motor2_dutyfactor);      // 设置正向速度
        MOTOR2_REV_DISABLE();
        MOTOR2_FWD_ENABLE();
    }
    else
    {
        MOTOR2_FWD_DISABLE();                           // 设置正向速度
        MOTOR2_SET_REV_COMPAER(motor2_dutyfactor);      // 设置反向速度
        MOTOR2_REV_ENABLE();
    }
}

/**
  * @brief  使能电机2
  * @param  无
  * @retval 无
  */
void set_motor2_enable(void)
{
    is_motor2_en = 1;
    MOTOR2_FWD_ENABLE();
    MOTOR2_REV_ENABLE();
}

/**
  * @brief  禁用电机2
  * @param  无
  * @retval 无
  */
void set_motor2_disable(void)
{
    is_motor2_en = 0;
    MOTOR2_FWD_DISABLE();
    MOTOR2_REV_DISABLE();
}

/**
  * @brief  设置电机3速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor3_speed(uint16_t v)
{
    motor3_dutyfactor = v;

    if (MOTOR3_direction == MOTOR_FWD)
    {
        MOTOR3_SET_FWD_COMPAER(motor3_dutyfactor);     // 设置速度
        MOTOR3_REV_DISABLE();
        MOTOR3_FWD_ENABLE();
    }
    else
    {
        MOTOR3_SET_REV_COMPAER(motor3_dutyfactor);     // 设置速度
        MOTOR3_FWD_DISABLE();
        MOTOR3_REV_ENABLE();
    }
}

/**
  * @brief  设置电机3方向
  * @param  无
  * @retval 无
  */
void set_motor3_direction(motor_dir_t dir)
{
    MOTOR3_direction = dir;

    if (MOTOR3_direction == MOTOR_FWD)
    {
        MOTOR3_SET_FWD_COMPAER(motor3_dutyfactor);      // 设置正向速度
        MOTOR3_REV_DISABLE();
        MOTOR3_FWD_ENABLE();
    }
    else
    {
        MOTOR3_FWD_DISABLE();                           // 设置正向速度
        MOTOR3_SET_REV_COMPAER(motor3_dutyfactor);      // 设置反向速度
        MOTOR3_REV_ENABLE();
    }
}

/**
  * @brief  使能电机3
  * @param  无
  * @retval 无
  */
void set_motor3_enable(void)
{
    is_motor3_en = 1;
    MOTOR3_FWD_ENABLE();
    MOTOR3_REV_ENABLE();
}

/**
  * @brief  禁用电机3
  * @param  无
  * @retval 无
  */
void set_motor3_disable(void)
{
    is_motor3_en = 0;
    MOTOR3_FWD_DISABLE();
    MOTOR3_REV_DISABLE();
}

/**
  * @brief  设置电机4速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor4_speed(uint16_t v)
{
    motor4_dutyfactor = v;

    if (MOTOR4_direction == MOTOR_FWD)
    {
        MOTOR4_SET_FWD_COMPAER(motor4_dutyfactor);     // 设置速度
        MOTOR4_REV_DISABLE();
        MOTOR4_FWD_ENABLE();
    }
    else
    {
        MOTOR4_SET_REV_COMPAER(motor4_dutyfactor);     // 设置速度
        MOTOR4_FWD_DISABLE();
        MOTOR4_REV_ENABLE();
    }
}

/**
  * @brief  设置电机4方向
  * @param  无
  * @retval 无
  */
void set_motor4_direction(motor_dir_t dir)
{
    MOTOR4_direction = dir;

    if (MOTOR4_direction == MOTOR_FWD)
    {
        MOTOR4_SET_FWD_COMPAER(motor4_dutyfactor);      // 设置正向速度
        MOTOR4_REV_DISABLE();
    }
    else
    {
        MOTOR4_FWD_DISABLE();                           // 设置正向速度
        MOTOR4_SET_REV_COMPAER(motor4_dutyfactor);      // 设置反向速度
    }
}

/**
  * @brief  使能电机4
  * @param  无
  * @retval 无
  */
void set_motor4_enable(void)
{
    is_motor4_en = 1;
    MOTOR4_FWD_ENABLE();
    MOTOR4_REV_ENABLE();
}

/**
  * @brief  禁用电机4
  * @param  无
  * @retval 无
  */
void set_motor4_disable(void)
{
    is_motor4_en = 0;
    MOTOR4_FWD_DISABLE();
    MOTOR4_REV_DISABLE();
}

/**
  * @brief  设置电机5速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_motor5_speed(uint16_t v)
{
    motor5_dutyfactor = v;

    if (MOTOR5_direction == MOTOR_FWD)
    {
        MOTOR5_SET_FWD_COMPAER(motor5_dutyfactor);     // 设置速度
        MOTOR5_REV_DISABLE();
        MOTOR5_FWD_ENABLE();
    }
    else
    {
        MOTOR5_SET_REV_COMPAER(motor5_dutyfactor);     // 设置速度
        MOTOR5_FWD_DISABLE();
        MOTOR5_REV_ENABLE();
    }
}

/**
  * @brief  设置电机5方向
  * @param  无
  * @retval 无
  */
void set_motor5_direction(motor_dir_t dir)
{
    MOTOR5_direction = dir;

    if (MOTOR5_direction == MOTOR_FWD)
    {
        MOTOR5_SET_FWD_COMPAER(motor5_dutyfactor);      // 设置正向速度
        MOTOR5_REV_DISABLE();
        MOTOR5_FWD_ENABLE();
    }
    else
    {
        MOTOR5_FWD_DISABLE();                           // 设置正向速度
        MOTOR5_SET_REV_COMPAER(motor5_dutyfactor);      // 设置反向速度
        MOTOR5_REV_ENABLE();
    }
}

/**
  * @brief  使能电机5
  * @param  无
  * @retval 无
  */
void set_motor5_enable(void)
{
    is_motor5_en = 1;
    MOTOR5_FWD_ENABLE();
    MOTOR5_REV_ENABLE();
}

/**
  * @brief  禁用电机5
  * @param  无
  * @retval 无
  */
void set_motor5_disable(void)
{
    is_motor5_en = 0;
    MOTOR5_FWD_DISABLE();
    MOTOR5_REV_DISABLE();
}

/**
  * @brief  下电机3增量式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
void motor3_pid_control(void)
{
    if (is_motor3_en == 1)    			 																										 										// 电机在使能状态下才进行控制处理
    {
        float cont_val = 0;    //存储PID计算的控制值
        int temp_val = 0;          //存储处理后的控制值   																										 										// 当前控制值

        cont_val = PID_realize(&pid3, positiondown_adc_mean, Pflag3, Iflag3, Iflagz3);    																 	 						// 将Pid3的值传递给函数进行 PID 计算

        if (cont_val > 0)   	 																														 										// 判断电机方向
        {
            set_motor3_direction(MOTOR_FWD);
        }
        else
        {
            set_motor3_direction(MOTOR_REV);
        }
        temp_val = (fabs(cont_val) > PWM_MAX_PERIOD_COUNT*0.9) ? PWM_MAX_PERIOD_COUNT*0.9 : fabs(cont_val);    // 速度上限处理    判断绝对值是否大于5500的80%，大于则是5500的80%，小于则是绝对值
        set_motor3_speed(temp_val);                                                                     			 // 设置 PWM 占空比
    }
}

/**
  * @brief  上电机4增量式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
void motor4_pid_control(void)
{
    if (is_motor4_en == 1)    			 																										 										// 电机在使能状态下才进行控制处理
    {
        float cont_val = 0;
        int temp_val = 0;             																										 										// 当前控制值
        if(positionup_adc_mean < 659)
        {
            positionup_adc_mean = 659;
        }
        if(positionup_adc_mean > 2124)
        {
            positionup_adc_mean = 2124;
        }
        cont_val = PID_realize(&pid4, positionup_adc_mean, Pflag4, Iflag4, Iflagz4);    																 	 							// 进行 PID 计算

        int32_t err = pid4.err;
        int32_t err_last = pid4.err_last;
        int32_t err_next = pid4.err_next;

        if (cont_val > 0)   	 																														 										// 判断电机方向
        {
            set_motor4_direction(MOTOR_REV);
        }
        else
        {
            set_motor4_direction(MOTOR_FWD);
        }
        temp_val = (fabs(cont_val) > PWM_MAX_PERIOD_COUNT*0.9) ? PWM_MAX_PERIOD_COUNT*0.9 : fabs(cont_val);    // 速度上限处理
        set_motor4_speed(temp_val);                                                                     			 // 设置 PWM 占空比
    }
}

int  currentSelectPosition = -1;
int  currentSelectrepeat_count = -1;

int  currentSelectmotor5_speed = 3000;
int  motor5_speed = 3000;

void BLE_control(void)
{
    char* redata;       //定义读数据的指针
    uint16_t len;       //定义数据大小
    if (Task_Delay[0] == 0 && IS_BLE_CONNECTED())      //定时并且判断INT引脚电平是否发生变化
    {
        BLE_WAKEUP_LOW;        //蓝牙wakeup引脚置0，启动蓝牙
        uint16_t linelen;     //定义数据的长度
        /*获取数据*/
        redata = get_rebuff(&len);
        linelen = get_line(linebuff, redata, len);
        /*检查数据是否有更新*/
        if (linelen < 200 && linelen != 0)
        {
            char first_char;
            char second_char[5] = {0};

            first_char = redata[0];
            second_char[0] = redata[1];
            second_char[1] = redata[2];
            second_char[2] = redata[3];
            second_char[3] = redata[4];
            second_char[4] = '\0';  // 添加字符串结尾

            switch (first_char)   //主模式区分
            {
                case '1':        //定点模式
                {
                    int position = Fixed_chose(second_char);     //判断点位
                    if (position != -1)
                    {
                        currentSelectPosition = position;
                        Fixed_flag = 1;                        //作为主程序中触发Fixed_control的标志位
                        Fixedcnt = 1;                          //作为Fixed_control首次触发信号
                    }
                    else
                    {

                    }
                    break;
                }
                case '2':
                    if(strcmp(second_char, "00aa") == 0)
                    {
                        clean_rebuff();
                        if(all_random_flag == 1)
                        {
                            all_random_flag = 0;
//                            set_motor5_disable();
//                            LED5_TOGGLE
                            LED5_TOGGLE
                            __set_FAULTMASK(1);
                            NVIC_SystemReset();
                        }
                        else
                        {
                            all_random_flag = 1;
                            randomcnt1 = 1;
                            LED4_TOGGLE
                        }
                    }
                    if(strcmp(second_char, "bb") == 0)
                    {
                        clean_rebuff();
                        if(left_random_flag == 1)
                        {
                            left_random_flag = 0;
//                            set_motor5_disable();
//                            LED4_TOGGLE
                            LED5_TOGGLE
                            __set_FAULTMASK(1);
                            NVIC_SystemReset();
                        }
                        else
                        {
                            left_random_flag = 1;
                            randomcnt2 = 1;
                            LED4_TOGGLE
                        }
                    }
                    if(strcmp(second_char, "cc") == 0)
                    {
                        clean_rebuff();
                        if(right_random_flag == 1)
                        {
                            right_random_flag = 0;
//                            set_motor5_disable();
//                            LED4_TOGGLE
                            LED5_TOGGLE
                            __set_FAULTMASK(1);
                            NVIC_SystemReset();
                        }
                        else
                        {
                            right_random_flag = 1;
                            randomcnt3 = 1;
                            LED4_TOGGLE
                        }
                    }
                    break;
                case 'A':
                {
                    int repeat_count = repeat_chose(second_char);   //比较second_char,返回对应的重复次数
                    if (repeat_count != -1)
                    {
                        currentSelectrepeat_count = repeat_count;
                        repeat_flag = 1;
                    }
                    else
                    {

                    }
                    break;
                }
                case 'B':
                {
//                    int speed_count = freq_chose(second_char);   //比较second_char,返回对应的重复次数
//                    if (speed_count != -1)
//                    {
                        currentSelectmotor5_speed = freq_chose(second_char);    //比较second_char,返回对应的M5速度
                        freq_flag = 1;
//                    }
//                    else
//                    {
//
//                    }
                    break;
                }
                default:

                    break;
            }
            /*处理数据后，清空接收蓝牙模块数据的缓冲区*/
            clean_rebuff();
            Task_Delay[0] = 200;                            //此值每1ms会减1，减到0才可以重新进来这里，所以执行的周期是200ms；
        }                                                   //增加 Task_Delay 的延时时间，以减少每次检查的频率，从而避免由于频繁检查导致的数据丢失
        BLE_WAKEUP_HIGHT;
    }
}

int Fixed_chose(char *second_char)       //根据second_char的判断，返回对应的值，作为数组的信号，确定对应的点位
{
    if (strcmp(second_char, "00A1") == 0)
    {
        return 0;
    }
    else if (strcmp(second_char, "00A2") == 0)
    {
        return 5;
    }
    else if (strcmp(second_char, "00A3") == 0)
    {
        return 10;
    }
    else if (strcmp(second_char, "00A4") == 0)
    {
        return 15;
    }
    else if (strcmp(second_char, "00A5") == 0)
    {
        return 20;
    }
    else if (strcmp(second_char, "11B1") == 0)
    {
        return 1;
    }
    else if (strcmp(second_char, "11B2") == 0)
    {
        return 6;
    }
    else if (strcmp(second_char, "11B3") == 0)
    {
        return 11;
    }
    else if (strcmp(second_char, "11B4") == 0)
    {
        return 16;
    }
    else if (strcmp(second_char, "11B5") == 0)
    {
        return 21;
    }
    else if (strcmp(second_char, "11C1") == 0)
    {
        return 2;
    }
    else if (strcmp(second_char, "11C2") == 0)
    {
        return 7;
    }
    else if (strcmp(second_char, "11C3") == 0)
    {
        return 12;
    }
    else if (strcmp(second_char, "11C4") == 0)
    {
        return 17;
    }
    else if (strcmp(second_char, "11C5") == 0)
    {
        return 22;
    }
    else if (strcmp(second_char, "11D1") == 0)
    {
        return 3;
    }
    else if (strcmp(second_char, "11D2") == 0)
    {
        return 8;
    }
    else if (strcmp(second_char, "11D3") == 0)
    {
        return 13;
    }
    else if (strcmp(second_char, "11D4") == 0)
    {
        return 18;
    }
    else if (strcmp(second_char, "11D5") == 0)
    {
        return 23;
    }
    else if (strcmp(second_char, "11E1") == 0)
    {
        return 4;
    }
    else if (strcmp(second_char, "11E2") == 0)
    {
        return 9;
    }
    else if (strcmp(second_char, "11E3") == 0)
    {
        return 14;
    }
    else if (strcmp(second_char, "11E4") == 0)
    {
        return 19;
    }
    else if (strcmp(second_char, "11E5") == 0)
    {
        return 24;
    }
    else
    {
        return  -1;
    }
}

int repeat_chose(char *second_char)
{
    if (strcmp(second_char, "1111") == 0)
    {
        return 1;
    }
    else if (strcmp(second_char, "2222") == 0)
    {
        return 2;
    }
    else if (strcmp(second_char, "3333") == 0)
    {
        return 3;
    }
    else if (strcmp(second_char, "4444") == 0)
    {
        return 4;
    }
    else if (strcmp(second_char, "5555") == 0)
    {
        return 5;
    }
    else
    {
        return  -1;
    }
}

int freq_chose(char *second_char)           //频率选择函数
{
    if (strcmp(second_char, "1111") == 0)
    {
        return 3500;
    }
    else if (strcmp(second_char, "2222") == 0)
    {
        return 4000;
    }
    else if (strcmp(second_char, "3333") == 0)
    {
        return 4500;
    }
    else if (strcmp(second_char, "4444") == 0)
    {
        return 5000;
    }
    else if (strcmp(second_char, "5555") == 0)
    {
        return 5500;
    }
    else
    {
        return  -1;
    }
}

void motor1_motor2_motor3_motor4_control(void)
{
    Position selected_position = all_positions[currentSelectPosition];
    set_pid_target3(&pid3, selected_position.vertical);
//                set_pid_target4(&pid4, selected_position.horizontal);
//                set_motor1_enable();
//                set_motor1_direction(MOTOR_FWD);
//                set_motor1_speed(selected_position.M1speed);
//
//                set_motor2_enable();
//                set_motor2_direction(MOTOR_REV);
//                set_motor2_speed(selected_position.M2speed);
}

void Fixed_control(void)
{
    static uint8_t state = 0;
    if (Fixedcnt == 1)
    {
        switch (state)
        {
            case 0: // 初始化并启动 M1、M2、M3、M4
            {
                motor1_motor2_motor3_motor4_control();  //控制M1、M2、M3、M4
                freq_function();                        //判断M5速度
                set_motor5_direction(MOTOR_REV);
                set_motor5_speed(motor5_speed);
                HAL_Delay(5000);    /** 五秒过后才打开电机的启动 */
                set_motor5_enable();
                state = 1;
                break;
            }
            case 1:
            {
                if (Dropping_adc_mean < 400)
                {
                    // 关闭电机
                    HAL_Delay(500);
                    set_motor5_disable();
                    state = 0; // 重置状态机
                    Fixedcnt = 0; // 重置控制标志位
                    Fixed_flag = 0;
                    sensor_triggered = 1;
                }
                break;
            }
        }
    }
}

void repeat_function(void)
{
    int loop_count = 0;
    int repeat_count_comparison_value = 0;
    repeat_count_comparison_value = currentSelectrepeat_count;       //传递需要重复的次数
    while (loop_count < repeat_count_comparison_value)              // 这里是一个示例，循环3次
    {
        sensor_triggered = 0;
        Fixedcnt = 1;
        // 等待 Fixed_control 完成其任务
        while (sensor_triggered == 0)
        {
            Fixed_control();                                       // 持续调用 Fixed_control，直到其完成任务并重置 Fixedcnt
        }
        loop_count++;  // 增加循环计数器1
    }
    repeat_flag = 0;
}

void freq_function(void)
{
    if(freq_flag == 1)
    {
        motor5_speed = currentSelectmotor5_speed;
    }
    else
    {
        motor5_speed = 3000;
    }
}

void random_control(void)
{
    //随机模式：全场
    if(1 == all_random_flag)
    {
        if(Dropping_adc_mean < 400)
        {
            HAL_Delay(500);
            randomcnt1 = 1;
            set_motor5_disable();
        }
/** 保证每个位置执行一次，要等待上方传感器产生低电平信号 */
        if(1 == randomcnt1)
        {
            randomcnt1 = 0;
            srand(HAL_GetTick()); // 初始化随机数发生器
            int index = generate_random_all_position(); // 获取随机位置索引
            Position selected_position = all_positions[index]; // 获取选定的位置数据

            set_pid_target3(&pid3, selected_position.vertical);
            set_pid_target4(&pid4, selected_position.horizontal);
            set_motor1_enable();
            set_motor1_direction(MOTOR_FWD);
            set_motor1_speed(selected_position.M1speed);
            set_motor2_enable();
            set_motor2_direction(MOTOR_REV);
            set_motor2_speed(selected_position.M2speed);
            HAL_Delay(2000);
/** 两秒过后才打开电机的启动 */
            set_motor5_direction(MOTOR_REV);
            set_motor5_speed(3500);
            set_motor5_enable();
        }
    }

    //随机模式：左半场
    if(1 == left_random_flag)
    {
        if(Dropping_adc_mean < 400)
        {
            HAL_Delay(500);
            randomcnt2 = 1;
            set_motor5_disable();
        }
/** 保证每个位置执行一次，要等待上方传感器产生低电平信号 */
        if(1 == randomcnt2)
        {
            randomcnt2 = 0;
            srand(HAL_GetTick()); // 初始化随机数发生器
            int index = generate_random_left_position(); // 获取随机位置索引
            Position selected_position = left_positions[index]; // 获取选定的位置数据

            set_pid_target3(&pid3, selected_position.vertical);
            set_pid_target4(&pid4, selected_position.horizontal);
            set_motor1_enable();                            //A1
            set_motor1_direction(MOTOR_FWD);
            set_motor1_speed(selected_position.M1speed);
            set_motor2_enable();
            set_motor2_direction(MOTOR_REV);
            set_motor2_speed(selected_position.M2speed);
            HAL_Delay(2000);
/** 两秒过后才打开电机的启动 */
            set_motor5_direction(MOTOR_REV);
            set_motor5_speed(3500);
            set_motor5_enable();
        }
    }

    //随机模式：右半场
    if(1 == right_random_flag)
    {
        if(Dropping_adc_mean < 400)
        {
            HAL_Delay(500);
            randomcnt3 = 1;
            set_motor5_disable();
        }
/** 保证每个位置执行一次，要等待上方传感器产生低电平信号 */
        if(1 == randomcnt3)
        {
            randomcnt3 = 0;
            srand(HAL_GetTick()); // 初始化随机数发生器
            int index = generate_random_right_position(); // 获取随机位置索引
            Position selected_position = right_positions[index]; // 获取选定的位置数据

            set_pid_target3(&pid3, selected_position.vertical);
            set_pid_target4(&pid4, selected_position.horizontal);
            set_motor1_enable();                            //A1
            set_motor1_direction(MOTOR_FWD);
            set_motor1_speed(selected_position.M1speed);
            set_motor2_enable();
            set_motor2_direction(MOTOR_REV);
            set_motor2_speed(selected_position.M2speed);
            HAL_Delay(2000);
/** 两秒过后才打开电机的启动 */
            set_motor5_direction(MOTOR_REV);
            set_motor5_speed(3500);
            set_motor5_enable();
        }
    }
}

void upward_control(void)
{
    LED5_TOGGLE
    uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
    if (temp_val >= 700) {
        temp_val -= 20; // 每次减少150
        if (temp_val < 700) {
            temp_val = 700; // 如果减少后的值小于700，则将其设置为700
        }
        set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
    }
}

void downward_control(void)
{
    LED2_TOGGLE
    uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
    if (temp_val <= 2100)
    {
        temp_val += 20; // 每次增加150
        if (temp_val > 2100)
        {
            temp_val = 2100; // 如果增加后的值大于2100，则将其设置为2100
        }
        set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
    }
}

void left_control(void)
{
    LED3_TOGGLE
    uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
    if(temp_val >= 700)
    {
        temp_val -= 20; // 每次减少150
        if(temp_val < 700)
        {
            temp_val = 700; // 如果减少后的值小于700，则将其设置为700
        }
        set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
    }
}

void right_control(void)
{
    LED4_TOGGLE
    uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
    if(temp_val <= 2200)
    {
        temp_val += 20; // 每次增加100
        if(temp_val > 2200) {
            temp_val = 2200; // 如果增加后的值大于2100，则将其设置为2100
        }
        set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
    }
}

void motor1_motor2_control(void)
{
    LED5_TOGGLE
    if(!is_motor1_en && !is_motor2_en)
    {
        set_motor1_enable();
        set_motor1_direction(MOTOR_FWD);
        set_motor1_speed(1800);

        set_motor2_enable();
        set_motor2_direction(MOTOR_REV);
        set_motor2_speed(1800);
    }
    else
    {
        set_motor1_disable();
        set_motor2_disable();
    }
}

void motor1_motor2_speedup(void)
{
    __IO uint16_t motor1_ChannelPulse = 2000; // 9000
    __IO uint16_t motor2_ChannelPulse = 2000;

    LED5_TOGGLE
    motor1_ChannelPulse += 100;
    motor2_ChannelPulse += 100;
    if((motor1_ChannelPulse > PWM_MAX_PERIOD_COUNT)||(motor2_ChannelPulse > PWM_MAX_PERIOD_COUNT))
    {
        motor1_ChannelPulse = PWM_MAX_PERIOD_COUNT;
        motor2_ChannelPulse = PWM_MAX_PERIOD_COUNT;
    }
    set_motor1_speed(motor1_ChannelPulse);
    set_motor2_speed(motor2_ChannelPulse);
}

void motor1_motor2_slowdown(void)
{
    __IO uint16_t motor1_ChannelPulse = 2000; // 9000
    __IO uint16_t motor2_ChannelPulse = 2000;
    LED5_TOGGLE
    motor1_ChannelPulse -= 100;
    motor2_ChannelPulse -= 100;
    if((motor1_ChannelPulse < PWM_MAX_PERIOD_COUNT/10)||(motor2_ChannelPulse < PWM_MAX_PERIOD_COUNT/10))
    {
        motor1_ChannelPulse = 0;
        motor2_ChannelPulse = 0;
    }

    set_motor1_speed(motor1_ChannelPulse);
    set_motor2_speed(motor2_ChannelPulse);
}

void motor_reset(void)
{
    LED5_TOGGLE
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

void motor5_control(void)
{
    if(1 == motor5_flag)
    {
        if(Dropping_adc_mean < 400)
        {
            HAL_Delay(500);
            randomcnt1 = 1;
            set_motor5_disable();
        }
/** 保证每个位置执行一次，要等待上方传感器产生低电平信号 */
        if(1 == randomcnt5)
        {
            randomcnt5 = 0;
            set_motor5_direction(MOTOR_REV);
            set_motor5_speed(3500);
            set_motor5_enable();
        }
    }
}

//            if (first_char == 'a')    //向上
//            {
//                clean_rebuff();
//                LED5_TOGGLE
//                uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
//                if (temp_val >= 700) {
//                    temp_val -= 150; // 每次减少150
//                    if (temp_val < 700) {
//                        temp_val = 700; // 如果减少后的值小于700，则将其设置为700
//                    }
//                    set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
//                }
//            }
//            if (first_char == 'b')       //向下
//            {
//                clean_rebuff();
//                LED2_TOGGLE
//                uint16_t temp_val = get_pid_target(&pid4); // 获取当前PID目标值
//                if (temp_val <= 2100) {
//                    temp_val += 150; // 每次增加150
//                    if (temp_val > 2100) {
//                        temp_val = 2100; // 如果增加后的值大于2100，则将其设置为2100
//                    }
//                    set_pid_target4(&pid4, temp_val); // 设置新的PID目标值
//                }
//            }
//            if (first_char == '3')          //左
//            {
//                clean_rebuff();
//                LED3_TOGGLE
//                uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
//                if (temp_val >= 700) {
//                    temp_val -= 150; // 每次减少150
//                    if (temp_val < 700) {
//                        temp_val = 700; // 如果减少后的值小于700，则将其设置为700
//                    }
//                    set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
//                }
//            }
//            if (first_char == '4')       //右
//            {
//                clean_rebuff();
//                LED4_TOGGLE
//                uint16_t temp_val = get_pid_target(&pid3); // 获取当前PID目标值
//                if (temp_val <= 2200) {
//                    temp_val += 150; // 每次增加100
//                    if (temp_val > 2200) {
//                        temp_val = 2200; // 如果增加后的值大于2100，则将其设置为2100
//                    }
//                    set_pid_target3(&pid3, temp_val); // 设置新的PID目标值
//                }
//            }
//            if (first_char == '5')       //启停M1、M2
//            {
//                clean_rebuff();
//                LED5_TOGGLE
//                if (!is_motor1_en && !is_motor2_en) {
//                    set_motor1_enable();
//                    set_motor1_direction(MOTOR_FWD);
//                    set_motor1_speed(1800);
//
//                    set_motor2_enable();
//                    set_motor2_direction(MOTOR_REV);
//                    set_motor2_speed(1800);
//                }
//                else {
//                    set_motor1_disable();
//                    set_motor2_disable();
//                }
//            }
//            if (first_char == 'c')       //设置M1、M2不同占空比
//            {
//                clean_rebuff();
//                LED5_TOGGLE
//                if (!is_motor1_en && !is_motor2_en)
//                {
//                    set_motor1_enable();
//                    set_motor1_direction(MOTOR_FWD);
//                    set_motor1_speed(1500);
//
//                    set_motor2_enable();
//                    set_motor2_direction(MOTOR_REV);
//                    set_motor2_speed(2500);
//                }
//                else
//                {
//                    set_motor1_disable();
//                    set_motor2_disable();
//                }
//            }
//            if (first_char == 'd')       //设置M1、M2不同占空比
//            {
//                clean_rebuff();
//                LED5_TOGGLE
//                if (!is_motor1_en && !is_motor2_en) {
//                    set_motor1_enable();
//                    set_motor1_direction(MOTOR_FWD);
//                    set_motor1_speed(2500);
//
//                    set_motor2_enable();
//                    set_motor2_direction(MOTOR_REV);
//                    set_motor2_speed(1500);
//                }
//                else {
//                    set_motor1_disable();
//                    set_motor2_disable();
//                }
//            }
//            if (first_char == '6')       //M1、M2加速
//            {
//                clean_rebuff();
//                __IO uint16_t motor1_ChannelPulse = 2000; // 9000
//                __IO uint16_t motor2_ChannelPulse = 2000;
//
//                LED5_TOGGLE
//                motor1_ChannelPulse += 100;
//                motor2_ChannelPulse += 100;
//                if ((motor1_ChannelPulse > PWM_MAX_PERIOD_COUNT) || (motor2_ChannelPulse > PWM_MAX_PERIOD_COUNT)) {
//                    motor1_ChannelPulse = PWM_MAX_PERIOD_COUNT;
//                    motor2_ChannelPulse = PWM_MAX_PERIOD_COUNT;
//                }
//                set_motor1_speed(motor1_ChannelPulse);
//                set_motor2_speed(motor2_ChannelPulse);
//            }
//            if (first_char == '7')           //M1、M2减速
//            {
//                clean_rebuff();
//                __IO uint16_t motor1_ChannelPulse = 2000; // 9000
//                __IO uint16_t motor2_ChannelPulse = 2000;
//                LED5_TOGGLE
//                motor1_ChannelPulse -= 100;
//                motor2_ChannelPulse -= 100;
//                if ((motor1_ChannelPulse < PWM_MAX_PERIOD_COUNT / 10) ||
//                    (motor2_ChannelPulse < PWM_MAX_PERIOD_COUNT / 10)) {
//                    motor1_ChannelPulse = 0;
//                    motor2_ChannelPulse = 0;
//                }
//
//                set_motor1_speed(motor1_ChannelPulse);
//                set_motor2_speed(motor2_ChannelPulse);
//            }
//            if (first_char == '8')           //启停M5
//            {
//                clean_rebuff();
//                LED5_TOGGLE
//                if (!is_motor5_en) {
//                    set_motor5_enable();
//                    set_motor5_direction(MOTOR_FWD);
//                    set_motor5_speed(3000);
//                } else {
//                    set_motor5_disable();
//                }
//            }
//            if (first_char == '9')           //复位
//            {
//                clean_rebuff();
//                LED5_TOGGLE
//                __set_FAULTMASK(1);
//                NVIC_SystemReset();
//            }