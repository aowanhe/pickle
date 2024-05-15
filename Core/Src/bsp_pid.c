#include "bsp_pid.h"
#include "bsp_motor_control.h"
#include <stdbool.h>  // 包含stdbool.h头文件以使用bool类型
#include "bsp_basic_tim.h"
#include "bsp_led.h"

//定义全局变量

_pid pid3;
_pid pid4;

// 在文件的开头或全局范围内定义全局变量
_Bool Pflag4 = 0;   // Pflag是布尔型变量
_Bool Iflag4 = 0;   // Iflag是布尔型变量
_Bool Iflagz4 = 0;  // Iflagz是布尔型变量


_Bool Pflag3 = 0;   // Pflag是布尔型变量
_Bool Iflag3 = 0;   // Iflag是布尔型变量
_Bool Iflagz3 = 0;  // Iflagz是布尔型变量

_Bool pid3_enable = 0;   // Iflag是布尔型变量
_Bool pid4_enable = 0;  // Iflagz是布尔型变量


/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init(void)
{
    /* 下电机初始化参数 */
    pid3.target_val = 0.0;
    pid3.actual_val = 0.0;
    pid3.err = 0.0;
    pid3.err_last = 0.0;
    pid3.err_next = 0.0;
    pid3.integral=0.0;
    pid3.integral_max = 8;
    pid3.integral_min = -8;
    pid3.Kp = 0.0;
    pid3.Ki = 0.0;
    pid3.Kd = 0.0;
    pid3.pid_enable = 0;

    /* 上电机初始化参数 */
    pid4.target_val = 0.0;
    pid4.actual_val = 0.0;
    pid4.err = 0.0;
    pid4.err_last = 0.0;
    pid4.err_next = 0.0;
    pid4.integral = 0.0;
    pid4.integral_max = 8;
    pid4.integral_min = -8;
    pid4.Kp = 0.0;
    pid4.Ki = 0.0;
    pid4.Kd = 0.0;
    pid4.pid_enable = 0;
}

/**
  * @brief  设置pid3目标值
  * @param  val目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target3(_pid *pid, float temp_val)
{
    pid->target_val = temp_val;    // 设置当前的目标值
    pid3_enable = 1;
    HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure5);
}

/**
  * @brief  设置pid4目标值
  * @param  val目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target4(_pid *pid, float temp_val)
{
    pid->target_val = temp_val;    // 设置当前的目标值
    pid4_enable = 1;
    HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure5);
}



/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
    return pid->target_val;    // 设置当前的目标值
}

/**
  * @brief  设置比例、积分、微分系数
  * @param  p：比例系数 P
  * @param  i：积分系数 i
  * @param  d：微分系数 d
	*	@note 	无
  * @retval 无
  */
void set_p_i_d(_pid *pid, float p, float i, float d)
{
    pid->Kp = p;    // 设置比例系数 P
    pid->Ki = i;    // 设置积分系数 I
    pid->Kd = d;    // 设置微分系数 D
}

/**
  * @brief  PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float PID_realize(_pid *pid, float actual_val, _Bool Pflag, _Bool Iflag, _Bool Iflagz)
{

    //	/*计算目标值与实际值的误差*（增量式PID）*/
    pid->err = pid->target_val - actual_val;  //目标值 - 实际值

    float ProportionalTerm = pid->Kp * (pid->err - pid->err_next);
    float IntegralTerm = pid->Ki * ((200 - fabs(pid->err))/150) * pid->err;  //pid->err需要为绝对值
    float IntegralTermz = pid->Ki * pid->err;

    if (pid->err > -1 && pid->err < 1)  //误差值：-3 ~ 3
    {
        pid->err = 0;
    }
	if (Pflag == 1)  //只用比例项
	{
		pid->actual_val += ProportionalTerm;
	}

	if (Iflag == 1)
	{
		pid->actual_val += IntegralTerm; //积分项
	}

	if (Iflagz == 1)
	{
		pid->actual_val += IntegralTermz;
	}

 	/*PID算法实现*/
//	pid->actual_val += pid->Kp * (pid->err - pid->err_next) +                             //比例项：（当前误差 - 上一次误差）* Kp
//                    pid->Ki * pid->err + 										         //积分项：误差的累积 * Ki
//                    pid->Kd * (pid->err - 2 * pid->err_next + pid->err_last);          //微分项：当前误差、上一次误差和上上次误差的差值，并乘以微分增益
    /* 添加积分项的上下限控制 */
//    if (pid->Ki != 0) {
//        pid->integral += pid->err;  // 更新积分项
//        /* 设置积分项的上下限 */
//        if (pid->integral > pid->integral_max)
//        {
//            pid->integral = pid->integral_max;
//        } else if (pid->integral < pid->integral_min)
//        {
//            pid->integral = pid->integral_min;
//        }
//    }

    /*传递误差*/
    pid->err_last = pid->err_next;   //上上一次传递给上一次
    pid->err_next = pid->err;        //上一次传递给当前一次


    /*返回当前实际值*/
    return pid->actual_val;

}











