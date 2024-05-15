#ifndef __ADVANCE_TIM_H
#define	__ADVANCE_TIM_H

#include "stm32f1xx.h"

/* 累计 TIM_Period个后产生一个更新或者中断*/
/* 当定时器从0计数到PWM_PERIOD_COUNT，即为PWM_PERIOD_COUNT+1次，为一个定时周期 */
#define PWM_PERIOD_COUNT     (4800)   //(3600)

/* 通用控制定时器时钟源TIMxCLK = HCLK=72MHz */
/* 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1) */
#define PWM_PRESCALER_COUNT     (2)

#define T1_PEM_motor1_dutyfactor    (2400)

/* 最大比较值 */
#define PWM_MAX_PERIOD_COUNT                    (PWM_PERIOD_COUNT - 100)

/* 定时器 */
#define ADVANCE_TIM           				    TIM1
#define ADVANCE_TIM_CLK_ENABLE()  			    __HAL_RCC_TIM1_CLK_ENABLE()

#define GENERAL_TIM                        	    TIM4
#define GENERAL_TIM_CLK_ENABLE()  				__HAL_RCC_TIM4_CLK_ENABLE()

#define ADVANCE_OCPWM_GPIO_CLK_ENABLE() 	    __HAL_RCC_GPIOE_CLK_ENABLE();
#define TIM4_GPIO_CLK_ENABLE() 	                __HAL_RCC_GPIOD_CLK_ENABLE();

/* TIM1通道1输出引脚 */
#define ADVANCE_OCPWM1_PIN           		    GPIO_PIN_9
#define ADVANCE_OCPWM1_GPIO_PORT     		    GPIOE


/* TIM1通道1互补输出引脚 */
#define ADVANCE_OCNPWM1_PIN            		    GPIO_PIN_8
#define ADVANCE_OCNPWM1_GPIO_PORT      		    GPIOE

/* TIM1通道2输出引脚 */
#define ADVANCE_OCPWM2_PIN           		    GPIO_PIN_11
#define ADVANCE_OCPWM2_GPIO_PORT     		    GPIOE

/* TIM1通道2互补输出引脚 */
#define ADVANCE_OCNPWM2_PIN            		    GPIO_PIN_10
#define ADVANCE_OCNPWM2_GPIO_PORT      		    GPIOE

/* TIM1通道3输出引脚 */
#define ADVANCE_OCPWM3_PIN           		    GPIO_PIN_13
#define ADVANCE_OCPWM3_GPIO_PORT     		    GPIOE

/* TIM1通道3互补输出引脚 */
#define ADVANCE_OCNPWM3_PIN            		    GPIO_PIN_12
#define ADVANCE_OCNPWM3_GPIO_PORT      		    GPIOE

/* TIM1通道4输出引脚 */
#define ADVANCE_OCPWM4_PIN            		    GPIO_PIN_14
#define ADVANCE_OCPWM4_GPIO_PORT      		    GPIOE

/* TIM4通道1输出引脚 */
#define PWM_TIM4_CH1_GPIO_PORT                  GPIOD
#define PWM_TIM4_CH1_PIN                        GPIO_PIN_13

/* TIM4通道2输出引脚 */
#define PWM_TIM4_CH2_GPIO_PORT                  GPIOD
#define PWM_TIM4_CH2_PIN                        GPIO_PIN_12

#define SHUTDOWN_PIN                            GPIO_PIN_7
#define SHUTDOWN_GPIO_PORT                      GPIOE
#define SHUTDOWN_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOE_CLK_ENABLE()

/* 电机 nSLEEP 使能脚 */
#define MOTOR_ENABLE_nSLEEP()                     HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_SET)      // 高电平打开-高电平使能
#define MOTOR_DISABLE_nSLEEP()                    HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_RESET)    // 低电平关断-低电平禁用

extern TIM_HandleTypeDef  TIM_TimeBaseStructure;
extern TIM_HandleTypeDef  TIM4_TimeBaseStructure;

void TIMx_Configuration(void);
void TIM1_SetPWM_pulse(uint32_t channel,int compare);
void TIM4_SetPWM_pulse(uint32_t channel,int compare);

#endif /* __ADVANCE_TIM_H */

