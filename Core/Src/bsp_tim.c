#include "bsp_tim.h"
#include "bsp_motor_control.h"

__IO uint16_t ChannelPulse = 4500;

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void TIMx_GPIO_Config(void)
{
    /*定义一个GPIO_InitTypeDef类型的结构体*/
    GPIO_InitTypeDef GPIO_InitStructure;

    /*开启定时器相关的GPIO外设时钟*/
    ADVANCE_OCPWM_GPIO_CLK_ENABLE()

    /* 定时器功能引脚初始化 */
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;    //推挽输出模式，配置为复用功能
    GPIO_InitStructure.Pull = GPIO_NOPULL;        //不上拉也不下拉
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    /* 开启TIM1的时钟 */
    __HAL_AFIO_REMAP_TIM1_ENABLE();

    //TIM1 channel1互补输出，配置引脚
    GPIO_InitStructure.Pin = ADVANCE_OCPWM1_PIN;
    HAL_GPIO_Init(ADVANCE_OCPWM1_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = ADVANCE_OCNPWM1_PIN;
    HAL_GPIO_Init(ADVANCE_OCNPWM1_GPIO_PORT, &GPIO_InitStructure);

    //TIM1 channel2互补输出
    GPIO_InitStructure.Pin = ADVANCE_OCPWM2_PIN;
    HAL_GPIO_Init(ADVANCE_OCPWM2_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = ADVANCE_OCNPWM2_PIN;
    HAL_GPIO_Init(ADVANCE_OCNPWM2_GPIO_PORT, &GPIO_InitStructure);

    //TIM1 channel3互补输出
    GPIO_InitStructure.Pin = ADVANCE_OCPWM3_PIN;
    HAL_GPIO_Init(ADVANCE_OCPWM3_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = ADVANCE_OCNPWM3_PIN;
    HAL_GPIO_Init(ADVANCE_OCNPWM3_GPIO_PORT, &GPIO_InitStructure);

    //TIM1 channel4配置PWM输出
    GPIO_InitStructure.Pin = ADVANCE_OCPWM4_PIN;
    HAL_GPIO_Init(ADVANCE_OCPWM4_GPIO_PORT, &GPIO_InitStructure);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
//配置TIM1的参数
TIM_HandleTypeDef  TIM_TimeBaseStructure;
static void TIM_Mode_Config(void)       //7.5 kHz
{
    TIM_OC_InitTypeDef TIM_OCInitStructure;                                              //配置TIM1的输出比较通道（output compare）
    /** 开启TIMx_CLK,x[1,8] */
    ADVANCE_TIM_CLK_ENABLE();

    /* 定义定时器的句柄即确定定时器寄存器的基地址*/
    TIM_TimeBaseStructure.Instance = ADVANCE_TIM;
    /* 累计 TIM_Period个后产生一个更新或者中断*/
    //当定时器从0计数到4799，即为4800次，为一个定时周期
    TIM_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT - 1;                            //-1为定时器的计数值从0开始
    // 控制定时器时钟源TIMxCLK = HCLK=72MHz
    // 设定定时器频率为=TIMxCLK/(PWM_PRESCALER_COUNT+1)
    TIM_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT - 1;                      //分频器：决定了定时器时钟源的频率与定时器计数器的时钟频率之间的比例关系
    // 采样时钟分频
    TIM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;                   //用于控制时钟信号进入计数器的速度 不分频
    // 计数方式
    TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;                         //向上计数
    // 重复计数器
    TIM_TimeBaseStructure.Init.RepetitionCounter = 0;                                    //不重复计数
    // 初始化定时器TIMx, x[1,8]
    HAL_TIM_PWM_Init(&TIM_TimeBaseStructure);

    /*PWM模式配置*/
    //配置为PWM模式1
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.Pulse = T1_PEM_motor1_dutyfactor;          //50%占空比
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;          //开启输出比较通道
    TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH;        //开启互补输出比较通道
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;       //空闲状态时为低电平
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;     //互补输出通道空闲状态时为低电平
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;           //禁用快速模式
    //初始化通道1输出PWM
    HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OCInitStructure,TIM_CHANNEL_4);

    /* 定时器通道1输出PWM */
    HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_1);
    /* 定时器通道1互补输出PWM */
    HAL_TIMEx_PWMN_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_1);

    /* 定时器通道2输出PWM */
    HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_2);
    /* 定时器通道2互补输出PWM */
    HAL_TIMEx_PWMN_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_2);

    /* 定时器通道3输出PWM */
    HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_3);
    /* 定时器通道3互补输出PWM */
    HAL_TIMEx_PWMN_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_3);

    /* 定时器通道4输出PWM */
    HAL_TIM_PWM_Start(&TIM_TimeBaseStructure,TIM_CHANNEL_4);
}

/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void TIM4_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    TIM4_GPIO_CLK_ENABLE()

    /* 定时器通道1功能引脚IO初始化 */
    /*设置输出类型*/
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    /*设置引脚速率 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    __HAL_AFIO_REMAP_TIM4_ENABLE();

    GPIO_InitStruct.Pin = PWM_TIM4_CH1_PIN;
    HAL_GPIO_Init(PWM_TIM4_CH1_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PWM_TIM4_CH2_PIN;
    HAL_GPIO_Init(PWM_TIM4_CH2_GPIO_PORT, &GPIO_InitStruct);

}

TIM_HandleTypeDef  TIM4_TimeBaseStructure;
static void TIM_PWMOUTPUT_Config(void)
{
    TIM_OC_InitTypeDef  TIM_OCInitStructure;
    int tim_per=5000;//定时器周期

    /*使能定时器*/
    GENERAL_TIM_CLK_ENABLE();

    TIM4_TimeBaseStructure.Instance = GENERAL_TIM;
    /* 累计 TIM_Period个后产生一个更新或者中断*/
    //当定时器从0计数到4799，即为4800次，为一个定时周期
    TIM4_TimeBaseStructure.Init.Period = PWM_PERIOD_COUNT - 1;
    //定时器时钟源TIMxCLK = 2 * PCLK1
    //				PCLK1 = HCLK / 2
    //				=> TIMxCLK=HCLK/2=SystemCoreClock/2*2=72MHz
    // 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10KHz
    TIM4_TimeBaseStructure.Init.Prescaler = PWM_PRESCALER_COUNT - 1;
    /*计数方式*/
    TIM4_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
    /*采样时钟分频*/
    TIM4_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    /*初始化定时器*/
    HAL_TIM_PWM_Init(&TIM4_TimeBaseStructure);

    /*PWM模式配置*/
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;//配置为PWM模式1
    TIM_OCInitStructure.Pulse = T1_PEM_motor1_dutyfactor;                              //默认占空比为50%
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    /*当定时器计数值小于CCR1_Val时为高电平*/
    TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;

    /*配置PWM通道*/
    HAL_TIM_PWM_ConfigChannel(&TIM4_TimeBaseStructure, &TIM_OCInitStructure, TIM_CHANNEL_1);
    /*开始输出PWM*/
    HAL_TIM_PWM_Start(&TIM4_TimeBaseStructure,TIM_CHANNEL_1);

    /*配置脉宽*/
    TIM_OCInitStructure.Pulse = T1_PEM_motor1_dutyfactor;                           //默认占空比为50%
    /*配置PWM通道*/
    HAL_TIM_PWM_ConfigChannel(&TIM4_TimeBaseStructure, &TIM_OCInitStructure, TIM_CHANNEL_2);
    /*开始输出PWM*/
    HAL_TIM_PWM_Start(&TIM4_TimeBaseStructure,TIM_CHANNEL_2);
}

void nSLEEP_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 定时器通道功能引脚端口时钟使能 */
    SHUTDOWN_GPIO_CLK_ENABLE();

    /* 引脚IO初始化 */
    /* 设置输出类型 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    /* 设置引脚速率 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /* 选择要控制的GPIO引脚 */
    GPIO_InitStruct.Pin = SHUTDOWN_PIN;

    /* 调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO */
    HAL_GPIO_Init(SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  设置TIM通道的占空比
	* @param  channel		通道	（1,2,3,4）
	* @param  compare		占空比
	*	@note 	无
  * @retval 无
  */
void TIM1_SetPWM_pulse(uint32_t channel,int compare)
{
    switch(channel)
    {
        case TIM_CHANNEL_1:     __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_1,compare);break;
        case TIM_CHANNEL_2:	    __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_2,compare);break;
        case TIM_CHANNEL_3:	    __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_3,compare);break;
        case TIM_CHANNEL_4:	    __HAL_TIM_SET_COMPARE(&TIM_TimeBaseStructure,TIM_CHANNEL_4,compare);break;
    }
}

void TIM4_SetPWM_pulse(uint32_t channel,int compare)
{
    switch(channel)
    {
        case TIM_CHANNEL_1:  	__HAL_TIM_SET_COMPARE(&TIM4_TimeBaseStructure,TIM_CHANNEL_1,compare);break;
        case TIM_CHANNEL_2:	    __HAL_TIM_SET_COMPARE(&TIM4_TimeBaseStructure,TIM_CHANNEL_2,compare);break;

    }
}

/**
  * @brief  初始化高级控制定时器定时，1s产生一次中断
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
    TIMx_GPIO_Config();
    TIM4_GPIO_Config();
    TIM_Mode_Config();
    TIM_PWMOUTPUT_Config();
    nSLEEP_gpio_config();
    set_motor1_disable();
    set_motor2_disable();
    set_motor3_disable();
    set_motor4_disable();
    set_motor5_disable();
}

/*********************************************END OF FILE**********************/
