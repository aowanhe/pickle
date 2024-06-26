//
// Created by Giselle on 2024/3/17.
//
/**
  ******************************************************************************
  * @file    bsp_SysTick.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   SysTick 系统滴答时钟10us中断函数库,中断时间可自由配置，
  *          常用的有 1us 10us 1ms 中断。
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */


#include "bsp_SysTick.h"



static __IO u32 TimingDelay;
__IO uint32_t g_ul_ms_ticks=0;
/**
  * @brief  启动系统滴答定时器 SysTick
  * @param  无
  * @retval 无
  */
void SysTick_Init(void)
{
    /* SystemFrequency / 1000    1ms中断一次
     * SystemFrequency / 100000	 10us中断一次
     * SystemFrequency / 1000000 1us中断一次
     */
    if (HAL_SYSTICK_Config(SystemCoreClock / 100))
    {
        /* Capture error */
        while (1);
    }
}

/**
  * @brief   us延时程序,10us为一个单位
  * @param
  *		@arg nTime: Delay_us( 1 ) 则实现的延时为 1 * 10us = 10us
  * @retval  无
  */
void Delay_us(__IO u32 nTime)
{
    TimingDelay = nTime;

    while(TimingDelay != 0);
}

int get_tick_count(unsigned long *count)
{
    count[0] = g_ul_ms_ticks;
    return 0;
}

/**
  * @brief  获取节拍程序
  * @param  无
  * @retval 无
  * @attention  在 SysTick 中断函数 SysTick_Handler()调用
  */
void TimingDelay_Decrement(void)            //递减全局变量 TimingDelay，直到其值为0。
{
    if (TimingDelay != 0x00)                //检查 TimingDelay 是否不等于 0; 如果 TimingDelay 不为 0，则将其值减 1。
    {
        TimingDelay--;
    }
}

void TimeStamp_Increment(void)   //记录系统运行的总时间，以毫秒为单位
{
    g_ul_ms_ticks++;
}



/*********************************************END OF FILE**********************/
