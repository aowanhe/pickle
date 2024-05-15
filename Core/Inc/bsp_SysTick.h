//
// Created by Giselle on 2024/3/17.
//

#ifndef CPROJECT_BSP_SYSTICK_H
#define CPROJECT_BSP_SYSTICK_H




#include "stm32f1xx.h"


#define NumOfTask 3

typedef uint32_t  u32;

extern void SysTick_Init(void);
extern void Delay_us(__IO u32 nTime);
extern void TimingDelay_Decrement(void);
int get_tick_count(unsigned long *count);





#endif //CPROJECT_BSP_SYSTICK_H
