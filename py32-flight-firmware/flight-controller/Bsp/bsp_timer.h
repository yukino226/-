#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "py32f0xx_hal.h"
#include "stdio.h"

void Bsp_Timer_Init(void);
void Bsp_Timer_Start_Timer(uint16_t time);  //time * 100us
void Bsp_Timer_Start(void);
void Bsp_Timer_Stop(void);

#endif
