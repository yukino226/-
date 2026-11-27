#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "py32f0xx_hal.h"
#include "stdio.h"


void Bsp_PWM_Init(void);
void Bsp_PWM_SetDuty(uint8_t channel, uint16_t duty);
#endif
