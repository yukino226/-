#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "py32f0xx_hal.h"

void Bsp_ADC_Init(void);
uint16_t Get_ADC_val(void);
#endif
