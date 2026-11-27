#ifndef __BSP_H
#define __BSP_H

#include "bsp_i2c.h"
#include "bsp_spi.h"
#include "bsp_flash.h"
#include "bsp_pwm.h"
#include "bsp_timer.h"
#include "bsp_adc.h"

void Bsp_Init(void);
void Bsp_Gpio_Toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Bsp_Gpio_Set(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Bsp_Gpio_Reset(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void delay_ms(uint32_t ms);
#endif
