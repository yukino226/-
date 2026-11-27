/**
 * @file bsp_beer.h
 * @author zhou.jie
 * @brief  蜂鸣器头文件
 * @version 0.1
 * @date 2024-11-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __BSP_BEER_H
#define __BSP_BEER_H

/* 包含其他头文件 */
#include "py32f0xx_hal.h"

/* 宏定义 */
// GPS BEER
#define BEER_GPIO_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE
#define BEER_GPIO_PORT GPIOB
#define BEER_GPIO_PIN GPIO_PIN_7
#define TIM14_PWM_CH1_GPIO_AF GPIO_AF5_TIM14 /* 端口复用到TIM1 */

typedef enum _Beer_statue
{
    BEER_OFF    = 0,
    BEER_ON     = 1,
    BEER_TOGGLE = 3,
} Beer_statue;

// 函数声明
void Bsp_Beer_Init(void);
void Beer_Ctlr(Beer_statue sta);

#endif /* __BSP_BEER_H */
