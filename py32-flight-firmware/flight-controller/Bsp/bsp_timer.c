#include <stdlib.h>
#include "bsp.h"
#include "App_Sound.h"
void Bsp_Timer_Init(void)
{
    // 使能TIM14时钟
    RCC->APBENR2 |= RCC_APBENR2_TIM14EN;

    // 关闭定时器
    TIM14->CR1 &= ~TIM_CR1_CEN;

    // 配置预分频器和自动重装载寄存器
    TIM14->PSC = 24 - 1;      // 预分频，24MHz/24=1MHz
    TIM14->ARR = 100 - 1;     // 自动重装载，1MHz/100=10kHz=100us

    // 计数方向向上
    TIM14->CR1 &= ~TIM_CR1_DIR;

    // 允许自动重装载寄存器预装载
    TIM14->CR1 |= TIM_CR1_ARPE;

    // 清除更新中断标志
    TIM14->SR = ~TIM_SR_UIF;

    // 使能更新中断
    TIM14->DIER |= TIM_DIER_UIE;

    // 配置中断优先级并使能
    NVIC_SetPriority(TIM14_IRQn, 0);
    NVIC_EnableIRQ(TIM14_IRQn);
}

void TIM14_IRQHandler(void)
{
    if (TIM14->SR & TIM_SR_UIF) // 检查更新中断标志
    {
        TIM14->SR &= ~TIM_SR_UIF; // 清除更新中断标志
        App_Sound_Update();
    }
}

void Bsp_Timer_Start(void)
{
    TIM14->CR1 |= TIM_CR1_CEN;
}

void Bsp_Timer_Stop(void)
{
    TIM14->CR1 &= (~TIM_CR1_CEN);
}

 


