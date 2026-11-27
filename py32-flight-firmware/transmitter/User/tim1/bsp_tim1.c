
/**
 * @file bsp_tim1.c
 * @author zhou.jie
 * @brief
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "tim1/bsp_tim1.h"
#include "app.h"

/* 定时器x句柄 */
TIM_HandleTypeDef Advanced_Timer1_UpdateIT_Handle;

/**
 * @brief       高级定时器TIM1更新中断初始化函数
 * @note
 *              高级定时器的时钟来自APB1, 而PCLK1 = 24Mhz, 我们设置PPRE1不分频, 因此
 *              高级定时器时钟 = 24Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *              当前配置 2ms
 *
 * @param       Prescaler: 	    时钟预分频数
 * @param       Period: 		自动重装值
 * @retval      无
 */
void Timer1_UpdateIT_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();

  Advanced_Timer1_UpdateIT_Handle.Instance = TIM1;
  /* 定时器x */
  Advanced_Timer1_UpdateIT_Handle.Init.Period = 6 - 1;                       /* arr 自动重装载值 */
  Advanced_Timer1_UpdateIT_Handle.Init.Prescaler = 8000 - 1;                   /* psc 定时器分频 */
  Advanced_Timer1_UpdateIT_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 向上计数模式 */
  Advanced_Timer1_UpdateIT_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* 时钟分频因子 */
  Advanced_Timer1_UpdateIT_Handle.Init.RepetitionCounter = 0;                  /* 不重复计数*/
  /* 自动重装载寄存器没有缓冲 */
  Advanced_Timer1_UpdateIT_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&Advanced_Timer1_UpdateIT_Handle); /* 初始化PWM */

  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);

  HAL_TIM_Base_Start_IT(&Advanced_Timer1_UpdateIT_Handle);
}

/**
 * @brief  TIM1更新中断回调函数
 * @param  无
 * @note  	无
 * @retval 无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  time_tick_increase();
}
/*********************************************END OF FILE**********************/
