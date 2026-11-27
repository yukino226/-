/**
 * @file bsp_beer.c
 * @author zhou.jie
 * @brief 蜂鸣器驱动
 * @version 0.1
 * @date 2024-11-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "beer/bsp_beer.h"
#include "systick/bsp_systick_delay.h"

/* 定时器x句柄 */
TIM_HandleTypeDef Timer14_PWM_Handle;

/**
 * @brief       定时器TIM14 通道Y PWM输出 初始化函数（使用PWM模式1）
 * @note
 *              定时器的时钟来自APB1, 而PCLK1 = 24Mhz, 我们设置PPRE1不分频, 因此
 *              定时器时钟 = 24Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       Prescaler: 	    时钟预分频数
 * @param       Period: 		自动重装值
 * @retval      无
 */
static void Timer14_PWM_Init(uint16_t Prescaler, uint16_t Period)
{
    TIM_OC_InitTypeDef Timer14_OC_PWM_Config; /* 定时器PWM输出配置 */

    Timer14_PWM_Handle.Instance = TIM14;
    /* 定时器x */
    Timer14_PWM_Handle.Init.Period = Period - 1;                        /* 自动重装载值 */
    Timer14_PWM_Handle.Init.Prescaler = Prescaler - 1;                  /* 定时器分频 */
    Timer14_PWM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 向上计数模式 */
    Timer14_PWM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* 时钟分频因子 */
    Timer14_PWM_Handle.Init.RepetitionCounter = 0;                  /* 不重复计数*/
                                                                    //    /* 自动重装载寄存器没有缓冲 */
                                                                    //    Advanced_Timer1_PWM_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&Timer14_PWM_Handle);                          /* 初始化PWM */

    Timer14_OC_PWM_Config.OCMode = TIM_OCMODE_PWM1;         /* 模式选择PWM1 */
    Timer14_OC_PWM_Config.Pulse = ((Period + 1) / 2);       /* ( Period + 1 ) / 2设置比较值,此值用来确定占空比,默认50% */
    Timer14_OC_PWM_Config.OCPolarity = TIM_OCPOLARITY_HIGH; /* 输出比较极性为高 */
    Timer14_OC_PWM_Config.OCFastMode = TIM_OCFAST_DISABLE;
    Timer14_OC_PWM_Config.OCIdleState = TIM_OCIDLESTATE_SET;
    Timer14_OC_PWM_Config.OCNIdleState = TIM_OCNIDLESTATE_RESET; /*空闲状态OC1N输出低电平 */
    Timer14_OC_PWM_Config.OCIdleState = TIM_OCIDLESTATE_RESET;   /*空闲状态OC1输出低电平*/

    /* 配置TIMx通道1 */
    HAL_TIM_PWM_ConfigChannel(&Timer14_PWM_Handle, &Timer14_OC_PWM_Config, TIM_CHANNEL_1);
    /* 使能定时器x和定时器x更新中断 */
    HAL_TIM_PWM_Start(&Timer14_PWM_Handle, TIM_CHANNEL_1);
}
/**
 * @brief       设置定时器PWM输出占空比函数
 * @param       可变参数
 * @retval      无
 */
static void SetTIM14PWMOutputChannalDuty(uint16_t CCR1_Value)
{
    Timer14_PWM_Handle.Instance->CCR1 = CCR1_Value;
}
/**
 * @brief       定时器底层驱动，开启时钟，设置中断优先级
                此函数会被HAL_TIM_PWM_Init函数调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14)
    {
        GPIO_InitTypeDef TIM14_PWM_Channal_GPIO_Config;

        /* 使能TIM时钟 */
        __HAL_RCC_TIM14_CLK_ENABLE();

        /* PWM Output Channal GPIO时钟使能 */
        BEER_GPIO_CLK_ENABLE();

        /* PWM Output Channal 1 */
        TIM14_PWM_Channal_GPIO_Config.Mode = GPIO_MODE_AF_PP;
        TIM14_PWM_Channal_GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;
        TIM14_PWM_Channal_GPIO_Config.Pull = GPIO_NOPULL;
        TIM14_PWM_Channal_GPIO_Config.Pin = BEER_GPIO_PIN;
        TIM14_PWM_Channal_GPIO_Config.Alternate = TIM14_PWM_CH1_GPIO_AF;
        HAL_GPIO_Init(BEER_GPIO_PORT, &TIM14_PWM_Channal_GPIO_Config);
    }
}
/**
 * @brief 蜂鸣器初始化
 *
 */
void Bsp_Beer_Init(void)
{
    Timer14_PWM_Init(4, 2400);
    SetTIM14PWMOutputChannalDuty(0);
}
/**
 * @brief 蜂鸣器控制
 * @param sta
 */
void Beer_Ctlr(Beer_statue sta)
{
    if (sta == BEER_ON)
    {
        SetTIM14PWMOutputChannalDuty(200);
    }
    else if (sta == BEER_TOGGLE)
    {
        SetTIM14PWMOutputChannalDuty(200);
        Delay_ms(20);
        SetTIM14PWMOutputChannalDuty(0);
    }
    else
    {
        SetTIM14PWMOutputChannalDuty(0);
    }
}
