/**
 * @file bsp_adc.c
 * @author zhou.jie
 * @brief
 * @version 0.1
 * @date 2024-11-01
 * @copyright Copyright (c) 2024
 *
 */
#include "adc/bsp_adc.h"
#include "usart/bsp_usart_debug.h"
#include "app.h"

/**
 * @brief  adc初始化
 * @param  无
 * @note  	轮询查询
 * @retval 无
 */
void Bsp_ADC_Init(void)
{
#if (ADC_MODE == ADC_SINGLE)
  /*时钟配置*/
  RCC->APBRSTR2 |= (1 << 20); // 复位ADC时钟
  RCC->APBRSTR2 &= ~(1 << 20);
  RCC->APBENR2 |= (1 << 20); // 使能ADC时钟
  /* ADC校准 */
  ADC1->CR |= (1UL << 31UL); // 启动ADC校准
  while ((ADC1->CR & (1UL << 31UL)) == 1)
    ; // 等待ADC校准完成

  /* ADC配置 */
  ADC1->CFGR1 |= (1 << 12); // 数据覆盖
  /* ADC时钟配置 */
  ADC1->CFGR2 |= (1 << 28); // 2分频
  /* ADC采样时间配置 */
  ADC1->SMPR |= (2 << 0); //7.5个ADC时钟
#else
  /*时钟配置*/
  RCC->APBRSTR2 |= (1 << 20); // 复位ADC时钟
  RCC->APBRSTR2 &= ~(1 << 20);
  RCC->APBENR2 |= (1 << 20); // 使能ADC时钟
  /* ADC校准 */
  ADC1->CR |= (1UL << 31UL); // 启动ADC校准
  while ((ADC1->CR & (1UL << 31UL)) == 1)
    ; // 等待ADC校准完成

  /* ADC配置 */
  /////////////////////////////////////////////////
  ADC1->CFGR1 |= (1 << 13); // 连续转换模式
  /////////////////////////////////////////////////
  ADC1->CFGR1 |= (1 << 2); // 扫描序列-->向下扫描    连续模式 必须开这个 从11-0扫描
  ////////////////////////////////////////////////
  ADC1->CFGR1 |= (1 << 12); // 数据覆盖
  /* ADC时钟配置 */
  ADC1->CFGR2 |= (1 << 28); // 2分频
  /* ADC采样时间配置 */
  ADC1->SMPR |= (6 << 0);

  /*ADC通道配置*/
  // // 配置通道需要在ADC启动之前
  ADC1->CHSELR |= (1 << LEFT_ROCKER_U_D);
  ADC1->CHSELR |= (1 << LEFT_ROCKER_L_R);
  ADC1->CHSELR |= (1 << LOW_VOL_DETECT);
  ADC1->CHSELR |= (1 << RIGHT_ROCKER_U_D);
  ADC1->CHSELR |= (1 << RIGHT_ROCKER_L_R);
  ADC1->CHSELR |= (1 << COMBINATION_KEY);

  /*启动ADC*/
  ADC1->CR |= (1 << 0); // 使能ADC ADEN位
  ADC1->CR |= (1 << 2); // 启动ADC ADSTART位
#endif
}

/* 功能             PIN         CHANNEL
 * 左摇杆上下       PA4         ADC_IN2
 * 左摇杆左/右      PA3         ADC_IN1
 * 低电压检测       PB1         ADC_IN0
 * 右摇杆上/下      PA6         ADC_IN3
 * 右摇杆左/右      PA7         ADC_IN4
 * 组合键           PB0         ADC_IN7
 */
void Get_ADC_val(uint16_t *val, uint8_t ch_num)
{
  if (val != NULL && ch_num == ADC_CH_USE)
  {
#if (ADC_MODE == ADC_SINGLE)
    HAL_Delay(10); // ADC再次启动时间需要大于8个ADC时钟

    /*ADC通道配置*/
    // 配置通道需要在ADC启动之前
    ADC1->CHSELR |= (1 << LEFT_ROCKER_U_D);
    ADC1->CHSELR |= (1 << LEFT_ROCKER_L_R);
    ADC1->CHSELR |= (1 << LOW_VOL_DETECT);
    ADC1->CHSELR |= (1 << RIGHT_ROCKER_U_D);
    ADC1->CHSELR |= (1 << RIGHT_ROCKER_L_R);
    ADC1->CHSELR |= (1 << COMBINATION_KEY);

    /*启动ADC*/
    ADC1->CR |= (1 << 0); // 使能ADC ADEN位
    ADC1->CR |= (1 << 2); // 启动ADC ADSTART位

    for (uint8_t i = 0; i < ADC_CH_USE; i++)
    {
      while ((ADC1->ISR & (1 << 2) && (1 << 3)) == 0)
        ; // 等待转换结束
      val[i] = ADC1->DR;
    }
    // 到这里ADC会停止 可以变换通道
#else
    for (uint8_t i = 0; i < ADC_CH_USE; i++)
    {
      while ((ADC1->ISR & (1 << 2) && (1 << 3)) == 0)
        ; // 等待转换结束
      val[i] = ADC1->DR;
    }
#endif
  }
}
