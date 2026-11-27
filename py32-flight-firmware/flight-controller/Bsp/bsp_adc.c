#include "bsp_adc.h"

void Bsp_ADC_Init(void)
{
  // 使能GPIOA时钟
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  // PA4 配置为模拟输入（模拟模式，无上下拉）
  GPIOA->MODER   = (GPIOA->MODER & ~(3U << (4 * 2))) | (3U << (4 * 2)); // 模拟模式
  GPIOA->PUPDR  &= ~(3U << (4 * 2));   

  /*时钟配置*/
  RCC->APBRSTR2 |= (1 << 20); // 复位AssDC时钟
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
  ADC1->CHSELR |= (1 << 0x00000002U);

  /*启动ADC*/
  ADC1->CR |= (1 << 0); // 使能ADC ADEN位
  ADC1->CR |= (1 << 2); // 启动ADC ADSTART位
}

uint16_t Get_ADC_val(void)
{
    while ((ADC1->ISR & (1 << 2) && (1 << 3)) == 0)
      ; // 等待转换结束
    return ADC1->DR;
}
