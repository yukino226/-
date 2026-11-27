/**
  ******************************************************************************
  * @file     bsp_gpio_led.c
  * @author   zhou.jie
  * @version  V1.0
  * @date     2024
  * @brief    led应用接口
  ******************************************************************************
  */
#include "led/bsp_gpio_led.h"
#include "app.h"
/**
  * @brief  LED初始化
  * @param  无
  * @note  	无
  * @retval 无
  */
void Bsp_Led_Init(void)
{
    GPIO_InitTypeDef	GPIO_Led_InitConfig;
    
    /* GPIOA时钟使能 */
    LED2_GPIO_CLK_ENABLE();
    LED1_GPIO_CLK_ENABLE();
#if (Debug_Mode)
    //void
#else
    LED3_GPIO_CLK_ENABLE();
#endif
    
    GPIO_Led_InitConfig.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Led_InitConfig.Pull = GPIO_PULLUP;
    GPIO_Led_InitConfig.Speed = GPIO_SPEED_FREQ_HIGH;
    /* LED2 GPIO初始化 */
    GPIO_Led_InitConfig.Pin = LED2_GPIO_PIN;
    HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_Led_InitConfig);
#if (Debug_Mode)
    //void
#else
    /* LED3 GPIO初始化 */
    GPIO_Led_InitConfig.Pin = LED3_GPIO_PIN;
    HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_Led_InitConfig);
#endif
    /* LED1 GPIO初始化 */
    GPIO_Led_InitConfig.Pin = LED1_GPIO_PIN;
    HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_Led_InitConfig);
    
    /* LED 关闭 */
    LED2(LED_OFF);
#if (Debug_Mode)
    //void
#else
    LED3(LED_OFF);
#endif
    LED1(LED_OFF);
}
/*********************************************END OF FILE**********************/
