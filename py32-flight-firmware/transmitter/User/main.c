/**
 * @file main.c
 * @author zhou.jie
 * @brief main
 * @version 0.1
 * @date 2024-10-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "main.h"
#include "led/bsp_gpio_led.h"
#include "usart/bsp_usart_debug.h"
#include "2_4G/bsp_rf_xn297.H"
#include "systick/bsp_systick_delay.h"
#include "tim1/bsp_tim1.h"
#include "app.h"
#include "beer/bsp_beer.h"
#include "key.h"
/**
 * @brief 模块初始化
 * 
 */
static void Module_Init(void)
{
#if (Debug_Mode)
    Delay_ms(1000);
#else
    //SWD SCK(PA2)作为LED IO使用
    Delay_ms(1000);
#endif
    DEBUG_USART_Config(115200); //串口初始化
    Bsp_Led_Init(); //LED初始化
    RF_Init(); //2.4G初始化
    key_init();
    Bsp_Beer_Init();
    Timer1_UpdateIT_Init(); //定时器初始化
}

/**
  * @brief  主函数
  * @param  无
  * @note  	无
  * @retval 无
  */
int main(void)
{
    HAL_Init(); //HAL库初始化

    Module_Init(); //模块初始化

    time_tick_set(0);

    LOG_Printf("remote ctrl start...\r\n");

    while (1)
    {
      Remote_CTL_Machine();
    }
}

/*********************************************END OF FILE**********************/

