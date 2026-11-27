#ifndef __BSP_GPIO_LED_H
#define __BSP_GPIO_LED_H

/* 包含其他头文件 */
#include "py32f0xx_hal.h"

 
/* 宏定义 */
// GPS LED
#define		LED1_GPIO_CLK_ENABLE		__HAL_RCC_GPIOA_CLK_ENABLE
#define		LED1_GPIO_PORT				GPIOA
#define		LED1_GPIO_PIN				GPIO_PIN_0
//返航 LED
#define		LED2_GPIO_CLK_ENABLE		__HAL_RCC_GPIOA_CLK_ENABLE
#define		LED2_GPIO_PORT				GPIOA
#define		LED2_GPIO_PIN				GPIO_PIN_1
//对频 LED
#define		LED3_GPIO_CLK_ENABLE		__HAL_RCC_GPIOA_CLK_ENABLE
#define		LED3_GPIO_PORT				GPIOA
#define		LED3_GPIO_PIN				GPIO_PIN_2

#define		LED_ON 		0
#define		LED_OFF 	1

#define 	LED1(x)   					x ? \
                                                        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET): \
                                                        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \

#define 	LED1_TOGGLE()     HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN)

#define 	LED2(x)   					x ? \
                                                        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_SET): \
                                                        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_RESET); \

#define 	LED2_TOGGLE()     HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN)


#define 	LED3(x)   					x ? \
                                                        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_PIN_SET): \
                                                        HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_PIN_RESET); \

#define 	LED3_TOGGLE()     HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_GPIO_PIN)

/* 全局变量声明 */


/* 函数声明 */
void Bsp_Led_Init(void);




#endif /* __BSP_GPIO_LED_H */

