#ifndef __BSP_ADC_H
#define __BSP_ADC_H

/* 包含其他头文件 */
#include "py32f0xx_hal.h"

/* 全局变量声明 */

#define ADC_SINGLE 0 //连续扫描
#define ADC_CONTINOUS 1 //单次扫描
#define ADC_MODE ADC_SINGLE

#define KEY_ADC_USE 5
#define VOL_DETECT_ADC_USE 1
#define ADC_CH_USE (KEY_ADC_USE + VOL_DETECT_ADC_USE) // ADC通道使用数量

/* 功能             PIN         CHANNEL
 * 左摇杆上下       PA4         ADC_IN2
 * 左摇杆左/右      PA3         ADC_IN1
 * 低电压检测       PB1         ADC_IN0
 * 右摇杆上/下      PA6         ADC_IN3
 * 右摇杆左/右      PA7         ADC_IN4
 * 组合键           PB0         ADC_IN7
 */
#define LOW_VOL_DETECT ADC_CHANNEL_0
#define LEFT_ROCKER_L_R ADC_CHANNEL_1
#define LEFT_ROCKER_U_D ADC_CHANNEL_2
#define RIGHT_ROCKER_U_D ADC_CHANNEL_3
#define RIGHT_ROCKER_L_R ADC_CHANNEL_4
#define COMBINATION_KEY ADC_CHANNEL_7

typedef enum _ADC_FUNC_MAP
{
    ADC_FUN_LOW_VOL_DETECT = 0x00,
    ADC_FUN_LEFT_ROCKER_L_R,
    ADC_FUN_LEFT_ROCKER_U_D,
    ADC_FUN_RIGHT_ROCKER_U_D,
    ADC_FUN_RIGHT_ROCKER_L_R,
    ADC_FUN_COMBINATION_KEY,
} ADC_FUNC_MAP;

/* 函数声明 */
void Bsp_ADC_Init(void);
void Get_ADC_val(uint16_t *val, uint8_t ch_num);

#endif /* __BSP_ADC_H */
