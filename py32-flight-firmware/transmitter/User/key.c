/**
 * @file key.c
 * @author zhou.jie
 * @brief  键盘处理
 * @version 0.1
 * @date 2024-11-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "key.h"
#include "app.h"
#include "adc/bsp_adc.h"

#include <string.h>
#include <stdlib.h>

static KEY_MSG key_msg;

static double convertRange(double value, double oldMin, double oldMax, double newMin, double newMax);

uint8_t *Get_Key_Statue_Data(void)
{ 
        return key_msg.key_sta;
}

/**
 * @brief 键盘初始化
 *
 */
void key_init(void)
{
        Bsp_ADC_Init();

        key_msg.key_sta[KEY_FUN_LEFT_ROCKER_L_R] = ROCKER_INIT_DEF;
        key_msg.key_sta[KEY_FUN_LEFT_ROCKER_U_D] = ROCKER_INIT_DEF;
        key_msg.key_sta[KEY_FUN_RIGHT_ROCKER_U_D] = ROCKER_INIT_DEF;
        key_msg.key_sta[KEY_FUN_RIGHT_ROCKER_L_R] = ROCKER_INIT_DEF;
        key_msg.key_sta[KEY_FUN_COMBINATION_KEY_1] = RELEASED;
        key_msg.key_sta[KEY_FUN_COMBINATION_KEY_2] = RELEASED;
}
/**
 * @brief 键盘扫描
 *
 */
void key_scan(void)
{
        uint8_t i = 0;
        uint16_t combination_key_sta = 0;

        Get_ADC_val(key_msg.key_adc_sta, ADC_CH_USE);

        for (i = 0; i < ADC_CH_USE; i++)
        {
                key_msg.key_adc_sta[i] = (uint16_t)((3.3 / 4096) * key_msg.key_adc_sta[i] * 1000);
        }

#if (Debug_Mode)
        // printf("*****ADC DEBUG*****\r\n");
        // printf("LEFT_ROCKER_L_R= %d\r\n", key_msg.key_adc_sta[ADC_FUN_LEFT_ROCKER_L_R]);
        // printf("LEFT_ROCKER_U_D= %d\r\n", key_msg.key_adc_sta[ADC_FUN_LEFT_ROCKER_U_D]);
        // printf("RIGHT_ROCKER_U_D= %d\r\n", key_msg.key_adc_sta[ADC_FUN_RIGHT_ROCKER_U_D]);
        // printf("RIGHT_ROCKER_L_R= %d\r\n", key_msg.key_adc_sta[ADC_FUN_RIGHT_ROCKER_L_R]);
        // printf("COMBINATION_KEY= %d\r\n", key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY]);
        // printf("*****END*****\r\n");
#endif
        key_msg.key_sta[KEY_FUN_LEFT_ROCKER_L_R] = (uint16_t)convertRange(key_msg.key_adc_sta[ADC_FUN_LEFT_ROCKER_L_R], LEFT_ROCKER_REF_VOL_R_LIMIT, LEFT_ROCKER_REF_VOL_L_LIMIT, ROCKER_LEVEL_MIN, ROCKER_LEVEL_MAX);
        key_msg.key_sta[KEY_FUN_LEFT_ROCKER_U_D] =  (uint16_t)convertRange(key_msg.key_adc_sta[ADC_FUN_LEFT_ROCKER_U_D], LEFT_ROCKER_REF_VOL_D_LIMIT, LEFT_ROCKER_REF_VOL_U_LIMIT, ROCKER_LEVEL_MIN, ROCKER_LEVEL_MAX);
        key_msg.key_sta[KEY_FUN_RIGHT_ROCKER_L_R] =  (uint16_t)convertRange(key_msg.key_adc_sta[ADC_FUN_RIGHT_ROCKER_L_R], RIGHT_ROCKER_REF_VOL_R_LIMIT, RIGHT_ROCKER_REF_VOL_L_LIMIT, ROCKER_LEVEL_MIN, ROCKER_LEVEL_MAX);
        key_msg.key_sta[KEY_FUN_RIGHT_ROCKER_U_D] = (uint16_t)convertRange(key_msg.key_adc_sta[ADC_FUN_RIGHT_ROCKER_U_D], RIGHT_ROCKER_REF_VOL_D_LIMIT, RIGHT_ROCKER_REF_VOL_U_LIMIT, ROCKER_LEVEL_MIN, ROCKER_LEVEL_MAX);

        if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_L_1_L_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_L_1_L;
                // printf("COMBINATION_KEY_L_1_L Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_L_1_R_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_L_1_R;
                // printf("COMBINATION_KEY_L_1_R Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_L_2_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_L_2;
                // printf("COMBINATION_KEY_L_2 Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_L_3_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_L_3;
                // printf("COMBINATION_KEY_L_3 Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_L_4_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_L_4;
                // printf("COMBINATION_KEY_L_4 Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_R_1_L_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_R_1_L;
                // printf("COMBINATION_KEY_R_1_L Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_R_1_R_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_R_1_R;
                // printf("COMBINATION_KEY_R_1_R Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_R_2_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_R_2;
                // printf("COMBINATION_KEY_R_2 Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_R_3_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_R_3;
                // printf("COMBINATION_KEY_R_3 Pressed");
        }
        else if (abs(key_msg.key_adc_sta[ADC_FUN_COMBINATION_KEY] - COMBINATION_KEY_R_4_REF_VOL) <= COMBINATION_KEY_REF_VOL_OFFSET)
        {
                combination_key_sta = PRESSED << COMBINATION_KEY_FUN_KEY_R_4;
                // printf("COMBINATION_KEY_R_4 Pressed");
        }
        else
        {
                combination_key_sta = RELEASED;
        }

        key_msg.key_sta[KEY_FUN_COMBINATION_KEY_2] = (uint8_t)((combination_key_sta >> 8) & 0x00FF);
        key_msg.key_sta[KEY_FUN_COMBINATION_KEY_1] = (uint8_t)(combination_key_sta & 0x00ff);
#if (Debug_Mode)
        // printf("KEY_FUN_LEFT_ROCKER_L_R : %02d\r\n", key_msg.key_sta[KEY_FUN_LEFT_ROCKER_L_R]);
        // printf("KEY_FUN_LEFT_ROCKER_U_D : %02d\r\n", key_msg.key_sta[KEY_FUN_LEFT_ROCKER_U_D]);
        // printf("KEY_FUN_RIGHT_ROCKER_L_R : %02d\r\n", key_msg.key_sta[KEY_FUN_RIGHT_ROCKER_L_R]);
        // printf("KEY_FUN_RIGHT_ROCKER_U_D : %02d\r\n", key_msg.key_sta[KEY_FUN_RIGHT_ROCKER_U_D]);
        // printf("KEY_FUN_COMBINATION_KEY_1 : 0x%02x\r\n", key_msg.key_sta[KEY_FUN_COMBINATION_KEY_1]);
        // printf("KEY_FUN_COMBINATION_KEY_2 : 0x%02x\r\n", key_msg.key_sta[KEY_FUN_COMBINATION_KEY_2]);
#endif
}
/* @brief 量程转换函数
        double value = 100.0; // 需要转换的原始值
        double oldMin = 0.0;  // 原始量程最小值
        double oldMax = 200.0; // 原始量程最大值
        double newMin = 1.0;   // 新量程最小值
        double newMax = 5.0;   // 新量程最大值
 */
double convertRange(double value, double oldMin, double oldMax, double newMin, double newMax)
{
        return ((newMax - newMin) * (value - oldMin) / (oldMax - oldMin)) + newMin;
}
