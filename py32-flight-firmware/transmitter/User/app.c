/**
 * @file app.c
 * @author zhou.jie
 * @brief
 * @version 0.1
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "app.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "led/bsp_gpio_led.h"
#include "usart/bsp_usart_debug.h"
#include "2_4G/bsp_rf_xn297.H"
#include "systick/bsp_systick_delay.h"
#include "tim1/bsp_tim1.h"
#include "beer/bsp_beer.h"
#include "key.h"

uint32_t time_tick_ms = 0;

TestStatus Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
machine_state_t current_state = STATE_START;

// 本地函数声明
static void Delay(__IO uint32_t nCount);
static int get_random(uint32_t ramd_num, int min, int max);

void time_tick_increase(void)
{
    time_tick_ms += 2;
}

uint32_t time_tick_get(void)
{
    return time_tick_ms;
}

void time_tick_set(uint32_t time_tick)
{
    time_tick_ms = time_tick;
}
/**
 * @brief 2.4G射频服务函数
 * @param timeout
 * @return app_task_state_t
 */
app_task_state_t rf_app_func(uint32_t timeout)
{
    app_task_state_t err_code = APP_TASK_NORMAL;
    uint8_t *key_data = NULL;
    uint8_t ucPayload[PAYLOAD_WIDTH] = {0};
    uint8_t i = 0;

    key_data = Get_Key_Statue_Data();

    ucPayload[0] = 0xA5;
    ucPayload[1] = key_data[0];
    ucPayload[2] = key_data[1];
    ucPayload[3] = key_data[2];
    ucPayload[4] = key_data[3];
    ucPayload[5] = key_data[4];
    ucPayload[6] = key_data[5];

    for (i = 0; i < RF_CH_TABLE_MAX; i++)
    {
        ucPayload[7] = i;
        RF_SendData_Multiple_CH(ucPayload, PAYLOAD_WIDTH, i);
    }

    return err_code;
}
/**
 * @brief 按键服务函数
 * @param timeout
 * @return app_task_state_t
 */
app_task_state_t key_app_func(uint32_t timeout)
{
    app_task_state_t err_code = APP_TASK_NORMAL;
    KEY_MSG key_last_msg;
    uint8_t *key_data = NULL;
    uint8_t *key_last_data = NULL;

    key_last_data = Get_Key_Statue_Data();
    memcpy(key_last_msg.key_sta, key_last_data, sizeof(key_last_msg.key_sta));

    key_scan();

    key_data = Get_Key_Statue_Data();

    if (*(key_data + KEY_FUN_COMBINATION_KEY_1) != RELEASED || *(key_data + KEY_FUN_COMBINATION_KEY_2) != RELEASED)
    {
        // 有按键被按下
        if (Buffercmp(key_data + 4, &key_last_msg.key_sta[4], 2) == FAILED)
        {
            Beer_Ctlr(BEER_TOGGLE);
        }
    }

    return err_code;
}
/**
 * @brief 系统自检
 * @param timeout
 * @return app_task_state_t
 */
app_task_state_t sys_check_app_func(uint32_t timeout)
{
    app_task_state_t err_code = APP_TASK_NORMAL;

    if (RF_Check() == FAILED)
    {
        err_code = APP_TASK_ERR;
#if (Debug_Mode)
        LOG_Printf("RF Err!\r\n");
#endif
        goto exit;
    }
exit:
    return err_code;
}
/**
 * @brief app 任务
 *
 */
app_task_state_t app_task(void)
{
    app_task_state_t err_code = APP_TASK_NORMAL;
    uint32_t start_time = time_tick_get();
    app_task_msg_t task_msg;

    task_msg.type = APP_TASK_T_KEY;

    RF_TxMode();

    while (1)
    {
        if (time_tick_get() - start_time < APP_TIMEOUT)
        {
            switch (task_msg.type)
            {
            case APP_TASK_T_KEY:
                err_code = key_app_func(KEY_APP_TIMEOUT);
                if (err_code == APP_TASK_NORMAL)
                {
                    task_msg.type = APP_TASK_T_RF;
                }
            case APP_TASK_T_RF:
                err_code = rf_app_func(RF_APP_TIMEOUT);
                if (err_code == APP_TASK_NORMAL)
                {
                    task_msg.type = APP_TASK_T_KEY;
                }
                break;
            default:
                break;
            }
        }
        else
        {
            // 任务超时
            LED1_TOGGLE();
            start_time = time_tick_get();
        }
    }

    return err_code;
}

/**
 * @brief 系统状态机函数
 *
 */
void Remote_CTL_Machine(void)
{
    app_task_state_t err_code = APP_TASK_NORMAL;

    // 根据当前状态进行逻辑处理
    switch (current_state)
    {
    case STATE_START:
        // 启动状态
        if (sys_check_app_func(APP_TIMEOUT) == APP_TASK_NORMAL)
        {
            LOG_Printf("sys check passed.\r\n");
            printf("mcu uid1: %d\r\n", HAL_GetUIDw0());
            printf("mcu uid2: %d\r\n", HAL_GetUIDw1());
            printf("mcu uid3: %d\r\n", HAL_GetUIDw2());
            printf("devid: %d\r\n", HAL_GetDEVID());

            printf("ch1: %d\r\n", get_random(HAL_GetUIDw2(), 0, 25));
            printf("ch2: %d\r\n", get_random(HAL_GetUIDw2(), 26, 50));
            printf("ch3: %d\r\n", get_random(HAL_GetUIDw2(), 51, 75));
            Beer_Ctlr(BEER_TOGGLE);
        }
        current_state = STATE_RUNNING;
        break;
    case STATE_IDLE:
        // 处于空闲状态的逻辑
        // 转换到运行状态
        current_state = STATE_RUNNING;
        break;
    case STATE_RUNNING:
        // 处于运行状态的逻辑
        // 根据条件转换到停止状态
        app_task();
        if (err_code == APP_TASK_ERR)
        {
            current_state = STATE_STOPPED;
        }
        break;
    case STATE_STOPPED:
        // 处于停止状态的逻辑
        // 可以选择重新进入空闲状态或其他状态
        current_state = STATE_IDLE;
        break;
    default:
        // 处理未知状态
        break;
    }
}

/*
 * 函数名：Buffercmp
 * 描述  ：比较两个缓冲区中的数据是否相等
 * 输入  ：-pBuffer1     src缓冲区指针
 *         -pBuffer2     dst缓冲区指针
 *         -BufferLength 缓冲区长度
 * 输出  ：无
 * 返回  ：-PASSED pBuffer1 等于   pBuffer2
 *         -FAILED pBuffer1 不同于 pBuffer2
 */
TestStatus Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }
    return PASSED;
}

static void Delay(__IO uint32_t nCount)
{
    for (; nCount != 0; nCount--)
        ;
}

// 获取随机数
static int get_random(uint32_t ramd_num, int min, int max)
{
    srand(ramd_num); // 使用当前时间作为随机数种子

    return min + rand() % (max - min + 1);
}
/*********************************************END OF FILE**********************/
