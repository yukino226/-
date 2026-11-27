#ifndef _APP_H_
#define _APP_H_
#include "py32f0xx_hal.h"
#include "usart/bsp_usart_debug.h"

typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

#define Debug_Mode  1

//任务超时时间
#define APP_TIMEOUT 100 //100MS
#define KEY_APP_TIMEOUT 10  //MS
#define RF_APP_TIMEOUT  10  //MS

// 定义状态
typedef enum {
    STATE_IDLE,
    STATE_START,
    STATE_RUNNING,
    STATE_STOPPED
} machine_state_t;

// 定义任务
typedef enum {
    APP_TASK_NULL,
    APP_TASK_T_RF, //射频通讯
    APP_TASK_T_KEY, //按键
    APP_TASK_T_LCD, //屏幕
} app_task_type_t;

typedef enum {
    APP_TASK_NORMAL,
    APP_TASK_ERR,
} app_task_state_t;

typedef struct 
{
    app_task_type_t type; /* 任务类型 */
} app_task_msg_t;


//函数声明
void time_tick_increase(void);
uint32_t time_tick_get(void);
void time_tick_set(uint32_t time_tick);
void Remote_CTL_Machine(void);

TestStatus Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);

#endif /* __MAIN_H */
