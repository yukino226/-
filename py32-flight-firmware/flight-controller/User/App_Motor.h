#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include <stdint.h>


#define MOTOR_NUM        4
#define MOTOR_LEFT_UP    3
#define MOTOR_LEFT_DOWN  2
#define MOTOR_RIGHT_UP   1
#define MOTOR_RIGHT_DOWN 0

typedef enum _eMotorState{
    ENUM_MOTOR_STOP,
    ENUM_MOTOR_PRESTART,
    ENUM_MOTOR_RUN,
}eMotorState;

typedef struct _stMotor{
    eMotorState eMotorSta;
    uint16_t u16MotorOut[MOTOR_NUM];
}stMotor;

void App_Motor_Init(void);
void App_Motor_Run(void);
extern stMotor g_stMotor;
void App_Motor_Set(uint8_t index, uint16_t thr);
void App_Motor_StartCheck(void);
void App_Motor_StopCheck(void);
void App_Motor_Stop(void);

#endif




