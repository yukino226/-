#ifndef __APP_HIGH_CTL_H
#define __APP_HIGH_CTL_H

#include <stdint.h>

typedef struct _stPid_High_Ctl
{
    float f32Target;
    float f32Current;
    float f32Error;
    float f32LastError;
    float f32Kp;
    float f32Ki;
    float f32Kd;
    float f32Pout;
    float f32Iout;
    float f32Dout;
    float f32Output;
}stPid_High_Ctl;
void App_High_Estimate_Reset(float f32CurrentAcc, float baro_h);
void App_High_Estimate(float current_acc_z, float baro_h, float baro_v);
void App_High_Hold(void);
void App_High_SetHighTarget(float target);
void App_High_HighRate_Ctl(float target_rate);
void App_High_ClerarIntegrator(void);
float App_High_Estimate_GetVelocity(void);
float App_High_Estimate_GetPosition(void);
float App_High_GetHighTarget(void);
#endif




