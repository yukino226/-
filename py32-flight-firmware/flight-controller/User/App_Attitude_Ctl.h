#ifndef __APP_ATTITUDE_CTL_H
#define __APP_ATTITUDE_CTL_H

#include <stdint.h>

typedef struct _stPid_Ctl
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
}stPid_Ctl;

void App_Attitude_ClearIntegrator(void);

void App_Attitude_GammaClearIntegrator(void);
void App_Attitude_SetGammaTarget(float f32Target);
void App_Attitude_GammaControl(void);

void App_Attitude_ThetaControl(void);
void App_Attitude_SetThetaTarget(float f32Target);
void App_Attitude_ThetaClearIntegrator(void);

void App_Attitude_YawClearIntegrator(void);
void App_Attitude_YawControl(void);
void App_Attitude_SetYawTarget(float f32Target);
void App_Plane_Yaw_RateControl(float rate);
void App_Plane_Yaw_AngleControl(void);

#endif




