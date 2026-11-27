#include "stdio.h"
#include "string.h"
#include "hal.h"
#include "commom.h"
#include "App_Ahrs.h"
#include "MadgwickAHRS.h"
#include "App_Plane.h"
#include "Filter_LPF.h"
#include "App_Motor.h"

#define ENABLE_AHRS_ACC_FILTER 1
#define AHRS_ACC_FILTER_LPF2   0
#define AHRS_ACC_FILTER_WIN    1
#define AHRS_ACC_FILTER_SEL    AHRS_ACC_FILTER_LPF2


#define AHRS_SAMPLE_CNT            (500)
#define AHRS_BEAT_DYNAMIC_ENABLE   0

Butter_Parameter Butter_60HZ_Parameter_Acce_ef_Z;
Butter_BufferData Butter_Buffer_SINS_EF_Z;

void App_AHRS_Init(void)
{
    Filter_LPF2_SetCutoffFrequency(250, 12,&Butter_60HZ_Parameter_Acce_ef_Z);
}

uint32_t u32AhsSample = 0;
extern float beta;
void App_AHRS_Update(void)
{
    if(u32AhsSample == AHRS_SAMPLE_CNT)
    {
        u32AhsSample++;
        MadgwickAHR_ClearInt();
        beta = 0.02f;
    }
    else if (u32AhsSample == 0)
    {
        beta = 1.0f;
        u32AhsSample++;
    }
    else if(u32AhsSample < AHRS_SAMPLE_CNT)
    {
        u32AhsSample++;
        beta -= 1.0f/AHRS_SAMPLE_CNT;
    }

    MadgwickAHRSupdateIMU(\
        g_stMpuData.f32Gyro_fx_rad,\
        g_stMpuData.f32Gyro_fy_rad,  \
        g_stMpuData.f32Gyro_fz_rad,  \
        g_stMpuData.f32Accel_x_g,    \
        g_stMpuData.f32Accel_y_g,    \
        g_stMpuData.f32Accel_z_g, 0.004f);


    g_stMpuData.f32Gyro_fz_dgree_lpf = T31 *  g_stMpuData.f32Gyro_fx_dgree + T32 * g_stMpuData.f32Gyro_fy_dgree + T33 * g_stMpuData.f32Gyro_fz_dgree;

    //Filter_LPF1(8, 0.004f,g_stMpuData.f32Gyro_fz_dgree, &g_stMpuData.f32Gyro_fz_dgree_lpf);//g_stMpuData.f32Gyro_fz_dgree
    g_stPlane.f32Yaw_Angle += g_stMpuData.f32Gyro_fz_dgree * 0.004f;
    g_stPlane.f32Yaw_Angle = Angle_Ceiling(g_stPlane.f32Yaw_Angle, 180);  
    g_stPlane.f32Phi   = g_stPlane.f32Yaw_Angle;

    g_stPlane.f32Theta = Theta;
    g_stPlane.f32Gamma = Gamma;

    g_stPlane.f32Acc_ef_z =  T13 * g_stMpuData.f32Accel_fx_g + T23 * g_stMpuData.f32Accel_fy_g + T33 * g_stMpuData.f32Accel_fz_g;
    g_stPlane.f32Acc_ef_fz = Filter_LPF2_Butterworth(g_stPlane.f32Acc_ef_z,\
	&Butter_Buffer_SINS_EF_Z,&Butter_60HZ_Parameter_Acce_ef_Z);
}
