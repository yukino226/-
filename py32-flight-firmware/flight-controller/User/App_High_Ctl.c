#include "stdio.h"
#include "string.h"
#include <math.h>
#include "hal.h"
#include "commom.h"
#include "App_High_Ctl.h"
#include "App_Plane.h"
#include "App_Motor.h"

#define HIGH_CTL_DT     0.01f

/* High */
stPid_High_Ctl g_stPid_High = 
{
    .f32Kp = 1.2f,
    .f32Ki = 0.0f,
    .f32Kd = 0.0f,
};

stPid_High_Ctl g_stPid_HighRate = 
{
    .f32Kp = 1000.0f,
    .f32Ki = 100.0f,
    .f32Kd = 10.0f,
};

#define TIME_CONTANST_ZER      (1.5f)
#define K_ACC_ZER 	           (1.5f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	           (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_POS_ZER              (3.0f / TIME_CONTANST_ZER)
#define HIGH_DELAY_CNT         5 //20ms


#define HIGH_POS_BUF_NUM       6
typedef struct _stHigh_Estimate
{
    float f32Pos_Baro;
    float f32Vel_Baro;
    float f32Pos;
    float f32PrePos;
    float f32Vel;
    float f32PreVel;
    float f32Acc;
    float f32LastAcc;
    float f32Pos_History[HIGH_POS_BUF_NUM];
    float f32AltDealt;
    float f32AltDealt_Filter;
    float f32AltDealt_Filter_Fabs;
    float f32AltGain;
    float f32VelDealt_Filter;
    float f32VelDealt_Filter_Fabs;
    float f32VelGain;
    float f32VelDealt;
    float f32Pos_Correction;
    float f32Vel_Correction;
    float f32Acc_correction;
}stHigh_Estimate;

stHigh_Estimate g_stHigh_Estimate;

void App_High_Estimate_Reset(float f32CurrentAcc, float baro_h)
{
    g_stHigh_Estimate.f32Pos_Baro = baro_h;
    g_stHigh_Estimate.f32Pos = baro_h;
    g_stHigh_Estimate.f32PrePos = baro_h;
    g_stHigh_Estimate.f32Vel = 0.0f;
    g_stHigh_Estimate.f32PreVel = 0.0f;
    g_stHigh_Estimate.f32Acc = f32CurrentAcc;
    g_stHigh_Estimate.f32LastAcc = f32CurrentAcc;
    for(uint8_t i = 0; i < HIGH_POS_BUF_NUM; i++)
    {
        g_stHigh_Estimate.f32Pos_History[i] = baro_h;
    }
    // g_stHigh_Estimate.f32Pos_Correction = 0;
    // g_stHigh_Estimate.f32Vel_Correction = 0;
    // g_stHigh_Estimate.f32Acc_correction = 0;
}

// void App_High_Estimate(float f32CurrentAcc, float baro_h,float baro_v)
// {
//     if((g_stPlane.take_off_process == 1) || (g_stMotor.eMotorSta == ENUM_MOTOR_PRESTART))
//     {
//         if(baro_h >= 0)
//         {
//             g_stHigh_Estimate.f32Pos_Baro = baro_h;
//         }
//         else
//         {
//             g_stHigh_Estimate.f32Pos_Baro = 0;
//         }
//     }
//     else
//     {
//         g_stHigh_Estimate.f32Pos_Baro = baro_h;
//     }


//     g_stHigh_Estimate.f32AltDealt=g_stHigh_Estimate.f32Pos_Baro-g_stHigh_Estimate.f32Pos_History[HIGH_DELAY_CNT];//气压计(超声波)与SINS估计量的差，单位cm
//     #define HIGH_ERR_FILTER  0.99f
//     g_stHigh_Estimate.f32AltDealt_Filter = g_stHigh_Estimate.f32AltDealt_Filter * HIGH_ERR_FILTER + (1 - HIGH_ERR_FILTER) * g_stHigh_Estimate.f32AltDealt;
//     g_stHigh_Estimate.f32AltDealt_Filter_Fabs = g_stHigh_Estimate.f32AltDealt_Filter_Fabs * HIGH_ERR_FILTER + (1 - HIGH_ERR_FILTER) * fabs(g_stHigh_Estimate.f32AltDealt);
//     g_stHigh_Estimate.f32AltGain = fabs(g_stHigh_Estimate.f32AltDealt_Filter/(g_stHigh_Estimate.f32AltDealt_Filter_Fabs + 1));
    
//     g_stHigh_Estimate.f32Pos_Correction +=  K_POS_ZER * g_stHigh_Estimate.f32AltDealt * HIGH_CTL_DT;
//     g_stHigh_Estimate.f32Vel_Correction +=  K_VEL_ZER * g_stHigh_Estimate.f32AltDealt * HIGH_CTL_DT;
//     g_stHigh_Estimate.f32Acc_correction +=  K_ACC_ZER * g_stHigh_Estimate.f32AltDealt * HIGH_CTL_DT;

//     g_stHigh_Estimate.f32LastAcc = g_stHigh_Estimate.f32Acc;
//     g_stHigh_Estimate.f32Acc = f32CurrentAcc  + g_stHigh_Estimate.f32Acc_correction;

//     g_stHigh_Estimate.f32VelDealt = (g_stHigh_Estimate.f32LastAcc + g_stHigh_Estimate.f32Acc) * HIGH_CTL_DT/2.0f;
//     g_stHigh_Estimate.f32PrePos += (g_stHigh_Estimate.f32Vel + 0.5f * g_stHigh_Estimate.f32VelDealt) * HIGH_CTL_DT;

//     g_stHigh_Estimate.f32Pos = g_stHigh_Estimate.f32PrePos + g_stHigh_Estimate.f32Pos_Correction;
//     g_stHigh_Estimate.f32PreVel += g_stHigh_Estimate.f32VelDealt;
//     g_stHigh_Estimate.f32Vel = g_stHigh_Estimate.f32PreVel + g_stHigh_Estimate.f32Vel_Correction;

//     for(uint8_t Cnt=HIGH_POS_BUF_NUM;Cnt>0;Cnt--)//20ms滑动一次
//     {
//         g_stHigh_Estimate.f32Pos_History[Cnt]=g_stHigh_Estimate.f32Pos_History[Cnt-1];
//     }
//     g_stHigh_Estimate.f32Pos_History[0]=g_stHigh_Estimate.f32Pos;  
// }

void App_High_Estimate(float f32CurrentAcc, float baro_h, float baro_v)
{
    if((g_stPlane.take_off_process == 1) || (g_stMotor.eMotorSta == ENUM_MOTOR_PRESTART))
    {
        if(baro_h >= 0)
        {
            g_stHigh_Estimate.f32Pos_Baro = baro_h;
            g_stHigh_Estimate.f32Vel_Baro = baro_v;
        }
        else
        {
            g_stHigh_Estimate.f32Pos_Baro = 0;
            g_stHigh_Estimate.f32Vel_Baro = 0;
        }
    }
    else
    {
        g_stHigh_Estimate.f32Pos_Baro = baro_h;
        g_stHigh_Estimate.f32Vel_Baro = baro_v;
    }
    g_stHigh_Estimate.f32Vel_Baro = constrain_float(g_stHigh_Estimate.f32Vel_Baro, -400.0f, 400.0f);
    
    g_stHigh_Estimate.f32AltDealt = g_stHigh_Estimate.f32Pos_Baro - g_stHigh_Estimate.f32Pos;
    g_stHigh_Estimate.f32AltDealt = constrain_float(g_stHigh_Estimate.f32AltDealt, -40.0f, 40.0f);
    g_stHigh_Estimate.f32VelDealt = g_stHigh_Estimate.f32Vel_Baro - g_stHigh_Estimate.f32Vel;
    g_stHigh_Estimate.f32VelDealt = constrain_float(g_stHigh_Estimate.f32VelDealt, -400.0f, 400.0f);

    #define HIGH_ERR_FILTER  0.99f
    g_stHigh_Estimate.f32AltDealt_Filter = g_stHigh_Estimate.f32AltDealt_Filter * HIGH_ERR_FILTER + (1 - HIGH_ERR_FILTER) * g_stHigh_Estimate.f32AltDealt;
    g_stHigh_Estimate.f32AltDealt_Filter_Fabs = g_stHigh_Estimate.f32AltDealt_Filter_Fabs * HIGH_ERR_FILTER + (1 - HIGH_ERR_FILTER) * fabs(g_stHigh_Estimate.f32AltDealt);
    g_stHigh_Estimate.f32AltGain = fabs(g_stHigh_Estimate.f32AltDealt_Filter/(g_stHigh_Estimate.f32AltDealt_Filter_Fabs + 1));
    
    g_stHigh_Estimate.f32VelDealt_Filter = g_stHigh_Estimate.f32VelDealt_Filter * HIGH_ERR_FILTER + (1 - HIGH_ERR_FILTER) * g_stHigh_Estimate.f32VelDealt;
    g_stHigh_Estimate.f32VelDealt_Filter_Fabs = g_stHigh_Estimate.f32VelDealt_Filter_Fabs * HIGH_ERR_FILTER + (1 - HIGH_ERR_FILTER) * fabs(g_stHigh_Estimate.f32VelDealt);
    g_stHigh_Estimate.f32VelGain = fabs(g_stHigh_Estimate.f32VelDealt_Filter/(g_stHigh_Estimate.f32VelDealt_Filter_Fabs + 1));    

    #define HIGH_ACC_COR   0.1f
    g_stHigh_Estimate.f32Acc_correction +=  HIGH_ACC_COR * g_stHigh_Estimate.f32VelGain * g_stHigh_Estimate.f32VelDealt * HIGH_CTL_DT;
    g_stHigh_Estimate.f32Acc = f32CurrentAcc  + g_stHigh_Estimate.f32Acc_correction;

    #define HIGH_VEL_COR   0.015f
    g_stHigh_Estimate.f32Vel += (g_stHigh_Estimate.f32Acc * HIGH_CTL_DT)\
                             + (g_stHigh_Estimate.f32AltGain * g_stHigh_Estimate.f32AltDealt *  HIGH_VEL_COR);

    g_stHigh_Estimate.f32Pos += (g_stHigh_Estimate.f32Vel * HIGH_CTL_DT);
    g_stHigh_Estimate.f32Pos = g_stHigh_Estimate.f32Pos * 0.99f + g_stHigh_Estimate.f32Pos_Baro * (1 - 0.99f);
}


float App_High_Estimate_GetVelocity(void)
{
    return  g_stHigh_Estimate.f32Vel;
}

float App_High_Estimate_GetPosition(void)
{
    return  g_stHigh_Estimate.f32Pos;
}

float App_High_GetHighTarget(void)
{
    return g_stPid_High.f32Target;
}

void App_High_Hold(void)
{
    float f32Desired_climb_rate = 0.0f; 
    g_stPid_High.f32Error = g_stPid_High.f32Target - g_stPlane.f32Postion_z;
    f32Desired_climb_rate = constrain_float(g_stPid_High.f32Error,-3.0f, 3.0f);
    g_stPid_High.f32Pout = f32Desired_climb_rate * g_stPid_High.f32Kp;
    g_stPid_High.f32Iout += g_stPid_High.f32Ki * g_stPid_High.f32Pout * HIGH_CTL_DT;
    g_stPid_High.f32Iout = constrain_float(g_stPid_High.f32Iout, -15.0, 15.0);
    g_stPid_High.f32Dout = g_stPid_High.f32Kd * (g_stPid_High.f32Pout - g_stPid_High.f32LastError)/HIGH_CTL_DT;
    g_stPid_High.f32Dout = constrain_float(g_stPid_High.f32Dout, -20.0, 20.0);
    g_stPid_High.f32LastError = g_stPid_High.f32Pout;
    g_stPid_High.f32Output = g_stPid_High.f32Pout + g_stPid_High.f32Iout + g_stPid_High.f32Dout;
	App_High_HighRate_Ctl(g_stPid_High.f32Output);
}


void App_High_HighRate_Ctl(float target_rate)
{
    g_stPid_HighRate.f32Error = target_rate - g_stPlane.f32Velocity_z;
    g_stPid_HighRate.f32Pout = g_stPid_HighRate.f32Error * g_stPid_HighRate.f32Kp;
    g_stPid_HighRate.f32Pout = constrain_float(g_stPid_HighRate.f32Pout, -200.0f, 200.0f);
    g_stPid_HighRate.f32Iout += g_stPid_HighRate.f32Error * g_stPid_HighRate.f32Ki * HIGH_CTL_DT;
    g_stPid_HighRate.f32Iout = constrain_float(g_stPid_HighRate.f32Iout, -200.0f, 200.0f);
    g_stPid_HighRate.f32Dout = (g_stPid_HighRate.f32Error - g_stPid_HighRate.f32LastError) * g_stPid_HighRate.f32Kd/HIGH_CTL_DT;
    g_stPid_HighRate.f32Dout = constrain_float(g_stPid_HighRate.f32Dout, -50.0f, 50.0f);
    g_stPid_HighRate.f32LastError = g_stPid_HighRate.f32Error;
    g_stPid_HighRate.f32Output = g_stPid_HighRate.f32Pout + g_stPid_HighRate.f32Iout + g_stPid_HighRate.f32Dout;
    g_stPlane.f32Thr_HighHold = (int16_t)constrain_float(g_stPid_HighRate.f32Output, -400.0f, 400.0f);
}

void App_High_SetHighTarget(float target)
{
    g_stPid_High.f32Target = target;
}

void App_High_ClerarIntegrator(void)
{
    g_stPid_HighRate.f32Iout = 0;
    g_stPid_HighRate.f32LastError = 0;
    g_stPid_High.f32Iout = 0;
    g_stPid_High.f32LastError = 0;
}
