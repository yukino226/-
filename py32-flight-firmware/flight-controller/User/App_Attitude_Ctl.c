#include "stdio.h"
#include "string.h"
#include "hal.h"
#include "commom.h"
#include "App_Attitude_Ctl.h"
#include "App_Plane.h"
#include <math.h>

#define ATTITUDE_CTL_DT 0.004f

/* Theta*/
stPid_Ctl g_stPid_Angle_Theta = 
{
    .f32Kp = 8.0f,
    .f32Ki = 0.0f,
    .f32Kd = 0.0f,
};

stPid_Ctl g_stPid_AngleRate_Theta = 
{
    .f32Kp = 0.4f,
    .f32Ki = 0.7f,
    .f32Kd = 50.0f,
};


/* Gamma*/

stPid_Ctl g_stPid_Angle_Gamma = 
{
    .f32Kp = 8.0f,
    .f32Ki = 0.0f,
    .f32Kd = 0.0f,
};

stPid_Ctl g_stPid_AngleRate_Gamma = 
{
    .f32Kp = 0.4f,
    .f32Ki = 0.7f,
    .f32Kd = 50.0f,
};

/* Yaw */
stPid_Ctl g_stPid_Angle_Yaw = 
{
    .f32Kp = 10.0f,
    .f32Ki = 0.0f,
    .f32Kd = 0.0f,
};

stPid_Ctl g_stPid_AngleRate_Yaw = 
{
    .f32Kp = 0.8f,
    .f32Ki = 0.5f,
    .f32Kd = 20.0f,
};

void App_Attitude_ClearIntegrator(void)
{
    App_Attitude_ThetaClearIntegrator();
    App_Attitude_GammaClearIntegrator();
    App_Attitude_YawClearIntegrator();
}


/*Function*/
void App_Attitude_SetThetaTarget(float f32Target)
{
    g_stPid_Angle_Theta.f32Target = f32Target;
}

void App_Attitude_ThetaClearIntegrator(void)
{
    g_stPid_Angle_Theta.f32Iout = 0;
    g_stPid_Angle_Theta.f32LastError = 0;
    g_stPid_AngleRate_Theta.f32Iout = 0;
    g_stPid_AngleRate_Theta.f32LastError = 0;
}

void App_Attitude_ThetaControl(void)
{
    g_stPid_Angle_Theta.f32Error = g_stPid_Angle_Theta.f32Target - g_stPlane.f32Theta;
    g_stPid_Angle_Theta.f32Error = Angle_Ceiling(g_stPid_Angle_Theta.f32Error, 100.0f);
    g_stPid_Angle_Theta.f32Pout = g_stPid_Angle_Theta.f32Error * g_stPid_Angle_Theta.f32Kp;
    g_stPid_Angle_Theta.f32Pout = Float_Ceiling(g_stPid_Angle_Theta.f32Pout, 150.0f);
    g_stPid_Angle_Theta.f32Iout += g_stPid_Angle_Theta.f32Error * g_stPid_Angle_Theta.f32Ki * ATTITUDE_CTL_DT;
    g_stPid_Angle_Theta.f32Iout = Float_Ceiling(g_stPid_Angle_Theta.f32Iout, 20.0f);
    g_stPid_Angle_Theta.f32Dout = (g_stPid_Angle_Theta.f32Error - g_stPid_Angle_Theta.f32LastError)  * g_stPid_Angle_Theta.f32Kd;
    g_stPid_Angle_Theta.f32Dout = Float_Ceiling(g_stPid_Angle_Theta.f32Dout, 150.0f);
    g_stPid_Angle_Theta.f32LastError = g_stPid_Angle_Theta.f32Error;
    g_stPid_Angle_Theta.f32Output = g_stPid_Angle_Theta.f32Pout + g_stPid_Angle_Theta.f32Iout + g_stPid_Angle_Theta.f32Dout;
    g_stPid_Angle_Theta.f32Output = Float_Ceiling(g_stPid_Angle_Theta.f32Output, 75.0f);
    

    g_stPid_AngleRate_Theta.f32Target = g_stPid_Angle_Theta.f32Output;
    g_stPid_AngleRate_Theta.f32Error = g_stPid_AngleRate_Theta.f32Target - g_stMpuData.f32Gyro_fy_dgree;
    g_stPid_AngleRate_Theta.f32Error = Float_Ceiling(g_stPid_AngleRate_Theta.f32Error, 100.0f);
    g_stPid_AngleRate_Theta.f32Pout = g_stPid_AngleRate_Theta.f32Error * g_stPid_AngleRate_Theta.f32Kp;
    g_stPid_AngleRate_Theta.f32Pout *= fabsf(g_stPid_AngleRate_Theta.f32Pout);

    g_stPid_AngleRate_Theta.f32Pout = Float_Ceiling(g_stPid_AngleRate_Theta.f32Pout, 300.0f);
    g_stPid_AngleRate_Theta.f32Iout += g_stPid_AngleRate_Theta.f32Error * g_stPid_AngleRate_Theta.f32Ki * ATTITUDE_CTL_DT;
    g_stPid_AngleRate_Theta.f32Iout = Float_Ceiling(g_stPid_AngleRate_Theta.f32Iout, 100.0f);


    g_stPid_AngleRate_Theta.f32Dout = (g_stPid_AngleRate_Theta.f32Error - g_stPid_AngleRate_Theta.f32LastError) * g_stPid_AngleRate_Theta.f32Kd;
    g_stPid_AngleRate_Theta.f32Dout = Float_Ceiling(g_stPid_AngleRate_Theta.f32Dout, 150.0f);

    g_stPid_AngleRate_Theta.f32LastError = g_stPid_AngleRate_Theta.f32Error;

    g_stPid_AngleRate_Theta.f32Output = g_stPid_AngleRate_Theta.f32Pout + g_stPid_AngleRate_Theta.f32Iout + g_stPid_AngleRate_Theta.f32Dout;
    g_stPid_AngleRate_Theta.f32Output = Float_Ceiling(g_stPid_AngleRate_Theta.f32Output, 500.0f);

    g_stPlane.u16Motor_Pitch = (int16_t)g_stPid_AngleRate_Theta.f32Output;
}

void App_Attitude_SetGammaTarget(float f32Target)
{
    g_stPid_Angle_Gamma.f32Target = f32Target;
}

void App_Attitude_GammaClearIntegrator(void)
{
    g_stPid_Angle_Gamma.f32Iout = 0;
    g_stPid_Angle_Gamma.f32LastError = 0;
    g_stPid_AngleRate_Gamma.f32Iout = 0;
    g_stPid_AngleRate_Gamma.f32LastError = 0;
}

void App_Attitude_GammaControl(void)
{
    g_stPid_Angle_Gamma.f32Error = g_stPid_Angle_Gamma.f32Target - g_stPlane.f32Gamma;
    g_stPid_Angle_Gamma.f32Error = Angle_Ceiling(g_stPid_Angle_Gamma.f32Error, 100.0f);
    g_stPid_Angle_Gamma.f32Pout = g_stPid_Angle_Gamma.f32Error * g_stPid_Angle_Gamma.f32Kp;
    g_stPid_Angle_Gamma.f32Pout = Float_Ceiling(g_stPid_Angle_Gamma.f32Pout, 150.0f);
    g_stPid_Angle_Gamma.f32Iout += g_stPid_Angle_Gamma.f32Error * g_stPid_Angle_Gamma.f32Ki * ATTITUDE_CTL_DT;
    g_stPid_Angle_Gamma.f32Iout = Float_Ceiling(g_stPid_Angle_Gamma.f32Iout, 20.0f);
    g_stPid_Angle_Gamma.f32Dout = (g_stPid_Angle_Gamma.f32Error - g_stPid_Angle_Gamma.f32LastError)  * g_stPid_Angle_Gamma.f32Kd;
    g_stPid_Angle_Gamma.f32Dout = Float_Ceiling(g_stPid_Angle_Gamma.f32Dout, 150.0f);
    g_stPid_Angle_Gamma.f32LastError = g_stPid_Angle_Gamma.f32Error;
    g_stPid_Angle_Gamma.f32Output = g_stPid_Angle_Gamma.f32Pout + g_stPid_Angle_Gamma.f32Iout + g_stPid_Angle_Gamma.f32Dout;
    g_stPid_Angle_Gamma.f32Output = Float_Ceiling(g_stPid_Angle_Gamma.f32Output, 75.0f);

    g_stPid_AngleRate_Gamma.f32Target = g_stPid_Angle_Gamma.f32Output;
    g_stPid_AngleRate_Gamma.f32Error = g_stPid_AngleRate_Gamma.f32Target - g_stMpuData.f32Gyro_fx_dgree;
    g_stPid_AngleRate_Gamma.f32Error = Float_Ceiling(g_stPid_AngleRate_Gamma.f32Error, 100.0f);
    g_stPid_AngleRate_Gamma.f32Pout = g_stPid_AngleRate_Gamma.f32Error * g_stPid_AngleRate_Gamma.f32Kp;
    g_stPid_AngleRate_Gamma.f32Pout *= fabsf(g_stPid_AngleRate_Gamma.f32Pout);
    g_stPid_AngleRate_Gamma.f32Pout = Float_Ceiling(g_stPid_AngleRate_Gamma.f32Pout, 300.0f);
    g_stPid_AngleRate_Gamma.f32Iout += g_stPid_AngleRate_Gamma.f32Error * g_stPid_AngleRate_Gamma.f32Ki * ATTITUDE_CTL_DT;
    g_stPid_AngleRate_Gamma.f32Iout = Float_Ceiling(g_stPid_AngleRate_Gamma.f32Iout, 100.0f);
    g_stPid_AngleRate_Gamma.f32Dout = (g_stPid_AngleRate_Gamma.f32Error - g_stPid_AngleRate_Gamma.f32LastError) * g_stPid_AngleRate_Gamma.f32Kd;
    g_stPid_AngleRate_Gamma.f32Dout = Float_Ceiling(g_stPid_AngleRate_Gamma.f32Dout, 150.0f);
    g_stPid_AngleRate_Gamma.f32LastError = g_stPid_AngleRate_Gamma.f32Error;
    g_stPid_AngleRate_Gamma.f32Output = g_stPid_AngleRate_Gamma.f32Pout + g_stPid_AngleRate_Gamma.f32Iout + g_stPid_AngleRate_Gamma.f32Dout;
    g_stPid_AngleRate_Gamma.f32Output = Float_Ceiling(g_stPid_AngleRate_Gamma.f32Output, 500.0f);

    g_stPlane.u16Motor_Roll = (int16_t)g_stPid_AngleRate_Gamma.f32Output;
}

void App_Attitude_SetYawTarget(float f32Target)
{
    g_stPid_Angle_Yaw.f32Target = f32Target;
}

void App_Attitude_SetYawRateTarget(float f32Target)
{
    g_stPid_AngleRate_Yaw.f32Target = f32Target;
}

void App_Attitude_YawClearIntegrator(void)
{
    g_stPid_Angle_Yaw.f32Iout = 0;
    g_stPid_Angle_Yaw.f32LastError = 0;
    g_stPid_AngleRate_Yaw.f32Iout = 0;
    g_stPid_AngleRate_Yaw.f32LastError = 0;
}

void App_Plane_Yaw_AngleControl(void)
{
    g_stPid_Angle_Yaw.f32Error = g_stPid_Angle_Yaw.f32Target - g_stPlane.f32Yaw_Angle;
    g_stPid_Angle_Yaw.f32Error = Angle_Ceiling(g_stPid_Angle_Yaw.f32Error, 180.0f);
    g_stPid_Angle_Yaw.f32Pout = g_stPid_Angle_Yaw.f32Error * g_stPid_Angle_Yaw.f32Kp;
    g_stPid_Angle_Yaw.f32Pout = Float_Ceiling(g_stPid_Angle_Yaw.f32Pout, 150.0f);
    g_stPid_Angle_Yaw.f32Iout += g_stPid_Angle_Yaw.f32Error * g_stPid_Angle_Yaw.f32Ki * ATTITUDE_CTL_DT;
    g_stPid_Angle_Yaw.f32Iout = Float_Ceiling(g_stPid_Angle_Yaw.f32Iout, 20.0f);
    g_stPid_Angle_Yaw.f32Dout = (g_stPid_Angle_Yaw.f32Error - g_stPid_Angle_Yaw.f32LastError)  * g_stPid_Angle_Yaw.f32Kd;
    g_stPid_Angle_Yaw.f32Dout = Float_Ceiling(g_stPid_Angle_Yaw.f32Dout, 150.0f);
    g_stPid_Angle_Yaw.f32LastError = g_stPid_Angle_Yaw.f32Error;
    g_stPid_Angle_Yaw.f32Output = g_stPid_Angle_Yaw.f32Pout + g_stPid_Angle_Yaw.f32Iout + g_stPid_Angle_Yaw.f32Dout;
    g_stPid_Angle_Yaw.f32Output = Float_Ceiling(g_stPid_Angle_Yaw.f32Output, 150.0f);

    g_stPid_AngleRate_Yaw.f32Target = g_stPid_Angle_Yaw.f32Output;
    g_stPid_AngleRate_Yaw.f32Error = g_stPid_AngleRate_Yaw.f32Target - g_stMpuData.f32Gyro_fz_dgree;
    g_stPid_AngleRate_Yaw.f32Error = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Error, 100.0f);
    g_stPid_AngleRate_Yaw.f32Pout = g_stPid_AngleRate_Yaw.f32Error * g_stPid_AngleRate_Yaw.f32Kp;
    g_stPid_AngleRate_Yaw.f32Pout *= fabs(g_stPid_AngleRate_Yaw.f32Pout);
    g_stPid_AngleRate_Yaw.f32Pout = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Pout, 500.0f);
    g_stPid_AngleRate_Yaw.f32Iout += g_stPid_AngleRate_Yaw.f32Error * g_stPid_AngleRate_Yaw.f32Ki * ATTITUDE_CTL_DT;
    g_stPid_AngleRate_Yaw.f32Iout = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Iout, 100.0f);
    g_stPid_AngleRate_Yaw.f32Dout = (g_stPid_AngleRate_Yaw.f32Error - g_stPid_AngleRate_Yaw.f32LastError) * g_stPid_AngleRate_Yaw.f32Kd;
    g_stPid_AngleRate_Yaw.f32Dout = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Dout, 50.0f);
    g_stPid_AngleRate_Yaw.f32LastError = g_stPid_AngleRate_Yaw.f32Error;
    g_stPid_AngleRate_Yaw.f32Output = g_stPid_AngleRate_Yaw.f32Pout + g_stPid_AngleRate_Yaw.f32Iout + g_stPid_AngleRate_Yaw.f32Dout;
    g_stPid_AngleRate_Yaw.f32Output = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Output, 500.0f);
    g_stPlane.u16Motor_Yaw = (int16_t)g_stPid_AngleRate_Yaw.f32Output;
}

void App_Plane_Yaw_RateControl(float rate)
{
    g_stPid_AngleRate_Yaw.f32Target = rate;
    g_stPid_AngleRate_Yaw.f32Error = g_stPid_AngleRate_Yaw.f32Target - g_stMpuData.f32Gyro_fz_dgree;
    g_stPid_AngleRate_Yaw.f32Error = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Error, 100.0f);
    g_stPid_AngleRate_Yaw.f32Pout = g_stPid_AngleRate_Yaw.f32Error * g_stPid_AngleRate_Yaw.f32Kp;
    g_stPid_AngleRate_Yaw.f32Pout *= fabs(g_stPid_AngleRate_Yaw.f32Pout);
    g_stPid_AngleRate_Yaw.f32Pout = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Pout, 500.0f);
    g_stPid_AngleRate_Yaw.f32Iout += g_stPid_AngleRate_Yaw.f32Error * g_stPid_AngleRate_Yaw.f32Ki * ATTITUDE_CTL_DT;
    g_stPid_AngleRate_Yaw.f32Iout = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Iout, 100.0f);
    g_stPid_AngleRate_Yaw.f32Dout = (g_stPid_AngleRate_Yaw.f32Error - g_stPid_AngleRate_Yaw.f32LastError) * g_stPid_AngleRate_Yaw.f32Kd;
    g_stPid_AngleRate_Yaw.f32Dout = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Dout, 50.0f);
    g_stPid_AngleRate_Yaw.f32LastError = g_stPid_AngleRate_Yaw.f32Error;
    g_stPid_AngleRate_Yaw.f32Output = g_stPid_AngleRate_Yaw.f32Pout + g_stPid_AngleRate_Yaw.f32Iout + g_stPid_AngleRate_Yaw.f32Dout;
    g_stPid_AngleRate_Yaw.f32Output = Float_Ceiling(g_stPid_AngleRate_Yaw.f32Output, 500.0f);

    g_stPlane.u16Motor_Yaw = (int16_t)g_stPid_AngleRate_Yaw.f32Output;
}
