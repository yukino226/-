#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "device.h"
#include "hal.h"
#include "commom.h"
#include "App_Storage.h"
#include "App_Plane.h"
#include "MadgwickAHRS.h"
//#include "rf_2_4G.h"
#include "App_High_Ctl.h"
#include "App_Attitude_Ctl.h"
#include "App_Motor.h"
#include "App_Voltage.h"

#define PLANE_RC_MID                        1520
#define PLANE_RC_MIN                        1020
#define PLANE_RC_MAX                        2020

//Gamma,Theta
#define PLANE_RC_TO_ANGLE_LOW               250
#define PLANE_RC_TO_ANGLE_HIGH              400
#define PLANE_RC_DEAD_ZONE                  100



//Phi
#define PLANE_RC_TO_YAWRATE_LOW             300
#define PLANE_RC_TO_YAWRATE_HIGH            400

#define PLANE_RC_DEAD_ZONE_YAW              100
#define PLANE_RC_TO_YAWRATE_SPEED           0.5f
//#define PLANE_RC_TO_YAWRATE_SPEED           0.15f
//THR
#define PLANE_RC_DEAD_ZONEE_THR             80   //ÓÍÃÅÐüÍ£µã

#define AP_MOTORS_DEFAULT_MIN_THROTTLE      150
#define AP_MOTORS_DEFAULT_MAX_THROTTLE      980
#define MOTORS_NEUTRAL                      1475
#define AP_MOTORS_MAX_NUM_MOTORS            4
#define radians(x)                          ((x)*(0.0174533f))
#define max(x,y)                            (((x)>(y)) ? (x) : (y))


stPlane g_stPlane;
//need Changge
extern RCTPParama g_stRCData;




void App_Plane_Init(void)
{
    g_stPlane.eCalSt = CALIBRATION_STATE_NONE;
    g_stPlane.f32Yaw_Angle = 0;
    g_stPlane.f32Throttle_neutral = 1550;
    g_stPlane.u8IsOnland = 1;
    g_stPlane.eFlySpeedLvl = FLY_SPEED_LVL_LOW;

    stMpuVector v;
    v.x = g_stStorage.stMpuFlashData.Falsh_Gyro_nx_off;
    v.y = g_stStorage.stMpuFlashData.Falsh_Gyro_ny_off;
    v.z = g_stStorage.stMpuFlashData.Falsh_Gyro_nz_off;
    Hal_Mpu_SetGyroOffset(&v);
    v.x = g_stStorage.stMpuFlashData.Falsh_Accel_nx_off;
    v.y = g_stStorage.stMpuFlashData.Falsh_Accel_ny_off;
    v.z = g_stStorage.stMpuFlashData.Falsh_Accel_nz_off;
    Hal_Mpu_SetAccOffset(&v);

    g_stPlane.f32Acc_ef_fz_Offset = 0.98f;
    //App_GPS_Init();
}
void App_Plane_Change_Speed(void)
{
    //flow_scaler++;
    //if(g_stPlane.u8MotorRun == 0)
    // {
        if(g_stPlane.eFlySpeedLvl == FLY_SPEED_LVL_HIGH)
            g_stPlane.eFlySpeedLvl = FLY_SPEED_LVL_LOW;
        else
            g_stPlane.eFlySpeedLvl = FLY_SPEED_LVL_HIGH;
    // }
}

void App_Plane_Change_mode(void)
{
    //flow_scaler++;
    //if(g_stPlane.u8MotorRun == 0)
    // {
    //     if(g_stPlane.eFlySpeedLvl == FLY_SPEED_LVL_HIGH)
    //         g_stPlane.eFlySpeedLvl = FLY_SPEED_LVL_LOW;
    //     else
    //         g_stPlane.eFlySpeedLvl = FLY_SPEED_LVL_HIGH;
    // }
}

static uint8_t  u8CalibrationCnt = 0;
static float i32Calibration_x_Sum = 0;
static float i32Calibration_y_Sum = 0;
static float i32Calibration_z_Sum = 0;


#define CALIBRATION_CNT     50
uint8_t App_Plane_Gyro_UpdateOffset(void)
{
    u8CalibrationCnt++;
    i32Calibration_x_Sum += g_stMpuData.f32Gyro_x;
    i32Calibration_y_Sum += g_stMpuData.f32Gyro_y;
    i32Calibration_z_Sum += g_stMpuData.f32Gyro_z;

    if (fabs(g_stMpuData.f32Accel_x_g) > 0.2f || \
        fabs(g_stMpuData.f32Accel_y_g) > 0.2f || \
        fabs(g_stMpuData.f32Accel_z_g - 0.98f) > 0.2f)
    {
        u8CalibrationCnt = 0;
        i32Calibration_x_Sum = 0;
        i32Calibration_y_Sum = 0;
        i32Calibration_z_Sum = 0;
        return 1;
    }

    if(u8CalibrationCnt >= CALIBRATION_CNT)
    {
        g_stMpuData.f32Gyro_x_off = (float)i32Calibration_x_Sum / (float)CALIBRATION_CNT;
        g_stMpuData.f32Gyro_y_off = (float)i32Calibration_y_Sum / (float)CALIBRATION_CNT;
        g_stMpuData.f32Gyro_z_off = (float)i32Calibration_z_Sum / (float)CALIBRATION_CNT;

        g_stStorage.stMpuFlashData.Falsh_Gyro_nx_off = g_stMpuData.f32Gyro_x_off;
        g_stStorage.stMpuFlashData.Falsh_Gyro_ny_off = g_stMpuData.f32Gyro_y_off;
        g_stStorage.stMpuFlashData.Falsh_Gyro_nz_off = g_stMpuData.f32Gyro_z_off;

        u8CalibrationCnt = 0;
        i32Calibration_x_Sum = 0;
        i32Calibration_y_Sum = 0;
        i32Calibration_z_Sum = 0;
        return 1;
    }
    else
    {
        return 0;
    }    
}

uint8_t App_Plane_Acc_UpdateOffset(void)
{
    u8CalibrationCnt++;
    i32Calibration_x_Sum += g_stMpuData.f32Accel_x;
    i32Calibration_y_Sum += g_stMpuData.f32Accel_y;
    i32Calibration_z_Sum += g_stMpuData.f32Accel_z;

    if(u8CalibrationCnt >= CALIBRATION_CNT)
    {
        g_stMpuData.f32Accel_x_off = (float)i32Calibration_x_Sum / (float)CALIBRATION_CNT;
        g_stMpuData.f32Accel_y_off = (float)i32Calibration_y_Sum / (float)CALIBRATION_CNT;
        g_stMpuData.f32Accel_z_off = (float)i32Calibration_z_Sum / (float)CALIBRATION_CNT - GRAVITY_MSS/ACC_SCALE/10;

        g_stStorage.stMpuFlashData.Falsh_Accel_nx_off = g_stMpuData.f32Accel_x_off;
        g_stStorage.stMpuFlashData.Falsh_Accel_ny_off = g_stMpuData.f32Accel_y_off;
        g_stStorage.stMpuFlashData.Falsh_Accel_nz_off = g_stMpuData.f32Accel_z_off;

        u8CalibrationCnt = 0;
        i32Calibration_x_Sum = 0;
        i32Calibration_y_Sum = 0;
        i32Calibration_z_Sum = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}


extern uint32_t u32AhsSample;
void App_Plane_Calibration(void)
{
    static uint16_t u16Calibration = 0;
    if ((g_stRCData.RC_TROTTLE_Receive  > 1850)
    && (g_stRCData.RC_ROLL_Receive  > 1850) 
    && (g_stRCData.RC_PITCH_Receive > 1850) 
    && (g_stRCData.RC_YAW_Receive <  1150)
    && (g_stMotor.eMotorSta == ENUM_MOTOR_STOP))
    {
        u16Calibration++;
        if (u16Calibration >= 10)
        {
            g_stPlane.eCalSt = CALIBRATION_STATE_ACC;
        }
    }
    else
    {
        u16Calibration = 0;
    }    

	switch (g_stPlane.eCalSt)
	{
	case CALIBRATION_STATE_NONE:
		break;
	case CALIBRATION_STATE_ACC:
		if(App_Plane_Acc_UpdateOffset())
		{
            g_stPlane.eCalSt = CALIBRATION_STATE_GYRO;
		}
		break;
	case CALIBRATION_STATE_GYRO:
		if(App_Plane_Gyro_UpdateOffset())
		{
			g_stPlane.eCalSt = CALIBRATION_STATE_AHRS;
            App_StorageSave();
            App_High_Estimate_Reset((g_stPlane.f32Acc_ef_fz - g_stPlane.f32Acc_ef_fz_Offset) * 980.0f, g_BaroData.f32Altitude * 100.0f);
            MadgwickAHR_ClearInt();
            u32AhsSample = 0;			
		}
		break;
    case CALIBRATION_STATE_AHRS:
        if(u32AhsSample == (501))
        {
            g_stPlane.eCalSt = CALIBRATION_STATE_NONE;
        }
        break;
    case CALIBRATION_STATE_GYRO_AUTO:
        if(App_Plane_Gyro_UpdateOffset())
        {
            g_stPlane.eCalSt = CALIBRATION_STATE_NONE;
        }
        break;
    default:
        break;
	}
}


void App_Plane_SetHighPositonOffset(void)
{
    Hal_Baro_UpdateOffset();
}


float App_Plane_RcToAngle(int16_t value)
{
    uint16_t u16Control_speed = 0;
    switch(g_stPlane.eFlySpeedLvl)
    {
        case FLY_SPEED_LVL_LOW:
            u16Control_speed = PLANE_RC_TO_ANGLE_LOW;
            break;
        case FLY_SPEED_LVL_MID:
        case FLY_SPEED_LVL_HIGH:
            u16Control_speed = PLANE_RC_TO_ANGLE_HIGH;
            break;
        default:
            u16Control_speed = PLANE_RC_TO_ANGLE_HIGH;
            break;
    }

	if (value > PLANE_RC_MID + PLANE_RC_DEAD_ZONE) 
    {
		return (0.1f  * ((u16Control_speed * (value - (PLANE_RC_MID + PLANE_RC_DEAD_ZONE))) / (PLANE_RC_MAX  - (PLANE_RC_MID + PLANE_RC_DEAD_ZONE))));
	}
	else if (value < PLANE_RC_MID - PLANE_RC_DEAD_ZONE)
    {
		return (0.1f  * ((u16Control_speed * (value - (PLANE_RC_MID - PLANE_RC_DEAD_ZONE))) / ((PLANE_RC_MID - PLANE_RC_DEAD_ZONE) - PLANE_RC_MIN)));
	}
	else
    {
		return 0;
	}
}

float App_Plane_RcToYawRate(int16_t value)
{
    float f32TargetYawRate = 0.0f;       
    uint16_t u16Control_speed = 0;
    switch(g_stPlane.eFlySpeedLvl)
    {
        case FLY_SPEED_LVL_LOW:
            u16Control_speed = PLANE_RC_TO_YAWRATE_LOW;
            break;
        case FLY_SPEED_LVL_MID:
        case FLY_SPEED_LVL_HIGH:
            u16Control_speed = PLANE_RC_TO_YAWRATE_HIGH;
            break;
        default:
            u16Control_speed = PLANE_RC_TO_YAWRATE_HIGH;
            break;
    }        
	if (value > PLANE_RC_MID + PLANE_RC_DEAD_ZONE_YAW) 
    {
        f32TargetYawRate = 0.1f * ((u16Control_speed * (value - (PLANE_RC_MID + PLANE_RC_DEAD_ZONE_YAW))) / (PLANE_RC_MAX  - (PLANE_RC_MID + PLANE_RC_DEAD_ZONE_YAW)));
        if(g_stPlane.f32Target_Yaw_Rate < f32TargetYawRate)
        {
            g_stPlane.f32Target_Yaw_Rate += PLANE_RC_TO_YAWRATE_SPEED;
        }
        else
        {
            g_stPlane.f32Target_Yaw_Rate = f32TargetYawRate;
        }
        
        return (g_stPlane.f32Target_Yaw_Rate);   
	}
	else if (value < (PLANE_RC_MID - PLANE_RC_DEAD_ZONE_YAW)) 
    {
        f32TargetYawRate = 0.1f * ((u16Control_speed * (value - (PLANE_RC_MID - PLANE_RC_DEAD_ZONE_YAW))) / ((PLANE_RC_MID - PLANE_RC_DEAD_ZONE_YAW) - PLANE_RC_MIN));
        if(g_stPlane.f32Target_Yaw_Rate > f32TargetYawRate)
        {
            g_stPlane.f32Target_Yaw_Rate -= PLANE_RC_TO_YAWRATE_SPEED;
        }
        else
        {
            g_stPlane.f32Target_Yaw_Rate = f32TargetYawRate;
        }
        
        return (g_stPlane.f32Target_Yaw_Rate);
	}
	else 
    {              
        g_stPlane.f32Target_Yaw_Rate = 0.0f;
        return 0;
	}
}

void App_Plane_ThetaControl(void)
{
    App_Attitude_ThetaControl();
}

void App_Plane_GammaControl(void)
{
    App_Attitude_GammaControl();
}

void App_Plane_Yaw_Control(void)
{
    static uint8_t  yaw_stop = 0;
    static uint16_t yaw_timer = 0;
    static float yaw_rate_stop = 0.0f;
    float  yaw_rate_control = 0.0f;

    g_stPlane.f32Yaw_Rate = App_Plane_RcToYawRate(g_stRCData.RC_YAW_Receive);


    if(g_stPlane.f32Yaw_Rate != 0.0f)
    {
        App_Plane_Yaw_RateControl(g_stPlane.f32Yaw_Rate * 3.0f);
        yaw_stop = 1;
    }
    else if(yaw_stop == 1)
    {
        App_Attitude_SetYawTarget(g_stPlane.f32Yaw_Angle);
        App_Plane_Yaw_AngleControl();
        yaw_stop = 0;
    }
    else
    {
        App_Plane_Yaw_AngleControl();
    }
    // {
    //     yaw_rate_control  = g_stPlane.f32Yaw_Rate * 3.0f;
    //     yaw_stop = 1;
    //     yaw_timer = 750;
    //     App_Plane_Yaw_RateControl(yaw_rate_control);
    // }
    // // else if(yaw_stop == 1)
    // // {
    // //     if (yaw_timer == 750) 
    // //     {
    // //         yaw_rate_stop = g_stMpuData.f32Gyro_fz_dgree;
    // //     }
    // //     else 
    // //     {
    // //         yaw_rate_stop += constrain_float(0 - yaw_rate_stop, -0.1f, 0.1f);
    // //     }
    // //     yaw_rate_control = yaw_rate_stop / 3.0f;
    // //     yaw_timer--;
    // //     if ((yaw_timer == 0) || (fabsf(g_stMpuData.f32Gyro_fz_dgree) < 10))
    // //     {
    // //         yaw_stop = 0;
    // //         App_Attitude_SetYawTarget(g_stPlane.f32Yaw_Angle);
    // //         yaw_timer = 0;
    // //     }
    // //     App_Plane_Yaw_RateControl(0);
    // // }
    // else
    // {
    //     App_Plane_Yaw_RateControl(0);
    //     //App_Plane_Yaw_AngleControl();
    // }
}

void App_Plane_Control(void)
{
    App_Attitude_SetGammaTarget(App_Plane_RcToAngle(g_stRCData.RC_ROLL_Receive));
    App_Attitude_SetThetaTarget(App_Plane_RcToAngle(g_stRCData.RC_PITCH_Receive));
    App_Plane_ThetaControl();
    App_Plane_GammaControl();
    App_Plane_Yaw_Control();
    App_Motor_Run();


    static uint16_t u16AngleProtectCnt = 0;
    if (((fabsf)(g_stPlane.f32Theta)) > 60.0f || ((fabsf)(g_stPlane.f32Gamma)) > 60.0f)
    {
        u16AngleProtectCnt++;
        if (u16AngleProtectCnt > 100)
        {
            App_Motor_Stop();
            u16AngleProtectCnt = 100;
        }
    }
    else
    {
        u16AngleProtectCnt = 0;
    }
}




float App_Plane_RcToClimbRate(int16_t radio_in)
{
	int16_t radio_trim_high = 1500 + PLANE_RC_DEAD_ZONEE_THR;
	int16_t radio_trim_low  = 1500 - PLANE_RC_DEAD_ZONEE_THR;

	static float current_climb_value = 0.0f;
	float target_climb_value = 0.0f;

	if (radio_in > radio_trim_high)
	{
		target_climb_value = 0.001f * ((3500 * (radio_in - radio_trim_high)) / (PLANE_RC_MAX - radio_trim_high));
		if (current_climb_value < target_climb_value)
		{
            if (g_stPlane.f32Postion_z < 1)
            {
                current_climb_value += 0.12f;
            }
            else
            {
                current_climb_value += 0.05f;
            }

            current_climb_value = constrain_float(current_climb_value, 0, 2.5f);

            if(g_stPlane.take_off_process == 1)
            {
                current_climb_value = constrain_float(current_climb_value, 0, 1.0f);
            }
		}
		else
        {
		    current_climb_value = target_climb_value;
        }
		return (current_climb_value);
	}
	else if (radio_in < radio_trim_low)
	{
		if (g_stPlane.f32Postion_z < 10)
		{
            target_climb_value = 0.001f * ((2200 * (radio_in - radio_trim_low)) / (radio_trim_low - PLANE_RC_MIN));
		}
		else
		{
		   target_climb_value = 0.001f * ((2000 * (radio_in - radio_trim_low)) / (radio_trim_low - PLANE_RC_MIN));
		}
                
		if (target_climb_value >= -0.3f)
        {
		   target_climb_value = -0.3f;
        }
    
		if (current_climb_value > target_climb_value)
        {
		   current_climb_value -= 0.02f;
        }
		else
        {
		   current_climb_value = target_climb_value;
        }

        if (g_stPlane.f32Postion_z < 2)
        {
            current_climb_value = constrain_float(current_climb_value, -0.7f, 0.0f);
        }

		return (current_climb_value);
	}
	else
	{
		//g_stPlane.u8DowmRateFlag = 1;
		current_climb_value = 0.0;
		return 0;
	}
}

uint8_t App_Plane_TakeOffCheck(void)
{
    if(fabs(g_stPlane.f32Acc_ef_fz - g_stPlane.f32Acc_ef_fz_Offset) < 0.03f)
        return 1;
    else
        return 0;
}

void App_Plane_TakeOffInit(void)
{
    App_High_ClerarIntegrator();
    App_Attitude_SetYawTarget(g_stPlane.f32Yaw_Angle);
    App_Attitude_ClearIntegrator();
    App_High_Estimate_Reset((g_stPlane.f32Acc_ef_fz - g_stPlane.f32Acc_ef_fz_Offset) * 980.0f, g_BaroData.f32Altitude * 100.0f);

    //MadgwickAHR_ClearInt();
}

uint8_t u8Overload = 0;

uint8_t App_Plane_IsOverLoad(void)
{
    return u8Overload;
}


static uint16_t take_off_count = 0;
static uint16_t take_off_rate = 0.0f;
extern stPid_High_Ctl g_stPid_HighRate;
uint16_t  takeoff_thr = 0;

void App_Plane_High_Ctl(void)
{
    static uint16_t cnt = 0;
    static float    climb_rate;
	static uint16_t climb_stopped = 1;
	static uint16_t climb_timer = 0;
    static uint8_t  u8BaroFirst = 0;

    if(g_BaroData.u8OffsetReady != 1)
    {   
        if(u8BaroFirst == 0)
        {
            u8BaroFirst = 1;
        }
        return;
    }

    if(u8BaroFirst == 1)
    {
        u8BaroFirst = 0;
        App_Plane_SetHighPositonOffset();
        App_High_Estimate_Reset((g_stPlane.f32Acc_ef_fz - g_stPlane.f32Acc_ef_fz_Offset) * 980.0f, g_BaroData.f32Altitude * 100.0f);
    }

    App_High_Estimate((g_stPlane.f32Acc_ef_fz - g_stPlane.f32Acc_ef_fz_Offset) * 980.0f, g_BaroData.f32Altitude * 100.0f, g_BaroData.f32AltitudeVelocity);
    g_stPlane.f32Velocity_z = App_High_Estimate_GetVelocity() * 0.01f;
    g_stPlane.f32Postion_z  = App_High_Estimate_GetPosition() * 0.01f;	
    if ((!Dev_XN297L_IsBind()) || App_Voltage_IsLow() || u8Overload)
    {
        climb_stopped = 1;
        if (g_stPlane.f32Postion_z > 20)
        {
            climb_rate = -2.4f;
        }
        else if (g_stPlane.f32Postion_z > 10)
        {
            climb_rate = -1.6f;
        }
        else if (g_stPlane.f32Postion_z > 5)
        {
            climb_rate = -1.2f;
        }
        else if (g_stPlane.f32Postion_z > 3)
        {
            climb_rate = -0.8f;
        }
        else
        {
            climb_rate = -0.5f;
            if ((g_stPlane.f32Postion_z < 4) && ((fabsf)(g_BaroData.f32Altitude) <= 4))
            {
                if (((fabsf)(g_stPlane.f32Theta)) > 40.0f || ((fabsf)(g_stPlane.f32Gamma)) > 40.0f)
                {
                    cnt++;
                    if (cnt >= 100)
                    {
                        App_Motor_Stop();
                    }
                }
            }
            else
            {
                cnt = 0;
            }
        }
        if (g_stRCData.RC_TROTTLE_Receive > 1550)
        {
            g_stPlane.f32Throttle_neutral = 1550;
        }
        else
        {
            g_stPlane.f32Throttle_neutral = 1550;
            if (climb_rate >= 0)
            {
                climb_rate = -0.5f;
            }
        }
    }
    else
    {
        climb_rate = App_Plane_RcToClimbRate(g_stRCData.RC_TROTTLE_Receive);
    }

    if (g_stPlane.u8IsOnland == 1)
    {
        u8Overload = 0;
        if(App_Voltage_GetFlyBattery() > 4.0f)
        {
            takeoff_thr = 50;
        }
        else if(App_Voltage_GetFlyBattery() > 3.9f)
        {
            takeoff_thr = 70;
        }
        else if(App_Voltage_GetFlyBattery() > 3.8f)
        {
            takeoff_thr = 100;
        }
        else if(App_Voltage_GetFlyBattery() > 3.7f)
        {
            takeoff_thr = 120;
        }
        else if(App_Voltage_GetFlyBattery() > 3.6f)
        {
            takeoff_thr = 140;
        }
        else if(App_Voltage_GetFlyBattery() > 3.5f)
        {
            takeoff_thr = 160;
        }
        else if(App_Voltage_GetFlyBattery() > 3.4f)
        {
            takeoff_thr = 180;
        }
        else
        {
            takeoff_thr = 0;
        }

        if ((!Dev_XN297L_IsBind()))
            climb_rate = 0;

        if (climb_rate > 0)
        {
            if(App_Plane_TakeOffCheck() && (takeoff_thr != 0))
            {
                g_stPlane.f32Throttle_neutral = 1550;
                App_Plane_TakeOffInit();
                g_stPlane.take_off_process = 1;
                g_stPlane.u8IsOnland = 0;
                u8Overload = 0;
            }
        }
        else
        {
            if ((g_stRCData.RC_TROTTLE_Receive < (1000 + AP_MOTORS_DEFAULT_MIN_THROTTLE + 130)))
            {
                g_stPlane.f32Throttle_neutral = g_stRCData.RC_TROTTLE_Receive;
            }
            if (g_stPlane.f32Throttle_neutral < 1150)
            {
                g_stPlane.f32Throttle_neutral = 1150;
            }
            return;
        }
    }

    if(g_stPlane.take_off_process == 1)
    {
        take_off_count++;
        if(App_Voltage_GetFlyBattery() > 4.2f)
            take_off_rate = 0.9f;
        else if(App_Voltage_GetFlyBattery() > 4.1f)
            take_off_rate = 0.95f;
        else if(App_Voltage_GetFlyBattery() > 4.0f)
            take_off_rate = 1.00f;
        else
            take_off_rate = 1.1f;
        App_High_HighRate_Ctl(take_off_rate);
        //g_stPlane.f32Thr_HighHold = takeoff_thr;
        if((climb_rate < 0) || (g_stPlane.f32Postion_z > 0.7f) || (take_off_count > 400))
        {
            if((take_off_count > 400) && ((g_stPlane.f32Postion_z <= 0.5f) || (g_stPlane.f32Velocity_z < 0.5f)))
            {
                u8Overload = 1;
            }

            climb_stopped = 0;
            climb_timer = 50;
            g_stPlane.take_off_process = 2;
            take_off_count = 0;
            //App_High_Estimate_Reset((g_stPlane.f32Acc_ef_fz - g_stPlane.f32Acc_ef_fz_Offset) * 980.0f, g_BaroData.f32Altitude * 100.0f);
        }
    }
    else
    {
        g_stPlane.take_off_process = 0;
        take_off_count = 0;
        if (climb_rate != 0)
        {
            App_High_HighRate_Ctl(climb_rate);
            climb_stopped = 0;
            climb_timer = 1;
        }
        else if(!climb_stopped)
        {
            if(take_off_rate > 0)
            {
                take_off_rate -= take_off_rate/climb_timer;
            }
            else
            {
                take_off_rate = 0;
            }
            App_High_HighRate_Ctl(take_off_rate);
            climb_timer--;
            if ((climb_timer == 0))
            {
                climb_stopped = 1;
                App_High_SetHighTarget(g_stPlane.f32Postion_z);
            }  
            // App_High_SetHighTarget(g_stPlane.f32Postion_z);      
            //App_High_Hold();
            // App_High_HighRate_Ctl(0);
            //climb_stopped = 1;
        }
        else
        {
            App_High_HighRate_Ctl(0);
            //App_High_Hold();
        }
        
    }
}

