#ifndef _APP_PLABE_H_
#define _APP_PLANE_H_

#include <stdint.h>

typedef enum _eCalibrationState
{
    CALIBRATION_STATE_NONE = 0,
    CALIBRATION_STATE_GYRO,
    CALIBRATION_STATE_ACC,
    CALIBRATION_STATE_AHRS,
    CALIBRATION_STATE_GYRO_AUTO,
    CALIBRATION_STATE_END
} eCalibrationState;

typedef enum _eFlySpeedLvl
{
    FLY_SPEED_LVL_LOW = 0,
    FLY_SPEED_LVL_MID,
    FLY_SPEED_LVL_HIGH,
}eFlySpeedLvl;


typedef struct _stPlane{
    eCalibrationState eCalSt;

    uint8_t eFlySpeedLvl;

    float  f32Acc_ef_z;
    float  f32Acc_ef_fz;
    float  f32Acc_ef_fz_Offset;

    float  f32Yaw_Angle;

    float  f32Thr_HighHold;

    float  f32Yaw_Rate;
    float  f32Target_Yaw_Rate;

    float  f32Theta;
    float  f32Gamma;
    float  f32Phi;

    float f32Velocity_x;
    float f32Position_x;
    float f32Velocity_y;
    float f32Position_y;

    float  f32Velocity_z;
    float  f32Postion_z;
    uint8_t take_off_process;


    float  f32Throttle_neutral;
    uint8_t u8IsOnland;

    //Motor
    int16_t u16Motor_Pitch;
    int16_t u16Motor_Roll;
    int16_t u16Motor_Yaw;
    int16_t u16Motor_Throttle;
    
}stPlane;


typedef struct _PID_Data_Type
{
  
    float target_rate,  angle_error; 
    float current_rate, rate_error;
    float target_angle;
  
    int16_t Pout;
    int16_t Iout;
    int16_t Dout;
    int16_t PIDout;
    
    int16_t PKp;
    int16_t PKi;
    
    int16_t RKp;
    int16_t RKi;
    int16_t RKd;
    
}PID_Data_Type;

typedef struct _PID_Altitude_Type
{
    float Pout;
    float Iout;
    float Dout;
    
    float P_Pout;
    float P_Iout;
    float P_Dout;
    
    float R_PIDout;
    float P_PIDout;
    
    float target_Position;
    
    float PKp;
    float PKi;
    float PKd;
    
    float RKp;
    float RKi;
    float RKd;
    
}PID_Altitude_Type;

extern stPlane g_stPlane;
void App_Plane_Init(void);
void App_Plane_Calibration(void);
void App_Plane_AhrUpdate(void);
void App_Plane_Mag_Cal(void);
void App_Plane_TakeOffInit(void);
void App_Plane_Control(void);
void App_Plane_High_Ctl(void);
void App_Plane_Navigation_UpdateWp(void);
void App_Plane_OptHorizon_Hold(void);
void App_Plane_Gps_Control(void);
void App_Plane_GPS_Fusion(void);
void GPS_High_Sins(void);
void App_GPS_Init(void);
void App_Gps_Acc_Update(void);
void App_Gps_PV_Update(void);
void clear_loiter_i(void);
void App_Plane_Change_Speed(void);
void App_Plane_OptHorizon_ClearInt(void);
void App_Plane_Change_mode(void);
float get_optimal_velocity_z(float acc_ef_z, float baro_velocity);
float get_optimal_position_z(float velocity, float baro_altitude);
void App_Plane_SetHighPositonOffset(void);
uint8_t App_Plane_IsOverLoad(void);

#endif
