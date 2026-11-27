#include "stdio.h"
#include "string.h"
#include "bsp.h"
#include "device.h"
#include "hal.h"
#include "commom.h"
#include "App_Motor.h"
#include "App_Plane.h"

#define MOTOR_PRESTART_VALUE   1200

stMotor g_stMotor;

void App_Motor_Init(void)
{
    g_stMotor.eMotorSta = ENUM_MOTOR_STOP;
}

void App_Motor_Set(uint8_t index, uint16_t thr)
{
    if(index < MOTOR_NUM)
    {
        if(g_stMotor.u16MotorOut[index] > 2000)
            g_stMotor.u16MotorOut[index] = 2000;
        else if(thr < 1000)
            g_stMotor.u16MotorOut[index] = 1000;
        else
            g_stMotor.u16MotorOut[index] = thr;
        Bsp_PWM_SetDuty(index, (g_stMotor.u16MotorOut[index] - 1000) * 3/2);
    }
}


void App_Motor_StartCheck(void)
{
    static uint16_t u16MotorCheckCnt = 0;
    static uint8_t  u8MotorStartFlag = 0;
    if((g_stMotor.eMotorSta == ENUM_MOTOR_STOP) && (g_BaroData.u8OffsetReady)\
     && (!App_Voltage_IsLow()) && (g_stPlane.eCalSt == CALIBRATION_STATE_NONE))
    {
        if((g_stRCData.RC_TROTTLE_Receive < 1150) && (g_stRCData.RC_ROLL_Receive > 1850)\
            && (g_stRCData.RC_PITCH_Receive > 1850) && (g_stRCData.RC_YAW_Receive < 1150))
        {
            u16MotorCheckCnt++;
            if(u16MotorCheckCnt >= 50)
            { 
                App_Plane_SetHighPositonOffset();
                u8MotorStartFlag = 1;
                g_stPlane.eCalSt = CALIBRATION_STATE_GYRO_AUTO;
            }
            else
            {
                g_stMotor.eMotorSta = ENUM_MOTOR_STOP;
            }
        }
        else
        {
            u16MotorCheckCnt = 0;
        }
    }
    if(u8MotorStartFlag  == 1)
    {
        if(g_stPlane.eCalSt == CALIBRATION_STATE_NONE)
        {
            u8MotorStartFlag  =0;
            g_stMotor.eMotorSta = ENUM_MOTOR_PRESTART;
        }     
    }
}


void App_Motor_StopCheck(void)
{
    static uint16_t Emergency_brake = 0;
	//紧急制动装置
	if ((((g_stRCData.RC_TROTTLE_Receive < 1150))
		&& (((g_stRCData.RC_ROLL_Receive < 1150) 
                && (g_stRCData.RC_PITCH_Receive > 1850) 
                && (g_stRCData.RC_YAW_Receive < 1150)) 
                || ((g_stRCData.RC_ROLL_Receive > 1850) 
                && (g_stRCData.RC_PITCH_Receive > 1850) 
                && (g_stRCData.RC_YAW_Receive > 1850)))
		&& (g_stMotor.eMotorSta != ENUM_MOTOR_STOP)))
	{
		Emergency_brake++;
		if (Emergency_brake >= 200)
        {
            g_stMotor.eMotorSta = ENUM_MOTOR_STOP;
        }
	}
	else
    {
        Emergency_brake = 0;
    }
}

void App_Motor_Stop(void)
{
    g_stMotor.eMotorSta = ENUM_MOTOR_STOP;
    
}

int16_t   vast_motor_out[MOTOR_NUM]; 

void App_Motor_Run(void)
{
	static uint16_t cnt_2sec = 0;
	static uint16_t cnt_4sec = 0;
    App_Motor_StartCheck();
    App_Motor_StopCheck();
    switch(g_stMotor.eMotorSta)
    {
        case ENUM_MOTOR_STOP:
            g_stPlane.u8IsOnland = 1;
            App_Motor_Set(MOTOR_LEFT_UP, 1000);
            App_Motor_Set(MOTOR_LEFT_DOWN, 1000);
            App_Motor_Set(MOTOR_RIGHT_UP, 1000);
            App_Motor_Set(MOTOR_RIGHT_DOWN, 1000);
            cnt_2sec = 0;
            break;
        case ENUM_MOTOR_PRESTART:
            App_Motor_Set(MOTOR_LEFT_UP, MOTOR_PRESTART_VALUE);
            App_Motor_Set(MOTOR_LEFT_DOWN, MOTOR_PRESTART_VALUE);
            App_Motor_Set(MOTOR_RIGHT_UP, MOTOR_PRESTART_VALUE);
            App_Motor_Set(MOTOR_RIGHT_DOWN, MOTOR_PRESTART_VALUE);
            if(g_stRCData.RC_TROTTLE_Receive >= 1590)
            {
                g_stMotor.eMotorSta = ENUM_MOTOR_RUN;
            }

            if (g_stRCData.RC_TROTTLE_Receive < 1150)
            {
                cnt_2sec++;
            }
            else
            {
                cnt_2sec = 0;
            }
            if (cnt_2sec >= 500)
            {
                g_stMotor.eMotorSta = ENUM_MOTOR_STOP;
            }
            break;
        case ENUM_MOTOR_RUN:
            g_stPlane.u16Motor_Throttle = g_stPlane.f32Throttle_neutral + g_stPlane.f32Thr_HighHold;
            if(g_stPlane.u16Motor_Throttle > 1280)
            {
                vast_motor_out[MOTOR_LEFT_UP] = (int16_t)(g_stPlane.u16Motor_Throttle\
                    + g_stPlane.u16Motor_Roll\
                    + g_stPlane.u16Motor_Pitch\
                    - g_stPlane.u16Motor_Yaw);
                
                vast_motor_out[MOTOR_LEFT_DOWN] = (int16_t)(g_stPlane.u16Motor_Throttle\
                    + g_stPlane.u16Motor_Roll\
                    - g_stPlane.u16Motor_Pitch\
                    + g_stPlane.u16Motor_Yaw);          
                
                vast_motor_out[MOTOR_RIGHT_UP] = (int16_t)(g_stPlane.u16Motor_Throttle\
                    - g_stPlane.u16Motor_Roll\
                    + g_stPlane.u16Motor_Pitch\
                    + g_stPlane.u16Motor_Yaw);
                
                vast_motor_out[MOTOR_RIGHT_DOWN] = (int16_t)(g_stPlane.u16Motor_Throttle\
                    - g_stPlane.u16Motor_Roll\
                    - g_stPlane.u16Motor_Pitch\
                    - g_stPlane.u16Motor_Yaw);
                
               for (uint8_t i = 0; i < MOTOR_NUM; i++)
               {
                   if (vast_motor_out[i] > 2000)
                   {
                       vast_motor_out[i] = 2000;
                   }
                   if (vast_motor_out[i] < 1280)
                   {
                       vast_motor_out[i] = 1280;
                   }
                   App_Motor_Set(i,vast_motor_out[i]);
               }
                // //2. 计算最大超限量
                // int16_t max_out = vast_motor_out[0];
                // int16_t min_out = vast_motor_out[0];
                // for (uint8_t i = 1; i < MOTOR_NUM; i++) {
                //     if (vast_motor_out[i] > max_out) max_out = vast_motor_out[i];
                //     if (vast_motor_out[i] < min_out) min_out = vast_motor_out[i];
                // }

                // int16_t offset = 0;
                // if (max_out > 2000) {
                //     offset = max_out - 2000;
                // }
                // // 可选：如果需要保证最低输出，也可以加下限调整
                // // if (min_out < 1280) {
                // //     int16_t offset_min = 1280 - min_out;
                // //     if (offset_min > offset) offset = -offset_min;
                // // }

                // // 3. 整体下调
                // for (uint8_t i = 0; i < MOTOR_NUM; i++) {
                //     int16_t out = vast_motor_out[i] - offset;
                //     // 最终限幅，防止下调后低于下限
                //     if (out > 2000) out = 2000;
                //     if (out < 1280) out = 1280;
                //     App_Motor_Set(i, out);
                // }                            
                cnt_4sec = 0;
            }
            else
            {
                //Land Check
                if ((g_stPlane.f32Postion_z < 2) \
                     //&& (g_stPlane.f32Velocity_z < 0.25f)
                    //  && (Hal_Mpu_GetGyroLen() < 20.0f)
                    && (g_stPlane.u16Motor_Throttle < 1280))
                {
                    cnt_4sec++;
                    if ((cnt_4sec > 125))
                    {
                        g_stPlane.u16Motor_Throttle = 900;
                        cnt_4sec = 0;	
                        g_stMotor.eMotorSta = ENUM_MOTOR_STOP;
                    }
                }
                else
                {
                    cnt_4sec/= 2;
                }

                        
                for (uint8_t i = 0; i < MOTOR_NUM; i++)
                {
                    App_Motor_Set(i, g_stPlane.u16Motor_Throttle);
                }    
            }


            // static float last_Acc_z = 0;
            // static float last_AccLen = 0;
            // if((g_stPlane.u16Motor_Throttle < 1520) &&\
            //  (g_stHigh_Estimate.f32Pos_Baro < -10.0f) &&\
            //   ((g_stPlane.f32Acc_ef_z - last_Acc_z) > 0.5f)&&\
            //   (g_stPlane.f32Acc_ef_z > 0)&&\

            //    && (fabs(Hal_Mpu_GetAccLen() - last_AccLen) > 3))
            // {
            //     g_stMotor.eMotorSta = ENUM_MOTOR_STOP;
            // }
            // last_Acc_z = g_stPlane.f32Acc_ef_z;
            // last_AccLen = Hal_Mpu_GetAccLen();

            // if(g_stPlane.u16Motor_Throttle <= 1280)
            // {
            //     g_stPlane.u16Motor_Throttle = 1280;
            // }

            break;
        default:
            break;
    }
}
