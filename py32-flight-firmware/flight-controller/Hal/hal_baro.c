#include <stdio.h>
#include <math.h>
#include "bsp.h"
#include "device.h"
#include "Hal_baro.h"

#define BARO_SAMPLE_CNT       200
#define BARO_SAMPLE_LPF       (1.0f/5.0f)
#define BARO_NORMAL_LPF       0.75f
#define BARO_VEL_LPF          0.75f

stBaroData g_BaroData;

uint8_t Hal_Baro_Init(void)
{
    if(Dev_TCBP001_Init() == 0)
    {
        while(1);
    }

    Dev_TCBP0001_GetBaroData();
    g_BaroData.f32PressureOffset  = g_BaroTcbp.f32Pressure;
    g_BaroData.f32LpfPressureLast = g_BaroTcbp.f32Pressure;
    g_BaroData.f32LpfPara = BARO_SAMPLE_LPF;
    delay_ms(5);
    return 1;
}

float Hal_Baro_GetRelAltitude(float Press, float Ref_Press)
{
    return 44330 * (1 - powf(((float)Press / (float)Ref_Press), (1/5.255f))); 
}

void Hal_Baro_GetData(void)
{
    Dev_TCBP0001_GetBaroData();
    g_BaroData.f32Pressure =    g_BaroTcbp.f32Pressure;
    g_BaroData.f32Temperature = g_BaroTcbp.f32Temperature;
}

void Hal_Baro_Altitude_Velocity_Update(void)
{
    g_BaroData.f32AltitudeVelocity = (g_BaroData.f32Altitude - g_BaroData.f32LastAltitude)  * 20.0f *  100.0f;
    g_BaroData.f32AltitudeVelocityF = g_BaroData.f32AltitudeVelocityF * BARO_VEL_LPF + (1 - BARO_VEL_LPF) * (g_BaroData.f32AltitudeVelocity * 20.0f);
}

void Hal_Baro_Update(void)
{
    Hal_Baro_GetData();
    g_BaroData.f32LpfPressure = g_BaroData.f32LpfPara * (g_BaroData.f32Pressure * 0.01f)
                                 + (1.0f - g_BaroData.f32LpfPara) * g_BaroData.f32LpfPressureLast;
    g_BaroData.f32LpfPressureLast = g_BaroData.f32LpfPressure;
   
    if(!g_BaroData.u8OffsetReady)
    {
        if(g_BaroData.u16OffsetReadyCnt > BARO_SAMPLE_CNT)
        {
            g_BaroData.f32PressureOffset     = g_BaroData.f32Pressure;
            g_BaroData.f32AltitudeRaw        = Hal_Baro_GetRelAltitude(g_BaroData.f32LpfPressure, g_BaroData.f32PressureOffset * 0.01f);
            g_BaroData.f32AltitudeOffset     = g_BaroData.f32AltitudeRaw;
            g_BaroData.f32LpfPara            = BARO_NORMAL_LPF;
            g_BaroData.u8OffsetReady         = 1;
        }
        else
        {
            g_BaroData.u16OffsetReadyCnt++;
        }
    }
    else
    {
        g_BaroData.f32AltitudeRaw = Hal_Baro_GetRelAltitude(g_BaroData.f32LpfPressure, g_BaroData.f32PressureOffset * 0.01f);
        g_BaroData.f32Altitude    = g_BaroData.f32AltitudeRaw - g_BaroData.f32AltitudeOffset;
        Hal_Baro_Altitude_Velocity_Update();
        g_BaroData.f32LastAltitude = g_BaroData.f32Altitude;
    }
}

uint8_t Hal_Baro_IsReady(void)
{
    return g_BaroData.u8OffsetReady;
}

uint8_t Hal_Baro_SetLpfPara(float lpf)
{
    g_BaroData.f32LpfPara = lpf;
    return 1;
}

void Hal_Baro_UpdateOffset(void)
{
    g_BaroData.f32AltitudeOffset  = g_BaroData.f32AltitudeRaw;
}
