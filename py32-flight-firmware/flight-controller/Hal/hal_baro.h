#ifndef __HAL_BARO_H__
#define __HAL_BARO_H__
#include <stdint.h>
#include "string.h"


#define Altitude_Velocity_BUF_LEN   8
typedef struct _stBaroData
{
    float   f32PressureOffset;
    float   f32Pressure;
    float   f32Temperature;

    uint8_t u8OffsetReady;
    uint16_t u16OffsetReadyCnt;

    float   f32LpfPara;
    float   f32LpfPressure;
    float   f32LpfPressureLast;

    float   f32AltitudeRaw;
    float   f32Altitude;
    float   f32LastAltitude;
    float   f32AltitudeOffset;

    float   f32AltitudeVelocityBuf[Altitude_Velocity_BUF_LEN];
    float   f32AltitudeVelocity;
    uint16_t u16AltitudeVelocityI;
    float   f32AltitudeVelocityF;
    float   f32AltitudeVelocityOld;

}stBaroData;


extern stBaroData g_BaroData;

uint8_t Hal_Baro_Init(void);
uint8_t Hal_Baro_IsReady(void);
void Hal_Baro_Update(void);
void Hal_Baro_UpdateOffset(void);
#endif
