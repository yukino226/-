#ifndef __APP_VOLTAGE_H
#define __APP_VOLTAGE_H

#include <stdint.h>

#define NUM_VOL_BUF  5

typedef struct _stVolSta
{
    float    f32VolValue;
    float    f32VolValueFilter;
    float    f32VolValueBuf[NUM_VOL_BUF];
}stVolSta;
extern stVolSta g_stVolSta;
uint8_t App_Voltage_Init(void);
void  App_Voltage_Check(void);
float App_Voltage_GetFlyBattery(void);
uint8_t App_Voltage_IsLow(void);

#endif




