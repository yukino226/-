#include <stdio.h>
#include "device.h"
#include "hal_mpu.h"
#include "commom.h"

stMpuData g_stMpuData;

#define GYRO_FILTER_ENABLE       1
#define ACC_FILTER_ENABLE        0

#define GYRO_FILTER_WINDOWS      0
#define GYRO_FILTER_LPF          1
#define GYRO_SELECT              GYRO_FILTER_WINDOWS


#if (GYRO_FILTER_ENABLE)
#if (GYRO_SELECT == GYRO_FILTER_WINDOWS)
#define GYRO_WINDOW_WIDE         4        
float Gyro_fx_buf[GYRO_WINDOW_WIDE];
float Gyro_fy_buf[GYRO_WINDOW_WIDE];
float Gyro_fz_buf[GYRO_WINDOW_WIDE];
uint8_t u8GyroWindowN = 0;
static void Hal_MPU_Gyro_Filter(void)
{
    float Gyro_sum_fx = 0.0f;
    float Gyro_sum_fy = 0.0f;
    float Gyro_sum_fz = 0.0f;

    Gyro_fx_buf[u8GyroWindowN] = g_stMpuData.f32Gyro_x_dgree;
    Gyro_fy_buf[u8GyroWindowN] = g_stMpuData.f32Gyro_y_dgree;
    Gyro_fz_buf[u8GyroWindowN] = g_stMpuData.f32Gyro_z_dgree;
    
    u8GyroWindowN++;

    if (u8GyroWindowN >= GYRO_WINDOW_WIDE)
    {
        u8GyroWindowN = 0;
    }

    Gyro_sum_fx = 0.0f;
    Gyro_sum_fy = 0.0f;    
    Gyro_sum_fz = 0.0f;
    
    for (uint16_t i = 0; i < GYRO_WINDOW_WIDE; i++)
    {
        Gyro_sum_fx += Gyro_fx_buf[i];
        Gyro_sum_fy += Gyro_fy_buf[i];
        Gyro_sum_fz += Gyro_fz_buf[i];
    }

    g_stMpuData.f32Gyro_fx_dgree = Gyro_sum_fx / (float)(GYRO_WINDOW_WIDE);
    g_stMpuData.f32Gyro_fy_dgree = Gyro_sum_fy / (float)(GYRO_WINDOW_WIDE);
    g_stMpuData.f32Gyro_fz_dgree = Gyro_sum_fz / (float)(GYRO_WINDOW_WIDE);
}

#elif (GYRO_SELECT == GYRO_FILTER_LPF)
#include  "Filter_LPF.h"
Butter_Parameter Gyro_Parameter={
    //200hz---30hz
    //1,  -0.7477891782585,    0.272214937925,
    //0.1311064399166,   0.2622128798333,   0.1311064399166
    //200hz---51hz
    1,  0.03680751639284,   0.1718123812701,
    0.3021549744157,   0.6043099488315,   0.3021549744157
};
Butter_BufferData Gyro_BufferData[3];

static void Hal_MPU_Gyro_Filter(void)
{
    g_stMpuData.f32Gyro_fx_dgree = LPButterworth(g_stMpuData.f32Gyro_x_dgree, &Gyro_BufferData[0], &Gyro_Parameter);
    g_stMpuData.f32Gyro_fy_dgree = LPButterworth(g_stMpuData.f32Gyro_y_dgree, &Gyro_BufferData[1], &Gyro_Parameter);
    g_stMpuData.f32Gyro_fz_dgree = LPButterworth(g_stMpuData.f32Gyro_z_dgree, &Gyro_BufferData[2], &Gyro_Parameter);
}

#else
#endif
#endif


#if (ACC_FILTER_ENABLE)
#define ACC_WINDOW_WIDE         8        
float Acc_fx_buf[ACC_WINDOW_WIDE];
float Acc_fy_buf[ACC_WINDOW_WIDE];
float Acc_fz_buf[ACC_WINDOW_WIDE];
uint8_t u8AccWindowN = 0;
static void Hal_MPU_Acc_Filter(void)
{
    float Acc_sum_fx = 0.0f;
    float Acc_sum_fy = 0.0f;
    float Acc_sum_fz = 0.0f;

    Acc_fx_buf[u8AccWindowN] = g_stMpuData.f32Accel_x_g;
    Acc_fy_buf[u8AccWindowN] = g_stMpuData.f32Accel_y_g;
    Acc_fz_buf[u8AccWindowN] = g_stMpuData.f32Accel_z_g;
    
    u8AccWindowN++;

    if (u8AccWindowN >= ACC_WINDOW_WIDE)
    {
        u8AccWindowN = 0;
    }

    Acc_sum_fx = 0.0f;
    Acc_sum_fy = 0.0f;    
    Acc_sum_fz = 0.0f;
    
    for (uint16_t i = 0; i < ACC_WINDOW_WIDE; i++)
    {
        Acc_sum_fx += Acc_fx_buf[i];
        Acc_sum_fy += Acc_fy_buf[i];
        Acc_sum_fz += Acc_fz_buf[i];
    }

    g_stMpuData.f32Accel_fx_g = Acc_sum_fx / (float)(ACC_WINDOW_WIDE);
    g_stMpuData.f32Accel_fy_g = Acc_sum_fy / (float)(ACC_WINDOW_WIDE);
    g_stMpuData.f32Accel_fz_g = Acc_sum_fz / (float)(ACC_WINDOW_WIDE);
}
#endif

/********************************************/
uint8_t Hal_Mpu_Init(void)
{ 
    if(Dev_MPU6887_Init() == 0)
    {
        while(1);
    }
    return 1;
}

static float Gyro_Len = 0.0f;
static float Acc_Len = 0.0f;

void Hal_Mpu_UpdateData(void)
{
    stMpuRawData RawData;
    Dev_MPU6887_GetData(&RawData);
    //Gyro Date Update
    g_stMpuData.f32Gyro_x = (float)RawData.s16Gyro_x;
    g_stMpuData.f32Gyro_y = (float)RawData.s16Gyro_y;
    g_stMpuData.f32Gyro_z = (float)RawData.s16Gyro_z;

    g_stMpuData.f32Gyro_x_dgree = (g_stMpuData.f32Gyro_x - g_stMpuData.f32Gyro_x_off) * GYRO_SCALE;
    g_stMpuData.f32Gyro_y_dgree = (g_stMpuData.f32Gyro_y - g_stMpuData.f32Gyro_y_off) * GYRO_SCALE;
    g_stMpuData.f32Gyro_z_dgree = (g_stMpuData.f32Gyro_z - g_stMpuData.f32Gyro_z_off) * GYRO_SCALE;

    static float f32Gyro_x_dgree_sum = 0;
    static float f32Gyro_y_dgree_sum = 0;
    f32Gyro_x_dgree_sum += g_stMpuData.f32Gyro_x_dgree;
    f32Gyro_y_dgree_sum += g_stMpuData.f32Gyro_y_dgree;

    g_stMpuData.f32Gyro_x_rad  = g_stMpuData.f32Gyro_x_dgree * DEG2RAD;
    g_stMpuData.f32Gyro_y_rad  = g_stMpuData.f32Gyro_y_dgree * DEG2RAD;
    g_stMpuData.f32Gyro_z_rad  = g_stMpuData.f32Gyro_z_dgree * DEG2RAD;

#if (GYRO_FILTER_ENABLE == 0)
    g_stMpuData.f32Gyro_fx_dgree = g_stMpuData.f32Gyro_x_dgree;
    g_stMpuData.f32Gyro_fy_dgree = g_stMpuData.f32Gyro_y_dgree;
    g_stMpuData.f32Gyro_fz_dgree = g_stMpuData.f32Gyro_z_dgree;
#else
    Hal_MPU_Gyro_Filter();
#endif
    g_stMpuData.f32Gyro_fx_rad  = g_stMpuData.f32Gyro_x_dgree * DEG2RAD;
    g_stMpuData.f32Gyro_fy_rad  = g_stMpuData.f32Gyro_y_dgree * DEG2RAD;
    g_stMpuData.f32Gyro_fz_rad  = g_stMpuData.f32Gyro_z_dgree * DEG2RAD;
    

    //Accel Data Update
    g_stMpuData.f32Accel_x = (float)RawData.s16Accel_x;
    g_stMpuData.f32Accel_y = (float)RawData.s16Accel_y;
    g_stMpuData.f32Accel_z = (float)RawData.s16Accel_z;

    g_stMpuData.f32Accel_x_g = (g_stMpuData.f32Accel_x - g_stMpuData.f32Accel_x_off) * ACC_SCALE;
    g_stMpuData.f32Accel_y_g = (g_stMpuData.f32Accel_y - g_stMpuData.f32Accel_y_off) * ACC_SCALE;
    g_stMpuData.f32Accel_z_g = (g_stMpuData.f32Accel_z - g_stMpuData.f32Accel_z_off) * ACC_SCALE;

#if (ACC_FILTER_ENABLE == 0)
    g_stMpuData.f32Accel_fx_g = g_stMpuData.f32Accel_x_g;
    g_stMpuData.f32Accel_fy_g = g_stMpuData.f32Accel_y_g;
    g_stMpuData.f32Accel_fz_g = g_stMpuData.f32Accel_z_g;
#else
    Hal_MPU_Acc_Filter();
#endif


    Gyro_Len = (g_stMpuData.f32Gyro_fx_dgree * g_stMpuData.f32Gyro_fx_dgree\
        + g_stMpuData.f32Gyro_fy_dgree * g_stMpuData.f32Gyro_fy_dgree\
        + g_stMpuData.f32Gyro_fz_dgree * g_stMpuData.f32Gyro_fz_dgree);
    Acc_Len = (g_stMpuData.f32Accel_fx_g * g_stMpuData.f32Accel_fx_g\
        + g_stMpuData.f32Accel_fy_g * g_stMpuData.f32Accel_fy_g\
        + g_stMpuData.f32Accel_fz_g * g_stMpuData.f32Accel_fz_g);
}

void Hal_Mpu_SetGyroOffset(stMpuVector *v)
{
    g_stMpuData.f32Gyro_x_off = v->x;
    g_stMpuData.f32Gyro_y_off = v->y;
    g_stMpuData.f32Gyro_z_off = v->z;
}

void Hal_Mpu_SetAccOffset(stMpuVector *v)
{
    g_stMpuData.f32Accel_x_off = v->x;
    g_stMpuData.f32Accel_y_off = v->y;
    g_stMpuData.f32Accel_z_off = v->z;
}

float Hal_Mpu_GetGyroLen(void)
{
    return Gyro_Len;
}

float Hal_Mpu_GetAccLen(void)
{
    return Acc_Len;
}
