#ifndef __HAL_MPU_H__
#define __HAL_MPU_H__

#include <stdint.h>
#include "string.h"

#define GYRO_SCALE                          0.06097561f
#define ACC_SCALE                           0.000244140625f

typedef struct _stMpuVector{
    float x;
    float y;
    float z;
}stMpuVector;

typedef struct _stMpuData {
 
    //Gyro Data Start

    float f32Gyro_x;
    float f32Gyro_y;
    float f32Gyro_z;

    //Filtered and Scaled Data
    float f32Gyro_x_dgree;
    float f32Gyro_y_dgree;
    float f32Gyro_z_dgree;

    float f32Gyro_x_off;
    float f32Gyro_y_off;
    float f32Gyro_z_off;
    
    float f32Gyro_fx_dgree;  
    float f32Gyro_fy_dgree;
    float f32Gyro_fz_dgree;
    float f32Gyro_fz_dgree_lpf;

    float f32Gyro_x_rad;
    float f32Gyro_y_rad;
    float f32Gyro_z_rad;

    float f32Gyro_fx_rad;
    float f32Gyro_fy_rad;
    float f32Gyro_fz_rad;

    //Gyro Data End

    //Accel Data Start
    float f32Accel_x;
    float f32Accel_y;
    float f32Accel_z;

    float f32Accel_x_g;
    float f32Accel_y_g;
    float f32Accel_z_g;

    float f32Accel_fx_g;
    float f32Accel_fy_g;
    float f32Accel_fz_g;

    float f32Accel_x_off;
    float f32Accel_y_off;
    float f32Accel_z_off;
    //Accel Data End
}stMpuData;
extern stMpuData g_stMpuData;

uint8_t Hal_Mpu_Init(void);
void Hal_Mpu_UpdateData(void);
void Hal_Mpu_SetGyroOffset(stMpuVector *v);
void Hal_Mpu_SetAccOffset(stMpuVector *v);
float Hal_Mpu_GetGyroLen(void);
float Hal_Mpu_GetAccLen(void);
#endif
