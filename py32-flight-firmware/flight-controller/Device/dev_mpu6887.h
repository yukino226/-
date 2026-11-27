#ifndef __DEV_MPU6887_H__
#define __DEV_MPU6887_H__

#include <stdint.h>

#define MPU6887_ADDR            0x68
#define MPU6887_WHO_AM_I        0x75
#define MPU6887_PWR_MGMT_1      0x6B
#define MPU6887_SMPLRT_DIV      0x19
#define MPU6887_CONFIG          0x1A
#define MPU6887_GYRO_CONFIG     0x1B
#define MPU6887_ACCEL_CONFIG    0x1C
#define MPU6887_ACCEL_CONFIG2   0x1D
#define MPU6887_INT_ENABLE      0x38
#define MPU6887_ACCEL_XOUT_H    0x3B
#define MPU6887_TEMP_OUT_H      0x41
#define MPU6887_GYRO_XOUT_H     0x43
#define MPU6887_USER_CTRL       0x6A
#define MPU6887_FIFO_EN         0x23
#define MPU6887_INT_PIN_CFG     0x37
#define MPU6887_INT_ENABLE      0x38
#define MPU6887_SIGNAL_PATH_RESET 0x68
#define MPU6887_USER_CTRL       0x6A
#define MPU6887_PWR_MGMT_1      0x6B
#define MPU6887_PWR_MGMT_2      0x6C
#define MPU6887_WHO_AM_I        0x75

#define MPU6887_RESET           0x80
#define MPU6887_CLOCK_PLL       0x03
#define MPU6887_GYRO_FS_250     0x00
#define MPU6887_GYRO_FS_500     0x08
#define MPU6887_GYRO_FS_1000    0x10
#define MPU6887_GYRO_FS_2000    0x18
#define MPU6887_ACCEL_FS_2      0x00
#define MPU6887_ACCEL_FS_4      0x08
#define MPU6887_ACCEL_FS_8      0x10
#define MPU6887_ACCEL_FS_16     0x18
#define MPU6887_WHO_AM_I_VALUE  0x0F

typedef struct _stMpuRawData {
    int16_t s16Gyro_x;
    int16_t s16Gyro_y;
    int16_t s16Gyro_z;
    int16_t s16Accel_x;
    int16_t s16Accel_y;
    int16_t s16Accel_z;
    int16_t s16Temp;
    int16_t reservations;
}stMpuRawData;

uint8_t Dev_MPU6887_Init(void);
void Dev_MPU6887_GetData(stMpuRawData *pData);
#endif
