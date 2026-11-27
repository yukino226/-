
#include <stdio.h>
#include <string.h>
#include "bsp.h"
#include "dev_mpu6887.h"


#define I2C_MPU6887_ADDRESS        (0x69 << 1)
stMpuRawData g_Mpu6887_Data;

void Dev_MPU6887_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf = data;
    Bsp_I2C_WriteBuf(I2C_MPU6887_ADDRESS, reg, &buf, 1);
}

uint8_t Dev_MPU6887_ReadReg(uint8_t reg)
{
    uint8_t buf = 0;
    Bsp_I2C_ReadBuf(I2C_MPU6887_ADDRESS, reg, &buf, 1);
    return buf;
}

void MPU6887_ReadRegs(uint8_t reg, uint8_t *pBuffer, uint16_t len)
{
    Bsp_I2C_ReadBuf(I2C_MPU6887_ADDRESS, reg, pBuffer, len);
}

uint8_t Dev_MPU6887_Init(void)
{
    if(Dev_MPU6887_ReadReg(MPU6887_WHO_AM_I)!= MPU6887_WHO_AM_I_VALUE)
    {
        return 0;
    }

    Dev_MPU6887_WriteReg(MPU6887_PWR_MGMT_1, MPU6887_RESET);
    delay_ms(1000); 
    Dev_MPU6887_WriteReg(MPU6887_PWR_MGMT_1, MPU6887_CLOCK_PLL);
    delay_ms(1000); 	
    Dev_MPU6887_WriteReg(MPU6887_SMPLRT_DIV, 0x00); // 1kHz / (1 + 1) = 500Hz
    Dev_MPU6887_WriteReg(MPU6887_CONFIG, 0x03); // 42Hz
    Dev_MPU6887_WriteReg(MPU6887_ACCEL_CONFIG2,0x05);
    Dev_MPU6887_WriteReg(MPU6887_GYRO_CONFIG, MPU6887_GYRO_FS_2000);
    Dev_MPU6887_WriteReg(MPU6887_ACCEL_CONFIG, MPU6887_ACCEL_FS_8);
    return 1;
}

void Dev_MPU6887_GetData(stMpuRawData *pData)
{
    uint8_t buffer[14];
    MPU6887_ReadRegs(MPU6887_ACCEL_XOUT_H|0x80, buffer, 14);
    g_Mpu6887_Data.s16Accel_x =  (int16_t)((buffer[0] << 8)  | buffer[1]);
    g_Mpu6887_Data.s16Accel_y =  (int16_t)((buffer[2] << 8)  | buffer[3]);
    g_Mpu6887_Data.s16Accel_z =  -(int16_t)((buffer[4] << 8)  | buffer[5]);
    g_Mpu6887_Data.s16Temp    =  (int16_t)((buffer[6] << 8)  | buffer[7]);
    g_Mpu6887_Data.s16Gyro_x  =  -(int16_t)((buffer[8] << 8) | buffer[9]);
    g_Mpu6887_Data.s16Gyro_y  =  -(int16_t)((buffer[10]  << 8) | buffer[11]);
    g_Mpu6887_Data.s16Gyro_z  =  (int16_t)((buffer[12] << 8) | buffer[13]);
    memcpy(pData, &g_Mpu6887_Data, sizeof(stMpuRawData));
    return;
}
