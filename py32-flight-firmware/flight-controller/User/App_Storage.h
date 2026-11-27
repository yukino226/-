
#ifndef __APP_STORAGE_H
#define __APP_STORAGE_H
#include "stdint.h"

#define SIGNED_TAG		        (0x55AA)
#pragma pack(1)
typedef struct _stMpuFlashData {
    float Falsh_Accel_nx_off;
    float Falsh_Accel_ny_off;
    float Falsh_Accel_nz_off;

    float Falsh_Gyro_nx_off;
    float Falsh_Gyro_ny_off;
    float Falsh_Gyro_nz_off;
}stMpuFlashData;

typedef struct _stFlashData
{
    uint16_t u16SignTag;
    stMpuFlashData stMpuFlashData;
    uint8_t u8CheckSum;
    uint8_t u8Reserved; // reserved for future use
}stFlashData;
#pragma pack()
extern stFlashData g_stStorage;
void App_StorageRecover(void);
void App_StorageInit(void);
uint8_t App_StorageSave(void);
#endif
