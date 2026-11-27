#include "stdio.h"
#include "App_Storage.h"
#include "string.h"
#include "bsp.h"

stFlashData g_stStorage;						   
void App_StorageRecover(void)
{
	g_stStorage.stMpuFlashData.Falsh_Gyro_nx_off = 0;
	g_stStorage.stMpuFlashData.Falsh_Gyro_ny_off = 0;
	g_stStorage.stMpuFlashData.Falsh_Gyro_nz_off = 0;

	g_stStorage.stMpuFlashData.Falsh_Accel_nx_off = 0;
	g_stStorage.stMpuFlashData.Falsh_Accel_ny_off = 0;
	g_stStorage.stMpuFlashData.Falsh_Accel_nz_off = 0;
	App_StorageSave();
}

uint8_t u8CheckSum = 0;
uint16_t u16SizeStorage; // exclude checksum
void App_StorageInit(void)
{
	uint8_t *data;
	u16SizeStorage = sizeof(stFlashData); // exclude checksum
	Bsp_Flash_Read((uint32_t*)&g_stStorage, sizeof(stFlashData));
	data = (uint8_t*)&(g_stStorage.stMpuFlashData);
	for(uint8_t i = 0; i < sizeof(stMpuFlashData); i++)
	{
		u8CheckSum += data[i];
	}

	if ((g_stStorage.u16SignTag != SIGNED_TAG)
	|| (u8CheckSum != g_stStorage.u8CheckSum))
	{
		App_StorageRecover();
	}
}


uint8_t App_StorageSave(void)
{
	uint8_t flag = 1;
	g_stStorage.u16SignTag = SIGNED_TAG;
	uint8_t *data;
	data = (uint8_t*)&(g_stStorage.stMpuFlashData);
	g_stStorage.u8CheckSum = 0;
	for(uint8_t i = 0; i < sizeof(stMpuFlashData); i++)
	{
		g_stStorage.u8CheckSum += data[i];
	}
	Bsp_Flash_Write((uint32_t*)&g_stStorage, sizeof(stFlashData));
	return flag;
}


