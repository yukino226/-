#include "bsp.h"
#include "main.h"
#include "hal.h"
#include "device.h"
#include "App_Storage.h"
#include "App_Ahrs.h"
#include "App_Plane.h"
#include "App_Led.h"
#include "App_Sound.h"


uint64_t tick_4ms = 0;
uint64_t tick_10ms = 0;
uint64_t tick_50ms = 0;
uint64_t tick_100ms = 0;
uint64_t tick_10s = 0;
volatile uint16_t timeout_poll = 0;

int main(void)
{
  /* Reset of all peripherals, Initializes the Systick. */
  HAL_Init();
  Bsp_Init();
  App_StorageInit();
  Dev_XN297L_Init();
  Hal_Mpu_Init();
  Hal_Baro_Init();
  App_Plane_Init();   
  App_AHRS_Init();
  App_Led_Init();
  App_Voltage_Init();
  tick_4ms = HAL_GetTick();
  tick_10ms = HAL_GetTick();
  tick_50ms = HAL_GetTick();
  tick_100ms = HAL_GetTick();
              
  while (1)
  {
      if((HAL_GetTick() - tick_4ms) >= 4)
      {
          if((HAL_GetTick() - tick_4ms) > 4)
          {
                timeout_poll++;
          }

          tick_4ms = HAL_GetTick();
          Hal_Mpu_UpdateData();
          App_AHRS_Update();
          App_Plane_Control();
      }

      if((HAL_GetTick() - tick_10ms) >= 10)
      {
          tick_10ms = HAL_GetTick();
          App_RcProtocol_Parser();
          App_Plane_High_Ctl();
      }

      if((HAL_GetTick() - tick_50ms) >= 50)
      {
          tick_50ms = HAL_GetTick();
          Hal_Baro_Update();
          App_Plane_Calibration(); 
      }

      if((HAL_GetTick() - tick_100ms) >= 100)
      {
          tick_100ms = HAL_GetTick();
          App_Led_Update();
      }

      if((HAL_GetTick() - tick_10s) >= 10000)
      {
            tick_10s = HAL_GetTick();
            App_Voltage_Check();
            // static uint8_t test_vol = 0;
            // test_vol++;
            // if(test_vol % 2)
            //     App_Sound_Play(1);
            // else
            //     App_Sound_Play(2);
      }
  }
}


/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
