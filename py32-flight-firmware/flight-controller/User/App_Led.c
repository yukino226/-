#include "bsp.h"
#include "App_Led.h"
#include "hal.h"
#include "device.h"
#include "App_Plane.h"
#include "App_Voltage.h"

stLed g_stLed;
void App_Led_Init(void)
{
    g_stLed = (stLed){
        .mode = LED_MODE_OFF,
        .duty = 0,
        .period = 1000
    };
}

void App_Led_Set(uint8_t mode, uint16_t duty, uint16_t period)
{
    g_stLed.mode = (eLedMode)mode;
    g_stLed.duty = duty;
    g_stLed.period = period;
}

void App_Led_Mode_Check(void)
{
    if(!g_BaroData.u8OffsetReady)
    {
       App_Led_Set(LED_MODE_BLINK, 3, 6);
    }
    else if((!Dev_XN297L_IsBind()) || App_Plane_IsOverLoad())
    {
        App_Led_Set(LED_MODE_BLINK, 15, 30);
    }
    else if(g_stPlane.eCalSt != CALIBRATION_STATE_NONE)
    {
        App_Led_Set(LED_MODE_BLINK, 2, 4);
    }
    else if (App_Voltage_IsLow())
    {
        App_Led_Set(LED_MODE_BLINK, 1, 2);
    }
    else
    {
        g_stLed.mode = LED_MODE_ON;
    }
}

void App_Led_Update(void)
{
    static uint32_t last_tick = 0;
    App_Led_Mode_Check();
    if (g_stLed.mode == LED_MODE_BLINK)
    {
        if ((HAL_GetTick() - last_tick) > g_stLed.period * 100)
        {
            last_tick = HAL_GetTick();
        }
        if ((HAL_GetTick() - last_tick) < g_stLed.duty * 100)
        {
            Bsp_Gpio_Set(GPIOA, GPIO_PIN_1);
        }
        else
        {
            Bsp_Gpio_Reset(GPIOA, GPIO_PIN_1);
        }
    }
    else if (g_stLed.mode == LED_MODE_ON)
    {
        Bsp_Gpio_Set(GPIOA, GPIO_PIN_1);
    }
    else
    {
        Bsp_Gpio_Reset(GPIOA, GPIO_PIN_1);
    }
}

