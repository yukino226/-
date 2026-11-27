#include "bsp.h"
#include "App_Sound.h"

stSound g_stSound;

void App_Sound_Play(uint8_t type)
{
    if(g_stSound.u8Playing == 1)
    {
        return;
    }
    g_stSound.u8Playing = 1;
    g_stSound.u8Process = 0;
    g_stSound.u8PlayType = type;
    g_stSound.u32Cnt = 0;
    Bsp_Gpio_Reset(GPIOB, GPIO_PIN_7);
    Bsp_Timer_Start();
}

void App_Sound_Update(void)
{
   g_stSound.u32Cnt++;
   if(g_stSound.u8Playing)
   {
        switch(g_stSound.u8Process)
        {
            case 0:
            if(g_stSound.u32Cnt) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Set(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process++;
            }
            break;
            case 1:
            if(g_stSound.u32Cnt) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Reset(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process++;
            }
            break;
            case 2:
            if(g_stSound.u32Cnt >= 10) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Set(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process++;
            }
            break;
            case 3:
            if(g_stSound.u32Cnt) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Reset(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process++;
            }
            break; 
            case 4:
            if(g_stSound.u32Cnt >= 10) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Set(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process++;
            }
            break;    
            case 5:
            if(g_stSound.u32Cnt) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Reset(GPIOB, GPIO_PIN_7);
                if(g_stSound.u8PlayType == 0)
                {
                    g_stSound.u8Process = 0;
                    g_stSound.u8Playing = 0;
                    Bsp_Timer_Stop();
                }
                else
                    g_stSound.u8Process++;
            }
            break;
            case 6:
            if(g_stSound.u32Cnt) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Set(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process++;
            }
            break;            
            case 7:
            if(g_stSound.u32Cnt) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Reset(GPIOB, GPIO_PIN_7);
                if(g_stSound.u8PlayType == 1)
                {
                    g_stSound.u8Process = 0;
                    g_stSound.u8Playing = 0;
                    Bsp_Timer_Stop();
                }
                else
                    g_stSound.u8Process++;
            }
            break;
            case 8:
            if(g_stSound.u32Cnt) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Set(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process++;
            }
            break;            
            case 9:
            if(g_stSound.u32Cnt) 
            {
                g_stSound.u32Cnt = 0;
                Bsp_Gpio_Reset(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process = 0;
                g_stSound.u8Playing = 0;
                Bsp_Timer_Stop();
            }
            break;
            default:
                Bsp_Gpio_Reset(GPIOB, GPIO_PIN_7);
                g_stSound.u8Process = 0;
                g_stSound.u8Playing = 0;
                Bsp_Timer_Stop();
            break;
        }
   }
}

