#include "stdio.h"
#include "bsp_adc.h"
#include "App_Voltage.h"

//need change
stVolSta g_stVolSta;

uint8_t App_Voltage_Init(void)
{
	g_stVolSta.f32VolValueFilter = g_stVolSta.f32VolValue = (Get_ADC_val() * 2.8f) / 4095.0f * 2.0f;
	for(uint8_t i = 0; i < NUM_VOL_BUF; i++)
			g_stVolSta.f32VolValueBuf[i] = g_stVolSta.f32VolValue;
	return 0;
}

float App_Voltage_GetFlyBattery(void)
{
	return g_stVolSta.f32VolValue;
}


void App_Voltage_Check(void)
{
	float f32Vol, lv, mv;
	float adc_value = 0;
	float v_out;
	static uint16_t index = 0;
	uint16_t k = 0;
	float sum = 0;
	adc_value = Get_ADC_val();
	v_out = (adc_value * 2.8f) / 4095.0f; // �?? ADC 数据�??�??为电�??
    f32Vol = v_out * 2.0f;    // 使用分压�??式�?�算实际电压
	g_stVolSta.f32VolValue = f32Vol;
	
	g_stVolSta.f32VolValueBuf[index]  = f32Vol;
	index++;
	if(index >= NUM_VOL_BUF)
	{
		index = 0;
	}

	for(uint8_t i = 0; i < NUM_VOL_BUF; i++)
	{
		sum += g_stVolSta.f32VolValueBuf[i];
	}

	g_stVolSta.f32VolValueFilter = sum / NUM_VOL_BUF;
#if 0
	g_stVolSta.f32VolWinData[g_stVolSta.u16VolWinIndex] = f32Vol;
	g_stVolSta.u16VolWinIndex++;
	if (g_stVolSta.u16VolWinIndex >= VOL_WINDOW_WIDE) 
        {
		g_stVolSta.u16VolWinIndex = 0;
        }
	g_stVolSta.f32VolSum = 0;
	for (k = 0; k < VOL_WINDOW_WIDE; k++)
	{
		g_stVolSta.f32VolSum += g_stVolSta.f32VolWinData[k];
	}
	f32Vol = (g_stVolSta.f32VolSum / (float)(VOL_WINDOW_WIDE));
	g_stVolSta.f32VolValue = f32Vol;

	if ((g_BaroData.position_z >= 40))
	{
		lv = 10.025f; 
		mv = 10.175f; 
	}
	else if (g_BaroData.position_z >= 25)
	{
		lv = 10.025f;
		mv = 10.145f; 
	}
	else if (g_BaroData.position_z >= 10)
	{
		lv = 10.98f;
		mv = 10.13f;
	}
	else
	{ 
		lv = 9.95f; 
		mv = 9.1f; 
	}
	if (lv <= 10.5f)
		lv += (10.5f/100);

	if (g_stVolSta.u8VolAlarmEvent != VOL_ALARM_EVENT_LOW_LANDS)
	{
		if (f32Vol == 0)
		{
			if (g_stVolSta.u8VolAlarmEvent != VOL_ALARM_EVENT_LOW)
			{
				i = 0;
			}
			j = 0;
			g_stVolSta.u8VolAlarmEvent = 0x00;
		}
		else if (f32Vol < lv)
		{
			j = 0;
			if (i > 100)
			{
				g_stVolSta.u8VolAlarmEvent = VOL_ALARM_EVENT_LOW_LANDS;
			}
			else if (i > 35)
			{
				g_stVolSta.u8VolAlarmEvent = VOL_ALARM_EVENT_LOW;
				i++;
			}
			else
			{
				i++;
			}
		}
		else if ((f32Vol < mv))
		{
			if (g_stVolSta.u8VolAlarmEvent != VOL_ALARM_EVENT_LOW)
                        {
				i = 0;
                        }
			if (j > 18)
			{
				g_stVolSta.u8VolAlarmEvent = VOL_ALARM_EVENT_LOW_LANDS;
			}
			else
                        {
				j++;
                        }
		}
		else
		{
			if (g_stVolSta.u8VolAlarmEvent != VOL_ALARM_EVENT_LOW)
                        {
				i = 0;
                        }
			j = 0;
			g_stVolSta.u8VolAlarmEvent = VOL_ALARM_EVENT_NORMAL;
		}
	}

	if (f32Vol <= 10.5f)
	{
		if (m >= 10)
		{
			g_stVolSta.u8VolAlarmEvent = VOL_ALARM_EVENT_LOW_LANDS;
			i = 130;
		}
		m++;
	}
	else
	{
		m = 0;
	}
	#endif
}

#define VOL_THR       3.3f
uint8_t App_Voltage_IsLow(void)
{
	return (g_stVolSta.f32VolValueFilter < VOL_THR)? 1:0;
}