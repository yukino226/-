#ifndef __APP_LED_H
#define __APP_LED_H

#include "bsp.h"

typedef enum _eLedMode {
    LED_MODE_OFF = 0,
    LED_MODE_ON,
    LED_MODE_BLINK,
} eLedMode;

typedef struct _stLed {
    eLedMode mode;
    uint16_t duty;   // 0-1000
    uint16_t period; // ms
} stLed;

void App_Led_Init(void);
void App_Led_Update(void);
void App_Led_Set(uint8_t mode, uint16_t duty, uint16_t period);

#endif
