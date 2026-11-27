#ifndef __APP_SOUND_H
#define __APP_SOUND_H

#include "bsp.h"

typedef struct _stSound
{
    uint8_t u8Playing;
    uint8_t u8PlayType;
    uint8_t u8Process;
    uint32_t u32Cnt;
}stSound;

void App_Sound_Play(uint8_t type);
void App_Sound_Update(void);

#endif
