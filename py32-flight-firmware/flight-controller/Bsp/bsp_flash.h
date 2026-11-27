#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include "py32f0xx_hal.h"
#include "stdio.h"

int8_t Bsp_Flash_Write(uint32_t *buffer, uint32_t size);
int8_t Bsp_Flash_Read(uint32_t *buffer, uint32_t size);
#endif
