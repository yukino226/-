#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "py32f0xx_hal.h"
#include "stdio.h"


void Bsp_I2C_Init(void);
int8_t Bsp_I2C_WriteBuf(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);
int8_t Bsp_I2C_ReadBuf(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);
#endif
