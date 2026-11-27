#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "py32f0xx_hal.h"
#include "stdio.h"

void Bsp_SPI_Init(void);
uint8_t Bsp_SPI_TransmitReceive(uint8_t data);
void Bsp_SPI_Transmit(uint8_t* data, uint16_t size);
void Bsp_SPI_Receive(uint8_t* data, uint16_t size);
void Bsp_SPI_CS_Low(void);
void Bsp_SPI_CS_High(void);
#endif
