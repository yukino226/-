#ifndef __DEV_TCBP001_H__
#define __DEV_TCBP001_H__
#include <stdint.h>
#include "string.h"

#define TCBP001TA_WRITE             0x00
#define TCBP001TA_READ              0x00

#define TCBP001TA_ADDR_CAL          0x10

#define TCBP001TA_ADDR_DATA         0x00
#define TCBP001TA_ADDR_PSR_DATA     0x00
#define TCBP001TA_ADDR_TMP_DATA     0x03

#define TCBP001TA_ADDR_PSR_CFG      0x06
#define TCBP001TA_ADDR_TMP_CFG      0x07
#define TCBP001TA_ADDR_MEAS_CFG     0x08
#define TCBP001TA_ADDR_CFG_REG      0x09

#define TCBP001TA_ADDR_INT_STS      0x0A
#define TCBP001TA_ADDR_FIFO_STS     0x0B
#define TCBP001TA_ADDR_RST          0x0C
#define TCBP001TA_ADDR_ID           0x0D

#define TCBP001TA_ADDR_TMP_COEF_SRCE   0x28

#define TCP001TA_ADDR_VALUE_ID       0x10
#define TCP001TA_ADDR_VALUE_RESET    0X89

#define TCBP001TA_MT_INIT            6400           //max measure time of initial p + t in us
#define TCBP001TA_MT                 2300          //max measure time of p or t in us

#define POW_2_23_MINUS_1             0x7FFFFF
#define POW_2_24                     0x1000000
#define POW_2_15_MINUS_1             0x7FFF
#define POW_2_16                     0x10000
#define POW_2_11_MINUS_1             0x7FF
#define POW_2_12                     0x1000
#define POW_2_20                     0x100000
#define POW_2_19_MINUS_1             0x7FFFF

#define Altitude_Velocity_BUF_LEN 8

typedef struct _stCalibration
{
    int16_t C0;
    int16_t C1;
    int32_t C00;
    int32_t C10;
    int16_t C01;
    int16_t C11;
    int16_t C20;
    int16_t C21;
    int16_t C30;
}stCalibration;

typedef struct _stBaroTcbp
{
    stCalibration Calibration;
    float   f32Pressure;
    float   f32Temperature;

    int32_t i32RawPressure;
    int32_t i32RawTemperature;
}stBaroTcbp;


extern stBaroTcbp g_BaroTcbp;
uint8_t Dev_TCBP001_Init(void);
void Dev_TCBP001_Update(void);
void Dev_TCBP0001_GetBaroData(void);
void Dev_TCBP001_Altitude_Velocity_Update(void);
#endif
