#include <stdio.h>
#include <math.h>
#include "bsp.h"
#include "dev_tcbp001.h"

//gyro (0x69 << 1)
#define I2C_TCBP001_ADDRESS        (0x76 << 1)
//#define I2C_TCBP001_ADDRESS        (0x77 << 1)
stBaroTcbp g_BaroTcbp;

void Dev_TCBP001_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf = data;
    Bsp_I2C_WriteBuf(I2C_TCBP001_ADDRESS, reg, &buf, 1);
}

uint8_t Dev_TCBP001_ReadReg(uint8_t reg)
{
    uint8_t buf = 0;
    Bsp_I2C_ReadBuf(I2C_TCBP001_ADDRESS, reg, &buf, 1);
    return buf;
}

void Dev_TCBP001_ReadRegs(uint8_t reg, uint8_t *pBuffer, uint16_t len)
{
    Bsp_I2C_ReadBuf(I2C_TCBP001_ADDRESS, reg, pBuffer, len);
    return;   
}

static uint32_t GetScalingCoef(uint8_t osr)
{
    switch(osr)
    {
        case 0:
            return 524288;
        case 1:
            return 1572864;
        case 2:
            return 3670016;
        case 3:
            return 7864320;
        case 4:
            return 253952;
        case 5:
            return 516096;
        case 6:
            return 1040384;
        case 7:
            return 2088960;
        default:
            return 524288;
    }
}

void Dev_TCBP001_Reset(void)
{
    Dev_TCBP001_WriteReg(TCBP001TA_ADDR_RST, TCBP001TA_WRITE|TCP001TA_ADDR_VALUE_RESET);
    delay_ms(1000);
}

uint8_t Dev_TCBP001_Init(void)
{
    uint8_t buf[18] = {0};
    Dev_TCBP001_Reset();

    delay_ms(1000);

    if(Dev_TCBP001_ReadReg(TCBP001TA_READ|TCBP001TA_ADDR_ID) != TCP001TA_ADDR_VALUE_ID)
    {
        return 0;
    }
    
    Dev_TCBP001_ReadRegs(TCBP001TA_READ|TCBP001TA_ADDR_CAL, buf, 18);

    g_BaroTcbp.Calibration.C0  = (buf[0] << 4) + ((buf[1] >> 4) & 0x0F);
    g_BaroTcbp.Calibration.C1  = (buf[2]) + ((buf[1] & 0x0F) << 8);
    g_BaroTcbp.Calibration.C00 = ((buf[3] << 12) + (buf[4] << 4) + ((buf[5] >> 4) & 0x0F)) & 0x000FFFFF;
    g_BaroTcbp.Calibration.C10 = (((buf[5] & 0x0F) << 16) + (buf[6] << 8) + (buf[7]))& 0x000FFFFF;
    g_BaroTcbp.Calibration.C01 = (buf[8] << 8) | (buf[9]);
    g_BaroTcbp.Calibration.C11 = (buf[10] << 8) | (buf[11]);
    g_BaroTcbp.Calibration.C20 = (buf[12] << 8) | (buf[13]);
    g_BaroTcbp.Calibration.C21 = (buf[14] << 8) | (buf[15]);
    g_BaroTcbp.Calibration.C30 = (buf[16] << 8) | (buf[17]);

    if(g_BaroTcbp.Calibration.C0 & 0x0800)
    {
        g_BaroTcbp.Calibration.C0 |= 0xF000;
    }
    
    if(g_BaroTcbp.Calibration.C1 & 0x0800)
    {
        g_BaroTcbp.Calibration.C1 |= 0xF000;
    }

    if(g_BaroTcbp.Calibration.C00 & 0x080000)
    {
        g_BaroTcbp.Calibration.C00 |= 0xFFF00000;
    }

    if(g_BaroTcbp.Calibration.C10 & 0x080000)
    {
        g_BaroTcbp.Calibration.C10 |= 0xFFF00000;
    }

    Dev_TCBP001_WriteReg(TCBP001TA_ADDR_PSR_CFG,  0x56);
    Dev_TCBP001_WriteReg(TCBP001TA_ADDR_TMP_CFG,  0xD6);
    Dev_TCBP001_WriteReg(TCBP001TA_ADDR_CFG_REG,  0x0C);

    Dev_TCBP001_WriteReg(TCBP001TA_ADDR_MEAS_CFG, 0x07);
    delay_ms(100);
    return 1;
}

void Dev_TCBP0001_GetBaroData(void)
{
    //if(g_BaroData.u8BaroReady)
    {
        uint8_t buf[3] = {0};
        buf[0] = Dev_TCBP001_ReadReg(TCBP001TA_READ|TCBP001TA_ADDR_MEAS_CFG);

        //check data ready and get data
        if(((buf[0]  & 0x10) == 0x10) ||((buf[0]  & 0x20) == 0x20))
        {
            if((buf[0]  & 0x10) == 0x10)
            {
                Dev_TCBP001_ReadRegs(TCBP001TA_READ|TCBP001TA_ADDR_PSR_DATA, buf, 3);
                g_BaroTcbp.i32RawPressure = (buf[0] << 16) | (buf[1] << 8) | buf[2];
                if(g_BaroTcbp.i32RawPressure & 0x800000)
                {
                    g_BaroTcbp.i32RawPressure |= 0xFF000000;
                }
            }

            if((buf[0]  & 0x20) == 0x20)
            {
                Dev_TCBP001_ReadRegs(TCBP001TA_READ|TCBP001TA_ADDR_TMP_DATA, buf, 3);
                g_BaroTcbp.i32RawTemperature = (buf[0] << 16) | (buf[1] << 8) | buf[2];
                if(g_BaroTcbp.i32RawTemperature & 0x800000)
                {
                    g_BaroTcbp.i32RawTemperature |= 0xFF000000;
                }
            }
    
            static float T_RawSc = 0;
            T_RawSc = ((float)g_BaroTcbp.i32RawTemperature/ GetScalingCoef(6));
            g_BaroTcbp.f32Temperature = (g_BaroTcbp.Calibration.C0 * 0.5f) + (float)(g_BaroTcbp.Calibration.C1 * T_RawSc);
            float P_RawSc = 0;
             P_RawSc = (float)g_BaroTcbp.i32RawPressure / GetScalingCoef(6);

            static float qua2 = 0;
             qua2 = g_BaroTcbp.Calibration.C10  +  P_RawSc * (g_BaroTcbp.Calibration.C20 + P_RawSc * g_BaroTcbp.Calibration.C30);
            static float qua3 = 0;
            qua3 = T_RawSc * P_RawSc * (g_BaroTcbp.Calibration.C11 + P_RawSc * g_BaroTcbp.Calibration.C21);

            g_BaroTcbp.f32Pressure = g_BaroTcbp.Calibration.C00 + 
                            P_RawSc * qua2 + T_RawSc * (float)g_BaroTcbp.Calibration.C01 + qua3;
        }
        else
        {
            return;
        }
    }
}
