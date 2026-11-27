// Copyright 2021 IOsetting <iosetting(at)outlook.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include "dev_xn297l.h"
#include "bsp.h"


#define SPI_NX297L_BUS    1
#define XN297L_CS_L()     Bsp_SPI_CS_Low()
#define XN297L_CS_H()     Bsp_SPI_CS_High()

#define XN297L_SEND_TIMEOUT     100
#define XN279L_RECV_TIMEOUT     100

const uint8_t XN297L_BIND_ADDR[XN297L_ADDR_LEN] = {0x58, 0x4E, 0x32, 0x37};

uint8_t  BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46};
uint8_t  RF_cal_data[]    = {0xF6,0x37,0x5D};
uint8_t  RF_cal2_data[]   = {0x45,0x21,0xEF,0xAC,0x5A,0x50};
uint8_t  Dem_cal_data[]   = {0x01};
uint8_t  Dem_cal2_data[]  = {0x0B,0xDF,0x02};

RCTPParama g_stRCData;

stXN297L g_XN297L = {
    .u8Ready = 0,
    .u8RecvDataReady = 0,
    .u8Channel = XN297L_CHANNEL,
    .u8Mode = 0,
    .u8PayloadWidth = XN297L_PLOAD_WIDTH,
    .u16SendTimeCnt = 0,
};


static void XN297L_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    XN297L_CS_L();
    Bsp_SPI_Transmit(&reg, 1);
    Bsp_SPI_Transmit(pBuf, len);
    XN297L_CS_H();
}

static void XN297L_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    XN297L_CS_L();
    Bsp_SPI_Transmit(&reg, 1);
    Bsp_SPI_Receive(pBuf, len);
    XN297L_CS_H();
}

static void XN297L_WriteReg(uint8_t reg, uint8_t byte)
{
    uint8_t buf[2] = {0};
    buf[0] = reg;
    buf[1] = byte;
    XN297L_CS_L();
    Bsp_SPI_Transmit(buf, 2);
    XN297L_CS_H();
}

static uint8_t XN297L_ReadReg(uint8_t reg)
{
    uint8_t byte = 0;
    XN297L_CS_L();
    Bsp_SPI_Transmit(&reg, 1);
    Bsp_SPI_Receive(&byte, 1);
    XN297L_CS_H();
    return byte;
}

static void XN297L_CE_HIGH()
{
    XN297L_WriteReg(XN297L_CMD_CE_FSPI_ON, 0x00);
}

static void XN297L_CE_LOW()
{
    XN297L_WriteReg(XN297L_CMD_CE_FSPI_OFF, 0x00);
}

static uint8_t XN297L_Check(void)
{
    uint8_t i;
    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_TX_ADDR, (uint8_t *)g_XN297L.u8Address,XN297L_ADDR_LEN);
    XN297L_ReadBuf(XN297L_CMD_R_REGISTER | XN297L_REG_TX_ADDR,  g_XN297L.pPayload, XN297L_ADDR_LEN);
    for (i = 0; i < XN297L_ADDR_LEN; i++) 
    {
        if (*(g_XN297L.pPayload + i) != g_XN297L.u8Address[i]) 
            return 0;
    }
    return 1;
}

static void XN297L_SetMode(uint8_t mode)
{
    if(mode == XN297L_TX_MODE)
    {
        XN297L_CE_LOW();
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_CMD_FLUSH_TX, 0);
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_CMD_FLUSH_RX, 0);
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG,  0X8E);
        g_XN297L.u8Mode = XN297L_TX_MODE;
    }
    else if(mode == XN297L_RX_MODE)
    {
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_CMD_FLUSH_TX, 0);
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_CMD_FLUSH_RX, 0);
        XN297L_CE_LOW();
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_CONFIG,  0X8F);
        XN297L_CE_HIGH();   
        g_XN297L.u8Mode = XN297L_RX_MODE;      
    }
}

static uint8_t XN297L_ReadStatus(void)
{
    return XN297L_ReadReg(XN297L_CMD_R_REGISTER | XN297L_REG_STATUS);
}

static void XN297L_ClearStatus(void)
{
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, 0x70);
}

//static void XN297L_ReuseTX(void)
//{
//    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, XN297L_FLAG_MAX_RT); //Clear max retry flag
//}

uint8_t Dev_XN297L_Init(void)
{
    //init spi cs pin pb12
    // gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    // XN297L_CS_H();

    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0x5A); // Soft reset

    delay_ms(10); // Wait for reset to complete

    XN297L_WriteReg(XN297L_CMD_RST_FSPI, 0XA5);

    if(XN297L_Check() == 0)
    {
        //printf("XN297L not found!\r\n");
        return 0;
    }
    else
    {
        //printf("XN297L found!\r\n");
    }

    if(XN297L_PLOAD_WIDTH >32)
    {
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_FEATURE, XN297L_FEATURE_BIT5_CE_SOFT|XN297L_FEATURE_BIT43_DATA_64BYTE);
    }
    else
    {
        XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_FEATURE, XN297L_FEATURE_BIT5_CE_SOFT);
    }

    XN297L_CE_LOW();
    XN297L_WriteReg(XN297L_CMD_FLUSH_TX, 0);
    XN297L_WriteReg(XN297L_CMD_FLUSH_RX, 0);
    Dev_XN297L_SetAddress(XN297L_BIND_ADDR);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_STATUS, 0x70);       // Clear status flags
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_EN_AA, 0x00);        // AutoAck on all pipes
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_EN_RXADDR, 0x21);    // Enable all pipes (P0 ~ P5, bit0 ~ bit5)
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_SETUP_AW, XN297L_SETUP_AW_4BYTE); // Address width
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CH, 65);          // Channel 78, 2478M HZ
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P0, XN297L_PLOAD_WIDTH ); // Payload width of P0
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P1, XN297L_PLOAD_WIDTH ); // Payload width of P1
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P2, XN297L_PLOAD_WIDTH ); // Payload width of P2
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P3, XN297L_PLOAD_WIDTH ); // Payload width of P3
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P4, XN297L_PLOAD_WIDTH ); // Payload width of P4
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RX_PW_P5, XN297L_PLOAD_WIDTH ); // Payload width of P5

    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_DYNPD, 0x21); // Dynamic payload width: off
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_SETUP,  XN297L_RF_POWER_P_9|XN297L_RF_DR_1M); // 9dbm 1Mbps
    XN297L_WriteReg(XN297L_CMD_ACTIVATE, 0x73);
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_SETUP_RETR, 0x25); // Retry interval 500µs, 5 times
    XN297L_CE_HIGH();
    XN297L_SetMode(XN297L_RX_MODE);
    g_XN297L.u8Ready = 1;
    return 1;
}

uint8_t Dev_XN297L_IsReady(void)
{
    return g_XN297L.u8Ready;
}

void Dev_XN297L_SetChannel(uint8_t channel)
{   
    g_XN297L.u8Channel = channel;
    XN297L_WriteReg(XN297L_CMD_W_REGISTER | XN297L_REG_RF_CH, channel);
    XN297L_SetMode(XN297L_RX_MODE);
}

void Dev_XN297L_SetAddress(const uint8_t *address)
{
    memcpy(g_XN297L.u8Address, address,XN297L_ADDR_LEN);
    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_TX_ADDR, (uint8_t *)address, XN297L_ADDR_LEN);
    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RX_ADDR_P0, (uint8_t *)address, XN297L_ADDR_LEN);    
    XN297L_WriteBuf(XN297L_CMD_W_REGISTER | XN297L_REG_RX_ADDR_P5, (uint8_t *)address, XN297L_ADDR_LEN);
}

uint8_t Dev_XN297L_SendData(uint8_t *ucPayload, uint8_t length)
{
    XN297L_SetMode(XN297L_TX_MODE);
    g_XN297L.u16SendTimeCnt = 0;
    XN297L_CE_HIGH();
    XN297L_WriteBuf(XN297L_CMD_W_TX_PAYLOAD, ucPayload, length);
    XN297L_CE_LOW();
    return 1;
}

uint8_t Dev_XN297L_IsBind(void)
{
    return g_XN297L.u8IsBind;
}

uint8_t Dev_XN297L_SetUnbind(void)
{
    g_XN297L.u8IsBind = 0;
    memcpy(g_XN297L.u8Address, XN297L_BIND_ADDR, XN297L_ADDR_LEN);
    g_XN297L.u8Channel = 78;    
    Dev_XN297L_SetAddress(g_XN297L.u8Address);
    Dev_XN297L_SetChannel(g_XN297L.u8Channel);
    XN297L_ClearStatus();
    XN297L_SetMode(XN297L_RX_MODE);
    return 1;
}

uint8_t Dev_XN297L_Process(void)
{
    uint8_t u8Status = 0;
    u8Status = XN297L_ReadStatus();
    #if 0
    if(!g_XN297L.u8IsBind)
    {
        if(g_XN297L.u8Mode == XN297L_RX_MODE)
        {
            if (u8Status & XN297L_FLAG_RX_DR)
            {
                g_XN297L.u8PayloadWidth = XN297L_ReadReg(XN297L_CMD_R_RX_PL_WID);
                XN297L_ReadBuf(XN297L_CMD_R_RX_PAYLOAD, g_XN297L.pPayload, g_XN297L.u8PayloadWidth);
                printf("Bind Msg Recv\n\r");
                printf("Address: %x %x %x %x\n\r",g_XN297L.pPayload[0],g_XN297L.pPayload[1],g_XN297L.pPayload[2],g_XN297L.pPayload[3]);
                printf("Channel: %x\n\r",g_XN297L.pPayload[4]);
                Dev_XN297L_SendData(g_XN297L.pPayload, g_XN297L.u8PayloadWidth);
                XN297L_ClearStatus();
                return 1;
            }
        }
        else if (g_XN297L.u8Mode == XN297L_TX_MODE)
        {
            if ((u8Status & XN297L_FLAG_TX_DS)
             || (u8Status & XN297L_FLAG_MAX_RT)
             || (g_XN297L.u16SendTimeCnt++ > XN297L_SEND_TIMEOUT))
            {
                g_XN297L.u8IsBind = 1;
                memcpy(g_XN297L.u8Address, g_XN297L.pPayload, XN297L_ADDR_LEN);
                g_XN297L.u8Channel = g_XN297L.pPayload[XN297L_ADDR_LEN];
                //printf("Bind remote\n\r");
                Dev_XN297L_SetAddress(g_XN297L.u8Address);
                Dev_XN297L_SetChannel(g_XN297L.u8Channel);
                XN297L_ClearStatus();
                XN297L_SetMode(XN297L_RX_MODE);
            }
        }
    }
    else
    #endif
    {
        if(g_XN297L.u8Mode == XN297L_RX_MODE)
        {
            if (u8Status & XN297L_FLAG_RX_DR)
            {
                g_XN297L.u8PayloadWidth = 8;//XN297L_ReadReg(XN297L_CMD_R_RX_PL_WID);
                XN297L_ReadBuf(XN297L_CMD_R_RX_PAYLOAD, g_XN297L.pPayload, g_XN297L.u8PayloadWidth);
                XN297L_ClearStatus();
                g_XN297L.u8RecvDataReady = 1;
                return 1;
            }
        }
        else if (g_XN297L.u8Mode == XN297L_TX_MODE)
        {
            if ((u8Status & XN297L_FLAG_TX_DS) || (u8Status & XN297L_FLAG_MAX_RT))
            {
                XN297L_ClearStatus();
                XN297L_SetMode(XN297L_RX_MODE);   
            }
        }
    }
    return 0;
}


uint8_t Dev_XN297L_GetPacket(uint8_t* buf)
{
    if(g_XN297L.u8RecvDataReady == 1)
    {
        memcpy(buf, g_XN297L.pPayload, g_XN297L.u8PayloadWidth);
        g_XN297L.u8RecvDataReady = 0;
        return g_XN297L.u8PayloadWidth;
    }
    return 0;
}

void App_RcProtocol_Parser(void)
{ 
    uint8_t RC_buf[8];
	static uint16_t u16RecvCnt = 0;
	Dev_XN297L_Process();
	if((Dev_XN297L_GetPacket(RC_buf) == 0))
	{	
            u16RecvCnt++;
            if(g_XN297L.u8IsBind == 1)
            {
                if(u16RecvCnt > 400)
                {	
                    g_XN297L.u8IsBind = 0;
                    u16RecvCnt = 0;
                }
                else if(u16RecvCnt > 200)
                {
                    g_stRCData.RC_TROTTLE_Receive   = 1520;	
                    g_stRCData.RC_YAW_Receive       = 1520;
                    g_stRCData.RC_ROLL_Receive      = 1520;
                    g_stRCData.RC_PITCH_Receive     = 1520;	
                }
            }
            return;
	}
	u16RecvCnt = 0;
	g_XN297L.u8IsBind = 1;

	//check header
	if(RC_buf[0] !=  0xA5)
	{
	   return;
	}
	g_stRCData.RC_YAW_Receive      = ((255 - RC_buf[1]) * 1000/255 + 1020);
	g_stRCData.RC_TROTTLE_Receive  = (RC_buf[2] * 1000/255 + 1020);
	g_stRCData.RC_PITCH_Receive    = ((255 - RC_buf[3]) * 1000/255 + 1020);
	g_stRCData.RC_ROLL_Receive     = ((RC_buf[4]) * 1000/255 + 1020);   
    
    static uint8_t u8Language1 = 0;
    if(u8Language1 != (RC_buf[6] & 0x01))
    {
        u8Language1 = (RC_buf[6] & 0x01);
        if(u8Language1 != 0)
        {
            App_Sound_Play(1);
        }
    }

    static uint8_t u8Language2 = 0;
    if(u8Language2 != (RC_buf[6] & 0x02))
    {
        u8Language2 = (RC_buf[6] & 0x02);
        if(u8Language2 != 0)
        {
            App_Sound_Play(2);
        }
    }

    
    static uint8_t u8Speed = 0;
    if(u8Speed != (RC_buf[5] & 0x08))
    {
        u8Speed = (RC_buf[5] & 0x08);
        if(u8Speed != 0)
        {
            App_Plane_Change_Speed(2);
        }
    }
}
