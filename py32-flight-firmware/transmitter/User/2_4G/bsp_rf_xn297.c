/**
 * @file rf_xn297.c
 * @author zhou.jie
 * @brief  XN297 2.4G驱动文件
 * @version 0.1
 * @date 2024-10-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "2_4G/bsp_rf_xn297.h"
#include "systick/bsp_systick_delay.h"

const uint8_t XN297L_BIND_ADDR[XN297L_ADDR_LEN] = {0x58, 0x4E, 0x32, 0x37};
uint8_t RF_CHANNEL_TABLE[RF_CH_TABLE_MAX] = {20, 40, 65};

/******************************************************************************/
// !!!!!!!!!!!!!!!!!!!!该部分内容需要修改!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// @brief  实现100us延时,该部分内容需要用户根据自己实际使用的MCU做修改
// @param  n_100us：需要延时的时间，单位为100us
// @retval  None
/******************************************************************************/
void delay_100us(uint16_t n_100us)
{
#if 0
    uint16_t i,j;
    for(i=0;i<n_100us;i++)
    {
        __NOP();
        __NOP();
        __NOP();
        j = 598u;
        while(j)
        {
            j--;
        }
    }
#else
    Delay_us(100 * n_100us);
#endif
}

/******************************************************************************/
// !!!!!!!!!!!!!!!!!!!!该部分内容需要修改!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// @brief  实现1ms延时,该部分内容需要用户根据自己实际使用的MCU做修改
// @param  n_ms：需要延时的时间，单位为ms
// @retval None
/******************************************************************************/
void delay_ms(uint16_t n_ms)
{
#if 0
	uint16_t i;

    __NOP();
    __NOP();
    __NOP();
	i = 5998u;
    while(i)
    {
        i--;
    }
#else
    Delay_ms(n_ms);
#endif
}
/**
 * @brief SOFT_SPI_init
 *
 */
void SOFT_SPI_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    SPI_CSN_GPIO_CLK_ENABLE();
    SPI_SCK_GPIO_CLK_ENABLE();
    SPI_DATA_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /* SPI_CSN GPIO*/
    GPIO_InitStruct.Pin = SPI_CSN_GPIO_PIN; 
    HAL_GPIO_Init(SPI_CSN_GPIO_PORT, &GPIO_InitStruct);
    /* SPI_SCK GPIO*/
    GPIO_InitStruct.Pin = SPI_SCK_GPIO_PIN;
    HAL_GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStruct);
    /* SPI_DATA GPIO*/
    GPIO_InitStruct.Pin = SPI_DATA_GPIO_PIN;
    HAL_GPIO_Init(SPI_DATA_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(SPI_CSN_GPIO_PORT, SPI_CSN_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI_SCK_GPIO_PORT, SPI_SCK_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SPI_DATA_GPIO_PORT, SPI_DATA_GPIO_PIN, GPIO_PIN_SET);
}
/**
 * @brief RX gong'f
 *
 */
void RF_RX_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RX_EN_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /* RX GPIO*/
    GPIO_InitStruct.Pin = RX_EN_GPIO_PIN;
    HAL_GPIO_Init(RX_EN_GPIO_PORT, &GPIO_InitStruct);

    RF_RX_HIGH;
    // RF_RX_LOW;
}

/**
 * @brief 设置SPI DATA引脚为输入模式
 *
 */
static void set_spi_data_gpio_input_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    SPI_DATA_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /* SPI_DATA GPIO */
    GPIO_InitStruct.Pin = SPI_DATA_GPIO_PIN;
    HAL_GPIO_Init(SPI_DATA_GPIO_PORT, &GPIO_InitStruct);
}
/**
 * @brief 设置SPID DATA引脚为输出模式
 *
 */
static void set_spi_data_gpio_output_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    SPI_DATA_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /* SPI_DATA GPIO */
    GPIO_InitStruct.Pin = SPI_DATA_GPIO_PIN;
    HAL_GPIO_Init(SPI_DATA_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SPI_DATA_GPIO_PORT, SPI_DATA_GPIO_PIN, GPIO_PIN_SET);
}

/******************************************************************************/
// @brief  SPI写入一个BYTE
// @param  R_REG：SPI写入的值
// @retval None
/******************************************************************************/
void SPI_WR(unsigned char R_REG)
{
    unsigned char i;
    set_spi_data_gpio_output_mode();
    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW;
        if (R_REG & 0x80)
        {
            SPI_DATA_HIGH;
        }
        else
        {
            SPI_DATA_LOW;
        }
        R_REG = R_REG << 1;
        SPI_SCK_HIGH;
    }
    SPI_DATA_HIGH;
    SPI_SCK_LOW;

}

/******************************************************************************/
// @brief  SPI读数据之前，先写入需要读的命令或地址
// @param  R_REG：SPI写入的值
// @retval None
/******************************************************************************/
void SPI_WR_forRD(unsigned char R_REG)
{
    unsigned char i;

    set_spi_data_gpio_output_mode();

    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW;
        if (R_REG & 0x80)
        {
            SPI_DATA_HIGH;
        }
        else
        {
            SPI_DATA_LOW;
        }
        R_REG = R_REG << 1;
        SPI_SCK_HIGH;
    }

    // SPI_DATA_HIGH;
    set_spi_data_gpio_input_mode();
    SPI_SCK_LOW;

}

/******************************************************************************/
// @brief  SPI读出一个BYTE
// @param  None
// @retval SPI读出的值
/******************************************************************************/
/******************************************************************************/
uint8_t SPI_RD(void)
{
    uint8_t i, data;

    data = 0;
    for (i = 0; i < 8; i++)
    {
        SPI_SCK_LOW;
        data = data << 1;
        SPI_SCK_HIGH;
        if (SPI_DATA_STATUS)
        {
            data |= 0x01;
        }
    }
    SPI_SCK_LOW;

    return data;
}

/******************************************************************************/
// @brief  RF寄存器读取
// @param  reg：读命令R_REGISTER+寄存器地址(比如CONFIG为0x00)
// @retval 读取到的寄存器值
/******************************************************************************/
uint8_t RF_ReadReg(uint8_t reg)
{
    uint8_t tmp;

    SPI_CSN_LOW;
    SPI_WR_forRD(reg);
    tmp = SPI_RD();
    SPI_CSN_HIGH;

    return tmp;
}

/******************************************************************************/
// @brief  RF寄存器设置
// @param  reg：写命令W_REGISTER+寄存器地址(比如CONFIG为0x00)
//         wdata：需要为寄存器设置的值
// @retval None
/******************************************************************************/
void RF_WriteReg(uint8_t reg, uint8_t wdata)
{
    SPI_CSN_LOW;
    SPI_WR(reg);
    SPI_WR(wdata);
    SPI_CSN_HIGH;
}
/******************************************************************************/
// @brief  RF寄存器设置
// @param  reg：写命令W_REGISTER+寄存器地址
//         pBuf：待写入数据的起始地址
//         length: 待写入数据的长度
// @retval None
/******************************************************************************/
void RF_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t length)
{
    uint8_t j;
    SPI_CSN_LOW;
    j = 0;
    SPI_WR(reg);
    for (j = 0; j < length; j++)
    {
        SPI_WR(pBuf[j]);
    }
    j = 0;
    SPI_CSN_HIGH;
}

/******************************************************************************/
//  RF_ReadBuf
//  Read Data(1 Byte Address ,length byte data read)
/******************************************************************************/
void RF_ReadBuf(uint8_t reg, unsigned char *pBuf, uint8_t length)
{
    uint8_t byte_ctr;

    SPI_CSN_LOW;
    SPI_WR_forRD(reg);
    for (byte_ctr = 0; byte_ctr < length; byte_ctr++)
        pBuf[byte_ctr] = SPI_RD();
    SPI_CSN_HIGH;
}

/******************************************************************************/
//  RF_TxMode
//  Set RF into TX mode
/******************************************************************************/
void RF_TxMode(void)
{
    RF_WriteReg(W_REGISTER + CONFIG, 0X8E); // 将RF设置成TX模式
    CE_LOW; 
}

/******************************************************************************/
//  RF_RxMode
//  将RF设置成RX模式，准备接收数据
/******************************************************************************/
void RF_RxMode(void)
{
    RF_WriteReg(W_REGISTER + CONFIG, 0X8F); // 将RF设置成RX模式
    delay_ms(10);
    CE_HIGH;
    delay_ms(10);
}

/******************************************************************************/
// @brief  读取RF状态标志
// @param  None
// @retval 读出的状态值
//         bit6：收到数据标志
//         bit5：发射成功标志
//         bit4：发射超时标志
//         bit3-1:读取到数据后，判断是哪个通道接收，此处用不到
//         bit0: TX FIFO状态标志，此处用不到
/******************************************************************************/
uint8_t RF_GetStatus(void)
{
    return RF_ReadReg(STATUS) & 0x70;
}

/******************************************************************************/
// @brief  清除RF状态标志
// @param  None
// @retval None
/******************************************************************************/
void RF_ClearStatus(void)
{
    RF_WriteReg(W_REGISTER + STATUS, 0x70);
}

/******************************************************************************/
//  ucRF_GetRSSI
//  读取rssi 值 ?
/******************************************************************************/
uint8_t RF_GetRSSI(void)
{
    return (RF_ReadReg(DATAOUT)); // 读取RF RSSI
}

/******************************************************************************/
//  RF_ClearFIFO
//  clear RF TX/RX FIFO
/******************************************************************************/
void RF_ClearFIFO(void)
{
    RF_WriteReg(FLUSH_TX, 0); // 清除RF 的 TX FIFO
    RF_WriteReg(FLUSH_RX, 0); // 清除RF 的 RX FIFO
}

/******************************************************************************/
//  RF_SetChannel
//  Set RF TX/RX channel:Channel
/******************************************************************************/
void RF_SetChannel(uint8_t Channel)
{
    RF_WriteReg(W_REGISTER + RF_CH, Channel);
}

/******************************************************************************/
//  发送数据：
//  参数：
//      1. ucPayload：需要发送的数据首地址
//      2. length:  需要发送的数据长度
//      length 通常等于 PAYLOAD_WIDTH
/******************************************************************************/
void RF_TxData(uint8_t *ucPayload, uint8_t length)
{
    CE_HIGH;
    delay_100us(1);
    RF_WriteBuf(W_TX_PAYLOAD, ucPayload, length);                                                                  		//rf entery tx mode start send data                                                                       		//rf entery stb3
    delay_ms(1);
    while(!(RF_GetStatus()&TX_DS_FLAG))
    {
        delay_ms(1);  
    }
    RF_WriteReg(FLUSH_TX, 0);	//清除RF的TX FIFO
    RF_ClearStatus();
    CE_LOW;
}
/**
 * @brief 射频数据发送
 * @param ucPayload
 * @param length
 */
void RF_SendData(uint8_t *ucPayload, uint8_t length)
{
    RF_TxData(ucPayload, PAYLOAD_WIDTH);
}
/**
 * @brief 多通道射频数据发送
 * @param ucPayload
 * @param length
 */
void RF_SendData_Multiple_CH(uint8_t *ucPayload, uint8_t length, uint8_t ch)
{
    RF_SetChannel(RF_CHANNEL_TABLE[ch]);
    RF_TxData(ucPayload, PAYLOAD_WIDTH);
}
/******************************************************************************/
//  RF_DumpRxData
//  读出接收到的数据：
//  参数：
//      1. ucPayload：存储读取到的数据的Buffer
//      2. length:    读取的数据长度
//  Return:
//      1. 0: 没有接收到数据
//      2. 1: 读取接收到的数据成功
//  note: Only use in Rx Mode
//  length 通常等于 PAYLOAD_WIDTH
/******************************************************************************/
uint8_t RF_DumpRxData(uint8_t *ucPayload, uint8_t length)
{
    if (!(RF_GetStatus() & RX_DR_FLAG))
    {
        return 0;
    }

    RF_ReadBuf(R_RX_PAYLOAD, ucPayload, length); // 将接收到的数据读出到ucPayload
    RF_WriteReg(FLUSH_RX, 0);                    // 清除RF的RX FIFO
    RF_ClearStatus();
    return 1;
}
/**
 * @brief RF自检
 * @return uint8_t 
 */
uint8_t RF_Check(void)
{
    uint8_t u8pPayload[PAYLOAD_WIDTH];

    uint8_t i;
    RF_ReadBuf(R_REGISTER | TX_ADDR, u8pPayload, XN297L_ADDR_LEN);
    for (i = 0; i < XN297L_ADDR_LEN; i++)
    {
        if (u8pPayload[i] != XN297L_BIND_ADDR[i])
            return 0;
    }

    if (RF_ReadReg(RF_CH) != DEFAULT_CHANNEL)
    {
        return 0;
    }

    return 1;
}
////////////////////////////////////////////////////////////////////////////////

//  以下部分与RF通信相关，不建议修改
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************/
//  PN006_Initial
//  Initial RF
/******************************************************************************/
void RF_Init(void)
{
    uint8_t  BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46};
    uint8_t  RF_cal_data[]    = {0xF6,0x37,0x5D};
    uint8_t  RF_cal2_data[]   = {0x45,0x21,0xEF,0xAC,0x5A,0x50};
    uint8_t  Dem_cal_data[]   = {0x01};
    uint8_t  Dem_cal2_data[]  = {0x0B,0xDF,0x02};

    uint8_t feature = 0x20; // 使用软件CE

    Delay_ms(200);
    SOFT_SPI_init();
    RF_RX_init();
    Delay_ms(2);

    RF_WriteReg(RST_FSPI, 0x5A); // Software Reset
    RF_WriteReg(RST_FSPI, 0XA5);

    RF_WriteReg(FLUSH_TX, 0);                                                                    // CLEAR TXFIFO
    RF_WriteReg(FLUSH_RX, 0);                                                                    // CLEAR  RXFIFO
    RF_WriteReg(W_REGISTER + STATUS, 0x70);                                                      // CLEAR  STATUS
    RF_WriteReg(W_REGISTER + EN_RXADDR, 0x01);                                                   // Enable Pipe0
    RF_WriteReg(W_REGISTER + SETUP_AW, 0x02);                                                    // address witdth is 4 bytes
    RF_WriteReg(W_REGISTER + RF_CH, DEFAULT_CHANNEL);                                            // 2478M HZ
    RF_WriteReg(W_REGISTER + RX_PW_P0, PAYLOAD_WIDTH);                                           // 8 bytes
    RF_WriteBuf(W_REGISTER + TX_ADDR, (uint8_t *)XN297L_BIND_ADDR, sizeof(XN297L_BIND_ADDR));    // Writes TX_Address to XN297L
    RF_WriteBuf(W_REGISTER + RX_ADDR_P0, (uint8_t *)XN297L_BIND_ADDR, sizeof(XN297L_BIND_ADDR)); // RX_Addr0 same as TX_Adr for Auto.Ack

    RF_WriteBuf(W_REGISTER + BB_CAL, BB_cal_data, sizeof(BB_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL2, RF_cal2_data, sizeof(RF_cal2_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL, Dem_cal_data, sizeof(Dem_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL, RF_cal_data, sizeof(RF_cal_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL2, Dem_cal2_data, sizeof(Dem_cal2_data));
    RF_WriteReg(W_REGISTER + DYNPD, 0x00);
    RF_WriteReg(W_REGISTER + RF_SETUP, RF_POWER); // 9dbm  1Mbps
    RF_WriteReg(ACTIVATE, 0x73);

    RF_WriteReg(W_REGISTER + SETUP_RETR, 0x00); // Disable retrans...
    RF_WriteReg(W_REGISTER + EN_AA, 0x00);      // Disable AutoAck

#if (TRANSMIT_TYPE == TRANS_ENHANCE_MODE)
    RF_WriteReg(W_REGISTER + SETUP_RETR, 0x03); //  3 retrans...
    RF_WriteReg(W_REGISTER + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
#elif (TRANSMIT_TYPE == TRANS_BURST_MODE)
    RF_WriteReg(W_REGISTER + SETUP_RETR, 0x00); // Disable retrans...
    RF_WriteReg(W_REGISTER + EN_AA, 0x00);      // Disable AutoAck
#endif

    if (PAYLOAD_WIDTH > 32)
        feature |= 0x18;

    RF_WriteReg(W_REGISTER + FEATURE, feature);

}

/******************************************************************************/
//  进入载波模式
/******************************************************************************/
void RF_Carrier(uint8_t ucChannel_Set)
{
    uint8_t BB_cal_data[] = {0x0A, 0x6D, 0x67, 0x9C, 0x46};
    uint8_t RF_cal_data[] = {0xF6, 0x37, 0x5D};
    uint8_t RF_cal2_data[] = {0x45, 0x21, 0xEF, 0xAC, 0x5A, 0x50};
    uint8_t Dem_cal_data[] = {0xE1};
    uint8_t Dem_cal2_data[] = {0x0B, 0xDF, 0x02};

    RF_WriteReg(RST_FSPI, 0x5A); // Software Reset
    RF_WriteReg(RST_FSPI, 0XA5);
    RF_WriteReg(W_REGISTER + FEATURE, 0x20);
    CE_LOW;
    // delay_ms(200);
    delay_ms(200);
    RF_WriteReg(W_REGISTER + RF_CH, ucChannel_Set); // 单载波频点
    RF_WriteReg(W_REGISTER + RF_SETUP, RF_POWER);   // 13dbm
    RF_WriteBuf(W_REGISTER + BB_CAL, BB_cal_data, sizeof(BB_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL2, RF_cal2_data, sizeof(RF_cal2_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL, Dem_cal_data, sizeof(Dem_cal_data));
    RF_WriteBuf(W_REGISTER + RF_CAL, RF_cal_data, sizeof(RF_cal_data));
    RF_WriteBuf(W_REGISTER + DEM_CAL2, Dem_cal2_data, sizeof(Dem_cal2_data));
}

/***************************************end of file ************************************/
