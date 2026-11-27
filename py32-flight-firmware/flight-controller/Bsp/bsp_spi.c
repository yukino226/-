#include "bsp_spi.h"

#define SPI_TIMEOUT    500

SPI_HandleTypeDef Spi1Handle;
void Bsp_SPI_Init(void)
{
    // GPIO_InitTypeDef  GPIO_InitStruct;
    // __HAL_RCC_GPIOA_CLK_ENABLE();
    // __HAL_RCC_GPIOB_CLK_ENABLE();
    // __HAL_RCC_SPI1_CLK_ENABLE();

    // GPIO_InitStruct.Pin = GPIO_PIN_6;// CS
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLUP;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

    // GPIO_InitStruct.Pin = GPIO_PIN_2; // CLK
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;    
    // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // GPIO_InitStruct.Pin = GPIO_PIN_0; // MOSI
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    // GPIO_InitStruct.Pull = GPIO_PULLUP;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;    
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 使能时钟
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;

    // PA6 (CS) 配置为推挽输出，上拉，VERY_HIGH
    GPIOA->MODER   = (GPIOA->MODER & ~(3U<<(6*2))) | (1U<<(6*2));      // 输出
    GPIOA->OTYPER &= ~(1U<<6);                                         // 推挽
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3U<<(6*2))) | (1U<<(6*2));      // 上拉
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U<<(6*2))) | (3U<<(6*2));    // VERY_HIGH
    GPIOA->BSRR = (1U<<6);                                             // CS拉高

    // PB2 (SCK) 配置为复用推挽输出，下拉，VERY_HIGH，AF0
    GPIOB->MODER   = (GPIOB->MODER & ~(3U<<(2*2))) | (2U<<(2*2));      // 复用
    GPIOB->OTYPER &= ~(1U<<2);                                         // 推挽
    GPIOB->PUPDR   = (GPIOB->PUPDR & ~(3U<<(2*2))) | (2U<<(2*2));      // 下拉
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(3U<<(2*2))) | (3U<<(2*2));    // VERY_HIGH
    GPIOB->AFR[0]  = (GPIOB->AFR[0] & ~(0xFU<<(2*4))) | (0U<<(2*4));   // AF0

    // PA0 (MOSI) 配置为复用开漏输出，上拉，VERY_HIGH，AF0
    GPIOA->MODER   = (GPIOA->MODER & ~(3U<<(0*2))) | (2U<<(0*2));      // 复用
    GPIOA->OTYPER |= (1U<<0);                                          // 开漏
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3U<<(0*2))) | (1U<<(0*2));      // 上拉
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U<<(0*2))) | (3U<<(0*2));    // VERY_HIGH
    GPIOA->AFR[0]  = (GPIOA->AFR[0] & ~(0xFU<<(0*4))) | (0U<<(0*4));   // AF0


    Spi1Handle.Instance               = SPI1;                       /* SPI1 */
    Spi1Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;   /* 32 frequency division */
    Spi1Handle.Init.Direction         = SPI_DIRECTION_1LINE;        /* full duplex */
    Spi1Handle.Init.CLKPolarity       = SPI_POLARITY_LOW;           /* Low clock polarity */
    Spi1Handle.Init.CLKPhase          = SPI_PHASE_1EDGE ;           /* Data sampling starts from the first clock edge */
    Spi1Handle.Init.DataSize          = SPI_DATASIZE_8BIT;          /* SPI data length is 8 bits */
    Spi1Handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;           /* Send MSB first */
    Spi1Handle.Init.NSS               = SPI_NSS_SOFT;               /* NSS software mode (hardware mode) */
    Spi1Handle.Init.Mode              = SPI_MODE_MASTER;            /* Configure as master */
    
    // if (HAL_SPI_DeInit(&Spi1Handle) != HAL_OK)
    // {
    //     while(1);
    // }
    // if (HAL_SPI_Init(&Spi1Handle) != HAL_OK)
    // {
    //     while(1);
    // }
    SPI1->CR1 = 0x00008324;
}


void Bsp_SPI_Transmit(uint8_t* data, uint16_t size)
{
    //HAL_SPI_Transmit(&Spi1Handle, data, size, SPI_TIMEOUT);
    uint32_t tickstart;
    tickstart = HAL_GetTick();
    CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); // Disable SPI
    SET_BIT(SPI1->CR1, SPI_CR1_BIDIOE);// Set 1Line direction (BIDIOE bit)
    SET_BIT(SPI1->CR1, SPI_CR1_SPE); // Enable SPI
    while(size > 0U)
    {
        if((SPI1->SR & SPI_SR_TXE) == SPI_SR_TXE)
        {
            *((__IO uint8_t *)&SPI1->DR) = (*data);
            data++;
            size--;
        }
        else
        {
            if(((HAL_GetTick() - tickstart) > SPI_TIMEOUT))
            {
                return; // Timeout
            }
        }
    }

    (void)SPI1->DR;
    while((SPI1->SR & SPI_SR_FTLVL) != 0U) // Wait until TXE flag is set
    {
        if(((HAL_GetTick() - tickstart) > SPI_TIMEOUT))
        {
            CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); // Disable the SPI
            return; // Timeout
        }
    }

    while((SPI1->SR & SPI_SR_BSY) != 0U) // Wait until BSY flag is reset
    {
        if(((HAL_GetTick() - tickstart) > SPI_TIMEOUT))
        {
            CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); // Disable the SPI
            return; // Timeout
        }
    }
    (void)SPI1->DR;
    while((SPI1->SR & SPI_SR_FRLVL) != 0U) // Wait until TXE flag is set
    {
        if(((HAL_GetTick() - tickstart) > SPI_TIMEOUT))
        {
            CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); // Disable the SPI
            return; // Timeout
        }
    }
}


void Bsp_SPI_Receive(uint8_t* data, uint16_t size)
{
    //HAL_SPI_Receive(&Spi1Handle, data, size, SPI_TIMEOUT);
    uint32_t tickstart;
    tickstart = HAL_GetTick();
    CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); // Disable SPI
    CLEAR_BIT(SPI1->CR1, SPI_CR1_BIDIOE);
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    while(size > 0U)
    {
        if((SPI1->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
        {
            (* (uint8_t *)data) = *(__IO uint8_t *)&SPI1->DR;
            data++;
            size--;
        }
        else
        {
            if(((HAL_GetTick() - tickstart) > SPI_TIMEOUT))
            {
                return; // Timeout
            }
        }
    }
    CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);

    while((SPI1->SR & SPI_SR_BSY) != 0U) // Wait until BSY flag is reset
    {
        if(((HAL_GetTick() - tickstart) > SPI_TIMEOUT))
        {
            CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); // Disable the SPI
            return; // Timeout
        }
    }
    (void)SPI1->DR;
    while((SPI1->SR & SPI_SR_FRLVL) != 0U) // Wait until TXE flag is set
    {
        (void)(__IO uint8_t *)&SPI1->DR;
        if(((HAL_GetTick() - tickstart) > SPI_TIMEOUT))
        {
            CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE); // Disable the SPI
            return; // Timeout
        }
    }
    return;
}

void Bsp_SPI_CS_Low(void)
{
    // 直接将PA6拉低
    GPIOA->BSRR = (1U << (6 + 16)); // 低16位为置位，高16位为复位
}

void Bsp_SPI_CS_High(void)
{
    // 直接将PA6拉高
    GPIOA->BSRR = (1U << 6);
}
