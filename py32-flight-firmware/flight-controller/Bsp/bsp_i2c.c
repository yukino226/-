#include <stdlib.h>
#include <string.h>
#include "bsp_i2c.h"


#define I2C_TIMEOUT    10

I2C_HandleTypeDef I2cHandle;
void Bsp_I2C_Init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
    RCC->APBENR1 |= RCC_APBENR1_I2CEN;

    // 配置 PB3 和 PB4 为复用开漏上拉，AF6（I2C1）
    // 1. 设置模式为复用功能 (MODER)
    GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (4 * 2))); // 清除PB3/PB4模式
    GPIOB->MODER |=  ((2U << (3 * 2)) | (2U << (4 * 2))); // 复用功能

    // 2. 设置输出类型为开漏 (OTYPER)
    GPIOB->OTYPER |= (1U << 3) | (1U << 4);

    // 3. 上拉 (PUPDR)
    GPIOB->PUPDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)));
    GPIOB->PUPDR |=  ((1U << (3 * 2)) | (1U << (4 * 2)));

    // 4. 速度为 VERY_HIGH (OSPEEDR)
    GPIOB->OSPEEDR &= ~((3U << (3 * 2)) | (3U << (4 * 2)));
    GPIOB->OSPEEDR |=  ((3U << (3 * 2)) | (3U << (4 * 2)));

    // 5. 复用功能 AF6 (AFRL)
    GPIOB->AFR[0] &= ~((0xFU << (3 * 4)) | (0xFU << (4 * 4)));
    GPIOB->AFR[0] |=  ((6U << (3 * 4)) | (6U << (4 * 4)));

    // I2C1 复位
    RCC->APBRSTR1 |= RCC_APBRSTR1_I2CRST;
    RCC->APBRSTR1 &= ~RCC_APBRSTR1_I2CRST;

    I2C1->CR2 = 0x00000018; // Disable I2C peripheral interrupts
    I2C1->TRISE = 0x00000008; // Set the maximum rise time for I2C    
    I2C1->CCR = 0x0000C003; // Clear the clock control register
    I2C1->CR1 = 0x00000001; // Enable I2C peripheral    
    I2C1->OAR1 = 0x000000A0; // Set the own address for I2C

//GPIO_InitTypeDef  GPIO_InitStruct;
//__HAL_RCC_GPIOB_CLK_ENABLE();
//__HAL_RCC_I2C_CLK_ENABLE();
//     GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4; // SDA and SCL pins
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//     GPIO_InitStruct.Pull = GPIO_PULLUP;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//   __HAL_RCC_I2C_FORCE_RESET();
//   __HAL_RCC_I2C_RELEASE_RESET();

// I2cHandle.Instance             = I2C1;                      /* I2C1 */
// I2cHandle.Init.ClockSpeed      = 400000;                    /* I2C communication speed  100K*/
// I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;        /* I2C duty cycle */
// I2cHandle.Init.OwnAddress1     = 0xA0;                      /* I2C address */
// I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;   /* Prohibit broadcast calls */
// I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;     /* Allow clock extension */
// if (HAL_I2C_Init(&I2cHandle) != HAL_OK)
// {
//     while(1);
// }

}

int8_t Bsp_I2C_WriteBuf(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    int8_t ret = 0;
    uint32_t tickstart = HAL_GetTick();
    while(READ_BIT(I2C1->SR2, I2C_SR2_BUSY))
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
        return -1; // I2C is busy
    }

    CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
    SET_BIT(I2C1->CR1, I2C_CR1_START);
    while(!READ_BIT(I2C1->SR1, I2C_SR1_SB)) // Wait for start bit
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }
    WRITE_REG(I2C1->DR, addr); // Send address
    while(!READ_BIT(I2C1->SR1, I2C_SR1_ADDR)) // Wait for address sent
    {
        if(READ_BIT(I2C1->SR1, I2C_SR1_AF)) // Check for acknowledge failure
        {
            SET_BIT(I2C1->CR1, I2C_CR1_STOP); // Send stop condition
            CLEAR_BIT(I2C1->SR1, I2C_SR1_AF); // Clear START bit
            return -1; // Acknowledge failure
        }

        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }

    (void)(I2C1->SR1); // Clear START bit
    (void)(I2C1->SR2); // Clear ADDR bit

    while(!READ_BIT(I2C1->SR1, I2C_SR1_TXE)) // Wait for data register empty
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }
    WRITE_REG(I2C1->DR, reg); // Send register address
    while(!READ_BIT(I2C1->SR1, I2C_SR1_BTF)) // Wait for data register empty
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }
    for (uint8_t i = 0; i < len; i++)
    {
        WRITE_REG(I2C1->DR, buffer[i]); // Send data
        while(!READ_BIT(I2C1->SR1, I2C_SR1_BTF)) // Wait for data register empty
        {
            if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
                return -1; // I2C is busy
        }
    }
    SET_BIT(I2C1->CR1, I2C_CR1_STOP); // Send stop condition

    // uint8_t *pData;
    // pData = malloc(len + 1);
    // if(pData == NULL)
    // {
    //     return -1; 
    // }
    // pData[0] = reg;
    // memcpy(pData + 1, buffer, len);
    // ret = HAL_I2C_Master_Transmit(&I2cHandle, addr, pData, len + 1, I2C_TIMEOUT);
    // free(pData);

    return ret;
}

int8_t Bsp_I2C_ReadBuf(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    int8_t ret = 0;
    uint32_t tickstart = HAL_GetTick();
    while(READ_BIT(I2C1->SR2, I2C_SR2_BUSY))
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
        return -1; // I2C is busy
    }

    //Send Register Address
    CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
    SET_BIT(I2C1->CR1, I2C_CR1_START);
    while(!READ_BIT(I2C1->SR1, I2C_SR1_SB)) // Wait for start bit
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }
    WRITE_REG(I2C1->DR, addr); // Send address
    while(!READ_BIT(I2C1->SR1, I2C_SR1_ADDR)) // Wait for address sent
    {
        if(READ_BIT(I2C1->SR1, I2C_SR1_AF)) // Check for acknowledge failure
        {
            SET_BIT(I2C1->CR1, I2C_CR1_STOP); // Send stop condition
            CLEAR_BIT(I2C1->SR1, I2C_SR1_AF); // Clear START bit
            return -1; // Acknowledge failure
        }

        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }
    (void)(I2C1->SR1); // Clear START bit
    (void)(I2C1->SR2); // Clear ADDR bit
    while(!READ_BIT(I2C1->SR1, I2C_SR1_TXE)) // Wait for data register empty
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }
    WRITE_REG(I2C1->DR, reg); // Send register address
    while(!READ_BIT(I2C1->SR1, I2C_SR1_BTF)) // Wait for data register empty
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }

    // Read Data
    CLEAR_BIT(I2C1->CR1, I2C_CR1_POS);
    SET_BIT(I2C1->CR1, I2C_CR1_ACK);
    SET_BIT(I2C1->CR1, I2C_CR1_START);
    while(!READ_BIT(I2C1->SR1, I2C_SR1_SB)) // Wait for start bit
    {
        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }
    WRITE_REG(I2C1->DR, addr|0x01); // Send address
    while(!READ_BIT(I2C1->SR1, I2C_SR1_ADDR)) // Wait for address sent
    {
        if(READ_BIT(I2C1->SR1, I2C_SR1_AF)) // Check for acknowledge failure
        {
            SET_BIT(I2C1->CR1, I2C_CR1_STOP); // Send stop condition
            CLEAR_BIT(I2C1->SR1, I2C_SR1_AF); // Clear START bit
            return -1; // Acknowledge failure
        }

        if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
            return -1; // I2C is busy
    }
    if(len == 1)
    {
        CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK); // Disable ACK for single byte read
        __disable_irq();
        (void)(I2C1->SR1); // Clear START bit
        (void)(I2C1->SR2); // Clear ADDR bit
        SET_BIT(I2C1->CR1, I2C_CR1_STOP); // Send stop condition
        __enable_irq();
    }
    else if (len == 2)
    {
        SET_BIT(I2C1->CR1, I2C_CR1_POS);
        __disable_irq();
        (void)(I2C1->SR1); // Clear START bit
        (void)(I2C1->SR2); // Clear ADDR bit
        CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK); // Disable ACK for last byte
        __enable_irq();        
    }
    else
    {
        SET_BIT(I2C1->CR1, I2C_CR1_ACK); // Enable ACK for multiple byte read
        (void)(I2C1->SR1); // Clear START bit
        (void)(I2C1->SR2); // Clear ADDR bit
    }
    while(len > 0)
    {
        if(len <= 3)
        {
            if(len == 1)
            {
                while(!READ_BIT(I2C1->SR1, I2C_SR1_RXNE)) // Wait for data register not empty
                {
                    if(READ_BIT(I2C1->SR1, I2C_SR1_STOPF)) // Check if data register is empty
                    {
                        CLEAR_BIT(I2C1->CR1, I2C_SR1_STOPF); // Send stop condition
                    }
                    if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
                        return -1; // I2C is busy
                }
                *buffer = READ_REG(I2C1->DR); // Read data
                buffer++;
                len--;
            }
            else if(len == 2)
            {
                while(!READ_BIT(I2C1->SR1, I2C_SR1_BTF)); // Wait for data register empty
                {
                    if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
                        return -1; // I2C is busy
                }
                __disable_irq();
                SET_BIT(I2C1->CR1, I2C_CR1_STOP); // Send stop condition
                *buffer = READ_REG(I2C1->DR); // Read data
                buffer++;
                len--;
                __enable_irq(); 
                *buffer = READ_REG(I2C1->DR); // Read data
                buffer++;
                len--;                

            }
            else
            {
                while(!READ_BIT(I2C1->SR1, I2C_SR1_BTF)); // Wait for data register empty
                {
                    if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
                        return -1; // I2C is busy
                }
                CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK); // Disable ACK for last byte
                __disable_irq();
                *buffer = READ_REG(I2C1->DR); // Read data
                buffer++;
                len--;
                while(!READ_BIT(I2C1->SR1, I2C_SR1_BTF)); // Wait for data register empty
                {
                    if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
                        return -1; // I2C is busy
                }
                SET_BIT(I2C1->CR1, I2C_CR1_STOP); // Send stop condition
                *buffer = READ_REG(I2C1->DR); // Read data
                buffer++;
                len--;
                __enable_irq(); 
                *buffer = READ_REG(I2C1->DR); // Read data
                buffer++;
                len--;                
            }
        }
        else
        {
            while(!READ_BIT(I2C1->SR1, I2C_SR1_RXNE)) // Wait for data register not empty
            {
                if(READ_BIT(I2C1->SR1, I2C_SR1_STOPF)) // Check if data register is empty
                {
                    CLEAR_BIT(I2C1->CR1, I2C_SR1_STOPF); // Send stop condition
                }
                if(((HAL_GetTick() - tickstart) > I2C_TIMEOUT))
                    return -1; // I2C is busy
            }
            *buffer = READ_REG(I2C1->DR); // Read data
            buffer++;
            len--;
            if(READ_BIT(I2C1->SR1, I2C_SR1_BTF)) // Check if data register is empty
            {
                *buffer = READ_REG(I2C1->DR); // Read data
                buffer++;
                len--;
            }

        }
    }
    

    // ret = HAL_I2C_Master_Transmit(&I2cHandle, addr, &reg, 1, I2C_TIMEOUT);
    // if (ret != HAL_OK)
    // {
    //     return ret;
    // }
    // ret = HAL_I2C_Master_Receive(&I2cHandle, addr, buffer, len, I2C_TIMEOUT);
    return ret;
}
