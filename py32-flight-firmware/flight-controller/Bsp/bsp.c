#include "bsp.h"

void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void Bsp_Gpio_Init(void)
{
    // 使能GPIOA时钟
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    // PA1 配置为推挽输出，无上拉下拉，低速
    GPIOA->MODER   = (GPIOA->MODER & ~(3U << (1 * 2))) | (1U << (1 * 2));   // 输出模式
    GPIOA->OTYPER &= ~(1U << 1);                                            // 推挽
    GPIOA->PUPDR  &= ~(3U << (1 * 2));                                      // 无上下拉
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (1 * 2)));                   // 低速
    //  置低PA1
    GPIOA->BRR = (1U << 1);

    // 使能GPIOB时钟
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // PB7 配置为推挽输出，无上拉下拉，低速
    GPIOB->MODER   = (GPIOB->MODER & ~(3U << (7 * 2))) | (1U << (7 * 2));   // 输出模式
    GPIOB->OTYPER &= ~(1U << 7);                                            // 推挽
    GPIOB->PUPDR  &= ~(3U << (7 * 2));                                      // 无上下拉
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(3U << (7 * 2)));                   // 低速
    // 置低PB7
    GPIOB->BRR = (1U << 7);
}


void Bsp_Init(void)
{
    Bsp_Gpio_Init();
    Bsp_I2C_Init();
    Bsp_SPI_Init();
    Bsp_PWM_Init();
    Bsp_Timer_Init();
    Bsp_ADC_Init();
}

void Bsp_Gpio_Set(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    if (GPIOx == GPIOA)
    {
        GPIOA->BSRR = GPIO_Pin;
    }
    else if (GPIOx == GPIOB)
    {
        GPIOB->BSRR = GPIO_Pin;
    }
}

void Bsp_Gpio_Reset(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    if (GPIOx == GPIOA)
    {
        GPIOA->BRR = GPIO_Pin;
    }
    else if (GPIOx == GPIOB)
    {
        GPIOB->BRR = GPIO_Pin;
    }
}

void Bsp_Gpio_Toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    if (GPIOx == GPIOA)
    {
        GPIOA->ODR ^= GPIO_Pin;
    }
    else if (GPIOx == GPIOB)
    {
        GPIOB->ODR ^= GPIO_Pin;
    }
}
