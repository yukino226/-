#include <stdlib.h>
#include "bsp_pwm.h"

// TIM_HandleTypeDef TimHandle;
// TIM_OC_InitTypeDef sConfig;
void Bsp_PWM_Init(void)
{
    // 使能时钟
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    // PA3、PA5、PA7、PB5 配置为复用推挽输出，上拉，高速，AF2（TIM1）

    // MODER: 复用功能
    GPIOA->MODER = (GPIOA->MODER & ~((3U<<(3*2))|(3U<<(5*2))|(3U<<(7*2))))
                | (2U<<(3*2)) | (2U<<(5*2)) | (2U<<(7*2));
    GPIOB->MODER = (GPIOB->MODER & ~(3U<<(5*2)))
                | (2U<<(5*2));

    // OTYPER: 推挽
    GPIOA->OTYPER &= ~((1U<<3)|(1U<<5)|(1U<<7));
    GPIOB->OTYPER &= ~(1U<<5);

    // PUPDR: 上拉
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3U<<(3*2))|(3U<<(5*2))|(3U<<(7*2))))
                | (1U<<(3*2)) | (1U<<(5*2)) | (1U<<(7*2));
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3U<<(5*2)))
                | (1U<<(5*2));

    // OSPEEDR: 高速
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~((3U<<(3*2))|(3U<<(5*2))|(3U<<(7*2))))
                | (3U<<(3*2)) | (3U<<(5*2)) | (3U<<(7*2));
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(3U<<(5*2)))
                | (3U<<(5*2));

    // AFRL: AF2
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFU<<(3*4))|(0xFU<<(5*4))|(0xFU<<(7*4))))
                | (2U<<(3*4)) | (2U<<(5*4)) | (2U<<(7*4));
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFU<<(5*4)))
                | (2U<<(5*4));
    // GPIO_InitTypeDef   GPIO_InitStruct = {0};
    // /* Enable TIM1 Clock */
    // __HAL_RCC_TIM1_CLK_ENABLE();                              
    // /* Enable GPIOA Clock */
    // __HAL_RCC_GPIOA_CLK_ENABLE();                             
    // /* Enable GPIOB Clock */
    // __HAL_RCC_GPIOB_CLK_ENABLE();                             
    // /* Mode AF-PP */
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;                   
    // /* PULLUP */
    // GPIO_InitStruct.Pull = GPIO_PULLUP;                       
    // /* SPEED HIGH */
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    // /* GPIO PA5 Initialize*/
    // GPIO_InitStruct.Pin = GPIO_PIN_5;
    // GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // /* GPIO PA3 Initialize */
    // GPIO_InitStruct.Pin = GPIO_PIN_3;
    // GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // /* GPIO PB5 Initialize */
    // GPIO_InitStruct.Pin = GPIO_PIN_5;
    // GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // /* GPIO PA7 Initialize */
    // GPIO_InitStruct.Pin = GPIO_PIN_7;
    // GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TimHandle.Instance = TIM1; 
    // // Initialize the timer for PWM
    // TimHandle.Init.Period            = 2500 - 1;                                     
    // /* Prescaler = 12 - 1 */
    // TimHandle.Init.Prescaler         = 24 - 1;                                 
    // /* ClockDivision = 0  */
    // TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;                  
    // /* Counter direction = Up */
    // TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;                      
    // /* Repetition = 0 */
    // TimHandle.Init.RepetitionCounter = 1 - 1;                                   
    // /* Auto-reload register not buffered  */
    // TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; 

    // if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
    // {
    //     while(1); // Initialization Error
    // }

    TIM1-> SR = 0x0000001F; // Clear control register
    TIM1-> CCMR1 = 0x00006868; // Clear capture/compare register
    TIM1-> CCMR2 = 0x00006868; // Clear capture/compare register
    TIM1-> CCER = 0x00001111; // Clear capture/compare enable register
    TIM1-> CNT = 0; // Clear control register
    //TIM1-> PSC = 0x00000017; // Set prescaler to 24 - 1
    TIM1-> PSC = 0x00000000; // Set prescaler to 0
    TIM1-> ARR =  0x000005db; // 1500 -1
    TIM1-> CCR1 = 0x00000000; // Clear capture/compare register
    TIM1-> CCR2 = 0x00000000; // Clear capture/compare register
    TIM1-> CCR3 = 0x00000000; // Clear capture/compare register
    TIM1-> CCR4 = 0x00000000; // Clear capture/compare register    
    TIM1-> BDTR = 0x00008000; // Clear break and dead-time register
    TIM1-> CR1   = 0x00000001;
    /* Set output compare mode: PWM1 */
    // sConfig.OCMode       = TIM_OCMODE_PWM1;                                     

    // /* OC channel output high level effective */
    // sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;                                 

    // /* Disable OC FastMode */
    // sConfig.OCFastMode   = TIM_OCFAST_DISABLE;                                  

    // /* OCN channel output high level effective */
    // sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                

    // /* Idle state OC1N output low level */
    // sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              

    // /* Idle state OC1 output low level*/
    // sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;                               

    // sConfig.Pulse = 1000;                                              

    // /* Channel 1 configuration */
    // if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
    // {
    //     while(1);
    // }                                           

    // /* Channel 2 configuration */
    // if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
    // {
    //     while(1);
    // }
                                            

    // /* Channel 3 configuration */
    // if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
    // {
    //     while(1);
    // }
                                            

    /* Channel 4 configuration */
//    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
//    {
//        while(1);
//    }

    // /* Channel1 Start PWM */
    // if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)                  
    // {
    //     while(1);
    // }

    // /* Channel2 Start PWM */
    // if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)                  
    // {
    //     while(1);
    // }

    // /* Channel3 Start PWM */
    // if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)                  
    // {
    //     while(1);
    // }

    // /* Channel4 Start PWM */
    // if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)                  
    // {
    //     while(1);   
    // }
}

void Bsp_PWM_SetDuty(uint8_t channel, uint16_t duty)
{
    if (channel == 0)
    {
        TIM1-> CCR1 = duty;
    }
    else if (channel == 1)
    {
        TIM1-> CCR2 = duty;
    }
    else if (channel == 2)
    {
        TIM1-> CCR3 = duty;
    }
    else if (channel == 3)
    {
        TIM1-> CCR4 = duty;
    }
}


