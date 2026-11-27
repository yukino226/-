#include "bsp_flash.h"

#define FLASH_USER_START_ADDR  (0x08005F80U)

int8_t Bsp_Flash_Write(uint32_t *buffer, uint32_t size)
{
    uint32_t PAGEError = 0;
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGEERASE;                                     /* Page Erase */
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;                                                       /* Erase Start Address */
    EraseInitStruct.NbPages  = size / FLASH_PAGE_SIZE;                                   /* Erase Page Number */
    if (size % FLASH_PAGE_SIZE) {                                                /* If size is not a multiple of page size */
        EraseInitStruct.NbPages += 1;                                 /* Adjust size to the next page boundary */
    }                                                /* Erase Page Number */
    HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);      
    uint32_t flash_program_start = FLASH_USER_START_ADDR ;                                                     /* flash program start address */
    uint32_t flash_program_end = (FLASH_USER_START_ADDR + size);                                               /* flash program end address */    
    while (flash_program_start < flash_program_end)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, flash_program_start, (uint32_t *)buffer) == HAL_OK)  /* Program */
        {
            flash_program_start += FLASH_PAGE_SIZE;                                           /* flash Start point first page */
            buffer += FLASH_PAGE_SIZE / 4;                                                       /* Update data point */
        }
    }
    HAL_FLASH_Lock();
    return 0;
}

int8_t Bsp_Flash_Read(uint32_t *buffer, uint32_t size)
{
    uint32_t addr = 0;

    while (addr < size)
    {
        buffer[addr / 4] = HW32_REG(FLASH_USER_START_ADDR + addr);
        addr += 4;
    }
    return 0;
}
