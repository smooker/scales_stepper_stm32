#include "eeprom.h"
#include "main.h"

void Flash_Program(uint8_t cell, uint32_t value)
{
    HAL_StatusTypeDef status;
    uint32_t address = FLASH_USER_START_ADDR;

    // 1. Unlock the Flash control register
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        // Handle error (e.g., return or an error LED)
        cdcprintf("FLASH ERR0\r\n");
        return;
    }

    // 2. Erase the Flash page before writing
    // The F1 series uses pages, not sectors like some other series (e.g., F4, F7)
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress   = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages       = 1; // Erase 1 page

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    if (status != HAL_OK)
    {
        // Handle erase error, maybe check PageError for details
        cdcprintf("FLASH ERR1\r\n");
        HAL_FLASH_Lock(); // Re-lock flash on error
        return;
    }

    // 3. Program the data (e.g., a 32-bit word)
    // For F103C8, program options include TYPEPROGRAM_HALFWORD (16-bit) and TYPEPROGRAM_WORD (32-bit)
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, value);

    if (status != HAL_OK)
    {
        // Handle program error
        cdcprintf("FLASH ERR2\r\n");
        HAL_FLASH_Lock(); // Re-lock flash on error
        return;
    }

    // 4. Lock the Flash control register
    HAL_FLASH_Lock();

    // Data is now saved at FLASH_USER_START_ADDR
}

uint32_t Read_Saved_Data(void)
{
    // Cast the flash address to a pointer of the data type you saved (e.g., uint32_t*)
    uint32_t* data_ptr = (uint32_t*)FLASH_USER_START_ADDR;
    return *data_ptr;
}
