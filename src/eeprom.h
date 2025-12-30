/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"


//FLASH EEPROM
#define FLASH_USER_START_ADDR   ((uint32_t)0x0800F800)
// #define FLASH_PAGE_SIZE         ((uint32_t)0x400)   // 1 KB
// #define DATA_TO_SAVE            ((uint32_t)0x12345678)

void Flash_Program(uint8_t cell, uint32_t value);
uint32_t Read_Saved_Data(void);

#endif /* __EEPROM_H */
