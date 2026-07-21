/**
 * 
 */

#include "dev_flash.hpp"

#include "stm_hal_shims.hpp"

DevFlash::DevFlash(uint32_t pageAddress) : _pageAddress(pageAddress) {}

uint32_t DevFlash::getPageNumber() const {
    // STM32F3 Pages are typically 2KB (0x800) each. 
    // Double-check your specific F3 reference manual (e.g., F303xC has 2KB pages, F303x8 has 1KB pages).
    return (_pageAddress - FLASH_BASE) / 0x800; 
}

bool DevFlash::erasePage() {
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;

    HAL_FLASH_Unlock();

    eraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = _pageAddress; // Some F3 HAL variants use PageAddress, others use Page
    eraseInitStruct.NbPages     = 1;

    // Clear all pending flags before performing flash operations
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    
    HAL_FLASH_Lock();

    return (status == HAL_OK);
}

bool DevFlash::writeWords(const uint32_t* data, size_t length) {
    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    uint32_t currentAddress = _pageAddress;
    bool success = true;

    for (size_t i = 0; i < length; ++i) {
        // STM32F3 standard embedded flash programs 32 bits at a time using FLASH_TYPEPROGRAM_WORD
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddress, data[i]) == HAL_OK) {
            currentAddress += 4; // Advance 4 bytes for 32-bit word
        } else {
            success = false;
            break;
        }
    }

    HAL_FLASH_Lock();
    return success;
}

void DevFlash::readWords(uint32_t* destination, size_t length) const {
    const uint32_t* flashSource = reinterpret_cast<const uint32_t*>(_pageAddress);
    
    for (size_t i = 0; i < length; ++i) {
        destination[i] = flashSource[i]; // Direct memory mapping read
    }
}
