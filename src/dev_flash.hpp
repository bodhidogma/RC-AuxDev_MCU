/**
 * 
 */

#ifndef DEV_FLASH_HPP
#define DEV_FLASH_HPP

#include "mymain.h"
#include "stm_console.hpp"

// Define a safe page address near the end of your Flash memory.
// Example: STM32F303VC has 256KB of flash (0x08000000 to 0x0803FFFF). 
// The last 2KB page starts at 0x0803F800.
#define CONFIG_FLASH_PAGE_ADDR  0x0803F800

class DevFlash {
public:
    // Pass the specific Page Address you want to use.
    // Ensure this page does not overlap with your compiled application code!
    explicit DevFlash(uint32_t pageAddress);

    bool erasePage();
    bool writeWords(const uint32_t* data, size_t length);
    void readWords(uint32_t* destination, size_t length) const;

private:
    uint32_t _pageAddress;
    
    // Auto-calculates page number based on physical address for STM32F3 HAL
    uint32_t getPageNumber() const;
};

#endif // DEV_FLASH_HPP
