/**
 * Basic ws2812 driver using SPI MOSI + DMA
 * 
 * Configure SPI1 for 2.25 MHz (72MHz / 32) to generate ~800kHz WS2812 data
 * Use 3-bit encoding: WS2812 bit '1' = 0b110, bit '0' = 0b100
 * Configure DMA to transmit SPI data buffer, trigger interrupt on complete
 * 
 */

#ifndef _DEV_WS2812_HPP_
#define _DEV_WS2812_HPP_

// location of all external HAL types
#include "mymain.h"

class DevWS2812 {
 public:
  DevWS2812(SPI_HandleTypeDef *hspi) {
    my_spi_ = hspi;
  }

  bool Initialize(void);

  bool GetConfig(void);

  bool Update(void);
  bool Loop(void);
  
  bool IsBusy(void);

  SPI_HandleTypeDef *my_spi_;

 protected:
 private:
  uint8_t pat_step_;
  uint32_t last_now_ms_;
  
};

#endif  // _DEV_WS2812_HPP_