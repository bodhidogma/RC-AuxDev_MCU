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
  static constexpr int kMaxLed = 13;
  static constexpr int kLeadInBytes = 3;
  static constexpr int kResetBytes = 50;
  static constexpr int kSpiBufferSize =
      (9 * kMaxLed + kLeadInBytes + kResetBytes);
  static constexpr int kMaxInstances = 2;

  explicit DevWS2812(SPI_HandleTypeDef *hspi);

  bool Initialize(void);

  bool GetConfig(void);

  bool Update(void);
  bool Loop(void);

  void SetLED(int led_num, int red, int green, int blue);
  void SetBrightness(int brightness);
  bool Send(void);
  bool IsBusy(void);
  void OnTxComplete(void);
  void OnTxError(void);
  static DevWS2812 *FindBySpiHandle(SPI_HandleTypeDef *hspi);

  SPI_HandleTypeDef *my_spi_;

 protected:
 private:
  static DevWS2812 *instances_[kMaxInstances];
  static void RegisterInstance(DevWS2812 *inst);
  static uint8_t ApplyBrightnessToValue(uint8_t value, uint8_t brightness);

  uint8_t led_data_[kMaxLed][4];
  uint8_t led_mod_[kMaxLed][4];
  uint8_t spi_data_[kSpiBufferSize];
  uint8_t current_brightness_;
  volatile uint8_t dma_busy_;
  uint8_t pat_step_;
  uint32_t last_now_ms_;
};

#endif  // _DEV_WS2812_HPP_