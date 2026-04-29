/**
 * SBUS RC receiver input driver.
 * Uses USART2: 100000 baud, 8E2, RX-only, RX pin inverted (configured by CubeMX).
 * Receives 25-byte SBUS frames, decodes 16 × 11-bit channels and flags.
 * Frame sync via inter-frame gap timing (>4 ms gap resets accumulator).
 */

#ifndef DEV_SBUS_HPP
#define DEV_SBUS_HPP

#include "mymain.h"

#define SBUS_CHANNELS       16u
#define SBUS_FRAME_LEN      25u
#define SBUS_STALE_MS       100u   // ms without update before data marked stale
#define SBUS_GAP_RESET_MS   4u     // inter-frame gap threshold for frame sync

// Flags byte (byte[23]) bit positions
#define SBUS_FLAG_CH17        (1u << 0)
#define SBUS_FLAG_CH18        (1u << 1)
#define SBUS_FLAG_FRAME_LOST  (1u << 2)
#define SBUS_FLAG_FAILSAFE    (1u << 3)

class DevSBus {
 public:
  DevSBus() {}

  // Store UART handle, reset state, arm single-byte IT receive. Returns true.
  bool Initialize(UART_HandleTypeDef *huart);

  // Call from HAL_UART_RxCpltCallback with each received byte.
  // Re-arms single-byte IT internally.
  void HandleRx(uint8_t byte);

  // Copy latest decoded channel values (SBUS_CHANNELS elements) into `channels`.
  // Returns false if no valid frame has been received yet.
  bool GetChannels(uint16_t *channels, int &channel_count) const;

  // Returns false if no valid frame within SBUS_STALE_MS or not yet valid.
  bool IsFresh(void) const;

  // Returns the raw flags byte from the last valid frame.
  uint8_t GetFlags(void) const;

  // Single-byte DMA/IT receive buffer — must be accessible from the callback.
  uint8_t rx_byte_ = 0;

 private:
  void DecodeFrame(void);

  UART_HandleTypeDef *my_huart_  = nullptr;
  uint8_t  rx_buffer_[SBUS_FRAME_LEN];
  int      rx_index_             = 0;
  uint32_t last_rx_ms_           = 0;   // HAL_GetTick() of last received byte

  uint16_t channels_[SBUS_CHANNELS];
  uint8_t  flags_                = 0;
  bool     valid_                = false;
  uint32_t last_update_ms_       = 0;   // HAL_GetTick() at last successfully decoded frame
};

#endif  // DEV_SBUS_HPP