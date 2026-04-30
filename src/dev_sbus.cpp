/* SBUS RC receiver input driver.
 *
 * IOC/CubeMX requirements:
 * - Configure USART2 for SBUS: 100000 baud, 8E2, RX enabled.
 * - Enable RX pin inversion (required for standard inverted SBUS receivers).
 * - Configure RX GPIO as USART alternate-function input.
 * - Keep UART/pin mapping aligned with kDefaultRxMap below unless using override.
 *
 * Implementation notes:
 * - Receives one byte at a time via HAL_UART_Receive_IT.
 * - 25-byte frames are accepted only with header 0x0F and footer 0x00.
 * - Unpacks 16 channels x 11-bit (bytes 1..22) and stores flags from byte 23.
 * - Uses inter-frame gap timing to resync frame assembly.
 */

/*
TODO:
improve implementation:
- Unsynced mode: receive 1 byte repeatedly until byte is 0x0F.
- Synced mode: receive full 25-byte frame into rx_buffer_.
- Validate header/footer (0x0F, 0x00) and decode.
- If invalid frame: drop back to unsynced 1-byte hunt mode.
- On UART error callback: clear error and drop to unsynced mode.
- and.. switch to DMA+IDLE calls
*/

#include "dev_sbus.hpp"
#include "stm_hal_shims.hpp"
#include <string.h>

// Forward declaration — global instance defined in mymain.cpp
extern DevSBus sbus;

// ---------------------------------------------------------------------------
// Default RX pin table — one entry per supported UART instance.
// Values must match the IOC/MspInit configuration for that peripheral so
// the default (no-override) path stays consistent with CubeMX output.
// Add entries here when enabling additional UART instances.
// ---------------------------------------------------------------------------
static const struct {
  USART_TypeDef     *instance;
  SBusRxConfig       config;
} kDefaultRxMap[] = {
  { USART2, { GPIOA, GPIO_PIN_3, GPIO_AF7_USART2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW } },
};

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

// Look up the default RX config for a given UART handle.
static bool ResolveDefaultConfig(UART_HandleTypeDef *huart, SBusRxConfig &out) {
  for (const auto &entry : kDefaultRxMap) {
    if (huart->Instance == entry.instance) {
      out = entry.config;
      return true;
    }
  }
  return false;  // unsupported UART instance
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool DevSBus::Initialize(UART_HandleTypeDef &huart, const SBusRxConfig *rx_override) {
  rx_index_       = 0;
  valid_          = false;
  last_rx_ms_     = 0;
  last_update_ms_ = 0;
  flags_          = 0;
  memset(channels_, 0, sizeof(channels_));
  memset(rx_buffer_, 0, sizeof(rx_buffer_));

  my_huart_ = &huart;

  // Validate that we have a default mapping for this UART instance.
  if (!ResolveDefaultConfig(my_huart_, rx_config_)) {
    return false;  // unsupported UART — do not arm IT
  }

  // Override path: re-init RX GPIO to caller-supplied pin/AF via HAL_GPIO_Init.
  // Default path: IOC/MspInit already configured the pin — nothing extra needed.
  if (rx_override != nullptr) {
    rx_config_ = *rx_override;
    if (!StmHalInitGpioAf(rx_config_.port, rx_config_.pin,
                          rx_config_.alternate, rx_config_.pull,
                          rx_config_.speed)) {
      return false;
    }
  }

  // Arm single-byte interrupt receive.
  HAL_UART_Receive_IT(my_huart_, &rx_byte_, 1);
  return true;
}

void DevSBus::HandleRx(uint8_t byte) {
  uint32_t now = HAL_GetTick();

  // Gap-based frame sync: if we've been idle longer than one inter-frame gap,
  // reset and expect a fresh header on the next byte.
  if ((now - last_rx_ms_) > SBUS_GAP_RESET_MS) {
    rx_index_ = 0;
  }
  last_rx_ms_ = now;

  // At index 0 we only accept the SBUS header byte
  if (rx_index_ == 0 && byte != 0x0F) {
    return;  // Not a header — wait for next gap
  }

  rx_buffer_[rx_index_++] = byte;

  if (rx_index_ == SBUS_FRAME_LEN) {
    rx_index_ = 0;
    // Validate header + footer
    if (rx_buffer_[0] == 0x0F && rx_buffer_[24] == 0x00) {
      DecodeFrame();
      valid_          = true;
      last_update_ms_ = now;
    }
  }

  // Re-arm for next byte
  HAL_UART_Receive_IT(my_huart_, &rx_byte_, 1);
}

bool DevSBus::GetChannels(uint16_t *channels, uint8_t &channel_count) const {
  if (!valid_) return false;
  memcpy(channels, channels_, sizeof(uint16_t) * SBUS_CHANNELS);
  channel_count = SBUS_CHANNELS;
  return true;
}

bool DevSBus::IsFresh(void) const {
  if (!valid_) return false;
  return (HAL_GetTick() - last_update_ms_) < SBUS_STALE_MS;
}

uint8_t DevSBus::GetFlags(void) const {
  return flags_;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/* Unpack 16 × 11-bit channels from rx_buffer_[1..22].
 * Standard SBUS bit layout, LSB of each channel first, little-endian across
 * bytes, continuous stream of bits (no padding between channels).
 */
void DevSBus::DecodeFrame(void) {
  const uint8_t *d = &rx_buffer_[1];

  channels_[ 0] = ((uint16_t)d[ 0]        | ((uint16_t)d[ 1] << 8))  & 0x07FF;
  channels_[ 1] = ((uint16_t)d[ 1] >> 3   | ((uint16_t)d[ 2] << 5))  & 0x07FF;
  channels_[ 2] = ((uint16_t)d[ 2] >> 6   | ((uint16_t)d[ 3] << 2)
                 | ((uint16_t)d[ 4] << 10)) & 0x07FF;
  channels_[ 3] = ((uint16_t)d[ 4] >> 1   | ((uint16_t)d[ 5] << 7))  & 0x07FF;
  channels_[ 4] = ((uint16_t)d[ 5] >> 4   | ((uint16_t)d[ 6] << 4))  & 0x07FF;
  channels_[ 5] = ((uint16_t)d[ 6] >> 7   | ((uint16_t)d[ 7] << 1)
                 | ((uint16_t)d[ 8] << 9))  & 0x07FF;
  channels_[ 6] = ((uint16_t)d[ 8] >> 2   | ((uint16_t)d[ 9] << 6))  & 0x07FF;
  channels_[ 7] = ((uint16_t)d[ 9] >> 5   | ((uint16_t)d[10] << 3))  & 0x07FF;
  channels_[ 8] = ((uint16_t)d[11]        | ((uint16_t)d[12] << 8))  & 0x07FF;
  channels_[ 9] = ((uint16_t)d[12] >> 3   | ((uint16_t)d[13] << 5))  & 0x07FF;
  channels_[10] = ((uint16_t)d[13] >> 6   | ((uint16_t)d[14] << 2)
                 | ((uint16_t)d[15] << 10)) & 0x07FF;
  channels_[11] = ((uint16_t)d[15] >> 1   | ((uint16_t)d[16] << 7))  & 0x07FF;
  channels_[12] = ((uint16_t)d[16] >> 4   | ((uint16_t)d[17] << 4))  & 0x07FF;
  channels_[13] = ((uint16_t)d[17] >> 7   | ((uint16_t)d[18] << 1)
                 | ((uint16_t)d[19] << 9))  & 0x07FF;
  channels_[14] = ((uint16_t)d[19] >> 2   | ((uint16_t)d[20] << 6))  & 0x07FF;
  channels_[15] = ((uint16_t)d[20] >> 5   | ((uint16_t)d[21] << 3))  & 0x07FF;

  flags_ = rx_buffer_[23];
}