/* SBUS RC receiver input driver.
 *
 * IOC/CubeMX requirements:
 * - Configure USART2 for SBUS: 100000 baud, 8E2, RX enabled.
 * - Enable RX pin inversion (required for standard inverted SBUS receivers).
 * - Configure RX GPIO as USART alternate-function input.
 * - Keep UART/pin mapping aligned with kDefaultRxMap below unless using
 * override.
 *
 * Implementation notes:
 * - Uses HAL_UARTEx_ReceiveToIdle_DMA when available; falls back to IT RX.
 * - 25-byte frames are accepted only with header 0x0F and footer 0x00.
 * - Unpacks 16 channels x 11-bit (bytes 1..22) and stores flags from byte 23.
 * - Uses gap-based reset plus in-frame header scan for fast re-alignment.
 */

#include "dev_sbus.hpp"

#include <string.h>

#include "stm_hal_shims.hpp"

// Forward declaration — global instance defined in mymain.cpp
extern DevSBus sbus;

// ---------------------------------------------------------------------------
// Default RX pin table — one entry per supported UART instance.
// Values must match the IOC/MspInit configuration for that peripheral so
// the default (no-override) path stays consistent with CubeMX output.
// Add entries here when enabling additional UART instances.
// ---------------------------------------------------------------------------
static const struct {
  USART_TypeDef* instance;
  SBusRxConfig config;
} kDefaultRxMap[] = {
    {USART2,
     {GPIOA, GPIO_PIN_3, GPIO_AF7_USART2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
};

static constexpr uint8_t kSbusHeader = 0x0F;
static constexpr uint8_t kSbusFooter = 0x00;

// Some SBUS2 receivers use footer variants 0x04/0x14/0x24/0x34.
static bool IsValidSbusFooter(uint8_t footer) {
  return (footer == kSbusFooter) || ((footer & 0x0FU) == 0x04U);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

// Look up the default RX config for a given UART handle.
static bool ResolveDefaultConfig(UART_HandleTypeDef* huart, SBusRxConfig& out) {
  for (const auto& entry : kDefaultRxMap) {
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

bool DevSBus::Initialize(UART_HandleTypeDef& huart,
                         const SBusRxConfig* rx_override) {
  rx_index_ = 0;
  valid_ = false;
  use_dma_rx_ = false;
  last_rx_ms_ = 0;
  last_update_ms_ = 0;
  flags_ = 0;
  memset(channels_, 0, sizeof(channels_));
  memset(dma_rx_buffer_, 0, sizeof(dma_rx_buffer_));
  memset(rx_buffer_, 0, sizeof(rx_buffer_));

  my_huart_ = &huart;

  // Validate that we have a default mapping for this UART instance.
  if (!ResolveDefaultConfig(my_huart_, rx_config_)) {
    return false;  // unsupported UART — do not arm IT
  }

  // Override path: re-init RX GPIO to caller-supplied pin/AF via HAL_GPIO_Init.
  // Default path: IOC/MspInit already configured the pin — nothing extra
  // needed.
  if (rx_override != nullptr) {
    rx_config_ = *rx_override;
    if (!StmHalInitGpioAf(rx_config_.port, rx_config_.pin, rx_config_.alternate,
                          rx_config_.pull, rx_config_.speed)) {
      return false;
    }
  }

  // Arm RX path: DMA+IDLE first, IT fallback if DMA arm fails.
  return ArmReceive();
}

void DevSBus::HandleRx(uint8_t byte) {
  ProcessRxBytes(&byte, 1);

  // IT path must be re-armed per byte.
  if (!use_dma_rx_) {
    (void)ArmItReceive();
  }
}

void DevSBus::HandleRxEvent(uint16_t size) {
  if (size > 0 && size <= SBUS_DMA_RX_BUF_LEN) {
    ProcessRxBytes(dma_rx_buffer_, size);
  }

  // DMA+IDLE path must always be re-armed after each event.
  (void)ArmDmaReceive();
}

void DevSBus::HandleError(void) {
  rx_index_ = 0;

  if (my_huart_ != nullptr) {
    (void)HAL_UART_AbortReceive(my_huart_);
  }

  (void)ArmReceive();
}

bool DevSBus::GetChannels(uint16_t* channels, uint8_t& channel_count) const {
  if (!valid_) return false;
  memcpy(channels, channels_, sizeof(uint16_t) * SBUS_CHANNELS);
  channel_count = SBUS_CHANNELS;
  return true;
}

bool DevSBus::IsFresh(void) const {
  if (!valid_) return false;
  return (HAL_GetTick() - last_update_ms_) < SBUS_STALE_MS;
}

bool DevSBus::_DumpState(StmConsole& console, uint8_t mode) const {
#if 1
  uint16_t sbus_ch[SBUS_CHANNELS];
  uint8_t sbus_count = 0;
  bool fresh = IsFresh();
  bool valid = GetChannels(sbus_ch, sbus_count);
  int len;
  uint8_t buf[64];
  if (!valid) {
    console.Send("SBUS: --\r\n", 10);
  } else {
    len = snprintf((char*)buf, sizeof(buf), "SBUS[%d]:%c ", sbus_count,
                   (fresh ? ' ' : '~'));
    console.Send((const char*)buf, len);
    for (int ch = 0; ch < sbus_count; ch++) {
      len =
          snprintf((char*)buf, sizeof(buf), "(%d) %4u\t", ch + 1, sbus_ch[ch]);
      console.Send((const char*)buf, len);
      if (sbus_ch[ch] == 0) {
        break;
      }
    }
    console.Send("\r\n", 2);
  }
#endif
  return true;
}

uint8_t DevSBus::GetFlags(void) const { return flags_; }

bool DevSBus::ArmReceive(void) {
  if (ArmDmaReceive()) return true;
  return ArmItReceive();
}

bool DevSBus::ArmDmaReceive(void) {
  if (my_huart_ == nullptr) return false;

  if (HAL_UARTEx_ReceiveToIdle_DMA(my_huart_, dma_rx_buffer_,
                                   SBUS_DMA_RX_BUF_LEN) != HAL_OK) {
    use_dma_rx_ = false;
    return false;
  }

  if (my_huart_->hdmarx != nullptr) {
    __HAL_DMA_DISABLE_IT(my_huart_->hdmarx, DMA_IT_HT);
  }
  use_dma_rx_ = true;
  return true;
}

bool DevSBus::ArmItReceive(void) {
  if (my_huart_ == nullptr) return false;
  if (HAL_UART_Receive_IT(my_huart_, &rx_byte_, 1) != HAL_OK) return false;
  use_dma_rx_ = false;
  return true;
}

void DevSBus::ProcessRxBytes(const uint8_t* data, uint16_t len) {
  if (data == nullptr || len == 0) return;

  const uint32_t now = HAL_GetTick();
  if ((now - last_rx_ms_) > SBUS_GAP_RESET_MS) {
    rx_index_ = 0;
  }
  last_rx_ms_ = now;

  for (uint16_t i = 0; i < len; i++) {
    const uint8_t byte = data[i];

    if (rx_index_ == 0 && byte != kSbusHeader) {
      continue;
    }

    rx_buffer_[rx_index_++] = byte;

    if (rx_index_ == SBUS_FRAME_LEN) {
      if (rx_buffer_[0] == kSbusHeader && IsValidSbusFooter(rx_buffer_[24])) {
        DecodeFrame();
        valid_ = true;
        last_update_ms_ = HAL_GetTick();
        rx_index_ = 0;
      } else {
        RealignAfterInvalidFrame();
      }
    }
  }
}

void DevSBus::RealignAfterInvalidFrame(void) {
  int new_start = -1;
  for (int i = 1; i < static_cast<int>(SBUS_FRAME_LEN); i++) {
    if (rx_buffer_[i] == kSbusHeader) {
      new_start = i;
      break;
    }
  }

  if (new_start < 0) {
    rx_index_ = 0;
    return;
  }

  const int remaining = static_cast<int>(SBUS_FRAME_LEN) - new_start;
  memmove(rx_buffer_, &rx_buffer_[new_start], static_cast<size_t>(remaining));
  rx_index_ = remaining;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/* Unpack 16 × 11-bit channels from rx_buffer_[1..22].
 * Standard SBUS bit layout, LSB of each channel first, little-endian across
 * bytes, continuous stream of bits (no padding between channels).
 */
void DevSBus::DecodeFrame(void) {
  const uint8_t* d = &rx_buffer_[1];

  channels_[0] = ((uint16_t)d[0] | ((uint16_t)d[1] << 8)) & 0x07FF;
  channels_[1] = ((uint16_t)d[1] >> 3 | ((uint16_t)d[2] << 5)) & 0x07FF;
  channels_[2] =
      ((uint16_t)d[2] >> 6 | ((uint16_t)d[3] << 2) | ((uint16_t)d[4] << 10)) &
      0x07FF;
  channels_[3] = ((uint16_t)d[4] >> 1 | ((uint16_t)d[5] << 7)) & 0x07FF;
  channels_[4] = ((uint16_t)d[5] >> 4 | ((uint16_t)d[6] << 4)) & 0x07FF;
  channels_[5] =
      ((uint16_t)d[6] >> 7 | ((uint16_t)d[7] << 1) | ((uint16_t)d[8] << 9)) &
      0x07FF;
  channels_[6] = ((uint16_t)d[8] >> 2 | ((uint16_t)d[9] << 6)) & 0x07FF;
  channels_[7] = ((uint16_t)d[9] >> 5 | ((uint16_t)d[10] << 3)) & 0x07FF;
  channels_[8] = ((uint16_t)d[11] | ((uint16_t)d[12] << 8)) & 0x07FF;
  channels_[9] = ((uint16_t)d[12] >> 3 | ((uint16_t)d[13] << 5)) & 0x07FF;
  channels_[10] = ((uint16_t)d[13] >> 6 | ((uint16_t)d[14] << 2) |
                   ((uint16_t)d[15] << 10)) &
                  0x07FF;
  channels_[11] = ((uint16_t)d[15] >> 1 | ((uint16_t)d[16] << 7)) & 0x07FF;
  channels_[12] = ((uint16_t)d[16] >> 4 | ((uint16_t)d[17] << 4)) & 0x07FF;
  channels_[13] =
      ((uint16_t)d[17] >> 7 | ((uint16_t)d[18] << 1) | ((uint16_t)d[19] << 9)) &
      0x07FF;
  channels_[14] = ((uint16_t)d[19] >> 2 | ((uint16_t)d[20] << 6)) & 0x07FF;
  channels_[15] = ((uint16_t)d[20] >> 5 | ((uint16_t)d[21] << 3)) & 0x07FF;

  flags_ = rx_buffer_[23];
}