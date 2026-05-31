/**
 * CRSF RC receiver input driver.
 * Uses USART2 by default: 420000 baud, 8N1, RX-only (configured by CubeMX).
 * Receives variable-length CRSF frames and decodes packed 16 × 11-bit channel
 * payloads (frame type 0x16).
 *
 * Hardware binding:
 *   - Default: IOC/CubeMX already configured the RX GPIO; Initialize() arms RX.
 *   - Override: pass a non-null CrsfRxConfig to re-map the RX pin at startup (e.g. for
 *     pin-stacking or multi-UART board variants). HAL_GPIO_Init is called for the new pin.
 */

#ifndef DEV_CRSF_HPP
#define DEV_CRSF_HPP

#include "mymain.h"
#include "stm_console.hpp"

#define CRSF_CHANNELS                    16u
#define CRSF_FRAME_TYPE_RC_CHANNELS      0x16u
#define CRSF_MIN_FRAME_LEN               4u   // addr + len + type + crc
#define CRSF_MAX_FRAME_LEN               64u
#define CRSF_MAX_LEN_FIELD               (CRSF_MAX_FRAME_LEN - 2u)
#define CRSF_DMA_RX_BUF_LEN              64u
#define CRSF_STALE_MS                    100u
#define CRSF_GAP_RESET_MS                4u

// RX GPIO pin configuration for startup override.
// Provide all fields; they map 1-to-1 to GPIO_InitTypeDef members.
struct CrsfRxConfig {
	GPIO_TypeDef *port;      // e.g. GPIOA
	uint16_t      pin;       // e.g. GPIO_PIN_3
	uint32_t      alternate; // e.g. GPIO_AF7_USART2
	uint32_t      pull;      // GPIO_NOPULL / GPIO_PULLUP / GPIO_PULLDOWN
	uint32_t      speed;     // GPIO_SPEED_FREQ_HIGH etc.
};

class DevCRSF {
 public:
	DevCRSF() {};

	// Reset state, resolve hardware binding, and arm receive path.
	// rx_override == nullptr -> trust IOC default GPIO config for the selected UART.
	// rx_override != nullptr -> re-init RX pin using HAL_GPIO_Init.
	// Returns false if UART instance is unsupported or GPIO clock enable fails.
	bool Initialize(UART_HandleTypeDef &huart,
									const CrsfRxConfig *rx_override = nullptr);

	// Returns active UART handle for callback routing.
	UART_HandleTypeDef *MyHuart() const { return my_huart_; }

	// Single-byte IT fallback receive buffer.
	uint8_t rx_byte_ = 0;

	// Call from HAL UART callbacks.
	void HandleRx(uint8_t byte);
	void HandleRxEvent(uint16_t size);
	void HandleError(void);

	// Copy latest decoded channel values into `channels`.
	// Returns false if no valid CRSF RC frame has been decoded yet.
	bool GetChannels(uint16_t *channels, uint8_t &channel_count) const;

	// Returns false if no valid frame within CRSF_STALE_MS.
	bool IsFresh(void) const;

	bool _DumpState(StmConsole& console, uint8_t mode = 0) const;  // for debugging

 private:
	bool ArmReceive(void);
	bool ArmDmaReceive(void);
	bool ArmItReceive(void);
	void ProcessRxBytes(const uint8_t *data, uint16_t len);
	bool TryDecodeFrame(const uint8_t *frame, uint8_t total_len);
	uint8_t Crc8DvbS2(const uint8_t *data, uint8_t len) const;

	UART_HandleTypeDef *my_huart_ = nullptr;
	CrsfRxConfig        rx_config_ = {};

	bool    use_dma_rx_ = false;
	uint8_t dma_rx_buffer_[CRSF_DMA_RX_BUF_LEN];

	uint8_t rx_buffer_[CRSF_MAX_FRAME_LEN];
	uint8_t rx_index_ = 0;
	uint8_t expected_frame_len_ = 0;
	uint32_t last_rx_ms_ = 0;

	uint16_t channels_[CRSF_CHANNELS];
	bool     valid_ = false;
	uint32_t last_update_ms_ = 0;
};

#endif  // DEV_CRSF_HPP