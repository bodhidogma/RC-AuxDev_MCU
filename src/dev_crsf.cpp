/* CRSF RC receiver input driver.
 *
 * IOC/CubeMX requirements:
 * - Configure USART2 (or selected UART) for CRSF: 420000 baud, 8N1, RX enabled.
 * - Configure RX GPIO as USART alternate-function input.
 * - Keep UART/pin mapping aligned with kDefaultRxMap below unless using
 *   override.
 *
 * Implementation notes:
 * - Uses HAL_UARTEx_ReceiveToIdle_DMA when available; falls back to IT RX.
 * - Parses variable-length CRSF frames: [addr][len][type][payload...][crc].
 * - Decodes frame type 0x16 (packed 16 x 11-bit channels).
 * 
 * TODO: add TX connected / link health
 */

#include "dev_crsf.hpp"

#include <stdio.h>
#include <string.h>

#include "stm_hal_shims.hpp"

// ---------------------------------------------------------------------------
// Default RX pin table — one entry per supported UART instance.
// Values must match the IOC/MspInit configuration for that peripheral so
// the default (no-override) path stays consistent with CubeMX output.
// Add entries here when enabling additional UART instances.
// ---------------------------------------------------------------------------
static const struct {
	USART_TypeDef* instance;
	CrsfRxConfig config;
} kDefaultRxMap[] = {
		{USART2,
		 {GPIOA, GPIO_PIN_3, GPIO_AF7_USART2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
};

static constexpr uint8_t kCrsfTxAddress = 0xEC;  // receiver device address

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------
static bool ResolveDefaultConfig(UART_HandleTypeDef* huart, CrsfRxConfig& out) {
	for (const auto& entry : kDefaultRxMap) {
		if (huart->Instance == entry.instance) {
			out = entry.config;
			return true;
		}
	}
	return false;
}


// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool DevCRSF::Initialize(UART_HandleTypeDef& huart,
												 const CrsfRxConfig* rx_override) {
	rx_index_ = 0;
	expected_frame_len_ = 0;
	valid_ = false;
	use_dma_rx_ = false;
	tx_busy_ = false;
	tx_len_ = 0;
	last_telemetry_tx_ms_ = 0;
	telemetry_slot_ = 0;
	telemetry_sent_count_ = 0;
	telemetry_drop_count_ = 0;
	last_rx_ms_ = 0;
	last_update_ms_ = 0;
	memset(channels_, 0, sizeof(channels_));
	memset(dma_rx_buffer_, 0, sizeof(dma_rx_buffer_));
	memset(rx_buffer_, 0, sizeof(rx_buffer_));
	memset(tx_buffer_, 0, sizeof(tx_buffer_));
	battery_ = {};
	attitude_ = {};
	flight_mode_ = {};

	my_huart_ = &huart;

	// Force UART into CRSF mode in case CubeMX is currently configured for SBUS.
	if (my_huart_ != nullptr) {
		(void)HAL_UART_DeInit(my_huart_);

		my_huart_->Init.BaudRate = 420000;
		my_huart_->Init.WordLength = UART_WORDLENGTH_8B;
		my_huart_->Init.StopBits = UART_STOPBITS_1;
		my_huart_->Init.Parity = UART_PARITY_NONE;
		my_huart_->Init.Mode = UART_MODE_TX_RX;
		my_huart_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
		my_huart_->Init.OverSampling = UART_OVERSAMPLING_16;
		my_huart_->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;

		my_huart_->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
		my_huart_->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_DISABLE;

		if (HAL_UART_Init(my_huart_) != HAL_OK) {
			return false;
		}
	}

	if (!ResolveDefaultConfig(my_huart_, rx_config_)) {
		return false;
	}

	if (rx_override != nullptr) {
		rx_config_ = *rx_override;
		if (!StmHalInitGpioAf(rx_config_.port, rx_config_.pin, rx_config_.alternate,
													rx_config_.pull, rx_config_.speed)) {
			return false;
		}
	}

	return ArmReceive();
}

void DevCRSF::HandleRx(uint8_t byte) {
	ProcessRxBytes(&byte, 1);
	if (!use_dma_rx_) {
		(void)ArmItReceive();
	}
}

void DevCRSF::HandleRxEvent(uint16_t size) {
	if (size > 0 && size <= CRSF_DMA_RX_BUF_LEN) {
		ProcessRxBytes(dma_rx_buffer_, size);
	}
	(void)ArmDmaReceive();
}

void DevCRSF::HandleError(void) {
	rx_index_ = 0;
	expected_frame_len_ = 0;

	if (my_huart_ != nullptr) {
		(void)HAL_UART_AbortReceive(my_huart_);
	}

	(void)ArmReceive();
}

void DevCRSF::HandleTxComplete(UART_HandleTypeDef* huart) {
	if (huart != my_huart_) return;
	tx_busy_ = false;
	tx_len_ = 0;
}

bool DevCRSF::GetChannels(uint16_t* channels, uint8_t& channel_count) const {
	if (!valid_) return false;
	memcpy(channels, channels_, sizeof(uint16_t) * CRSF_CHANNELS);
	channel_count = CRSF_CHANNELS;
	return true;
}

bool DevCRSF::IsFresh(void) const {
	if (!valid_) return false;
	return (HAL_GetTick() - last_update_ms_) < CRSF_STALE_MS;
}

void DevCRSF::UpdateBatteryTelemetry(uint16_t voltage_cV, uint16_t current_cA,
														 uint32_t capacity_mAh,
														 uint8_t remaining_pct) {
	battery_.voltage_cV = voltage_cV;
	battery_.current_cA = current_cA;
	battery_.capacity_mAh = capacity_mAh;
	battery_.remaining_pct = (remaining_pct > 100u) ? 100u : remaining_pct;
	battery_.valid = true;
	battery_.dirty = true;
	battery_.last_update_ms = HAL_GetTick();
}

void DevCRSF::UpdateAttitudeTelemetry(int16_t pitch_rad_1e4,
														 int16_t roll_rad_1e4,
														 int16_t yaw_rad_1e4) {
	attitude_.pitch_rad_1e4 = pitch_rad_1e4;
	attitude_.roll_rad_1e4 = roll_rad_1e4;
	attitude_.yaw_rad_1e4 = yaw_rad_1e4;
	attitude_.valid = true;
	attitude_.dirty = true;
	attitude_.last_update_ms = HAL_GetTick();
}

void DevCRSF::UpdateFlightModeTelemetry(const char* mode_text) {
	if (mode_text == nullptr) return;
	const size_t max_copy = static_cast<size_t>(CRSF_TELEMETRY_TEXT_MAX - 1u);
	strncpy(flight_mode_.text, mode_text, max_copy);
	flight_mode_.text[max_copy] = '\0';
	flight_mode_.valid = true;
	flight_mode_.dirty = true;
	flight_mode_.last_update_ms = HAL_GetTick();
}

void DevCRSF::SendTelemetryTick(uint32_t now_ms) {
	if (my_huart_ == nullptr) return;
	if (tx_busy_) return;

	const uint32_t telemetry_interval_ms = 1000u / static_cast<uint32_t>(CRSF_TELEMETRY_RATE_HZ);
	if ((now_ms - last_telemetry_tx_ms_) < telemetry_interval_ms) return;

	uint8_t payload[CRSF_MAX_FRAME_LEN] = {};
	uint8_t payload_len = 0;
	uint8_t frame_type = 0;
	bool built = false;

	for (uint8_t attempt = 0; attempt < 3u; ++attempt) {
		switch (telemetry_slot_) {
			case 0:
				built = BuildBatteryPayload(payload, payload_len);
				frame_type = CRSF_FRAME_TYPE_BATTERY_SENSOR;
				break;
			case 1:
				built = BuildAttitudePayload(payload, payload_len);
				frame_type = CRSF_FRAME_TYPE_ATTITUDE;
				break;
			default:
				built = BuildFlightModePayload(payload, payload_len);
				frame_type = CRSF_FRAME_TYPE_FLIGHT_MODE;
				break;
		}

		telemetry_slot_ = (telemetry_slot_ + 1u) % 3u;
		if (built) break;
	}

	if (!built) return;

	uint8_t frame_len = 0;
	if (!BuildTelemetryFrame(frame_type, payload, payload_len, tx_buffer_, frame_len)) {
		telemetry_drop_count_++;
		return;
	}

	if (!StartTxFrame(tx_buffer_, frame_len)) {
		telemetry_drop_count_++;
		return;
	}

	last_telemetry_tx_ms_ = now_ms;
	telemetry_sent_count_++;
}

bool DevCRSF::_DumpState(StmConsole& console, uint8_t mode) const {
	(void)mode;
	uint16_t crsf_ch[CRSF_CHANNELS];
	uint8_t crsf_count = 0;
	bool fresh = IsFresh();
	bool valid = GetChannels(crsf_ch, crsf_count);
	int len;
	uint8_t buf[64];
	if (!valid) {
		console.Send("CRSF: --", 8);
	} else {
		len = snprintf((char*)buf, sizeof(buf), "CRSF[%d]:%c ", crsf_count,
									 (fresh ? ' ' : '~'));
		console.Send((const char*)buf, len);
		for (int ch = 0; ch < crsf_count; ch++) {
			len =
					snprintf((char*)buf, sizeof(buf), "(%d) %4u\t", ch + 1, crsf_ch[ch]);
			console.Send((const char*)buf, len);
			if (crsf_ch[ch] == 0) {
				break;
			}
		}
	}
  console.Send("\r\n", 2);
	return true;
}

bool DevCRSF::ArmReceive(void) {
	if (ArmDmaReceive()) return true;
	return ArmItReceive();
}

bool DevCRSF::ArmDmaReceive(void) {
	if (my_huart_ == nullptr) return false;

	if (HAL_UARTEx_ReceiveToIdle_DMA(my_huart_, dma_rx_buffer_,
																	 CRSF_DMA_RX_BUF_LEN) != HAL_OK) {
		use_dma_rx_ = false;
		return false;
	}

	if (my_huart_->hdmarx != nullptr) {
		__HAL_DMA_DISABLE_IT(my_huart_->hdmarx, DMA_IT_HT);
	}
	use_dma_rx_ = true;
	return true;
}

bool DevCRSF::ArmItReceive(void) {
	if (my_huart_ == nullptr) return false;
	if (HAL_UART_Receive_IT(my_huart_, &rx_byte_, 1) != HAL_OK) return false;
	use_dma_rx_ = false;
	return true;
}

void DevCRSF::ProcessRxBytes(const uint8_t* data, uint16_t len) {
	if (data == nullptr || len == 0) return;

	const uint32_t now = HAL_GetTick();
	if ((now - last_rx_ms_) > CRSF_GAP_RESET_MS) {
		rx_index_ = 0;
		expected_frame_len_ = 0;
	}
	last_rx_ms_ = now;

	for (uint16_t i = 0; i < len; i++) {
		const uint8_t byte = data[i];

		if (rx_index_ >= CRSF_MAX_FRAME_LEN) {
			rx_index_ = 0;
			expected_frame_len_ = 0;
		}

		rx_buffer_[rx_index_++] = byte;

		if (rx_index_ == 2) {
			const uint8_t len_field = rx_buffer_[1];
			if (len_field < 2 || len_field > CRSF_MAX_LEN_FIELD) {
				rx_index_ = 0;
				expected_frame_len_ = 0;
				continue;
			}
			expected_frame_len_ = static_cast<uint8_t>(len_field + 2u);
			if (expected_frame_len_ < CRSF_MIN_FRAME_LEN ||
					expected_frame_len_ > CRSF_MAX_FRAME_LEN) {
				rx_index_ = 0;
				expected_frame_len_ = 0;
			}
			continue;
		}

		if (expected_frame_len_ != 0 && rx_index_ == expected_frame_len_) {
			if (TryDecodeFrame(rx_buffer_, expected_frame_len_)) {
				valid_ = true;
				last_update_ms_ = HAL_GetTick();
			}
			rx_index_ = 0;
			expected_frame_len_ = 0;
		}
	}
}

bool DevCRSF::TryDecodeFrame(const uint8_t* frame, uint8_t total_len) {
	if (frame == nullptr || total_len < CRSF_MIN_FRAME_LEN) return false;

	const uint8_t len_field = frame[1];
	if (total_len != static_cast<uint8_t>(len_field + 2u)) return false;

	const uint8_t type = frame[2];
	const uint8_t payload_len = static_cast<uint8_t>(len_field - 2u);
	const uint8_t* payload = &frame[3];
	const uint8_t rx_crc = frame[total_len - 1u];
	const uint8_t calc_crc = Crc8DvbS2(&frame[2], static_cast<uint8_t>(len_field - 1u));
	if (calc_crc != rx_crc) return false;

	if (type != CRSF_FRAME_TYPE_RC_CHANNELS || payload_len < 22u) {
		return false;
	}

	const uint8_t* d = payload;
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

	return true;
}

bool DevCRSF::BuildTelemetryFrame(uint8_t frame_type, const uint8_t* payload,
																uint8_t payload_len,
																uint8_t* out,
																uint8_t& out_len) const {
	if (out == nullptr || payload == nullptr) return false;
	const uint8_t len_field = static_cast<uint8_t>(payload_len + 2u);
	const uint8_t total = static_cast<uint8_t>(payload_len + 4u);
	if (total > CRSF_MAX_FRAME_LEN) return false;

	out[0] = kCrsfTxAddress;
	out[1] = len_field;
	out[2] = frame_type;
	memcpy(&out[3], payload, payload_len);
	out[3 + payload_len] = Crc8DvbS2(&out[2], static_cast<uint8_t>(payload_len + 1u));
	out_len = total;
	return true;
}

bool DevCRSF::BuildBatteryPayload(uint8_t* payload, uint8_t& payload_len) {
	if (payload == nullptr || !battery_.valid) return false;
	payload[0] = static_cast<uint8_t>((battery_.voltage_cV >> 8) & 0xFFu);
	payload[1] = static_cast<uint8_t>(battery_.voltage_cV & 0xFFu);
	payload[2] = static_cast<uint8_t>((battery_.current_cA >> 8) & 0xFFu);
	payload[3] = static_cast<uint8_t>(battery_.current_cA & 0xFFu);
	payload[4] = static_cast<uint8_t>((battery_.capacity_mAh >> 16) & 0xFFu);
	payload[5] = static_cast<uint8_t>((battery_.capacity_mAh >> 8) & 0xFFu);
	payload[6] = static_cast<uint8_t>(battery_.capacity_mAh & 0xFFu);
	payload[7] = battery_.remaining_pct;
	payload_len = 8u;
	battery_.dirty = false;
	return true;
}

bool DevCRSF::BuildAttitudePayload(uint8_t* payload, uint8_t& payload_len) {
	if (payload == nullptr || !attitude_.valid) return false;
	payload[0] = static_cast<uint8_t>((attitude_.pitch_rad_1e4 >> 8) & 0xFFu);
	payload[1] = static_cast<uint8_t>(attitude_.pitch_rad_1e4 & 0xFFu);
	payload[2] = static_cast<uint8_t>((attitude_.roll_rad_1e4 >> 8) & 0xFFu);
	payload[3] = static_cast<uint8_t>(attitude_.roll_rad_1e4 & 0xFFu);
	payload[4] = static_cast<uint8_t>((attitude_.yaw_rad_1e4 >> 8) & 0xFFu);
	payload[5] = static_cast<uint8_t>(attitude_.yaw_rad_1e4 & 0xFFu);
	payload_len = 6u;
	attitude_.dirty = false;
	return true;
}

bool DevCRSF::BuildFlightModePayload(uint8_t* payload, uint8_t& payload_len) {
	if (payload == nullptr || !flight_mode_.valid) return false;
	const size_t max_copy = static_cast<size_t>(CRSF_TELEMETRY_TEXT_MAX - 1u);
	const size_t text_len = strnlen(flight_mode_.text, max_copy);
	if (text_len == 0u) return false;
	memcpy(payload, flight_mode_.text, text_len);
	payload[text_len] = '\0';
	payload_len = static_cast<uint8_t>(text_len + 1u);
	flight_mode_.dirty = false;
	return true;
}

bool DevCRSF::StartTxFrame(const uint8_t* frame, uint8_t frame_len) {
	if (my_huart_ == nullptr || frame == nullptr || frame_len == 0u) return false;
	if (tx_busy_) return false;

	tx_len_ = frame_len;
	tx_busy_ = true;
	if (HAL_UART_Transmit_IT(my_huart_, const_cast<uint8_t*>(frame), frame_len) != HAL_OK) {
		tx_busy_ = false;
		tx_len_ = 0;
		return false;
	}
	return true;
}

uint8_t DevCRSF::Crc8DvbS2(const uint8_t* data, uint8_t len) const {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x80u) {
				crc = (uint8_t)((crc << 1) ^ 0xD5u);
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

