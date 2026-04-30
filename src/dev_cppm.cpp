/* CPPM (PPM Sum) RC receiver input capture driver.
 *
 * IOC/CubeMX requirements:
 * - Enable timer input capture with 1 MHz timer tick (1 tick = 1 us).
 * - Capture polarity: rising edge.
 * - Configure GPIO as timer alternate-function input for each used channel.
 * - Keep pin mapping aligned with kDefaultInputMap below unless using overrides.
 *
 * Implementation notes:
 * - Gap >= CPPM_SYNC_GAP_US marks frame sync.
 * - Gap <  CPPM_SYNC_GAP_US is interpreted as channel pulse width in us.
 * - On each sync gap, any collected channels are latched as one frame.
 * - GetChannels()/IsFresh() report data from the first registered CPPM input.
 */

#include "dev_cppm.hpp"
#include "stm_hal_shims.hpp"
#include <string.h>

// Forward declaration — global instance defined in mymain.cpp
extern DevCPPM cppm;

// Map TIM_CHANNEL_x to HAL_TIM_ActiveChannel (same helper as dev_pwm_duty)
static inline HAL_TIM_ActiveChannel chan_to_active(uint32_t hal_channel) {
  return static_cast<HAL_TIM_ActiveChannel>(1u << (hal_channel >> 2u));
}

// ---------------------------------------------------------------------------
// Default input map and helpers
// ---------------------------------------------------------------------------

static const struct {
  TIM_TypeDef* tim_instance;
  uint32_t hal_channel;
  CppmGpioConfig gpio;
} kDefaultInputMap[] = {
    // IOC TIM2 input-capture defaults
		{TIM2, TIM_CHANNEL_1,
		 {GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
		{TIM2, TIM_CHANNEL_2,
		 {GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
		{TIM2, TIM_CHANNEL_3,
		 {GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
		{TIM2, TIM_CHANNEL_4,
		 {GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
};

static bool ResolveDefaultInputConfig(TIM_HandleTypeDef* htim,
                                      uint32_t hal_channel,
                                      CppmGpioConfig& out) {
  for (const auto& entry : kDefaultInputMap) {
    if (entry.tim_instance == htim->Instance && entry.hal_channel == hal_channel) {
      out = entry.gpio;
      return true;
    }
  }
  return false;
}

static bool ConfigureTimerInputCapture(TIM_HandleTypeDef* htim,
                                       uint32_t hal_channel) {
  TIM_IC_InitTypeDef sConfigIC = {};
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = CPPM_IC_FILTER;

  if (HAL_TIM_IC_Init(htim) != HAL_OK) return false;
  if (HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, hal_channel) != HAL_OK) return false;
  return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

int DevCPPM::Register(const CppmInputConfig& cfg) {
  if (input_count_ >= static_cast<int>(CPPM_MAX_INPUTS)) return -1;
  CppmInput &inp       = inputs_[input_count_];
  inp.htim             = cfg.htim;
  inp.hal_channel      = cfg.hal_channel;
  inp.gpio_cfg         = cfg.gpio;
  inp.use_gpio_override = (cfg.gpio.port != nullptr);
  inp.prev_tick        = 0;
  inp.frame_period_us  = 0;
  inp.frame_start_tick = 0;
  inp.last_update_ms   = 0;
  inp.ch_idx           = 0;
  inp.num_channels     = 0;
  inp.synced           = false;
  inp.valid            = false;
  memset(inp.channels, 0, sizeof(inp.channels));
  return input_count_++;
}

bool DevCPPM::Initialize(const CppmInputConfig* configs, int count) {
  if (configs == nullptr || count <= 0) return false;

  input_count_ = 0;
  memset(inputs_, 0, sizeof(inputs_));

  for (int i = 0; i < count; i++) {
    if (Register(configs[i]) < 0) return false;
  }

  for (int i = 0; i < input_count_; i++) {
    CppmInput &inp = inputs_[i];
    const CppmGpioConfig* gpio_cfg = &inp.gpio_cfg;
    CppmGpioConfig default_cfg = {};

    // Default path requires a known IOC mapping.
    // Override path is allowed to bind pins not present in default map.
    if (!inp.use_gpio_override) {
      if (!ResolveDefaultInputConfig(inp.htim, inp.hal_channel, default_cfg)) {
        return false;
      }
      gpio_cfg = &default_cfg;
    }

    if (!StmHalInitGpioAf(gpio_cfg->port, gpio_cfg->pin,
                          gpio_cfg->alternate, gpio_cfg->pull,
                          gpio_cfg->speed)) {
      return false;
    }

    if (!ConfigureTimerInputCapture(inp.htim, inp.hal_channel)) return false;

    __HAL_TIM_SET_CAPTUREPOLARITY(inp.htim, inp.hal_channel,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    if (HAL_TIM_IC_Start_IT(inp.htim, inp.hal_channel) != HAL_OK) {
      return false;
    }
  }
  return true;
}

bool DevCPPM::Initialize(TIM_HandleTypeDef* htim, uint32_t hal_channel) {
  const CppmInputConfig cfg = {htim, hal_channel};
  return Initialize(&cfg, 1);
}

void DevCPPM::HandleCapture(TIM_HandleTypeDef *htim) {
  HAL_TIM_ActiveChannel active = htim->Channel;

  for (int i = 0; i < input_count_; i++) {
    CppmInput &inp = inputs_[i];
    if (inp.htim->Instance != htim->Instance) continue;
    if (chan_to_active(inp.hal_channel) != active) continue;

    uint16_t tick = static_cast<uint16_t>(
        HAL_TIM_ReadCapturedValue(htim, inp.hal_channel));

    uint32_t gap_us = static_cast<uint32_t>(
        static_cast<uint16_t>(tick - inp.prev_tick));
    inp.prev_tick = tick;

    if (gap_us >= CPPM_SYNC_GAP_US) {
      // Sync gap detected: complete any in-progress frame
      if (inp.synced && inp.ch_idx > 0) {
        inp.frame_period_us = static_cast<uint32_t>(
            static_cast<uint16_t>(tick - inp.frame_start_tick));
        inp.num_channels    = inp.ch_idx;
        inp.last_update_ms  = HAL_GetTick();
        inp.valid           = true;
      }
      inp.frame_start_tick = tick;
      inp.ch_idx           = 0;
      inp.synced           = true;
    } else if (inp.synced) {
      // Channel value: store if within bounds
      if (inp.ch_idx < CPPM_CHANNELS) {
        inp.channels[inp.ch_idx++] = gap_us;
      }
    }
    break;  // Only one active channel per callback invocation
  }
}

bool DevCPPM::GetChannels(uint16_t *channels, uint8_t &channel_count, uint16_t &period_us) const {
  if (input_count_ == 0) return false;
  const CppmInput &inp = inputs_[0];  // use first registered CPPM input
  if (!inp.valid) return false;
  memcpy(channels, inp.channels, sizeof(uint16_t) * CPPM_CHANNELS);
  channel_count = static_cast<int>(inp.num_channels);
  period_us = inp.frame_period_us;
  return true;
}

bool DevCPPM::IsFresh() const {
  if (input_count_ == 0) return false;
  const CppmInput &inp = inputs_[0];
  if (!inp.valid) return false;
  return (HAL_GetTick() - inp.last_update_ms) < CPPM_STALE_MS;
}

// ---------------------------------------------------------------------------
// HAL callback
// ---------------------------------------------------------------------------


