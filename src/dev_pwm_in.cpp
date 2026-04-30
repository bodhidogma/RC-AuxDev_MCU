/* RC PWM multi-channel input capture.
 *
 * IOC/CubeMX requirements:
 * - Enable TIM3 input-capture channels used for RC input.
 * - Set timer tick to 1 MHz (1 tick = 1 us).
 * - Configure corresponding GPIO pins as TIM3 alternate-function inputs.
 * - Default pin map expected by this module: PA6(CH1), PA4(CH2), PB0(CH3), PB1(CH4).
 *
 * Implementation notes:
 * - Pins can be overridden in code; if no override is provided, IOC GPIO setup is used.
 * - ISR toggles rising/falling polarity per channel.
 * - Rising edge stores timestamp and period (rise-to-rise); falling edge stores pulse width.
 */

#include "dev_pwm_in.hpp"
#include "stm_hal_shims.hpp"

// Reference to the global instance defined in mymain.cpp
extern DevPWMIn pwm_dev_in;

// Map TIM_CHANNEL_x to HAL_TIM_ACTIVE_CHANNEL_x
// TIM_CHANNEL_1=0, CH2=4, CH3=8, CH4=12 -> shift right by 2, then use as bit index
static inline HAL_TIM_ActiveChannel chan_to_active(uint32_t hal_channel) {
  return static_cast<HAL_TIM_ActiveChannel>(1u << (hal_channel >> 2u));
}

// ---------------------------------------------------------------------------
// DevPWMIn implementation
// ---------------------------------------------------------------------------

bool DevPWMIn::Initialize(const PwmInChanConfig *configs, int count) {
  for (int i = 0; i < count; i++) {
    Register(configs[i]);
  }
  for (int i = 0; i < channel_count_; i++) {
    PwmChannel &ch = channels_[i];
    // Override path: re-init GPIO via HAL_GPIO_Init before starting IC.
    // Default path (port == nullptr): IOC/MspInit already configured the pin.
    if (ch.gpio_cfg.port != nullptr) {
      if (!StmHalInitGpioAf(ch.gpio_cfg.port, ch.gpio_cfg.pin,
                             ch.gpio_cfg.alternate, ch.gpio_cfg.pull,
                             ch.gpio_cfg.speed)) return false;
    }
    __HAL_TIM_SET_CAPTUREPOLARITY(ch.htim, ch.hal_channel,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(ch.htim, ch.hal_channel);
    ch.expect_rise = true;
  }
  return true;
}

int DevPWMIn::Register(const PwmInChanConfig &cfg) {
  if (channel_count_ >= PWM_IN_MAX_CHANNELS) return -1;
  PwmChannel &ch    = channels_[channel_count_];
  ch.htim           = cfg.htim;
  ch.hal_channel    = cfg.hal_channel;
  ch.gpio_cfg       = cfg;
  ch.rise_tick      = 0;
  ch.prev_rise_tick = 0;
  ch.pulse_us       = 0;
  ch.period_us      = 0;
  ch.last_update_ms = 0;
  ch.expect_rise    = true;
  ch.valid          = false;
  return channel_count_++;
}

void DevPWMIn::HandleCapture(TIM_HandleTypeDef *htim) {
  HAL_TIM_ActiveChannel active = htim->Channel;

  for (int i = 0; i < channel_count_; i++) {
    PwmChannel &ch = channels_[i];
    if (ch.htim->Instance != htim->Instance) continue;
    if (chan_to_active(ch.hal_channel) != active) continue;

    uint16_t tick = static_cast<uint16_t>(
        HAL_TIM_ReadCapturedValue(htim, ch.hal_channel));

    if (ch.expect_rise) {
      // Rising edge: compute period from previous rise
      if (ch.valid) {
        ch.period_us = static_cast<uint32_t>(
            static_cast<uint16_t>(tick - ch.prev_rise_tick));
      }
      ch.prev_rise_tick = tick;
      ch.rise_tick      = tick;
      ch.expect_rise    = false;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, ch.hal_channel,
                                    TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
      // Falling edge: compute pulse width
      ch.pulse_us       = static_cast<uint32_t>(
          static_cast<uint16_t>(tick - ch.rise_tick));
      ch.last_update_ms = HAL_GetTick();
      ch.valid          = true;
      ch.expect_rise    = true;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, ch.hal_channel,
                                    TIM_INPUTCHANNELPOLARITY_RISING);
    }
    break;  // Only one active channel per callback invocation
  }
}

bool DevPWMIn::GetChannel(int idx, uint32_t &pulse_us,
                             uint32_t &period_us) const {
  if (idx < 0 || idx >= channel_count_) return false;
  const PwmChannel &ch = channels_[idx];
  if (!ch.valid) return false;
  pulse_us  = ch.pulse_us;
  period_us = ch.period_us;
  return true;
}

bool DevPWMIn::IsFresh(int idx) const {
  if (idx < 0 || idx >= channel_count_) return false;
  const PwmChannel &ch = channels_[idx];
  if (!ch.valid) return false;
  return (HAL_GetTick() - ch.last_update_ms) < PWM_IN_STALE_MS;
}
