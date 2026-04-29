/* RC PWM multi-channel input capture via TIM3.
 * 
 * GPIO pins are shared between PWM IN / OUT (read RC input or drive RC output)
 *
 * TIM3 config: prescaler=71, 1 MHz tick (1 tick = 1 us).
 * Default channels (IOC/CubeMX config): PA6(CH1), PA4(CH2), PB0(CH3), PB1(CH4).
 * Each channel uses independent edge-toggling capture:
 *   rising  -> record timestamp, switch to falling
 *   falling -> compute pulse_us, switch back to rising
 * Period measured rise-to-rise.
 *
 * Hardware binding:
 *   Default path  — port == nullptr in PwmInChanConfig; IOC/MspInit already
 *                   configured the GPIO; Initialize() skips GPIO init for that channel.
 *   Override path — port != nullptr; Initialize() calls HAL_GPIO_Init before
 *                   HAL_TIM_IC_Start_IT for that channel.
 */

#include "dev_pwm_in.hpp"

// Reference to the global instance defined in mymain.cpp
extern DevPWMIn pwm_dev_in;

// Map TIM_CHANNEL_x to HAL_TIM_ACTIVE_CHANNEL_x
// TIM_CHANNEL_1=0, CH2=4, CH3=8, CH4=12 -> shift right by 2, then use as bit index
static inline HAL_TIM_ActiveChannel chan_to_active(uint32_t hal_channel) {
  return static_cast<HAL_TIM_ActiveChannel>(1u << (hal_channel >> 2u));
}

// ---------------------------------------------------------------------------
// GPIO helpers
// ---------------------------------------------------------------------------

// Enable the AHB peripheral clock for the given GPIO port.
// Uses HAL RCC helper macros — idempotent if clock is already enabled.
static bool EnableGpioClock(GPIO_TypeDef *port) {
  if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
  else if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
  else { return false; }
  return true;
}

// Apply GPIO override for one channel using HAL_GPIO_Init.
static bool ApplyChannelGpio(const PwmInChanConfig &cfg) {
  if (!EnableGpioClock(cfg.port)) return false;
  GPIO_InitTypeDef gpio = {};
  gpio.Pin       = cfg.pin;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = cfg.pull;
  gpio.Speed     = cfg.speed;
  gpio.Alternate = cfg.alternate;
  HAL_GPIO_Init(cfg.port, &gpio);
  return true;
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
      if (!ApplyChannelGpio(ch.gpio_cfg)) return false;
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
