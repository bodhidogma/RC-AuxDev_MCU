/* RC PWM multi-channel input capture via TIM3.
 *
 * TIM3 config: prescaler=71, 1 MHz tick (1 tick = 1 us).
 * Channels: PA6(CH1), PA4(CH2), PB0(CH3), PB1(CH4).
 * Each channel uses independent edge-toggling capture:
 *   rising  -> record timestamp, switch to falling
 *   falling -> compute pulse_us, switch back to rising
 * Period measured rise-to-rise.
 */

#include "dev_pwm_duty.hpp"

// Reference to the global instance defined in mymain.cpp
extern DevPWMDuty pwm_duty;

// Map TIM_CHANNEL_x to HAL_TIM_ACTIVE_CHANNEL_x
// TIM_CHANNEL_1=0, CH2=4, CH3=8, CH4=12 -> shift right by 2, then use as bit index
static inline HAL_TIM_ActiveChannel chan_to_active(uint32_t hal_channel) {
  return static_cast<HAL_TIM_ActiveChannel>(1u << (hal_channel >> 2u));
}

int DevPWMDuty::Register(TIM_HandleTypeDef *htim, uint32_t hal_channel) {
  if (channel_count_ >= PWM_DUTY_MAX_CHANNELS) return -1;
  PwmChannel &ch    = channels_[channel_count_];
  ch.htim           = htim;
  ch.hal_channel    = hal_channel;
  ch.rise_tick      = 0;
  ch.prev_rise_tick = 0;
  ch.pulse_us       = 0;
  ch.period_us      = 0;
  ch.last_update_ms = 0;
  ch.expect_rise    = true;
  ch.valid          = false;
  return channel_count_++;
}

bool DevPWMDuty::Initialize(void) {
  for (int i = 0; i < channel_count_; i++) {
    PwmChannel &ch = channels_[i];
    __HAL_TIM_SET_CAPTUREPOLARITY(ch.htim, ch.hal_channel,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(ch.htim, ch.hal_channel);
    ch.expect_rise = true;
  }
  return true;
}

void DevPWMDuty::HandleCapture(TIM_HandleTypeDef *htim) {
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

bool DevPWMDuty::GetChannel(int idx, uint32_t &pulse_us,
                             uint32_t &period_us) const {
  if (idx < 0 || idx >= channel_count_) return false;
  const PwmChannel &ch = channels_[idx];
  if (!ch.valid) return false;
  pulse_us  = ch.pulse_us;
  period_us = ch.period_us;
  return true;
}

bool DevPWMDuty::IsFresh(int idx) const {
  if (idx < 0 || idx >= channel_count_) return false;
  const PwmChannel &ch = channels_[idx];
  if (!ch.valid) return false;
  return (HAL_GetTick() - ch.last_update_ms) < PWM_DUTY_STALE_MS;
}

/** HAL input capture callback — dispatches to the capture manager. */
#if USE_PWM_DUTY
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  pwm_duty.HandleCapture(htim);
}
#endif
