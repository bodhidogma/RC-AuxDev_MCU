/**
 * Multi-channel RC PWM input capture driver.
 * Uses TIM3 independent IC mode, 1 MHz tick (prescaler=71).
 * Supports up to 4 channels: PA6(CH1), PA4(CH2), PB0(CH3), PB1(CH4).
 */

#ifndef _DEV_PWM_DUTY_HPP_
#define _DEV_PWM_DUTY_HPP_

#include "mymain.h"

#define USE_PWM_DUTY 0

#define PWM_DUTY_MAX_CHANNELS  4
#define PWM_DUTY_STALE_MS      100u   // ms without update before channel marked stale

struct PwmChannel {
  TIM_HandleTypeDef *htim;
  uint32_t  hal_channel;       // TIM_CHANNEL_1 .. TIM_CHANNEL_4
  uint16_t  rise_tick;         // counter value at last rising edge
  uint16_t  prev_rise_tick;    // counter value at previous rising edge (for period)
  uint32_t  pulse_us;          // high pulse width in microseconds
  uint32_t  period_us;         // full period in microseconds
  uint32_t  last_update_ms;    // HAL_GetTick() at last completed capture
  bool      expect_rise;       // true = waiting for rising edge
  bool      valid;             // true after first full pulse captured
};

class DevPWMDuty {
 public:
  DevPWMDuty() {}

  // Register a channel prior to Initialize(). Returns 0-based index, or -1 if full.
  int  Register(TIM_HandleTypeDef *htim, uint32_t hal_channel);

  // Start IC interrupts on all registered channels.
  bool Initialize(void);

  // Call from HAL_TIM_IC_CaptureCallback.
  void HandleCapture(TIM_HandleTypeDef *htim);

  // Read latest measurement for channel idx. Returns false if not yet valid.
  bool GetChannel(int idx, uint32_t &pulse_us, uint32_t &period_us) const;

  // Returns false if no capture within PWM_DUTY_STALE_MS or not yet valid.
  bool IsFresh(int idx) const;

  int        channel_count_ = 0;
  PwmChannel channels_[PWM_DUTY_MAX_CHANNELS];
};

#endif  // _DEV_PWM_DUTY_HPP_