/* read PWM duty cycle from TIMER combined channel
 */

#include "dev_pwm_duty.hpp"

// static reference to global object
extern DevPWMDuty pwm_duty;

bool DevPWMDuty::Initialize(void) {

  // PWM receiver input capture
  HAL_TIM_IC_Start_IT(this->my_tim_, TIM_CHANNEL_1);  // Signal Input Channel (Main)
  HAL_TIM_IC_Start(this->my_tim_, TIM_CHANNEL_2);     // Secondary Channel

  return true;
}

/**
 *
 */
bool DevPWMDuty::GetConfig(void) {
#if 0
  char buff[32];

  uint32_t addr = (uint32_t)my_adc_;

  snprintf(buff, 32, "adc(0x%lx)" NL, addr);
  CON_PRINTf(buff);
#endif
  return true;
}

/** TIM Input Capture callback
 *
 * APB timer == SystemCoreClock (72Mhz)
 * Timer clock source = Internal
 * Combined channels = PWM In on CH1 (CH1 rising, CH2 falling)
 * Prescaler = 71 (72Mhz / (71 + 1) = 1Mhz), So timer ticks every 1us
 * RC transmitter signal = 50Hz, 20ms period, 1-2ms pulse width
 */

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == pwm_duty.my_tim_->Instance) {
    pwm_duty.cycle_time_ = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    if (pwm_duty.cycle_time_ != 0) {
      // SystemCoreClock == 72Mhz
      pwm_duty.freq_ = SystemCoreClock / (1 * pwm_duty.cycle_time_);    // 72Mhz
      pwm_duty.duty_ = 10000 * HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) / pwm_duty.cycle_time_;
    }
  }
}
