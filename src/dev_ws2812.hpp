/**
 * Basic ws2812 driver using PWM + DMA
 * 
 * Configure TIMx for PWM output at ~800kHz frequency: 72MHz / 90 = 800kHz
 * Confifure TIMx, enable INT
 * Configure DMA to feed PWM duty cycle data to TIMx CCRx register
 * Bit values: 1 = 66% duty cycle, 0 = 33% duty cycle
 * 
 */

#ifndef _DEV_WS2812_HPP_
#define _DEV_WS2812_HPP_

// location of all external HAL types
#include "mymain.h"


class DevWS2812 {
 public:
  DevWS2812(TIM_HandleTypeDef *htim, uint32_t chan) {
    my_tim_ = htim;
    channel_ = chan;
  }

  bool Initialize(void);

  bool GetConfig(void);

  bool Loop(void);

  TIM_HandleTypeDef *my_tim_;
  uint32_t channel_;

 protected:
 private:
  // void *my_adc_;
};

#endif  // _DEV_WS2812_HPP_