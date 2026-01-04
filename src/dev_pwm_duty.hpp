/**
 *
 */

#ifndef _DEV_PWM_DUTY_HPP_
#define _DEV_PWM_DUTY_HPP_

// location of all external HAL types
#include "mymain.h"

class DevPWMDuty {
 public:
  DevPWMDuty(TIM_HandleTypeDef *htim) { my_tim_ = htim; }

  bool Initialize(void);

  bool GetConfig(void);

  uint32_t freq_, cycle_time_ = 0;
  uint32_t duty_ = 0;

  TIM_HandleTypeDef *my_tim_;

 protected:
 private:

  //void *my_adc_;
};

#endif  // _DEV_PWM_DUTY_HPP_