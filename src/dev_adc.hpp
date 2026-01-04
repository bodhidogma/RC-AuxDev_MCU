/**
 *
 */

#ifndef _DEV_ADC_HPP_
#define _DEV_ADC_HPP_

#include "mymain.h"

class DevADC {
 public:
  DevADC(ADC_HandleTypeDef *hadc) { my_adc_ = hadc; }

  bool Initialize(void);

  bool GetConfig(void);

 protected:
 private:
  uint32_t adc_val;

  ADC_HandleTypeDef *my_adc_;
  //void *my_adc_;
};

#endif  // _DEV_ADC_HPP_
