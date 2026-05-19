/**
 *
 */

#ifndef _DEV_ADC_HPP_
#define _DEV_ADC_HPP_


#include "mymain.h"

// Configuration for each ADC channel/interface
struct AdcConfig {
  ADC_HandleTypeDef *hadc;
  uint32_t channel;           // Use ADC_CHANNEL_TEMPSENSOR for temp sensor
  bool is_temp_sensor;
};


class DevADC {
 public:
  // New constructor for config struct
  DevADC(const AdcConfig& config) {
    my_adc_ = config.hadc;
    my_channel_ = config.channel;
    temp_measurement_ = config.is_temp_sensor;
    adc_val = 0;
    initialized_ = false;
    conversion_in_flight_ = false;
  }

  // Legacy constructors for compatibility (optional, can be removed if not needed)
  DevADC(ADC_HandleTypeDef &hadc, uint32_t channel)
    : DevADC(AdcConfig{&hadc, channel, false}) {}
  DevADC(ADC_HandleTypeDef &hadc, bool temp_measurement)
    : DevADC(AdcConfig{&hadc, 0, temp_measurement}) {}

  bool Initialize(void);
  bool GetConfig(void);
  bool Update(void);
  int16_t GetValue(void) { return adc_val; }
  bool IsBusy(void) const { return conversion_in_flight_; }
  void HandleConvCplt(ADC_HandleTypeDef *hadc);
  void HandleError(ADC_HandleTypeDef *hadc);

 private:
  int16_t adc_val;
  bool initialized_;
  volatile bool conversion_in_flight_;
  bool temp_measurement_;
  ADC_HandleTypeDef *my_adc_;
  uint32_t my_channel_;
};

#endif  // _DEV_ADC_HPP_
