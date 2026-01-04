/*
 */

#include "dev_adc.hpp"

bool DevADC::Initialize(void) {

  // init and start conversions

  //HAL_ADCEx_Calibration_Start(my_adc_);
  HAL_ADC_Start_DMA(my_adc_, (uint32_t*)&adc_val, 1);
  return true;
}

// bool DevADC::Update(void) { return true; }

/**
 *
 */
bool DevADC::GetConfig(void) {
  char buff[32];

  uint32_t addr = (uint32_t)my_adc_;

  snprintf(buff, 32, "adc(0x%lx)" NL, addr);
  CON_PRINTf(buff);

  return true;
}

/** ADC conversion complete callback
 * 
*/
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

}
