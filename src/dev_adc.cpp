/*
 */

#include "dev_adc.hpp"

#include "stm32f3xx_ll_adc.h"

namespace {

volatile DevADC *g_active_adc1_owner = nullptr;
volatile DevADC *g_active_adc2_owner = nullptr;
volatile DevADC *g_active_adc3_owner = nullptr;
volatile DevADC *g_active_adc4_owner = nullptr;

volatile DevADC **GetActiveOwnerSlot(ADC_HandleTypeDef *hadc) {
  if (hadc == NULL || hadc->Instance == NULL) {
    return nullptr;
  }

  if (hadc->Instance == ADC1) return &g_active_adc1_owner;
  if (hadc->Instance == ADC2) return &g_active_adc2_owner;
  if (hadc->Instance == ADC3) return &g_active_adc3_owner;
  if (hadc->Instance == ADC4) return &g_active_adc4_owner;
  return nullptr;
}

}  // namespace

bool DevADC::Initialize(void) {
  if (my_adc_ == NULL) {
    return false;
  }

  // Ensure ADC IRQ is enabled for the selected ADC instance.
  if (my_adc_->Instance == ADC1 || my_adc_->Instance == ADC2) {
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  } else if (my_adc_->Instance == ADC3) {
    HAL_NVIC_SetPriority(ADC3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC3_IRQn);
  } else if (my_adc_->Instance == ADC4) {
    HAL_NVIC_SetPriority(ADC4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC4_IRQn);
  }

  initialized_ = true;
  conversion_in_flight_ = false;
  return true;
}

bool DevADC::Update(void) {
  if (!initialized_ || my_adc_ == NULL) {
    return false;
  }

  volatile DevADC **owner_slot = GetActiveOwnerSlot(my_adc_);
  if (owner_slot != nullptr && *owner_slot != nullptr && *owner_slot != this) {
    return true;
  }

  if ((HAL_ADC_GetState(my_adc_) & HAL_ADC_STATE_REG_BUSY) != 0U) {
    return true;
  }

  // A conversion is already active; keep non-blocking behavior and return.
  if (conversion_in_flight_) {
    return true;
  }

  ADC_ChannelConfTypeDef cfg = {};
  cfg.Channel = temp_measurement_ ? ADC_CHANNEL_TEMPSENSOR : my_channel_;
  cfg.Rank = ADC_REGULAR_RANK_1;
  cfg.SingleDiff = ADC_SINGLE_ENDED;
  cfg.SamplingTime = temp_measurement_ ? ADC_SAMPLETIME_61CYCLES_5 : ADC_SAMPLETIME_1CYCLE_5;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  cfg.Offset = 0;

  if (HAL_ADC_ConfigChannel(my_adc_, &cfg) != HAL_OK) {
    return false;
  }

  conversion_in_flight_ = true;
  if (owner_slot != nullptr) {
    *owner_slot = this;
  }
  if (HAL_ADC_Start_IT(my_adc_) != HAL_OK) {
    conversion_in_flight_ = false;
    if (owner_slot != nullptr && *owner_slot == this) {
      *owner_slot = nullptr;
    }
    return false;
  }
  return true;
}

void DevADC::HandleConvCplt(ADC_HandleTypeDef *hadc) {
  if (hadc != my_adc_) {
    return;
  }

  volatile DevADC **owner_slot = GetActiveOwnerSlot(hadc);
  if (owner_slot != nullptr && *owner_slot != this) {
    return;
  }

  uint32_t raw = HAL_ADC_GetValue(my_adc_);
  if (temp_measurement_) {
    int32_t temp_c = __LL_ADC_CALC_TEMPERATURE(3300U, raw, LL_ADC_RESOLUTION_12B);
    adc_val = (int16_t)temp_c;
  } else {
    adc_val = (int16_t)raw;
  }
  conversion_in_flight_ = false;
  if (owner_slot != nullptr && *owner_slot == this) {
    *owner_slot = nullptr;
  }
  HAL_ADC_Stop_IT(my_adc_);
}

void DevADC::HandleError(ADC_HandleTypeDef *hadc) {
  if (hadc != my_adc_) {
    return;
  }

  volatile DevADC **owner_slot = GetActiveOwnerSlot(hadc);
  if (owner_slot != nullptr && *owner_slot != this) {
    return;
  }

  conversion_in_flight_ = false;
  if (owner_slot != nullptr && *owner_slot == this) {
    *owner_slot = nullptr;
  }
  HAL_ADC_Stop_IT(my_adc_);
}

/**
 *
 */
bool DevADC::GetConfig(void) {
  char buff[32];

  uint32_t addr = (uint32_t)my_adc_;

  snprintf(buff, 32, "adc(0x%x) temp=%d" NL, addr, temp_measurement_ ? 1 : 0);
  CON_PRINTf("%s",buff);

  return true;
}
