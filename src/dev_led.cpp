/*
 */

#include "dev_led.hpp"
#include <cstdio>

bool DevLED::Update(void) {
  uint32_t now_ms = millis();

  switch (pattern_) {
    case BLINK1:
      if (now_ms - last_now_ms_ > 1000) {
        last_now_ms_ = now_ms;
        HAL_GPIO_WritePin(port_, pin_, (GPIO_PinState)pat_step_);  //
        pat_step_ = !pat_step_;
      }
      break;

    case BLINK2:
      if (now_ms - last_now_ms_ > 250) {
        last_now_ms_ = now_ms;
        HAL_GPIO_WritePin(port_, pin_, (GPIO_PinState)pat_step_);  //
        pat_step_ = !pat_step_;
      }
      break;

    case BLINK3:
      if (now_ms - last_now_ms_ > 250) {
        last_now_ms_ = now_ms;
        switch (pat_step_) {
          case 0:
          case 1:
            HAL_GPIO_WritePin(port_, pin_, (GPIO_PinState)pat_step_);  //

          case 2:
          case 3:
            pat_step_++;
            break;
          default:
            pat_step_ = 0;
            break;
        }
      }
      break;

    case OFF:
    default:
      HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);  // RESET
      break;
  }

  return true;
}

/**
 * 
*/
bool DevLED::GetConfig(void) {

  char buff[32];

  uint32_t addr = (uint32_t)port_ - (uint32_t)GPIOA;

  snprintf(buff, 32, "led(0x%lx,0x%x) = %d" NL, addr, pin_, (int)pattern_);
  CON_PRINTf(buff);

  return true;
}