/**
 *
 */

#ifndef _DEV_LED_HPP_
#define _DEV_LED_HPP_

#include "mymain.h"

class DevLED {
 public:
  typedef enum {
    OFF = 0,  //!< off
    BLINK1,   //!<
    BLINK2,   //!<
    BLINK3,   //!<
    BLINK4,   //!<
    BLINK5,   //!<
  } led_pattern_t;

  DevLED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    port_ = GPIOx;
    pin_ = GPIO_Pin;

    pattern_ = OFF;
    pat_step_ = 0;
    last_now_ms_ = 0;
  }

  void SetPattern(led_pattern_t pat) { pattern_ = pat; }
  bool Update(void);

  bool GetConfig(void);

 protected:
 private:
  GPIO_TypeDef *port_;
  uint16_t pin_;
  led_pattern_t pattern_;
  uint8_t pat_step_;
  uint32_t last_now_ms_;
};

// extern DevLED led0;

#endif  // _DEV_ADC_HPP_
