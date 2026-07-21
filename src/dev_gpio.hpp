/**
 * @file dev_button.hpp
 */

#ifndef _DEV_GPIN_HPP_
#define _DEV_GPIN_HPP_

#include "mymain.h"

class DevGpioPin {
 private:
  GPIO_TypeDef* gpioPort;
  uint16_t gpioPin;
  bool lastState;
  bool pressed;
  bool inputPin;   // true = input pin, false = output pin
  bool activeLow;  // true = active low, false = active high
  uint32_t lastDebounceTime;
  const uint32_t debounceDelay = 50;  // Debounce time in milliseconds

 public:
  // Constructor accepts the GPIO Port and Pin
  DevGpioPin(GPIO_TypeDef* port, uint16_t pin, bool input = true,
             bool act_low = false)
      : gpioPort(port),
        gpioPin(pin),
        lastState(GPIO_PIN_RESET),
        inputPin(input),
        activeLow(act_low),
        lastDebounceTime(0) {}

  void Initialize();

  bool IsEnabled();

  void SetOutputState(bool state);
};

#endif  // _DEV_GPIN_HPP_
