/**
 * @file dev_button.hpp
 */

#ifndef _DEV_BUTTON_HPP_
#define _DEV_BUTTON_HPP_

#include "mymain.h"

class DevButton {
  private:
    GPIO_TypeDef* gpioPort;
    uint16_t gpioPin;
    bool lastState;
    bool pressed;
    uint32_t lastDebounceTime;
    const uint32_t debounceDelay = 50; // Debounce time in milliseconds

  public:
    // Constructor accepts the GPIO Port and Pin
    DevButton(GPIO_TypeDef* port, uint16_t pin)
        : gpioPort(port), gpioPin(pin), lastState(GPIO_PIN_RESET), lastDebounceTime(0) {}

    void Initialize();
    bool IsPressed();
};

#endif // _DEV_BUTTON_HPP_
