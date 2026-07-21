/**
 *
 */

#include "dev_gpio.hpp"

#include "stm32f3xx_hal_gpio.h"

/** Initialize GPIO pin for input with pull-up and debounce handling.
 *
 */
void DevGpioPin::Initialize() {
  // GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIO port is already configured in CubeMX
  // Note: Ensure the GPIO port clock is enabled in your main code (e.g.,
  // __HAL_RCC_GPIOA_CLK_ENABLE())
  // GPIO_InitStruct.Pin = gpioPin;
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;  // Or GPIO_PULLDOWN depending on
  // circuit GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);

  // Read / set initial state
  lastState = HAL_GPIO_ReadPin(gpioPort, gpioPin);
  pressed = (lastState == (activeLow ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

/** if GPIO pin is enabled (pressed)
 *
 */
bool DevGpioPin::IsEnabled() {
  bool currentState = HAL_GPIO_ReadPin(gpioPort, gpioPin);

  if (inputPin == true) {
    uint32_t currentTime = HAL_GetTick();

    // Check if the pin state changed
    if (currentState != lastState) {
      // Check if enough time has passed to ignore bouncing
      if ((currentTime - lastDebounceTime) > debounceDelay) {
        lastDebounceTime = currentTime;
        lastState = currentState;

        // Return true if button is pressed (assuming active-low / Pull-up
        // configuration)
        pressed = (currentState == (activeLow ? GPIO_PIN_RESET : GPIO_PIN_SET));
      }
    }
  } else {
    // For output pin, just return the current state
    pressed = currentState;
  }
  return pressed;
}

/**
 *
 */

void DevGpioPin::SetOutputState(bool state) {
  if (!inputPin) {
    HAL_GPIO_WritePin(gpioPort, gpioPin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}
