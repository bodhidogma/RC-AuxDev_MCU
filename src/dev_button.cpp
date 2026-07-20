/**
 *
 */

#include "dev_button.hpp"

#include "stm32f3xx_ll_adc.h"

void DevButton::Initialize() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Note: Ensure the GPIO port clock is enabled in your main code (e.g.,
  // __HAL_RCC_GPIOA_CLK_ENABLE())
  GPIO_InitStruct.Pin = gpioPin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // Or GPIO_PULLDOWN depending on circuit
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(gpioPort, &GPIO_InitStruct);

  // Read initial state
  lastState = HAL_GPIO_ReadPin(gpioPort, gpioPin);
}

// Call this function inside your main while(1) loop
bool DevButton::IsPressed() {
  bool currentState = HAL_GPIO_ReadPin(gpioPort, gpioPin);
  uint32_t currentTime = HAL_GetTick();

  // Check if the pin state changed
  if (currentState != lastState) {
    // Check if enough time has passed to ignore bouncing
    if ((currentTime - lastDebounceTime) > debounceDelay) {
      lastDebounceTime = currentTime;
      lastState = currentState;

      // Return true if button is pressed (assuming active-low / Pull-up
      // configuration)
      pressed = (currentState == GPIO_PIN_RESET);
    }
  }
  return pressed;
}
