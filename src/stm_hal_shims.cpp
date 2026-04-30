/**
 * STM32 HAL callback implementations. These are called by the HAL library when
 * configured events occur (e.g. UART byte received, timer capture, etc).
 */

// #include <cstring>

// #include "mymain.h"
#include "stm_hal_shims.hpp"
#include "dev_cppm.hpp"
#include "dev_pwm_in.hpp"
#include "dev_sbus.hpp"
#include "stm_console.hpp"

// ---------------------------------------------------------------------------
// Shared GPIO helpers
// ---------------------------------------------------------------------------

bool StmHalEnableGpioClock(GPIO_TypeDef *port) {
  if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
  else if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
  else { return false; }
  return true;
}

bool StmHalInitGpioAf(GPIO_TypeDef *port, uint16_t pin,
                      uint32_t alternate, uint32_t pull, uint32_t speed) {
  if (!StmHalEnableGpioClock(port)) return false;
  GPIO_InitTypeDef gpio = {};
  gpio.Pin       = pin;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = pull;
  gpio.Speed     = speed;
  gpio.Alternate = alternate;
  HAL_GPIO_Init(port, &gpio);
  return true;
}

extern uint8_t console_uart_rx_buffer_[1];
extern DevSBus sbus;
extern DevCPPM cppm;
extern DevPWMIn pwm_dev_in;

/** UART RX complete callback
 *
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
#if USE_SBUS
  if (huart == sbus.MyHuart()) {
    // SBUS: HandleRx re-arms IT internally
    sbus.HandleRx(sbus.rx_byte_);
  } else
#endif
      if (huart == console.MyHuart()) {
    console.update_rx_buffer(console_uart_rx_buffer_[0]);

    // re-initiate a rx request
    HAL_UART_Receive_IT(huart, console_uart_rx_buffer_, sizeof(console_uart_rx_buffer_));
    // HAL_UART_Receive_IT(huart, &uart_buffer_, 1);
    // HAL_UART_Receive_DMA(huart, &uart_buffer_, 1);
  }
}

/** HAL input capture callback — dispatches to the appropriate decoder.
 * 
*/
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
#if USE_CPPM
  // assume all CCPM inputs use same timer
  if (cppm.input_count_ > 0 && htim == cppm.inputs_[0].htim) {
    cppm.HandleCapture(htim);
  }
#endif
#if USE_PWM_IN
  // assume all PWM inputs use same timer
  if (pwm_dev_in.channel_count_ > 0 && htim == pwm_dev_in.channels_[0].htim) {
    pwm_dev_in.HandleCapture(htim);
  }
#endif
  return;
}
