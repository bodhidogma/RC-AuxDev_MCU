/**
 * STM32 HAL callback implementations. These are called by the HAL library when
 * configured events occur (e.g. UART byte received, timer capture, etc).
 */

// #include <cstring>

// #include "mymain.h"
#include "dev_cppm.hpp"
#include "dev_pwm_in.hpp"
#include "dev_sbus.hpp"
#include "stm_console.hpp"

extern uint8_t uart1_rx_buffer_[1];
extern DevSBus sbus;
extern DevCPPM cppm;
extern DevPWMIn pwm_dev_in;

/** UART RX complete callback
 *
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  // callback is for our UART
  // if (huart->Instance == USART1) {
#if USE_SBUS
  if (huart == sbus.MyHuart()) {
    // SBUS: HandleRx re-arms IT internally
    sbus.HandleRx(sbus.rx_byte_);
  } else
#endif
      if (huart == console.MyHuart()) {
    // console.update_buffer(uart1_rx_buffer_, sizeof(uart1_rx_buffer_));
    console.update_rx_buffer(uart1_rx_buffer_[0]);

    // re-initiate a rx request
    HAL_UART_Receive_IT(huart, uart1_rx_buffer_, sizeof(uart1_rx_buffer_));
    // HAL_UART_Receive_IT(huart, &uart_buffer_, 1);
    // HAL_UART_Receive_DMA(huart, &uart_buffer_, 1);
  }
}

/** HAL input capture callback — dispatches to the appropriate decoder. */
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
#if USE_CPPM
  // assume all CCPM inputs use same timer
  if (htim == cppm.inputs_[0].htim) {
    cppm.HandleCapture(htim);
  }
#endif
#if USE_PWM_IN
  // assume all PWM inputs use same timer
  if (htim == pwm_dev_in.channels_[0].htim) {
    pwm_dev_in.HandleCapture(htim);
  }
#endif
  return;
}
