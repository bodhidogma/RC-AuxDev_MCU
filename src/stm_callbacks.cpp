/**
 * STM32 HAL callback implementations. These are called by the HAL library when
 * configured events occur (e.g. UART byte received, timer capture, etc).
 */


//#include <cstring>

//#include "mymain.h"
#include "stm_console.hpp"
#include "dev_sbus.hpp"

extern uint8_t uart1_rx_buffer_[1];
extern DevSBus sbus;

 /** UART RX complete callback
 *
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // callback is for our UART
  // if (huart->Instance == USART1) {
  if (huart == console.MyHuart()) {
    // console.update_buffer(uart1_rx_buffer_, sizeof(uart1_rx_buffer_));
    console.update_rx_buffer(uart1_rx_buffer_[0]);

    // re-initiate a rx request
    HAL_UART_Receive_IT(huart, uart1_rx_buffer_, sizeof(uart1_rx_buffer_));
    // HAL_UART_Receive_IT(huart, &uart_buffer_, 1);
    // HAL_UART_Receive_DMA(huart, &uart_buffer_, 1);
  }
#if USE_SBUS
  else if (huart == sbus.MyHuart()) {
    // SBUS: HandleRx re-arms IT internally
    sbus.HandleRx(sbus.rx_byte_);
  }
#endif
}
