/*
 */

#ifndef __STM_CONSOLE_HPP__
#define __STM_CONSOLE_HPP__

#include <stdbool.h>
#include <stdint.h>

#include "mymain.h"

class StmConsole {
 public:
  StmConsole(UART_HandleTypeDef *huart, bool cdc_uart);

  UART_HandleTypeDef *MyHuart() { return my_huart_; }

  bool Initialize();
  bool Update();

  uint8_t Send(const char *buf, uint16_t len);

  // bool update_buffer(uint8_t *buff, uint8_t buff_len);
  bool update_rx_buffer(uint8_t data);
  uint8_t update_tx_head(void);

 protected:
 private:
  bool process_cmd(void);

  UART_HandleTypeDef *my_huart_;
  bool my_cdc_uart_;

  bool cmd_ready_;

  const static uint16_t kTxBuffLen = 128;
  char tx_buffer_[kTxBuffLen];
  uint16_t tx_buffer_head_, tx_buffer_tail_;
  uint16_t tx_active_buff_len_;

  const static uint16_t kRxBuffLen = 16;
  char rx_buffer_[kRxBuffLen];
  uint16_t rx_buffer_tail_;
};

extern StmConsole console;

#endif