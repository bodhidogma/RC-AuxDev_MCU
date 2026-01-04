/*

https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx

 */

#include "stm_console.hpp"

#include <cstring>

#include "mymain.h"

// external objects

#include "dev_led.hpp"

extern DevLED led0;
extern DevLED led1;

// global uart rx buffer
static uint8_t uart1_rx_buffer_[1];

/**
 *
 */
StmConsole::StmConsole(UART_HandleTypeDef *huart, bool cdc_uart) {
  my_huart_ = nullptr;
  my_cdc_uart_ = false;

  // define which uarts to use
  if (huart != nullptr) {
    my_huart_ = huart;
  } else {
    my_cdc_uart_ = cdc_uart;
  }
}

/** init console class
 *
 * @return true
 */
bool StmConsole::Initialize(void) {
  memset(tx_buffer_, 0, kTxBuffLen);
  tx_buffer_head_ = tx_buffer_tail_ = 0;
  tx_active_buff_len_ = 0;

  memset(rx_buffer_, 0, kRxBuffLen);
  rx_buffer_tail_ = 0;

  // kick off UART rx - trigger callback when RX data
  if (my_huart_ != nullptr) {
    HAL_UART_Receive_IT(my_huart_, uart1_rx_buffer_, 1);
    // HAL_UART_Receive_IT(my_huart_, uart1_rx_buffer_, 1);
    // HAL_UART_Receive_DMA(my_huart_, uart1_rx_buffer_, 1);
  } else {
  }
  return true;
}

/** Process console input and commands
 *
 * @return true if input was processed
 */
bool StmConsole::Update(void) {
  bool status = false;

  if (cmd_ready_) {
    status = process_cmd();

    // rx_buffer_[kRxBuffLen - 1] = '\0';
    // CON_PRINTf(" %s<~>", rx_buffer_);
    cmd_ready_ = false;
    rx_buffer_tail_ = 0;
  }

  return status;
}

/**
 *
 */
bool StmConsole::process_cmd(void) {
  char *cmd, *tok = nullptr;
  char *args[6] = {nullptr};
  int c;

  cmd = strtok(rx_buffer_, " ");  // find cmd separated by space
  tok = cmd;
  c = 0;

  // capture all space separated args
  while (tok && c < 6) {
    tok = strtok(NULL, " ");
    args[c++] = tok;
  }

  // help output
  if (!std::strcmp(cmd, "?") || !std::strcmp(cmd, "h")) {
    CON_PRINTf("?|h: <help>" NL);
    CON_PRINTf("cfg: <config settings>" NL);
    CON_PRINTf("led(0|1) <#>: led pattern set" NL NL);
  }
  // string starts with led
  else if (std::strcmp(cmd, "led") >= 0) {
    uint8_t l = (cmd[4] == '1');  // 0 || 1
    c = atoi(args[0]);
    if (l) {
      led1.SetPattern((DevLED::led_pattern_t)c);
    } else {
      led0.SetPattern((DevLED::led_pattern_t)c);
    }
    CON_PRINTf("Set LED%d = %d" NL NL, l, c);
  }
  else if (!std::strcmp(cmd, "cfg")) {
    CON_PRINTf("Config:" NL);
    led0.GetConfig();
    led1.GetConfig();
  }
  CON_PRINTf("> ");
  return true;
}

/** check if USB CDC is ready to transmit
 *
 */
#ifndef CDC_TransmitReady_FS
USBD_StatusTypeDef CDC_TransmitReady_FS(void) {
  return USBD_OK;
}
#endif

/** update tx pointers after a completed TX
 *
 */
uint8_t StmConsole::update_tx_head(void) {
  HAL_StatusTypeDef status = HAL_OK;

  // entire buffer has been TX'd
  if (tx_buffer_head_ == tx_buffer_tail_) {
    tx_active_buff_len_ = 0;
    tx_buffer_head_ = tx_buffer_tail_ = 0;
  }
  // need to trigger a new TX operation (still active)
  else {
    tx_active_buff_len_ = tx_buffer_tail_ - tx_buffer_head_;
    if (my_huart_ != nullptr) {
      status = HAL_UART_Transmit_IT(my_huart_,
                                    (uint8_t *)&tx_buffer_[tx_buffer_head_],
                                    tx_active_buff_len_);
    } else {
      if (CDC_TransmitReady_FS() == USBD_OK)
      {
        CDC_Transmit_FS((uint8_t *)&tx_buffer_[tx_buffer_head_],
                        tx_active_buff_len_);
      }
    }

    // TX submitted, head is now @ tail
    tx_buffer_head_ = tx_buffer_tail_;
  }

  return status;
}

/**
 *
 */
uint8_t StmConsole::Send(const char *buf, uint16_t len) {
  HAL_StatusTypeDef status = HAL_OK;
  // enough space to output our buffer?
  if (len > (kTxBuffLen - tx_buffer_tail_)) {
    return HAL_BUSY;
  }

  // append data to end of buffer (tail)
  memcpy(&tx_buffer_[tx_buffer_tail_], buf, len);
  tx_buffer_tail_ += len;

  if (tx_active_buff_len_ == 0) {
    tx_active_buff_len_ = len;
    if (my_huart_ != nullptr) {
      status = HAL_UART_Transmit_IT(my_huart_,
                                    (uint8_t *)&tx_buffer_[tx_buffer_head_],
                                    tx_active_buff_len_);
      // status = HAL_UART_Transmit_DMA(my_huart_,
      //                               (uint8_t *)&tx_buffer_[tx_buffer_head_],
      //                               tx_active_buff_len_);
    } else {
      if (CDC_TransmitReady_FS() == USBD_OK)
      {
        CDC_Transmit_FS((uint8_t *)&tx_buffer_[tx_buffer_head_],
                        tx_active_buff_len_);
      }
    }

    // TX submitted, head is now @ tail
    tx_buffer_head_ = tx_buffer_tail_;
  }

  return status;
}

/** append data to rx buffer checking for end of cmd
 *
 */
bool StmConsole::update_rx_buffer(uint8_t data) {
  // unable to add any more data to buffer till processed
  if ((rx_buffer_tail_) >= kRxBuffLen) {
    return false;
  }

  // append new data
  rx_buffer_[rx_buffer_tail_++] = data;

  // CON_PRINTf("%c/%d ", (char)data, buffer_pos_);

  // check for end of buffer
  if (rx_buffer_tail_ >= kRxBuffLen) {
    CON_PRINTf("<!>" NL);
    cmd_ready_ = true;
  }
  // check for end of line
  else if (rx_buffer_[rx_buffer_tail_ - 1] == '\r' ||
           rx_buffer_[rx_buffer_tail_ - 1] == '\n') {
    rx_buffer_[--rx_buffer_tail_] = '\0';

    // check for non-empty line / have some bytes
    if (rx_buffer_tail_) {
      // CON_PRINTf("<%d>" NL, rx_buffer_tail_);
      CON_PRINTf(NL);
      cmd_ready_ = true;
    } else {
      CON_PRINTf(NL "> ");
    }
  }
  // misc chars to process (backspace / delete)
  else if (rx_buffer_[rx_buffer_tail_ - 1] == '\b' ||
           rx_buffer_[rx_buffer_tail_ - 1] == '\x7f') {
    rx_buffer_[--rx_buffer_tail_] = '\0';
    if (rx_buffer_tail_ > 0) {
      rx_buffer_[--rx_buffer_tail_] = '\0';
      CON_PRINTf("\b \b");
    }
  }
  // echo received char back to serial port
  else {
    // CON_PRINTf("%c", data);
    Send((char *)&data, 1);
  }
  return true;
}

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
}

/** UART TX complete callback
 *
 */
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // callback is for our UART
  // if (huart->Instance == USART1) {
  if (huart == console.MyHuart()) {
    console.update_tx_head();
  }
}

/** USB CDC RX complete callback
 *
 */
extern "C" void USBD_CDC_RxCpltCallback(uint8_t *buf, uint32_t len) {
  // do something with USB UART data
  do {
    console.update_rx_buffer(*(buf++));
  } while (--len);
}

/** USB CDC TX complete callback
 *
 */
extern "C" void USBD_CDC_TxCpltCallback(uint8_t *buf, uint32_t len) {
  // do something with USB UART data
  console.update_tx_head();
}
