/*
 */

#include <mymain.h>
#include <stdio.h>

//#include "dev_adc.hpp"
#include "dev_led.hpp"
#include "stm_console.hpp"
#include "dev_pwm_duty.hpp"
#include "dev_ws2812.hpp"

// global objects

// extern ADC_HandleTypeDef hadc1;

extern USBD_HandleTypeDef hUsbDeviceFS;

/** F103 - USB interface needs to be re-inserted to enumerate properly.
 *
 */
StmConsole console(&huart1, false);
// StmConsole console(NULL, true);

DevLED led0(LED0_GPIO_Port, LED0_Pin);
DevLED led1(LED1_GPIO_Port, LED1_Pin);

// DevADC adc0(&hadc1);
// RC receiver PWM input capture — TIM3, 4 channels, 1 MHz tick
// CH1=PA6, CH2=PA4, CH3=PB0, CH4=PB1
DevPWMDuty pwm_duty;
// WS2812 RGB LED strip driver, using SPI1/2 (PA7/PB15=MOSI)
DevWS2812 ws2812_1(&hspi1);
DevWS2812 ws2812_2(&hspi2);


static uint32_t sys_now_ms = 0;

bool usb_connected = true;

/**
 *
 */
void main_loop(void) {
  uint32_t last_now_ms_ = 0;

  //HAL_Delay(2000);

  uint8_t buf[32];
  uint8_t buffer[] = "<<START>>\r\n";

  //CDC_Transmit_FS(buffer, sizeof(buffer));
  //HAL_Delay(100);
  HAL_UART_Transmit_IT(&huart1, buffer, sizeof(buffer));
  //HAL_Delay(100);

  // disable stdio buffering
  setbuf(stdout, NULL);

  // init global classes
  console.Initialize();
  // adc0.Initialize();
  pwm_duty.Register(&htim3, TIM_CHANNEL_1);  // PB4
  pwm_duty.Register(&htim3, TIM_CHANNEL_2);  // PB5
  pwm_duty.Register(&htim3, TIM_CHANNEL_3);  // PB0
  pwm_duty.Register(&htim3, TIM_CHANNEL_4);  // PB1
  pwm_duty.Initialize();
  ws2812_1.Initialize();
  ws2812_2.Initialize();
  led0.SetPattern(DevLED::BLINK1);
  led1.SetPattern(DevLED::BLINK3);

  //ws2812.Loop();

  // HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED0_Pin, GPIO_PIN_RESET);	// RESET
  // HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  while (1) {
    sys_now_ms = millis();

    if (sys_now_ms - last_now_ms_ > 1000) {
      last_now_ms_ = sys_now_ms;

      // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      // fprintf( stderr, ".");
      // printf(".");
      // console.Send(".", 1);
      // console.Send("some data --", 12);
      // console.Send("++ more bytes! ", 15);
      if (usb_connected) {
        // console.Send("USB OK\r\n", 8);
        console.Send(".", 1);
        //CDC_Transmit_FS((uint8_t *)".", 1);
      }
      // Print pulse width for each RC channel
      for (int ch = 0; ch < pwm_duty.channel_count_; ch++) {
        uint32_t pulse_us = 0, period_us = 0;
        bool fresh = pwm_duty.IsFresh(ch);
        pwm_duty.GetChannel(ch, pulse_us, period_us);
        int len = snprintf((char *)buf, sizeof(buf),
                           "CH%d: %4lu us %s\r\n",
                           ch + 1, pulse_us, fresh ? "OK" : "--");
        //console.Send((const char *)buf, len);
        if (usb_connected) {
          CDC_Transmit_FS(buf, len);
        }
      }
    }

    console.Update();
    led0.Update();
    led1.Update();
  
    ws2812_1.Update();
    ws2812_2.Update();

    if (usb_connected == false and hUsbDeviceFS.pClassData != 0) {
      usb_connected = true;
      // console.Send("USB CDC Connected!" NL, 20);
      //HAL_Delay(1000);
      //CDC_Transmit_FS((uint8_t *)"<<USB CDC Connected>>" NL, 22);
      //HAL_Delay(100);
    }
  }
}
