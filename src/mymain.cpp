/*
 */

#include <mymain.h>
#include <stdio.h>

// #include "dev_adc.hpp"
#include "dev_cppm.hpp"
#include "dev_led.hpp"
#include "dev_pwm_duty.hpp"
#include "dev_sbus.hpp"
#include "dev_ws2812.hpp"
#include "WS2812FX.h"
#include "stm_console.hpp"

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
#if USE_PWM_DUTY
// RC receiver PWM input capture — TIM3, 4 channels, 1 MHz tick
// CH1=PA6, CH2=PA4, CH3=PB0, CH4=PB1
DevPWMDuty pwm_duty;
#endif
#if USE_SBUS
// SBUS RC receiver input — USART2, 100000 baud 8E2, RX-pin inverted
DevSBus sbus;
#endif
#if USE_CPPM
// CPPM RC receiver input — single-wire PPM sum on one TIM IC pin
DevCPPM cppm;
#endif
// WS2812 RGB LED strip driver, using SPI1/2 (PA7/PB15=MOSI)
DevWS2812 ws2812_1(&hspi1);
DevWS2812 ws2812_2(&hspi2);

// WS2812FX effects engine — drives ws2812_1 via customShow callback
// pin=0 and type=0 are unused (DevWS2812 owns the hardware)
WS2812FX ws2812fx_1(DevWS2812::kMaxLed, 0, 0);
WS2812FX ws2812fx_2(DevWS2812::kMaxLed, 0, 0);

static uint32_t sys_now_ms = 0;
static uint32_t count_s = 0;
uint8_t led_mode = 1;

bool usb_connected = true;

/**
 *
 */
void main_loop(void) {
  uint32_t last_now_ms_ = 0;

  // HAL_Delay(2000);

  uint8_t buf[64];
  uint8_t buffer[] = "<<START>>\r\n";

  // CDC_Transmit_FS(buffer, sizeof(buffer));
  // HAL_Delay(100);
  HAL_UART_Transmit_IT(&huart1, buffer, sizeof(buffer));
  // HAL_Delay(100);

  // disable stdio buffering
  setbuf(stdout, NULL);

  // init global classes
  console.Initialize();
  // adc0.Initialize();
#if USE_PWM_DUTY
  pwm_duty.Register(&htim3, TIM_CHANNEL_1);  // PB4
  pwm_duty.Register(&htim3, TIM_CHANNEL_2);  // PB5
  pwm_duty.Register(&htim3, TIM_CHANNEL_3);  // PB0
  pwm_duty.Register(&htim3, TIM_CHANNEL_4);  // PB1
  pwm_duty.Initialize();
#endif
#if USE_SBUS
  sbus.Initialize(&huart2);
#endif
#if USE_CPPM
  cppm.Register(&htim3, TIM_CHANNEL_1);  // PB4
  cppm.Initialize();
#endif

  ws2812_1.Initialize();
  ws2812_2.Initialize();

  // WS2812FX setup for ws2812_1 (library handles customShow bridge internally)
  ws2812fx_1.init(&ws2812_1);
  ws2812fx_1.setBrightness(64);  // 
  ws2812fx_2.init(&ws2812_2);
  ws2812fx_2.setBrightness(64);  //

  // setSegment configures LED range and initial effect; use setMode()/setSpeed() at runtime
  ws2812fx_1.setSegment(0, 0, DevWS2812::kMaxLed - 1);
  ws2812fx_1.start();
  ws2812fx_2.setSegment(0, 0, DevWS2812::kMaxLed - 1);
  ws2812fx_2.start();

  //ws2812fx_1.setColors(0, (uint32_t[]){0xFF0000, 0x00FF00, 0x0000FF});
  ws2812fx_1.setSpeed(0, 1000);
  ws2812fx_1.setMode(0, led_mode);
  ws2812fx_2.setSpeed(0, 1000);
  ws2812fx_2.setMode(0, led_mode++);
  ws2812fx_2.setOptions(0, REVERSE);

  led0.SetPattern(DevLED::BLINK1);
  led1.SetPattern(DevLED::BLINK3);

  // ws2812.Loop();

  // HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED0_Pin, GPIO_PIN_RESET);	// RESET
  // HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  while (1) {
    sys_now_ms = millis();

    if (sys_now_ms - last_now_ms_ > 1000) {
      last_now_ms_ = sys_now_ms;
      count_s++;

      // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
      // fprintf( stderr, ".");
      // printf(".");
      // console.Send(".", 1);
      // console.Send("some data --", 12);
      // console.Send("++ more bytes! ", 15);
      if (usb_connected) {
        // console.Send("USB OK\r\n", 8);
        // console.Send(".", 1);
        // CDC_Transmit_FS((uint8_t *)".", 1);
      }

      if (count_s % 10 == 0) {
        snprintf((char*)buf, sizeof(buf), "led_mode: %d\r\n", led_mode);
        console.Send((const char*)buf, strlen((const char*)buf));

        ws2812fx_1.setMode(0, led_mode);
        ws2812fx_2.setMode(0, led_mode++);
        
        if (led_mode > FX_MODE_RAIN) {
          led_mode = 0;
        }
      }

      // Print pulse width for each RC channel
#if USE_PWM_DUTY
      for (int ch = 0; ch < pwm_duty.channel_count_; ch++) {
        uint32_t pulse_us = 0, period_us = 0;
        bool fresh = pwm_duty.IsFresh(ch);
        pwm_duty.GetChannel(ch, pulse_us, period_us);
        int len = snprintf((char*)buf, sizeof(buf), "CH%d: %4lu us %s\t",
                           ch + 1, pulse_us, fresh ? "OK" : "--");
        console.Send((const char*)buf, len);
      }
      console.Send("\r\n", 2);
#endif
      // Print SBUS status (first 4 channels)
#if USE_SBUS
      {
        uint16_t sbus_ch[SBUS_CHANNELS];
        int sbus_count = 0;
        bool fresh = sbus.IsFresh();
        bool valid = sbus.GetChannels(sbus_ch, sbus_count);
        if (usb_connected) {
          if (!valid) {
            console.Send("SBUS: --\r\n", 10);
          } else {
            for (int ch = 0; ch < 4; ch++) {
              int len = snprintf((char*)buf, sizeof(buf), "SBUS%d: %4u %s\t",
                                 ch + 1, sbus_ch[ch], fresh ? "OK" : "STALE");
              console.Send((const char*)buf, len);
            }
            console.Send("\r\n", 2);
          }
        }
      }
#endif
#if USE_CPPM
      {
        uint32_t cppm_pulse_us = 0, cppm_period_us = 0;
        bool fresh = cppm.IsFresh(0);
        bool valid = cppm.GetChannel(0, cppm_pulse_us, cppm_period_us);
        if (!valid) {
          console.Send("CPPM: --\r\n", 11);
        } else {
          int len = snprintf(
              (char*)buf, sizeof(buf), "CPPM CH1: %4lu us / %4lu us %s\r\n",
              cppm_pulse_us, cppm_period_us, fresh ? "OK" : "STALE");
          console.Send((const char*)buf, len);
        }
      }
#endif
    }

    console.Update();
    led0.Update();
    led1.Update();

    ws2812fx_1.service();   // runs WS2812FX effect and fires customShow → ws2812_1
    ws2812fx_2.service();   // runs WS2812FX effect and fires customShow → ws2812_2

    if (usb_connected == false and hUsbDeviceFS.pClassData != 0) {
      usb_connected = true;
      // console.Send("USB CDC Connected!" NL, 20);
      // HAL_Delay(1000);
      // CDC_Transmit_FS((uint8_t *)"<<USB CDC Connected>>" NL, 22);
      // HAL_Delay(100);
    }
  }
}
