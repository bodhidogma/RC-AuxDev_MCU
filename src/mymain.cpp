/*
 */

#include <mymain.h>
#include <stdio.h>

// #include "dev_adc.hpp"
#include "WS2812FX.h"
#include "dev_cppm.hpp"
#include "dev_led.hpp"
#include "dev_pwm_in.hpp"
#include "dev_pwm_out.hpp"
#include "dev_sbus.hpp"
#include "dev_ws2812.hpp"
#include "stm_console.hpp"

// global objects

// transmitter modes: AETR, TAER

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
DevPWMIn pwm_dev_in;
// SBUS RC receiver input — USARTx, 100000 baud 8E2, RX-pin inverted (TAER)
DevSBus sbus;
// CPPM RC receiver input — single-wire PPM sum on one TIM IC pin (AETR)
DevCPPM cppm;
// PWM output — TIM4, 4 channels, 1 MHz tick
DevPWMOut pwm_dev_out;

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
#if USE_PWM_IN
  // Default IOC pins: PA6(CH1), PA4(CH2), PB0(CH3), PB1(CH4) — null port =
  // trust IOC Override example for pin-stacking:
  //   { TIM_CHANNEL_1, GPIOB, GPIO_PIN_4, GPIO_AF2_TIM3, GPIO_NOPULL,
  //   GPIO_SPEED_FREQ_LOW }
  static const PwmInChanConfig kPwmInChannels[] = {
      {&htim3, TIM_CHANNEL_1},  // PA6
      {&htim3, TIM_CHANNEL_2},  // PA4
      {&htim3, TIM_CHANNEL_3},  // PB0
      {&htim3, TIM_CHANNEL_4},  // PB1
  };
  pwm_dev_in.Initialize(kPwmInChannels, 4);
#elif USE_PWM_OUT
  // Default IOC pins: PA6(CH1), PA4(CH2), PB0(CH3), PB1(CH4)
  static const PwmOutChanConfig kPwmOutChannels[] = {
      {&htim3, TIM_CHANNEL_1},  // PA6
      {&htim3, TIM_CHANNEL_2},  // PA4
      {&htim3, TIM_CHANNEL_3},  // PB0
      {&htim3, TIM_CHANNEL_4},  // PB1
  };
  pwm_dev_out.Initialize(kPwmOutChannels, 4);
#endif
#if USE_SBUS
  // Default: IOC/CubeMX already configured the RX GPIO for USART3 on PB11.
  // If PB11 is assigned to CPPM, SBUS must remain disabled at boot.
  sbus.Initialize(huart3);
#elif USE_CPPM
  // Default CPPM target on shared PB11: TIM2_CH4.
  // Non-null port means runtime override is applied by DevCPPM.
  // TIM2_CH4 on PB11 (shared stack with USART3_RX).
  // GPIO must be configured explicitly — TIM2 MSP only sets up clock + IRQ,
  // no pin config. Override applies PB11 as AF1 (TIM2_CH4) input.
  static const CppmInputConfig kCppmConfig = {
      &htim2,
      TIM_CHANNEL_4,
      {GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}};
  cppm.Initialize(&kCppmConfig, 1);
  // cppm.Initialize(&htim3, TIM_CHANNEL_1);  // alternative: PA6 = TIM3_CH1
#endif

  ws2812_1.Initialize();
  ws2812_2.Initialize();

  // WS2812FX setup for ws2812_1 (library handles customShow bridge internally)
  ws2812fx_1.init(&ws2812_1);
  ws2812fx_1.setBrightness(64);  //
  ws2812fx_2.init(&ws2812_2);
  ws2812fx_2.setBrightness(64);  //

  // setSegment configures LED range and initial effect; use
  // setMode()/setSpeed() at runtime
  ws2812fx_1.setSegment(0, 0, DevWS2812::kMaxLed - 1);
  ws2812fx_1.start();
  ws2812fx_2.setSegment(0, 0, DevWS2812::kMaxLed - 1);
  ws2812fx_2.start();

  // ws2812fx_1.setColors(0, (uint32_t[]){0xFF0000, 0x00FF00, 0x0000FF});
  ws2812fx_1.setSpeed(0, 1000);
  ws2812fx_1.setMode(0, led_mode);
  ws2812fx_2.setSpeed(0, 1000);
  ws2812fx_2.setMode(0, led_mode++);
  ws2812fx_2.setOptions(0, REVERSE);

  led0.SetPattern(DevLED::BLINK1);
  led1.SetPattern(DevLED::BLINK3);

  while (1) {
    sys_now_ms = millis();

    if (sys_now_ms - last_now_ms_ > 1000) {
      last_now_ms_ = sys_now_ms;
      count_s++;

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

#if USE_PWM_IN  // Print pulse width for each RC channel
      {
        uint32_t pulse_us = 0, period_us = 0;
        bool fresh;
        int len;
        len = snprintf((char*)buf, sizeof(buf), "PWM IN: ");
        console.Send((const char*)buf, len);
        for (int ch = 0; ch < pwm_dev_in.channel_count_; ch++) {
          fresh = pwm_dev_in.IsFresh(ch);
          pwm_dev_in.GetChannel(ch, pulse_us, period_us);
          len = snprintf((char*)buf, sizeof(buf), "%d:%c%4lu\t", ch + 1,
                         fresh ? ' ' : '~', pulse_us);
          console.Send((const char*)buf, len);
        }
        console.Send("\r\n", 2);
      }
#endif
#if USE_SBUS  // Print SBUS status (first 4 channels)
      {
        uint16_t sbus_ch[SBUS_CHANNELS];
        uint8_t sbus_count = 0;
        bool fresh = sbus.IsFresh();
        bool valid = sbus.GetChannels(sbus_ch, sbus_count);
        int len;
        if (!valid) {
          console.Send("SBUS: --\r\n", 10);
        } else {
          len = snprintf((char*)buf, sizeof(buf), "SBUS[%d]:%c ", sbus_count,
                         (fresh ? ' ' : '~'));
          console.Send((const char*)buf, len);
          for (int ch = 0; ch < sbus_count; ch++) {
            len = snprintf((char*)buf, sizeof(buf), "(%d) %4u\t", ch + 1,
                           sbus_ch[ch]);
            console.Send((const char*)buf, len);
          }
          console.Send("\r\n", 2);
        }
      }
#elif USE_CPPM  // Print CPPM status (first 4 channels)
      {
        uint16_t cppm_ch[CPPM_CHANNELS];
        uint8_t cppm_count = 0;
        uint16_t period_us = 0;
        bool fresh = cppm.IsFresh();
        bool valid = cppm.GetChannels(cppm_ch, cppm_count, period_us);
        int len = 0;
        if (!valid) {
          console.Send("CPPM: --\r\n", 10);
        } else {
          len = snprintf((char*)buf, sizeof(buf), "CPPM[%d @ %dms]:%c ",
                         cppm_count, period_us / 1000, (fresh ? ' ' : '~'));
          console.Send((const char*)buf, len);
          for (int ch = 0; ch < cppm_count; ch++) {
            len = snprintf((char*)buf, sizeof(buf), "(%d) %4u\t", ch + 1,
                           cppm_ch[ch]);
            console.Send((const char*)buf, len);
          }
          console.Send("\r\n", 2);
        }
      }
#endif
    }

#if USE_PWM_OUT
    uint16_t servo_pos = 0;
    bool fresh = false, valid = false;
#if USE_PWM_IN
#elif USE_SBUS
    uint16_t sbus_ch[SBUS_CHANNELS];
    uint8_t sbus_count = 0;
    fresh = sbus.IsFresh();
    valid = sbus.GetChannels(sbus_ch, sbus_count);
    if (valid && fresh) {
      // Example: map first SBUS channel to a servo PWM output
      // pwm = 1000 + (sbus - 172) * (2000-1000) / (1811-172)
      // pwm = 1000 + (sbus - 172) * 0.61012
      servo_pos =
          1000 + (uint16_t)((sbus_ch[2] - 172) * 0.61012);  // #3 == elev
    }
#elif USE_CPPM
    uint16_t cppm_ch[CPPM_CHANNELS];
    uint8_t cppm_count = 0;
    uint16_t period_us = 0;
    fresh = cppm.IsFresh();
    valid = cppm.GetChannels(cppm_ch, cppm_count, period_us);
    if (valid && fresh) {
      // Example: map first CPPM channel to a servo PWM output
      servo_pos = cppm_ch[1];  // #1 == elevator
    }
#endif
    if (servo_pos) {
      pwm_dev_out.SetPulseUs(3, servo_pos);
    }
#endif

    console.Update();
    led0.Update();
    led1.Update();

    ws2812fx_1
        .service();  // runs WS2812FX effect and fires customShow → ws2812_1
    ws2812fx_2
        .service();  // runs WS2812FX effect and fires customShow → ws2812_2

    if (usb_connected == false and hUsbDeviceFS.pClassData != 0) {
      usb_connected = true;
      // console.Send("USB CDC Connected!" NL, 20);
      // HAL_Delay(1000);
      // CDC_Transmit_FS((uint8_t *)"<<USB CDC Connected>>" NL, 22);
      // HAL_Delay(100);
    }
  }
}
