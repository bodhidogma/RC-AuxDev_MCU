/*
 */

#include <mymain.h>
#include <stdio.h>

#include "dev_adc.hpp"
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

extern USBD_HandleTypeDef hUsbDeviceFS;

/** F103 - USB interface needs to be re-inserted to enumerate properly.
 *
 */
//StmConsole console(&huart1, false); // UART
StmConsole console(NULL, true); // USB CDC

DevLED led0(LED_G_GPIO_Port, LED_G_Pin);
DevLED led1(LED_R_GPIO_Port, LED_R_Pin);


// Multi-ADC configuration: add more entries as needed
static const AdcConfig kAdcConfigs[] = {
  {&hadc1, ADC_CHANNEL_TEMPSENSOR, true},   // Internal temp sensor
  {&hadc1, ADC_CHANNEL_1, false},           // A1.1 - igniter
  {&hadc3, ADC_CHANNEL_1, false},           // A3.1 - battery voltage
};

extern const size_t kNumAdcs = sizeof(kAdcConfigs) / sizeof(kAdcConfigs[0]);
DevADC adc_devs[kNumAdcs] = {
  DevADC(kAdcConfigs[0]),
  DevADC(kAdcConfigs[1]),
  DevADC(kAdcConfigs[2])
};

// RC receiver PWM input capture — TIM3, 4 channels, 1 MHz tick
DevPWMIn pwm_dev_in;
// SBUS RC receiver input — USARTx, 100000 baud 8E2, RX-pin inverted (TAER)
DevSBus sbus;
// CPPM RC receiver input — single-wire PPM sum on one TIM IC pin (AETR)
DevCPPM cppm;
// PWM output — TIM4, 4 channels, 1 MHz tick
DevPWMOut pwm_dev_out;

// WS2812 RGB LED strip driver, using SPI1/2 (PA7/PB15=MOSI)
DevWS2812 ws2812_1(&hspi2);
DevWS2812 ws2812_2(&hspi3);

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
  HAL_Delay(100);

  // disable stdio buffering
  setbuf(stdout, NULL);

  // init global classes
  console.Initialize();
  for (size_t i = 0; i < kNumAdcs; ++i) {
    adc_devs[i].Initialize();
  }

#if USE_PWM_IN
  static const PwmInChanConfig kPwmInChannels[] = {
      {&htim2, TIM_CHANNEL_1},
      {&htim2, TIM_CHANNEL_2},
      {&htim2, TIM_CHANNEL_3},
      {&htim2, TIM_CHANNEL_4},
  };
  pwm_dev_in.Initialize(kPwmInChannels, 4);
#elif USE_PWM_OUT
  static const PwmOutChanConfig kPwmOutChannels[] = {
      {&htim2, TIM_CHANNEL_1},
      {&htim2, TIM_CHANNEL_2},
      {&htim2, TIM_CHANNEL_3},
      {&htim2, TIM_CHANNEL_4},
  };
  pwm_dev_out.Initialize(kPwmOutChannels, 4);
#endif
#if USE_SBUS
  // Default: IOC/CubeMX already configured the RX GPIO for USART3 on PB11.
  // If PB11 is assigned to CPPM, SBUS must remain disabled at boot.
  sbus.Initialize(huart2);
#elif USE_CPPM
  // Default CPPM target on shared PA3: TIM15_CH2.
  // Non-null port means runtime override is applied by DevCPPM.
  // GPIO must be configured explicitly — TIM15 MSP only sets up clock + IRQ,
  static const CppmInputConfig kCppmConfig = {
      &htim15,
      TIM_CHANNEL_2,
      {GPIOA, GPIO_PIN_3, GPIO_AF9_TIM15, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}};
  cppm.Initialize(&kCppmConfig, 1);
  //cppm.Initialize(&htim2, TIM_CHANNEL_4);
#endif

#if USE_WS2812
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
  ws2812fx_2.setMode(0, led_mode);
  ws2812fx_2.setOptions(0, REVERSE);
  led_mode++;
#endif

  led0.SetPattern(DevLED::BLINK1);
  led1.SetPattern(DevLED::BLINK3);

  while (1) {
    for (size_t i = 0; i < kNumAdcs; ++i) {
      adc_devs[i].Update();
    }

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

#if USE_WS2812        
        ws2812fx_1.setMode(0, led_mode);
        ws2812fx_2.setMode(0, led_mode);
        led_mode++;
#endif
        if (led_mode > FX_MODE_RAIN) {
          led_mode = 0;
        }
      }


      // Print all ADC values
      for (size_t i = 0; i < kNumAdcs; ++i) {
        snprintf((char*)buf, sizeof(buf), "adc%u: %d \t", i, adc_devs[i].GetValue());
        console.Send((const char*)buf, strlen((const char*)buf));
      }

#if USE_PWM_IN  // Print pulse width for each RC channel
      pwm_dev_in._DumpState(console, 0);  // for debugging
#endif
#if USE_SBUS  // Print SBUS status (first 4 channels)
      sbus._DumpState(console, 0);  // for debugging
#elif USE_CPPM  // Print CPPM status (first 4 channels)
      cppm._DumpState(console, 0);  // for debugging
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
          1000 + (uint16_t)((sbus_ch[0] - 172) * 0.61012);  // #1 == thr
    }
#elif USE_CPPM
    uint16_t cppm_ch[CPPM_CHANNELS];
    uint8_t cppm_count = 0;
    uint16_t period_us = 0;
    fresh = cppm.IsFresh();
    valid = cppm.GetChannels(cppm_ch, cppm_count, period_us);
    if (valid && fresh) {
      // Example: map first CPPM channel to a servo PWM output
      servo_pos = cppm_ch[2];  // #3 == thr
    }
#endif
    if (servo_pos) {
      pwm_dev_out.SetPulseUs(USE_PWM_OUT-1, servo_pos);
    }
#endif

    console.Update();
    led0.Update();
    led1.Update();

    // runs WS2812FX effect and fires customShow → ws2812_1
#if USE_WS2812    
    ws2812fx_1.service();
    ws2812fx_2.service();
#endif

    if (usb_connected == false and hUsbDeviceFS.pClassData != 0) {
      usb_connected = true;
      // console.Send("USB CDC Connected!" NL, 20);
      // HAL_Delay(1000);
      // CDC_Transmit_FS((uint8_t *)"<<USB CDC Connected>>" NL, 22);
      // HAL_Delay(100);
    }
  }
}
