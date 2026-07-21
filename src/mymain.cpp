/*
 */

#include <mymain.h>
#include <stdio.h>

#include "WS2812FX.h"
#include "dev_adc.hpp"
#include "dev_baro_ms5611.hpp"
#include "dev_crsf.hpp"
#include "dev_flash.hpp"
#include "dev_gpio.hpp"
#include "dev_imu_mpu6050.hpp"
#include "dev_led.hpp"
#include "dev_pwm_out.hpp"
#include "dev_ws2812.hpp"
#include "stm_console.hpp"
#include "dev_msp_handler.hpp"

// global objects

// transmitter modes: AETR, TAER

extern USBD_HandleTypeDef hUsbDeviceFS;

/** F103 - USB interface needs to be re-inserted to enumerate properly.
 *
 */
StmConsole console(&huart1, false);  // UART
// StmConsole console(NULL, true); // USB CDC
MspHandler msp;  // MSP command handler (USB CDC)

// blink LED on board (green) and external LED (red)
DevLED led0(LED_G_GPIO_Port, LED_G_Pin);
DevLED led1(LED_R_GPIO_Port, LED_R_Pin);

// track GPIO Input state(s)
DevGpioPin button1(GPIOB, GPIO_PIN_0, true, true);  // (PB0) User button
DevGpioPin usb_detect(GPIOC, GPIO_PIN_15, true);    // (PC15) USB VBUS detect
DevGpioPin igniter(GPIOB, GPIO_PIN_2, false);       // (PB2) Igniter output

// Multi-ADC configuration: add more entries as needed
static const AdcConfig kAdcConfigs[] = {
    {&hadc1, ADC_CHANNEL_TEMPSENSOR, true},  // Internal temp sensor
    {&hadc1, ADC_CHANNEL_1, false},          // A1.1 (PA0) - igniter
    {&hadc3, ADC_CHANNEL_1, false},          // A3.1 (PB1) - battery voltage
};

extern const size_t kNumAdcs = sizeof(kAdcConfigs) / sizeof(kAdcConfigs[0]);
DevADC adc_devs[kNumAdcs] = {DevADC(kAdcConfigs[0]), DevADC(kAdcConfigs[1]),
                             DevADC(kAdcConfigs[2])};

// CRSF RC receiver input — USARTx, 420000 baud 8N1
DevCRSF crsf;
// PWM output — TIM4, 4 channels, 1 MHz tick
DevPWMOut pwm_dev_out;

// WS2812 RGB LED strip driver, using SPI1/2 (PA7/PB15=MOSI)
DevWS2812 ws2812_1(&hspi2);
DevWS2812 ws2812_2(&hspi3);

// WS2812FX effects engine — drives ws2812_1 via customShow callback
// pin=0 and type=0 are unused (DevWS2812 owns the hardware)
WS2812FX ws2812fx_1(DevWS2812::kMaxLed, 0, 0);
WS2812FX ws2812fx_2(DevWS2812::kMaxLed, 0, 0);

// pressure sensor (MS5611) on I2C1 (PB6=SCL, PB7=SDA)
DevMS5611 ms5611(&hi2c1);  // MS5611

// IMU sensor (MPU6050) on I2C1 (PB6=SCL, PB7=SDA)
DevMPU6050 mpu6050(&hi2c1);  // MPU6050

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

  DevFlash flash(CONFIG_FLASH_PAGE_ADDR);

  // disable stdio buffering
  setbuf(stdout, NULL);

  // init global classes
  console.Initialize();
  for (size_t i = 0; i < kNumAdcs; ++i) {
    adc_devs[i].Initialize();
  }
  button1.Initialize();
  usb_detect.Initialize();
  igniter.Initialize();
  igniter.SetOutputState(false);  // ensure igniter is off

#if USE_MS5611_BARO
  if (!ms5611.begin()) {
    console.Send("MS5611 init failed\r\n", 20);
  }
#endif
#if USE_MPU6050_IMU
  if (!mpu6050.begin(DevMPU6050::GyroScale::FS_512,
                     DevMPU6050::AccelScale::FS_4G)) {
    console.Send("MPU6050 init failed\r\n", 22);
  }
#endif

#if 1  // FLASH
  uint32_t dataToWrite[3] = {0xDEADBEEF, 0x12345678, 0xAAAA5555};
  uint32_t dataToRead[3] = {0};

  flash.readWords(dataToRead, 3);
  snprintf((char*)buf, sizeof(buf), "Read back: 0x%08X 0x%08X 0x%08X\r\n",
           dataToRead[0], dataToRead[1], dataToRead[2]);
  console.Send((const char*)buf, strlen((const char*)buf));

  if (dataToRead[0] != 0xDEADBEEF && flash.erasePage()) {
    console.Send("Flash page erased\r\n", 20);

    if (flash.writeWords(dataToWrite, 3)) {
      console.Send("Flash write successful\r\n", 25);
    } else {
      console.Send("Flash write failed\r\n", 22);
    }
  }
#endif

#if USE_PWM_OUT
  static const PwmOutChanConfig kPwmOutChannels[] = {
      {&htim2, TIM_CHANNEL_1},
      {&htim2, TIM_CHANNEL_2},
      {&htim2, TIM_CHANNEL_3},
      {&htim2, TIM_CHANNEL_4},
  };
  pwm_dev_out.Initialize(kPwmOutChannels, 4);
#endif  // USE_PWM_OUT
#if USE_CRSF
  crsf.Initialize(huart2);
#if USE_CRSF_TELEMETRY
  crsf.UpdateFlightModeTelemetry("AUXDEV");
#endif
#endif  // USE_CRSF

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
#endif  // USE_WS2812

  led0.SetPattern(DevLED::BLINK1);
  led1.SetPattern(DevLED::BLINK3);

  while (1) {
    for (size_t i = 0; i < kNumAdcs; ++i) {
      adc_devs[i].Update();
    }

    sys_now_ms = millis();

#if USE_CRSF && USE_CRSF_TELEMETRY
    // Push telemetry values from app modules into CRSF store.
    // Battery value currently uses raw ADC units as a placeholder until scaled
    // calibration is added.
    const uint16_t battery_cV = static_cast<uint16_t>(adc_devs[2].GetValue());
    crsf.UpdateBatteryTelemetry(battery_cV, 0, 0, led_mode);
    crsf.UpdateAttitudeTelemetry(0, 0, 0);
    crsf.SendTelemetryTick(sys_now_ms);
#endif  // USE_CRSF && USE_CRSF_TELEMETRY

    // -- do something every 1s
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
#endif  // USE_WS2812
        if (led_mode > FX_MODE_RAIN) {
          led_mode = 0;
        }
      }

      // update igniter state (for testing)
      if (button1.IsEnabled()) {
        igniter.SetOutputState(true);
      } else {
        igniter.SetOutputState(false);
      }

      // print GPIO state(s)
      snprintf((char*)buf, sizeof(buf), "b1= %d u= %d i= %d ",
               button1.IsEnabled() ? 1 : 0, usb_detect.IsEnabled() ? 1 : 0,
               igniter.IsEnabled() ? 1 : 0);
      console.Send((const char*)buf, strlen((const char*)buf));

      // Print all ADC values
      for (size_t i = 0; i < kNumAdcs; ++i) {
        snprintf((char*)buf, sizeof(buf), "adc%u= %3d ", i,
                 adc_devs[i].GetValue());
        console.Send((const char*)buf, strlen((const char*)buf));
      }

#if USE_CRSF  // Print CRSF status (first 4 channels)
              // crsf._DumpState(console, 0);  // for debugging
#endif

#if USE_MS5611_BARO
      // Print MS5611 pressure and temperature
      if (ms5611.read()) {
        snprintf((char*)buf, sizeof(buf), "P= %.2f T= %.2f ",
                 ms5611.getPressure(), ms5611.getTemperature());
        console.Send((const char*)buf, strlen((const char*)buf));
      }
#endif
#if USE_MPU6050_IMU
      if (mpu6050.read()) {
        snprintf((char*)buf, sizeof(buf), "aX= %.2f gZ= %.2f T= %.2f ",
                 mpu6050.getAccX(), mpu6050.getGyroZ(),
                 mpu6050.getTemperature());
        console.Send((const char*)buf, strlen((const char*)buf));
      }
#endif
      // print EOL
      console.Send(NL, 2);
    }

    // update PWM output from CRSF input (if available)
#if USE_PWM_OUT
    uint16_t servo_pos = 0;
    bool fresh = false, valid = false;
#if USE_CRSF
    uint16_t crsf_ch[CRSF_CHANNELS];
    uint8_t crsf_count = 0;
    fresh = crsf.IsFresh();
    valid = crsf.GetChannels(crsf_ch, crsf_count);
    if (valid && fresh) {
      // Example: map 4 CRSF channel to a servo PWM output
      for (uint8_t ch = 0; ch < 4; ch++) {
        servo_pos = 1000 + (uint16_t)((crsf_ch[ch] - 172) * 0.61012);
        pwm_dev_out.SetPulseUs(ch, servo_pos);
      }
      // pwm = 1000 + (crsf - 172) * (2000-1000) / (1811-172)
    }
#endif  // USE_CRSF
#endif  // USE_PWM_OUT

    console.Update();
    led0.Update();
    led1.Update();

    // runs WS2812FX effect and fires customShow → ws2812_1
#if USE_WS2812
    ws2812fx_1.service();
    ws2812fx_2.service();
#endif  // USE_WS2812

    if (usb_connected == false and hUsbDeviceFS.pClassData != 0) {
      usb_connected = true;
      // console.Send("USB CDC Connected!" NL, 20);
      // HAL_Delay(1000);
      // CDC_Transmit_FS((uint8_t *)"<<USB CDC Connected>>" NL, 22);
      // HAL_Delay(100);
    }
  }
}
