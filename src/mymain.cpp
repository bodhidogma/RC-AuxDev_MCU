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
// RC receiver PWM duty cycle reader (timer4 CH1+CH2)
DevPWMDuty pwm_duty(&htim4);
DevWS2812 ws2812(&htim2, TIM_CHANNEL_3);


static uint32_t sys_now_ms = 0;

bool usb_connected = false;

/**
 *
 */
void main_pre_loop(void){

};

/**
 *
 */
void main_loop(void) {
  uint32_t last_now_ms_ = 0;

  uint8_t buf[32];
  uint8_t buffer[] = "<<START>>\r\n";
  CDC_Transmit_FS(buffer, sizeof(buffer));
  HAL_Delay(100);

  HAL_UART_Transmit_IT(&huart1, buffer, sizeof(buffer));
  HAL_Delay(100);

  // disable stdio buffering
  setbuf(stdout, NULL);

  // init global classes
  console.Initialize();
  // adc0.Initialize();
  //pwm_duty.Initialize();
  //ws2812.Initialize();

  led0.SetPattern(DevLED::BLINK1);
  led1.SetPattern(DevLED::BLINK3);

  // HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED0_Pin, GPIO_PIN_RESET);	// RESET
  // HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  //ws2812.Loop();

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
        CDC_Transmit_FS((uint8_t *)".", 1);
      }
      //sprintf((char *)buf, "Freq: %lu Hz, Duty: %ld %%\r\n", pwm_duty.freq_, pwm_duty.duty_);
      // CDC_Transmit_FS(buf, strlen((char *)buf));
      console.Send((const char *)buf, strlen((char *)buf));
    }

    console.Update();
    led0.Update();
    led1.Update();

    if (usb_connected == false and hUsbDeviceFS.pClassData != 0) {
      usb_connected = true;
      // console.Send("USB CDC Connected!" NL, 20);
      // HAL_Delay(100);
      CDC_Transmit_FS((uint8_t *)"<<USB CDC Connected>>" NL, 22);
      HAL_Delay(100);
    }
  }
}
