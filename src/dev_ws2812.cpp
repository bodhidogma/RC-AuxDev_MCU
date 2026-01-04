/* read PWM duty cycle from TIMER combined channel
 */

#include "dev_ws2812.hpp"

#include <math.h>

// static reference to global object
extern DevWS2812 ws2812;

#define MAX_LED 4
#define USE_BRIGHTNESS 1

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

// GBR data + 50uS of reset
uint16_t pwmData[(24 * MAX_LED) + 50];
volatile int datasentflag = 0;

void Set_LED(int LEDnum, int Red, int Green, int Blue) {
  LED_Data[LEDnum][0] = LEDnum;
  LED_Data[LEDnum][1] = Green;
  LED_Data[LEDnum][2] = Red;
  LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness(int brightness)  // 0-45
{
#if USE_BRIGHTNESS
  if (brightness > 45) brightness = 45;
  for (int i = 0; i < MAX_LED; i++) {
    LED_Mod[i][0] = LED_Data[i][0];
    for (int j = 1; j < 4; j++) {
      float angle = 90 - brightness;  // in degrees
      angle = angle * PI / 180;       // in rad
      LED_Mod[i][j] = (LED_Data[i][j]) / (tan(angle));
    }
  }
#endif
}

void WS2812_Send(void) {
  uint32_t indx = 0;
  uint32_t color;

  for (int i = 0; i < MAX_LED; i++) {
#if USE_BRIGHTNESS
    color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));
#else
    color = ((LED_Data[i][1] << 16) | (LED_Data[i][2] << 8) | (LED_Data[i][3]));
#endif

    for (int i = 23; i >= 0; i--) {
      if (color & (1 << i)) {
        pwmData[indx] = 60;  // 2/3 of 90
      }

      else
        pwmData[indx] = 30;  // 1/3 of 90

      indx++;
    }
  }
  // set 50uS of low to reset to 1st led
  for (int i = 0; i < 50; i++) {
    pwmData[indx] = 0;
    indx++;
  }

  HAL_TIM_PWM_Start_DMA(ws2812.my_tim_, ws2812.channel_, (uint32_t *)pwmData,
                        indx);
  while (!datasentflag) {
  };
  datasentflag = 0;
}

// --

bool DevWS2812::Initialize(void) {
  Set_LED(0, 255, 0, 0);
  Set_LED(1, 0, 255, 0);
  Set_LED(2, 0, 0, 255);
  Set_LED(3, 46, 89, 128);
  // Set_LED(4, 156, 233, 100);
  // Set_LED(5, 102, 0, 235);
  // Set_LED(6, 47, 38, 77);
  // Set_LED(7, 255, 200, 0);

  return true;
}

/**
 *
 */
bool DevWS2812::GetConfig(void) {
#if 0
  char buff[32];

  uint32_t addr = (uint32_t)my_adc_;

  snprintf(buff, 32, "adc(0x%lx)" NL, addr);
  CON_PRINTf(buff);
#endif
  return true;
}

/**
 *
 */
bool DevWS2812::Loop(void) {
  while (1) {
    for (int i = 0; i < 46; i++) {
      Set_Brightness(i);
      WS2812_Send();
      HAL_Delay(50);
    }

    for (int i = 45; i >= 0; i--) {
      Set_Brightness(i);
      WS2812_Send();
      HAL_Delay(50);
    }
  }
  return true;
}

/** */
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == ws2812.my_tim_->Instance) {
    HAL_TIM_PWM_Stop_DMA(htim, ws2812.channel_);
    datasentflag = 1;
  }
}
