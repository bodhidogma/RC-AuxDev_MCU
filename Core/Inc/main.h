/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_G_Pin GPIO_PIN_13
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOC
#define I_VBUS_SENS_Pin GPIO_PIN_15
#define I_VBUS_SENS_GPIO_Port GPIOC
#define A11_IGNITER_SENS_Pin GPIO_PIN_0
#define A11_IGNITER_SENS_GPIO_Port GPIOA
#define T2C2_PWMIO2_Pin GPIO_PIN_1
#define T2C2_PWMIO2_GPIO_Port GPIOA
#define U2_RX_OUT_Pin GPIO_PIN_2
#define U2_RX_OUT_GPIO_Port GPIOA
#define U2_RX_IN_Pin GPIO_PIN_3
#define U2_RX_IN_GPIO_Port GPIOA
#define I_BUTTON_Pin GPIO_PIN_0
#define I_BUTTON_GPIO_Port GPIOB
#define I_VBAT_SENS_Pin GPIO_PIN_1
#define I_VBAT_SENS_GPIO_Port GPIOB
#define O_LAUNCH_TRIG_Pin GPIO_PIN_2
#define O_LAUNCH_TRIG_GPIO_Port GPIOB
#define T2C3_PWMIO3_Pin GPIO_PIN_10
#define T2C3_PWMIO3_GPIO_Port GPIOB
#define T2C4_PWMIO4_Pin GPIO_PIN_11
#define T2C4_PWMIO4_GPIO_Port GPIOB
#define _spi2_sck_Pin GPIO_PIN_13
#define _spi2_sck_GPIO_Port GPIOB
#define S2_WS2812_CH1_Pin GPIO_PIN_15
#define S2_WS2812_CH1_GPIO_Port GPIOB
#define T2C1_PWMIO1_Pin GPIO_PIN_15
#define T2C1_PWMIO1_GPIO_Port GPIOA
#define S3_WS2812_CH2_Pin GPIO_PIN_5
#define S3_WS2812_CH2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
