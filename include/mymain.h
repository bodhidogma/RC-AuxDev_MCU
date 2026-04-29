/**
 *
 */

#ifndef __MYMAIN_H__
#define __MYMAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdlib.h>

#include "main.h"
//#include "usart.h"
#include "usbd_cdc_if.h"

/*
 *  Primary entry points from main.c
 */
void main_loop();

extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
//extern SPI_HandleTypeDef hspi3;

extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_tx;


#ifdef __cplusplus
}
#endif

// enable at startup / runtime, but for now just choose 1
#define USE_PWM_IN 0
#if !USE_PWM_IN
#define USE_PWM_OUT 1
#endif
#define USE_SBUS 0
#if !USE_SBUS
#define USE_CPPM 1
#endif

// arduino like macros
#define millis() HAL_GetTick()  // get current ms elapsed
#define delay(x) HAL_Delay(x)   // delay ms

#define CON_PRINTf printf
#define NL "\r\n"

#endif
