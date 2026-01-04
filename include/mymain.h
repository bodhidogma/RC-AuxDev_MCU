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
void main_pre_loop();
void main_loop();

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


#ifdef __cplusplus
}
#endif

// arduino like macros
#define millis() HAL_GetTick()  // get current ms elapsed
#define delay(x) HAL_Delay(x)   // delay ms

#define CON_PRINTf printf
#define NL "\r\n"

#endif
