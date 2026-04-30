/**
 * Shared STM32 HAL utility helpers.
 *
 * Provides low-level GPIO helpers that are generic enough to be reused across
 * RC drivers (CPPM, PWM in/out, SBUS).  Protocol-specific setup (TIM IC config,
 * TIM PWM config, UART receive arming, default pin maps) stays local to each
 * driver.
 *
 * All functions are normal C++ (not extern "C").  The HAL interrupt callbacks
 * are implemented in stm_hal_shims.cpp but are not declared here; they are
 * registered with the HAL via the weak-symbol override mechanism.
 */

#ifndef STM_HAL_SHIMS_HPP
#define STM_HAL_SHIMS_HPP

#include "mymain.h"

/**
 * Enable the AHB peripheral clock for the given GPIO port.
 * Uses HAL RCC helper macros — idempotent if the clock is already on.
 * Returns false if the port pointer is not recognised.
 */
bool StmHalEnableGpioClock(GPIO_TypeDef *port);

/**
 * Enable the GPIO clock and call HAL_GPIO_Init for an alternate-function pin.
 * Configures Mode = GPIO_MODE_AF_PP.  pull and speed are forwarded as-is.
 * Returns false if the port is not recognised (clock enable fails).
 */
bool StmHalInitGpioAf(GPIO_TypeDef *port, uint16_t pin,
                      uint32_t alternate, uint32_t pull, uint32_t speed);

#endif  // STM_HAL_SHIMS_HPP
