/**
 * CRSF RC receiver input driver.
 * Uses USART2 by default: 100000 baud, 8E2, RX-only, RX pin inverted (configured by CubeMX).
 *
 * Hardware binding:
 *   - Default: IOC/CubeMX already configured the RX GPIO; Initialize() simply arms IT.
 *   - Override: pass a non-null CRSFRxConfig to re-map the RX pin at startup (e.g. for
 *     pin-stacking or multi-UART board variants). HAL_GPIO_Init is called for the new pin.
 */

#ifndef DEV_CRSF_HPP
#define DEV_CRSF_HPP

#include "mymain.h"
#include "stm_console.hpp"


#endif  // DEV_CRSF_HPP