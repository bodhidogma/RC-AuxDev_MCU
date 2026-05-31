/* SBUS RC receiver input driver.
 *
 * IOC/CubeMX requirements:
 * - Configure USART2 for SBUS: 100000 baud, 8E2, RX enabled.
 * - Enable RX pin inversion (required for standard inverted SBUS receivers).
 * - Configure RX GPIO as USART alternate-function input.
 * - Keep UART/pin mapping aligned with kDefaultRxMap below unless using
 * override.
 *
 * Implementation notes:
 * - Uses HAL_UARTEx_ReceiveToIdle_DMA when available; falls back to IT RX.
 * - 25-byte frames are accepted only with header 0x0F and footer 0x00.
 * - Unpacks 16 channels x 11-bit (bytes 1..22) and stores flags from byte 23.
 * - Uses gap-based reset plus in-frame header scan for fast re-alignment.
 */

#include "dev_crsf.hpp"

#include <string.h>

#include "stm_hal_shims.hpp"

// Forward declaration — global instance defined in mymain.cpp
//extern DevCSRF csrf;

// ---------------------------------------------------------------------------
// Default RX pin table — one entry per supported UART instance.
// Values must match the IOC/MspInit configuration for that peripheral so
// the default (no-override) path stays consistent with CubeMX output.
// Add entries here when enabling additional UART instances.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

