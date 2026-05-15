/**
 * Multi-channel RC PWM input capture driver.
 * Uses TIM3 independent IC mode, 1 MHz tick (prescaler=71).
 * Default channels (IOC/CubeMX config): PA6(CH1), PA4(CH2), PB0(CH3), PB1(CH4).
 *
 * Hardware binding:
 *   - Default: set port = nullptr in PwmInChanConfig → IOC already configured the GPIO.
 *   - Override: provide port/pin/alternate fields → HAL_GPIO_Init is called per-channel
 *     during Initialize() (e.g. for pin-stacking or alternate pad routing).
 */

#ifndef _DEV_PWM_IN_HPP_
#define _DEV_PWM_IN_HPP_

#include "mymain.h"
#include "stm_console.hpp"

#define PWM_IN_MAX_CHANNELS  4
#define PWM_IN_STALE_MS      100u   // ms without update before channel marked stale
#define PWM_IN_MIN_PULSE_US  750u   // lower bound for valid RC high pulse width
#define PWM_IN_MAX_PULSE_US  2250u  // upper bound for valid RC high pulse width
#define PWM_IN_MIN_PERIOD_US 3000u  // lower bound for valid RC frame period
#define PWM_IN_MAX_PERIOD_US 30000u // upper bound for valid RC frame period
#define PWM_IN_EDGE_TIMEOUT_MS 30u  // timeout while waiting for falling edge
#define PWM_IN_IC_FILTER     4u     // timer IC digital filter for glitch rejection

// Per-channel GPIO configuration.
// Set port = nullptr to use the IOC/CubeMX default pin assignment for this channel.
// Set all fields to override: HAL_GPIO_Init is called during Initialize().
struct PwmInChanConfig {
  TIM_HandleTypeDef  *htim;   // ptr to timer handle (required)  
  uint32_t      hal_channel;  // TIM_CHANNEL_1 .. TIM_CHANNEL_4 (required)
  GPIO_TypeDef *port;         // nullptr = use IOC default; non-null = override
  uint16_t      pin;          // e.g. GPIO_PIN_6
  uint32_t      alternate;    // e.g. GPIO_AF2_TIM3
  uint32_t      pull;         // GPIO_NOPULL / GPIO_PULLUP / GPIO_PULLDOWN
  uint32_t      speed;        // GPIO_SPEED_FREQ_LOW etc.
};

struct PwmChannel {
  TIM_HandleTypeDef  *htim;
  uint32_t            hal_channel;       // TIM_CHANNEL_1 .. TIM_CHANNEL_4
  PwmInChanConfig     gpio_cfg;          // resolved GPIO config for this channel
  uint16_t            rise_tick;         // counter value at last rising edge
  uint16_t            prev_rise_tick;    // counter value at previous rising edge (for period)
  uint32_t            pulse_us;          // high pulse width in microseconds
  uint32_t            period_us;         // full period in microseconds
  uint32_t            pending_period_us; // candidate period from latest rising edge
  uint32_t            last_update_ms;    // HAL_GetTick() at last completed capture
  uint32_t            last_rise_ms;      // HAL_GetTick() at last rising edge
  bool                expect_rise;       // true = waiting for rising edge
  bool                have_prev_rise;    // true after at least one rising edge captured
  bool                pending_period_valid; // true if pending_period_us is inside bounds
  bool                valid;             // true after first full pulse captured
};

class DevPWMIn {
 public:
  // Construct with timer + channel config array.
  // Constructor calls Register() for each entry; Initialize() activates capture.
  DevPWMIn() {};

  // Apply GPIO overrides (where specified) and start IC interrupts on all registered channels.
  bool Initialize(const PwmInChanConfig *configs, int count);

  // Register a channel manually (e.g. for dynamic or post-construction use).
  // Returns 0-based index, or -1 if full.
  int  Register(const PwmInChanConfig &cfg);

  // Call from HAL_TIM_IC_CaptureCallback.
  void HandleCapture(TIM_HandleTypeDef *htim);

  // Read latest measurement for channel idx. Returns false if not yet valid.
  bool GetChannel(int idx, uint32_t &pulse_us, uint32_t &period_us) const;

  // Returns false if no capture within PWM_IN_STALE_MS or not yet valid.
  bool IsFresh(int idx) const;

  // Dump state for debugging.
  bool _DumpState(StmConsole &console, uint8_t mode) const;

  int                channel_count_ = 0;
  PwmChannel         channels_[PWM_IN_MAX_CHANNELS];
};

#endif  // _DEV_PWM_IN_HPP_