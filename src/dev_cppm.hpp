/**
 * CPPM (PPM Sum) RC receiver input capture driver.
 * The gap between consecutive rising edges encodes each channel:
 *   gap >= CPPM_SYNC_GAP_US (3000 µs) : frame sync — next gap = CH1
 *   gap  < CPPM_SYNC_GAP_US           : RC channel value in µs (1000–2000 µs)
 * Timer config: prescaler must yield 1 MHz tick (1 tick = 1 µs), e.g.
 *   APB clock 72 MHz → prescaler = 71.
 * Capture is triggered on every rising edge.
 * Up to CPPM_MAX_INPUTS separate CPPM wires may be registered.
 *
 */

#ifndef DEV_CPPM_HPP
#define DEV_CPPM_HPP

#include "mymain.h"
#include "stm_console.hpp"

#define CPPM_MAX_INPUTS 1u      // max registered CPPM input wires
#define CPPM_CHANNELS 18u       // max RC channels per CPPM frame
#define CPPM_SYNC_GAP_US 3000u  // gap >= this is treated as frame sync
#define CPPM_STALE_MS 100u      // ms without update before data marked stale
#define CPPM_IC_FILTER 4u       // timer IC digital filter for glitch rejection

// Optional GPIO override for CPPM input capture pin.
// Set port = nullptr to trust IOC/CubeMX default pin assignment.
struct CppmGpioConfig {
  GPIO_TypeDef* port;      // nullptr = use IOC default
  uint16_t pin;            // e.g. GPIO_PIN_11
  uint32_t alternate;      // e.g. GPIO_AF1_TIM2
  uint32_t pull;           // GPIO_NOPULL / GPIO_PULLUP / GPIO_PULLDOWN
  uint32_t speed;          // GPIO_SPEED_FREQ_LOW / HIGH
};

// CPPM input registration config.
struct CppmInputConfig {
  TIM_HandleTypeDef* htim;   // required
  uint32_t hal_channel;      // TIM_CHANNEL_1 .. TIM_CHANNEL_4
  CppmGpioConfig gpio = {nullptr, 0, 0, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW};
};

struct CppmInput {
  TIM_HandleTypeDef* htim;
  uint32_t hal_channel;              // TIM_CHANNEL_1 .. TIM_CHANNEL_4
  CppmGpioConfig gpio_cfg;           // requested/active GPIO config
  bool use_gpio_override;            // true when gpio_cfg should be applied
  uint16_t prev_tick;                // counter value at last rising edge
  uint16_t channels[CPPM_CHANNELS];  // decoded pulse widths in µs
  uint16_t frame_period_us;          // full frame period in µs
  uint16_t frame_start_tick;         // tick at start of current frame
  uint32_t last_update_ms;           // HAL_GetTick() at last complete frame
  uint8_t ch_idx;                    // next RC channel slot to fill
  uint8_t num_channels;              // channels decoded in last complete frame
  bool synced;                       // true after first sync gap seen
  bool valid;                        // true after first complete frame decoded
};

class DevCPPM {
 public:
  DevCPPM() {}

  // Register a CPPM input prior to Initialize(). Returns 0-based index, or -1
  // if full.
  int Register(const CppmInputConfig& cfg);

  // Start IC interrupts on all provided inputs.
  // Call with gpio.port == nullptr to trust IOC defaults.
  bool Initialize(const CppmInputConfig* configs, int count);

  // Backward-compatible wrapper.
  bool Initialize(TIM_HandleTypeDef* htim, uint32_t hal_channel);

  // Call from HAL_TIM_IC_CaptureCallback.
  void HandleCapture(TIM_HandleTypeDef* htim);

  // Copy latest decoded channel values (RC_CHANNELS) into `channels`.
  // Returns false if no valid frame has been received yet.
  bool GetChannels(uint16_t* channels, uint8_t& channel_count,
                   uint16_t& period_us) const;

  // Returns false if no complete frame within CPPM_STALE_MS or not yet valid.
  bool IsFresh() const;

  bool _DumpState(StmConsole& console, uint8_t mode = 0) const;  // for debugging

  int input_count_ = 0;
  CppmInput inputs_[CPPM_MAX_INPUTS];
};

#endif  // DEV_CPPM_HPP