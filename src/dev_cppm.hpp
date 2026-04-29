/**
 * 
 */

#ifndef DEV_CPPM_HPP
#define DEV_CPPM_HPP

#include "mymain.h"

#define CPPM_MAX_INPUTS       2u    // max registered CPPM input wires
#define CPPM_MAX_RC_CHANNELS  18u   // max RC channels per CPPM frame
#define CPPM_SYNC_GAP_US      3000u // gap >= this is treated as frame sync
#define CPPM_STALE_MS         100u  // ms without update before data marked stale

struct CppmInput {
  TIM_HandleTypeDef *htim;
  uint32_t  hal_channel;                        // TIM_CHANNEL_1 .. TIM_CHANNEL_4
  uint16_t  prev_tick;                          // counter value at last rising edge
  uint32_t  rc_channels[CPPM_MAX_RC_CHANNELS];  // decoded pulse widths in µs
  uint32_t  frame_period_us;                    // full frame period in µs
  uint32_t  frame_start_tick;                   // tick at start of current frame
  uint32_t  last_update_ms;                     // HAL_GetTick() at last complete frame
  uint8_t   ch_idx;                             // next RC channel slot to fill
  uint8_t   num_channels;                       // channels decoded in last complete frame
  bool      synced;                             // true after first sync gap seen
  bool      valid;                              // true after first complete frame decoded
};

class DevCPPM {
 public:
  DevCPPM() {}

  // Register a CPPM input prior to Initialize(). Returns 0-based index, or -1 if full.
  int  Register(TIM_HandleTypeDef *htim, uint32_t hal_channel);

  // Start IC interrupts on all registered inputs.
  bool Initialize(TIM_HandleTypeDef *htim, uint32_t hal_channel);

  // Call from HAL_TIM_IC_CaptureCallback.
  void HandleCapture(TIM_HandleTypeDef *htim);

  // Read decoded RC channel `idx` from the first registered CPPM input.
  // pulse_us = channel value in µs; period_us = full frame period.
  // Returns false if not yet valid.
  bool GetChannel(int idx, uint32_t &pulse_us, uint32_t &period_us) const;

  // Returns false if no complete frame within CPPM_STALE_MS or not yet valid.
  bool IsFresh(int idx) const;

  int        channel_count_ = 0;
  CppmInput  inputs_[CPPM_MAX_INPUTS];
};

#endif  // DEV_CPPM_HPP