/* CPPM (PPM Sum) RC receiver input capture driver.
 *
 * CPPM encodes multiple RC channels on a single wire as a series of pulses.
 * The gap between consecutive rising edges encodes each channel:
 *   gap >= CPPM_SYNC_GAP_US (3000 µs) : frame sync — next gap = CH1
 *   gap  < CPPM_SYNC_GAP_US           : RC channel value in µs (1000–2000 µs)
 *
 * Timer config: prescaler must yield 1 MHz tick (1 tick = 1 µs), e.g.
 *   APB clock 72 MHz → prescaler = 71.
 * Capture is triggered on every rising edge.
 * Up to CPPM_MAX_INPUTS separate CPPM wires may be registered.
 */

#include "dev_cppm.hpp"
#include <string.h>

// Forward declaration — global instance defined in mymain.cpp
extern DevCPPM cppm;

// Map TIM_CHANNEL_x to HAL_TIM_ActiveChannel (same helper as dev_pwm_duty)
static inline HAL_TIM_ActiveChannel chan_to_active(uint32_t hal_channel) {
  return static_cast<HAL_TIM_ActiveChannel>(1u << (hal_channel >> 2u));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

int DevCPPM::Register(TIM_HandleTypeDef *htim, uint32_t hal_channel) {
  if (channel_count_ >= static_cast<int>(CPPM_MAX_INPUTS)) return -1;
  CppmInput &inp       = inputs_[channel_count_];
  inp.htim             = htim;
  inp.hal_channel      = hal_channel;
  inp.prev_tick        = 0;
  inp.frame_period_us  = 0;
  inp.frame_start_tick = 0;
  inp.last_update_ms   = 0;
  inp.ch_idx           = 0;
  inp.num_channels     = 0;
  inp.synced           = false;
  inp.valid            = false;
  memset(inp.rc_channels, 0, sizeof(inp.rc_channels));
  return channel_count_++;
}

bool DevCPPM::Initialize(TIM_HandleTypeDef *htim, uint32_t hal_channel) {
  Register(htim, hal_channel);
  for (int i = 0; i < channel_count_; i++) {
    CppmInput &inp = inputs_[i];
    __HAL_TIM_SET_CAPTUREPOLARITY(inp.htim, inp.hal_channel,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(inp.htim, inp.hal_channel);
  }
  return true;
}

void DevCPPM::HandleCapture(TIM_HandleTypeDef *htim) {
  HAL_TIM_ActiveChannel active = htim->Channel;

  for (int i = 0; i < channel_count_; i++) {
    CppmInput &inp = inputs_[i];
    if (inp.htim->Instance != htim->Instance) continue;
    if (chan_to_active(inp.hal_channel) != active) continue;

    uint16_t tick = static_cast<uint16_t>(
        HAL_TIM_ReadCapturedValue(htim, inp.hal_channel));

    uint32_t gap_us = static_cast<uint32_t>(
        static_cast<uint16_t>(tick - inp.prev_tick));
    inp.prev_tick = tick;

    if (gap_us >= CPPM_SYNC_GAP_US) {
      // Sync gap detected: complete any in-progress frame
      if (inp.synced && inp.ch_idx > 0) {
        inp.frame_period_us = static_cast<uint32_t>(
            static_cast<uint16_t>(tick - inp.frame_start_tick));
        inp.num_channels    = inp.ch_idx;
        inp.last_update_ms  = HAL_GetTick();
        inp.valid           = true;
      }
      inp.frame_start_tick = tick;
      inp.ch_idx           = 0;
      inp.synced           = true;
    } else if (inp.synced) {
      // Channel value: store if within bounds
      if (inp.ch_idx < CPPM_MAX_RC_CHANNELS) {
        inp.rc_channels[inp.ch_idx++] = gap_us;
      }
    }
    break;  // Only one active channel per callback invocation
  }
}

bool DevCPPM::GetChannel(int idx, uint32_t &pulse_us,
                          uint32_t &period_us) const {
  if (channel_count_ == 0) return false;
  const CppmInput &inp = inputs_[0];  // use first registered CPPM input
  if (!inp.valid) return false;
  if (idx < 0 || idx >= static_cast<int>(inp.num_channels)) return false;
  pulse_us  = inp.rc_channels[idx];
  period_us = inp.frame_period_us;
  return true;
}

bool DevCPPM::IsFresh(int idx) const {
  if (channel_count_ == 0) return false;
  const CppmInput &inp = inputs_[0];
  if (!inp.valid) return false;
  if (idx < 0 || idx >= static_cast<int>(inp.num_channels)) return false;
  return (HAL_GetTick() - inp.last_update_ms) < CPPM_STALE_MS;
}

// ---------------------------------------------------------------------------
// HAL callback
// ---------------------------------------------------------------------------


