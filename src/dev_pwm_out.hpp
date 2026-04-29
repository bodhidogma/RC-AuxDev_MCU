/**
 * Multi-channel RC servo PWM output driver.
 *
 * Timer assumptions follow project defaults: prescaler=71 (1 MHz tick), so
 * pulse width is expressed directly in microseconds.
 *
 * Classic analog-servo defaults:
 *   - frame period: 20000 us (50 Hz)
 *   - pulse range : 1000..2000 us
 *   - neutral     : 1500 us
 */

#ifndef _DEV_PWM_OUT_HPP_
#define _DEV_PWM_OUT_HPP_

#include "mymain.h"

#define PWM_OUT_MAX_CHANNELS      4
#define PWM_OUT_PERIOD_US_DEFAULT 20000u
#define PWM_OUT_PULSE_US_MIN      1000u
#define PWM_OUT_PULSE_US_MAX      2000u
#define PWM_OUT_PULSE_US_NEUTRAL  1500u

// Optional GPIO override for output channel pin.
// Set port = nullptr to use IOC/CubeMX default pin assignment.
struct PwmOutGpioConfig {
    GPIO_TypeDef *port;     // nullptr = use IOC default
    uint16_t      pin;      // e.g. GPIO_PIN_6
    uint32_t      alternate;// e.g. GPIO_AF2_TIM3
    uint32_t      pull;     // GPIO_NOPULL / GPIO_PULLUP / GPIO_PULLDOWN
    uint32_t      speed;    // GPIO_SPEED_FREQ_LOW / HIGH
};

// Per-channel startup config.
struct PwmOutChanConfig {
    TIM_HandleTypeDef *htim;       // required
    uint32_t           hal_channel;// TIM_CHANNEL_1 .. TIM_CHANNEL_4
    PwmOutGpioConfig   gpio = {nullptr, 0, 0, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW};
    uint16_t           startup_pulse_us = PWM_OUT_PULSE_US_NEUTRAL;
};

struct PwmOutChannel {
    TIM_HandleTypeDef *htim;
    uint32_t           hal_channel;
    PwmOutGpioConfig   gpio_cfg;
    bool               use_gpio_override;
    uint16_t           pulse_us;
    bool               started;
};

class DevPWMOut {
 public:
    DevPWMOut() {}

    // Initialize channel list and start PWM output.
    // period_us defaults to classic analog-servo frame period (20 ms).
    bool Initialize(const PwmOutChanConfig *configs, int count,
                                    uint32_t period_us = PWM_OUT_PERIOD_US_DEFAULT);

    // Register a channel manually (advanced/dynamic use).
    // Returns 0-based index, or -1 if full.
    int Register(const PwmOutChanConfig &cfg);

    // Servo-friendly API (microseconds).
    bool SetPulseUs(int idx, uint16_t pulse_us);
    bool GetPulseUs(int idx, uint16_t &pulse_us) const;

    bool Enable(int idx);
    bool Disable(int idx);

    // Compatibility wrappers.
    void setDutyCycle(uint8_t channel, uint16_t dutyCycle);
    uint16_t getDutyCycle(uint8_t channel) const;

    int           channel_count_ = 0;
    uint32_t      period_us_ = PWM_OUT_PERIOD_US_DEFAULT;
    PwmOutChannel channels_[PWM_OUT_MAX_CHANNELS];
};

#endif