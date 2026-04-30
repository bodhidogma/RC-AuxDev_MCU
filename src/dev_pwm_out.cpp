/* RC PWM multi-channel output driver.
 *
 * IOC/CubeMX requirements:
 * - Enable TIM2 PWM outputs for channels in use.
 * - Set timer base to 1 MHz tick (1 tick = 1 us).
 * - Configure GPIO pins as TIM2 alternate-function outputs.
 * - Keep pin mapping aligned with kDefaultOutputMap below unless using overrides.
 *
 * Implementation notes:
 * - Pulse values are clamped to PWM_OUT_PULSE_US_MIN..PWM_OUT_PULSE_US_MAX.
 * - Pulse is also constrained to stay below the configured period boundary.
 * - Typical RC output period is 20000 us (50 Hz), configured at Initialize().
 */

#include "dev_pwm_out.hpp"
#include "stm_hal_shims.hpp"

// Default output map for IOC/CubeMX TIM2 pins in this project.
static const struct {
	TIM_TypeDef *tim_instance;
	uint32_t hal_channel;
	PwmOutGpioConfig gpio;
} kDefaultOutputMap[] = {
		{TIM2, TIM_CHANNEL_1,
		 {GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
		{TIM2, TIM_CHANNEL_2,
		 {GPIOA, GPIO_PIN_1, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
		{TIM2, TIM_CHANNEL_3,
		 {GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
		{TIM2, TIM_CHANNEL_4,
		 {GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}},
};

static bool ResolveDefaultOutputConfig(TIM_HandleTypeDef *htim,
																			 uint32_t hal_channel,
																			 PwmOutGpioConfig &out) {
	for (const auto &entry : kDefaultOutputMap) {
		if (entry.tim_instance == htim->Instance && entry.hal_channel == hal_channel) {
			out = entry.gpio;
			return true;
		}
	}
	return false;
}

static bool IsValidHalChannel(uint32_t hal_channel) {
	return hal_channel == TIM_CHANNEL_1 || hal_channel == TIM_CHANNEL_2 ||
				 hal_channel == TIM_CHANNEL_3 || hal_channel == TIM_CHANNEL_4;
}

static uint16_t ClampPulseUs(uint16_t pulse_us, uint32_t period_us) {
	uint32_t clamped = pulse_us;
	if (clamped < PWM_OUT_PULSE_US_MIN) clamped = PWM_OUT_PULSE_US_MIN;
	if (clamped > PWM_OUT_PULSE_US_MAX) clamped = PWM_OUT_PULSE_US_MAX;

	// Keep pulse strictly below ARR+1 period boundary.
	if (period_us > 0 && clamped >= period_us) clamped = period_us - 1u;

	return static_cast<uint16_t>(clamped);
}

static bool ConfigureTimerPwmBase(TIM_HandleTypeDef *htim, uint32_t period_us) {
	htim->Init.Period = period_us - 1u;
	if (HAL_TIM_PWM_Init(htim) != HAL_OK) return false;
	return true;
}

int DevPWMOut::Register(const PwmOutChanConfig &cfg) {
	if (channel_count_ >= PWM_OUT_MAX_CHANNELS) return -1;
	if (cfg.htim == nullptr) return -1;
	if (!IsValidHalChannel(cfg.hal_channel)) return -1;

	for (int i = 0; i < channel_count_; i++) {
		if (channels_[i].htim == cfg.htim && channels_[i].hal_channel == cfg.hal_channel) {
			return -1;
		}
	}

	PwmOutChannel &ch = channels_[channel_count_];
	ch.htim = cfg.htim;
	ch.hal_channel = cfg.hal_channel;
	ch.gpio_cfg = cfg.gpio;
	ch.use_gpio_override = (cfg.gpio.port != nullptr);
	ch.pulse_us = cfg.startup_pulse_us;
	ch.started = false;
	return channel_count_++;
}

bool DevPWMOut::Initialize(const PwmOutChanConfig *configs, int count,
													 uint32_t period_us) {
	if (configs == nullptr || count <= 0) return false;
	if (period_us < 1000u || period_us > 65535u) return false;

	period_us_ = period_us;
	channel_count_ = 0;

	for (int i = 0; i < count; i++) {
		if (Register(configs[i]) < 0) return false;
	}

	TIM_HandleTypeDef *configured_timers[PWM_OUT_MAX_CHANNELS] = {nullptr};
	int configured_count = 0;

	for (int i = 0; i < channel_count_; i++) {
		PwmOutChannel &ch = channels_[i];

		// Validate mapping against known defaults even when override is used.
		PwmOutGpioConfig default_cfg = {};
		if (!ResolveDefaultOutputConfig(ch.htim, ch.hal_channel, default_cfg)) {
			return false;
		}

		if (ch.use_gpio_override) {
			if (!StmHalInitGpioAf(ch.gpio_cfg.port, ch.gpio_cfg.pin,
			                      ch.gpio_cfg.alternate, ch.gpio_cfg.pull,
			                      ch.gpio_cfg.speed)) return false;
		}

		bool needs_timer_init = true;
		for (int t = 0; t < configured_count; t++) {
			if (configured_timers[t] == ch.htim) {
				needs_timer_init = false;
				break;
			}
		}
		if (needs_timer_init) {
			if (!ConfigureTimerPwmBase(ch.htim, period_us_)) return false;
			configured_timers[configured_count++] = ch.htim;
		}

		TIM_OC_InitTypeDef sConfigOC = {};
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		ch.pulse_us = ClampPulseUs(ch.pulse_us, period_us_);
		sConfigOC.Pulse = ch.pulse_us;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

		if (HAL_TIM_PWM_ConfigChannel(ch.htim, &sConfigOC, ch.hal_channel) != HAL_OK) {
			return false;
		}
		if (HAL_TIM_PWM_Start(ch.htim, ch.hal_channel) != HAL_OK) {
			return false;
		}
		ch.started = true;
	}

	return true;
}

bool DevPWMOut::SetPulseUs(int idx, uint16_t pulse_us) {
	if (idx < 0 || idx >= channel_count_) return false;
	PwmOutChannel &ch = channels_[idx];
	if (!ch.started) return false;

	ch.pulse_us = ClampPulseUs(pulse_us, period_us_);
	__HAL_TIM_SET_COMPARE(ch.htim, ch.hal_channel, ch.pulse_us);
	return true;
}

bool DevPWMOut::GetPulseUs(int idx, uint16_t &pulse_us) const {
	if (idx < 0 || idx >= channel_count_) return false;
	pulse_us = channels_[idx].pulse_us;
	return true;
}

bool DevPWMOut::Enable(int idx) {
	if (idx < 0 || idx >= channel_count_) return false;
	PwmOutChannel &ch = channels_[idx];
	if (ch.started) return true;
	if (HAL_TIM_PWM_Start(ch.htim, ch.hal_channel) != HAL_OK) return false;
	ch.started = true;
	return true;
}

bool DevPWMOut::Disable(int idx) {
	if (idx < 0 || idx >= channel_count_) return false;
	PwmOutChannel &ch = channels_[idx];
	if (!ch.started) return true;
	if (HAL_TIM_PWM_Stop(ch.htim, ch.hal_channel) != HAL_OK) return false;
	ch.started = false;
	return true;
}

void DevPWMOut::setDutyCycle(uint8_t channel, uint16_t dutyCycle) {
	SetPulseUs(static_cast<int>(channel), dutyCycle);
}

uint16_t DevPWMOut::getDutyCycle(uint8_t channel) const {
	uint16_t pulse = 0;
	if (!GetPulseUs(static_cast<int>(channel), pulse)) return 0;
	return pulse;
}