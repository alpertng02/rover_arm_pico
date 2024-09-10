#include "PwmController.hpp"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

namespace motor {
    PwmController::PwmController(uint pwmPin, uint32_t frequency) : mPin(pwmPin), mSlice(pwm_gpio_to_slice_num(pwmPin)), mChan(pwm_gpio_to_channel(pwmPin)), mClockDiv(1.0f) {
        gpio_set_function(pwmPin, GPIO_FUNC_PWM);

        pwm_config config = pwm_get_default_config();

        uint32_t sysClockHz = clock_get_hz(clk_sys);
        if (sysClockHz / UINT16_MAX > frequency * mClockDiv) {
            mClockDiv = sysClockHz / UINT16_MAX / frequency;
            if (mClockDiv < 1.0f) {
                mClockDiv = 1.0f;
            } else if (mClockDiv > 259.0f + 15.0f / 16.0f) {
                mClockDiv = 259.0f + 15.0f / 16.0f;
            }
        }

        mWrap = static_cast<float>(sysClockHz) / (frequency * mClockDiv);
        if (mWrap > UINT16_MAX) {
            mWrap = UINT16_MAX;
        }

        pwm_config_set_clkdiv(&config, mClockDiv);
        pwm_config_set_wrap(&config, static_cast<uint16_t>(mWrap));
        pwm_init(mSlice, &config, false);
    }

    void PwmController::setDutyCycle(uint32_t dutyCycleScl) {
        if (dutyCycleScl > mMaxDutyCycleScl) {
            dutyCycleScl = mMaxDutyCycleScl;
        }
        pwm_set_chan_level(mSlice, mChan, (dutyCycleScl * mWrap) / 10000);

    }
    // 0x00000000
    void PwmController::setDutyCycle(float dutyCycle) {
        setDutyCycle(static_cast<uint32_t>(dutyCycle * 100));
    }

    void PwmController::setHz(uint32_t frequency) {
        const uint32_t sysClockHz = clock_get_hz(clk_sys);
        if (sysClockHz / UINT16_MAX > frequency * mClockDiv) {
            mClockDiv = sysClockHz / UINT16_MAX / frequency;
            if (mClockDiv < 1.0f) {
                mClockDiv = 1.0f;
            } else if (mClockDiv > 259.0f + 15.0f / 16.0f) {
                mClockDiv = 259.0f + 15.0f / 16.0f;
            }
        }

        mWrap = static_cast<float>(sysClockHz) / (frequency * mClockDiv);
        if (mWrap > UINT16_MAX) {
            mWrap = UINT16_MAX;
        }
        pwm_set_clkdiv(mSlice, mClockDiv);
        pwm_set_wrap(mSlice, mWrap);
    }

    void PwmController::setMaxDutyCycle(uint32_t maxDutyCycleScl) {
        mMaxDutyCycleScl = maxDutyCycleScl;
    }

    PwmController::~PwmController() {
        gpio_deinit(mPin);
    }


}