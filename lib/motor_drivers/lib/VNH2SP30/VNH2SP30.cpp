#include "VNH2SP30.hpp"

#include "hardware/gpio.h"
#include "hardware/adc.h"

namespace motor {
    VNH2SP30::VNH2SP30(uint inA, uint inB, uint pwm, uint cs, uint en, uint32_t frequency) : mInA(inA), mInB(inB), mPwm(pwm, frequency), mCs(cs), mEn(en) {
        gpio_init(mInA);
        gpio_set_dir(mInA, GPIO_OUT);
        gpio_put(mInA, 0);

        gpio_init(mInB);
        gpio_set_dir(mInB, GPIO_OUT);
        gpio_put(mInB, 0);

        if (cs != UnsetPin) {
            adc_init();
            adc_gpio_init(cs);
        }

        if (en != UnsetPin) {
            gpio_init(en);
            gpio_set_dir(en, GPIO_IN);
            gpio_pull_up(en);
        }
    }

    void VNH2SP30::setSpeed(int32_t dutyCycleScl) {

        if (dutyCycleScl == 0) {
            gpio_put(mInA, 0);
            gpio_put(mInB, 0);
        } else if (dutyCycleScl > 0) {
            gpio_put(mInA, 1);
            gpio_put(mInB, 0);
        } else {
            gpio_put(mInA, 0);
            gpio_put(mInB, 1);
        }
        mPwm.setDutyCycle(dutyCycleScl >= 0 ? static_cast<uint32_t>(dutyCycleScl) : static_cast<uint32_t>(-dutyCycleScl));
    }

    void VNH2SP30::setSpeed(float dutyCycle) {
        setSpeed(static_cast<int32_t>(dutyCycle * 100));
    }

    void VNH2SP30::setMaxSpeed(int32_t dutyCycleScl) {
        mPwm.setMaxDutyCycle(dutyCycleScl);
    }

    void VNH2SP30::setMaxSpeed(float dutyCycle) {
        setMaxSpeed(static_cast<int32_t>(dutyCycle * 100));
    }

    float VNH2SP30::readCurrent() {
        if (mCs == UnsetPin) {
            return 0;
        } else {
            constexpr float conversionFactor = 3.3f / 4095;
            constexpr float ampsPerVolt = 0.13f;
            adc_select_input(mCs);
            return adc_read() * conversionFactor * ampsPerVolt;
        }
    }

    VNH2SP30::~VNH2SP30() {
        gpio_deinit(mInA);
        gpio_deinit(mInB);
        if (mCs != UnsetPin) {
            gpio_deinit(mCs);
        }
        if (mEn != UnsetPin) {
            gpio_deinit(mEn);
        }
    }



}