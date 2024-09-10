#include "BTS7960.hpp"

#include "hardware/gpio.h"

namespace motor {

    BTS7960::BTS7960(uint lpwmPin, uint rpwmPin, uint32_t frequency, uint lis, uint ris) : mLpwm(lpwmPin, frequency), mRpwm(rpwmPin, frequency),
        mLis(lis), mRis(ris) {
        if (lis != UnsetPin) {
            gpio_init(mLis);
            gpio_set_dir(mLis, GPIO_IN);
            gpio_pull_down(mLis);
        }

        if (ris != UnsetPin) {
            gpio_init(mRis);
            gpio_set_dir(mRis, GPIO_IN);
            gpio_pull_down(mRis);
        }
    }

    void BTS7960::setSpeed(int32_t dutyCycleScl) {
        if (dutyCycleScl >= 0) {
            mLpwm.setDutyCycle(static_cast<uint32_t>(dutyCycleScl));
            mRpwm.setDutyCycle(static_cast<uint32_t>(0));
        } else {
            mLpwm.setDutyCycle(static_cast<uint32_t>(0));
            mRpwm.setDutyCycle(static_cast<uint32_t>(-dutyCycleScl));
        }
    }

    void BTS7960::setSpeed(float dutyCycle) {
        setSpeed(static_cast<int32_t>(dutyCycle * 100));
    }
    
    void BTS7960::setMaxSpeed(int32_t dutyCycleScl)
    {
        mLpwm.setMaxDutyCycle(dutyCycleScl);
        mRpwm.setMaxDutyCycle(dutyCycleScl);
    }
    
    void BTS7960::setMaxSpeed(float dutyCycle)
    {
        setMaxSpeed(static_cast<int32_t>(dutyCycle * 100));
    }

    BTS7960::Alarm BTS7960::checkAlarm() {
        int alarm = 0;

        if (mLis != UnsetPin) {
            alarm |= gpio_get(mLis);
        } else if (mRis != UnsetPin) {
            alarm |= gpio_get(mLis) << 1;
        }
        return static_cast<BTS7960::Alarm>(alarm);
    }

    BTS7960::~BTS7960() {
        if (mLis != UnsetPin) gpio_deinit(mLis);
        if (mRis != UnsetPin) gpio_deinit(mRis);
    }
}