#include "L298N.hpp"

#include "hardware/gpio.h"

namespace motor {

    L298N::L298N(uint in1, uint in2, uint en, uint32_t frequency) : mIn1(in1, frequency), mIn2(in2, frequency), mEn(en) {
        gpio_init(mEn);
        gpio_set_dir(mEn, GPIO_OUT);
        gpio_put(mEn, 0);
    }

    void L298N::setSpeed(int32_t dutyCycleScl) {
        gpio_put(mEn, dutyCycleScl != 0);
        if (dutyCycleScl >= 0) {
            mIn1.setDutyCycle(static_cast<uint32_t>(dutyCycleScl));
            mIn2.setDutyCycle(static_cast<uint32_t>(0));
        } else {
            mIn1.setDutyCycle(static_cast<uint32_t>(0));
            mIn2.setDutyCycle(static_cast<uint32_t>(-dutyCycleScl));
        }
    }

    void L298N::setSpeed(float dutyCycle) {
        setSpeed(static_cast<int32_t>(dutyCycle * 100));
    }
    
    void L298N::setMaxSpeed(int32_t dutyCycleScl)
    {
        mIn1.setMaxDutyCycle(dutyCycleScl);
        mIn2.setMaxDutyCycle(dutyCycleScl);
    }
    
    void L298N::setMaxSpeed(float dutyCycle)
    {
        setMaxSpeed(static_cast<int32_t>(dutyCycle * 100));
    }

    L298N::~L298N() {
        gpio_deinit(mEn);
    }



}