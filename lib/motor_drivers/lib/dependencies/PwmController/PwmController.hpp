#ifndef PWMCONTROLLER_HPP
#define PWMCONTROLLER_HPP

#include "hardware/pwm.h"

namespace motor {
    class PwmController {

    public:
        PwmController(uint pwmPin, uint32_t frequency = 20000);

        /**
         * @brief Sets the duty cycle of the PWM controller.
         *
         * This function sets the duty cycle of the PWM controller to the specified value.
         * The duty cycle is represented as a fixed-point number with 24 bits of precision.
         * The value should be in the range of 0 to 1, where 0 represents 0% duty cycle and
         * 1 represents 100% duty cycle.
         *
         * @param dutyCycleFp The duty cycle value as a fixed-point number.
         */
        void setDutyCycle(uint32_t dutyCycleScl);
        void setDutyCycle(float dutyCycle);
        void setHz(uint32_t frequency);

        void setMaxDutyCycle(uint32_t maxDutyCycleScl);
        ~PwmController();
    private:
        const uint mPin;
        const uint mSlice;
        const uint mChan;
        uint32_t mWrap;
        float mClockDiv = 1.0f;

        uint32_t mMaxDutyCycleScl = 10000;
    };
}

#endif // PWMCONTROLLER_HPP