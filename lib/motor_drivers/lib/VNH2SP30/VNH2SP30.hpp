#ifndef VNH2SP30_HPP
#define VNH2SP30_HPP

#include "IMotor.hpp"
#include "PwmController.hpp"

namespace motor {

    class VNH2SP30 : public IMotor {

    public:

        VNH2SP30(uint inA, uint inB, uint pwm, uint cs = UnsetPin, uint en = UnsetPin, uint32_t frequency = 20000);

        virtual void setSpeed(int32_t dutyCycleScl);

        virtual void setSpeed(float dutyCycle);

        virtual void setMaxSpeed(int32_t dutyCycleScl);

        virtual void setMaxSpeed(float dutyCycle);

        float readCurrent();

        virtual ~VNH2SP30();

    private:
        uint mInA;
        uint mInB;
        PwmController mPwm;
        uint mCs;
        uint mEn;
    };
}
#endif // VNH2SP30_HPP