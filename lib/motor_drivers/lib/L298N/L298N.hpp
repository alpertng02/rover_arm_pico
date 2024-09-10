#ifndef L298N_HPP
#define L298N_HPP

#include "IMotor.hpp"
#include "PwmController.hpp"

namespace motor {

    class L298N : public IMotor {

    public:


        L298N(uint in1, uint in2, uint en = UnsetPin, uint32_t frequency = 2000);

        virtual void setSpeed(int32_t dutyCycleScl);

        virtual void setSpeed(float dutyCycle);

        virtual void setMaxSpeed(int32_t dutyCycleScl);

        virtual void setMaxSpeed(float dutyCycle);
        
        virtual ~L298N();

    private:
        PwmController mIn1;
        PwmController mIn2;
        uint mEn;
    };
}
#endif // L298N_HPP