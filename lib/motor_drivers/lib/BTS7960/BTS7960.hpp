#ifndef BTS7960_HPP
#define BTS7960_HPP

#include "IMotor.hpp"
#include "PwmController.hpp"

namespace motor {

    class BTS7960 : public IMotor {

    public:

        enum {
            UnsetPin = 100,
        };

        enum Alarm {
            noAlarm = 0,
            rIsAlarm = 1,
            lIsAlarm = 2,
            bothAlarm = 3
        };

        BTS7960(uint lpwmPin, uint rpwmPin, uint32_t frequency = 10000, uint lis = UnsetPin, uint ris = UnsetPin);

        virtual void setSpeed(int32_t dutyCycleScl);

        virtual void setSpeed(float dutyCycle);

        virtual void setMaxSpeed(int32_t dutyCycleScl);

        virtual void setMaxSpeed(float dutyCycle);

        Alarm checkAlarm();

        virtual ~BTS7960();

    private:
        PwmController mLpwm;
        PwmController mRpwm;

        uint mLis;
        uint mRis;
    };
}
#endif // BTS7960_HPP