#ifndef IMOTOR_HPP
#define IMOTOR_HPP

#include "pico/types.h"

namespace motor {

    class IMotor {

    public:

        virtual void setSpeed(int32_t dutyCycleScl) = 0;

        virtual void setSpeed(float dutyCycle) = 0;

        virtual void setMaxSpeed(int32_t dutyCycleScl) = 0;

        virtual void setMaxSpeed(float dutyCycle) = 0;

        virtual ~IMotor() = default;

        enum {
            UnsetPin = UINT16_MAX
        };
    };


}

#endif // IMOTOR_HPP