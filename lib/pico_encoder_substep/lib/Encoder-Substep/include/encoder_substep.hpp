#ifndef __ENCODER_SUBSTEP_HPP__
#define __ENCODER_SUBSTEP_HPP__

#include "encoder_substep.h"

namespace encoder {

class EncoderSubstep {
public:
  EncoderSubstep() = delete;
  EncoderSubstep(EncoderSubstep &encoder) = delete;

  EncoderSubstep(const PIO pio, const uint sm, const uint pinA, const uint stepsPerRev = 16);

  bool calibrate();

  uint getStepCount();

  int getSpeed();

  int getSpeed_2_20();

  float getRpm();

  void setStepsPerRev(uint stepsPerRev);
  
  struct Data {
    int speed_2_20;
    int speed;
    uint step;
  };

  Data getAll();

private:
  const uint m_pinA{};
  substep_state_t m_substepState{};
  uint m_stepsPerRev{};
};
} // namespace encoder

#endif // __ENCODER_SUBSTEP_HPP__