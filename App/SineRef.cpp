

#include "SineRef.h"

#include <cmath>

namespace rm {

void SineRef::configure(float amplitude, float freq_hz, float phase_rad) {
  amplitude_=amplitude;
  freq_hz_=freq_hz;
  phase_rad_=phase_rad;
}

float SineRef::step() {
  ++ticks_;
  const float t = ticks_ * kSampleTime ;
  value_=amplitude_ * sinf(kTwoPi*freq_hz_* t + phase_rad_);
  return value_;
}

void SineRef::reset() {
  ticks_=0;
  value_=0.0f;
}

}  
