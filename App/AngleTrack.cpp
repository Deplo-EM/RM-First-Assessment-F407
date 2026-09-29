

#include "AngleTrack.h"

#include "RmConfig.h"

namespace rm {

void AngleTrack::reset() {

  started_ = false;
  last_raw_ = 0 ;
  total_ticks_ = 0; 
}

void AngleTrack::update(uint16_t raw) {

  if (!started_){
    started_ = true;
    last_raw_ = raw;
    return ;
  }

  int32_t delta = raw - last_raw_;
  if (delta > kHalfTurnRaw){
    delta -= kRawPerTurn;
  }else if(delta < -kHalfTurnRaw){
    delta += kRawPerTurn;
  }

  total_ticks_ += delta;
  last_raw_ = raw;
}

float AngleTrack::rad() const {

  return static_cast<float>(total_ticks_) * kRawToRad;
}

float AngleTrack::turns() const {

  return static_cast<float>(total_ticks_) / kRawPerTurn;
}

int32_t AngleTrack::totalTicks() const {

  return total_ticks_;
}

}  // namespace rm
