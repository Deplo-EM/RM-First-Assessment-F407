
#include "SafetyGuard.h"

#include "RmConfig.h"

namespace rm {

void SafetyGuard::reset() {
  started_ = false;
  ticks_since_feedback_ = 0;
  temp_c_ = 0.0f;
  over_temp_ = false;
}

void SafetyGuard::onFeedback() {
  started_ = true;
  ticks_since_feedback_ = 0;
}

void SafetyGuard::tick(float temp_c) {

  temp_c_ = temp_c;
  ticks_since_feedback_++;
  if (temp_c_  > kTempLimitC){
    over_temp_ = true;
  } else if (temp_c_ < kTempLimitC -kTempHysteresisC){
    over_temp_ = false;
  }
}

bool SafetyGuard::feedbackOk() const {
  if(started_ && ticks_since_feedback_ <= kFeedbackTimeoutMs){
    return true;
  }
  return false;
}

bool SafetyGuard::tempOk() const {
  // TODO(B4-5)
  if (!over_temp_){
    return true;
  }
  return false;
}

bool SafetyGuard::allowed() const {
  if (feedbackOk() && tempOk()){
    return true;
  }
  return false;
}

float SafetyGuard::limit(float u) const {
  if (!allowed()) return 0.0f;
  if (u > kSafeVoltageMax){
    u = kSafeVoltageMax;
  }else if(u < -kSafeVoltageMax){
    u = -kSafeVoltageMax;
  }
  
  return u;
}

}  // namespace rm
