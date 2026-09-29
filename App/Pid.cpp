
#include "Pid.h"

#include "RmConfig.h"

namespace rm {

Pid::Pid(float kp, float ki, float kd, float out_max, float out_min) {
  // TODO(B2-1)：把 5 个参数存进成员变量
  //   提示：积分 integ_ 和上一拍误差 last_err_ 已经在 Pid.h 里默认初始化为 0 了，
  //        所以这里只要存 5 个参数
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  out_max_ = out_max;
  out_min_ = out_min;
}

void Pid::setParams(float kp, float ki, float kd) {
  // TODO(B2-2)：在线改增益（注意：改参数**不要**清掉积分，否则调参时电机会一顿一顿）
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void Pid::setOutputLimit(float out_max, float out_min) {
  // TODO(B2-3)
  out_max_ = out_max;
  out_min_ = out_min;
}

void Pid::reset() {
  // TODO(B2-4)：把"记忆"清零（积分累加、上一拍误差）
  integ_ = 0.0f;
  last_err_ = 0.0f;
}

float Pid::calc(float ref, float fdb) {
  // TODO(B2-5)：五步
  //   1) err   = ref - fdb
  //   2) deriv = err - last_err_                       （这一拍误差变化了多少）
  //   3) integ_ += err                                 （把误差累加进"账本"）
  //   4) out = kp_*err
  //          + ki_*integ_*kSampleTime                   （累加值 × 周期 = 真正的时间积分）
  //          + kd_*deriv/kSampleTime                    （变化量 ÷ 周期 = 变化率）
  //   5) 限幅 + 积分回退：
  //        如果 out > out_max_  → out = out_max_，并且 integ_ -= err（撤销本拍累加）
  //        如果 out < out_min_  → out = out_min_，并且 integ_ -= err
  //   最后：last_err_ = err;  return out;

  const float err = ref - fdb;
  const float deriv = err - last_err_;
  integ_ += err;
  float out = kp_* err + ki_ * integ_ * kSampleTime + kd_*deriv/kSampleTime;
  if(out>out_max_){
    out = out_max_;
    integ_ -= err;
  }else if(out<out_min_){
    out = out_min_;
    integ_ -= err;
  }
  last_err_ =err;
  return out;
}

}  // namespace rm
