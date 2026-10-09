// ============================================================================
//  Pid.h —— 位置式 PID 控制器（B2 关）
//
//  离散公式（T = 控制周期 = kSampleTime = 0.001 s）：
//
//      e(k) = ref - fdb
//      u(k) = Kp*e(k) + Ki*Σe(j)*T + Kd*(e(k) - e(k-1))/T
//
//  必须内置两件事（少一个，电机在现场就会出问题）：
//    1) 输出限幅：输出夹在 [out_min, out_max] 之间
//    2) 积分回退（抗积分饱和）：输出已经顶到限幅时，撤销本拍的积分累加
//
//  接口与老师给的 ref\Pid.h 保持一致（方便对照学习，也方便以后接 ControlTask）
// ============================================================================

#pragma once

namespace rm {

class Pid {
 public:
  // kp, ki, kd: 三个增益；out_max, out_min: 输出上下限（如 ±3000 起步）
  Pid(float kp, float ki, float kd, float out_max, float out_min);
  ~Pid() = default;

  void setParams(float kp, float ki, float kd);
  void setOutputLimit(float out_max, float out_min);

  // 清掉"记忆"：积分累加、上一拍误差
  void reset();

  // 计算一次控制量：ref 目标值，fdb 反馈值，返回**限幅后**的输出
  float calc(float ref, float fdb);

  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }

 private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float out_max_ = 0.0f;
  float out_min_ = 0.0f;

  float integ_ = 0.0f;     // Σe(j)：误差**累加**（注意：这里存的是误差之和，还没乘 T）
  float last_err_ = 0.0f;  // e(k-1)：上一拍的误差（算微分项要用）
};

}  // namespace rm
