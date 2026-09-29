// ============================================================================
//  SineRef.h —— 正弦目标发生器（B1 关）
//
//  这一关只干一件事：按固定节拍，产出一条正弦曲线上的值，当作电机的目标。
//
//  【重要设计约定】这个文件里不许出现任何 STM32 / HAL 相关的东西。
//  原因：只有"不碰硬件"的代码，才能在电脑上单独测试。
//  这条约定叫"算法与硬件解耦"，是嵌入式里最重要的一条工程习惯。
// ============================================================================

#pragma once

#include <cstdint>

#include "RmConfig.h"  // kSampleTime / kTwoPi 在这里定义（全工程共用一份）

namespace rm {

// 正弦目标发生器
//
// 用法：
//   rm::SineRef ref;
//   ref.configure(2.0f, 0.2f, 0.0f);   // 幅值 2、频率 0.2 Hz、初相位 0
//   ...每个控制周期调用一次...
//   float target = ref.step();
//
class SineRef {
 public:
  // 设定/修改曲线参数（运行中随时可改，用于在线调参）
  void configure(float amplitude, float freq_hz, float phase_rad);

  // 推进一个控制周期（1 ms），返回【推进之后】那个时刻的目标值。
  // 也就是说：reset() 之后第 n 次调用 step()，返回的是 t = n 毫秒 时刻的值。
  float step();

  // 时间归零（ticks_ = 0），并让 value() 回到 0
  void reset();

  // 最近一次 step() 算出的值（不推进时间）
  float value() const { return value_; }

  // 已经推进了多少个控制周期（从上次 reset() 算起）
  uint32_t ticks() const { return ticks_; }

 private:
  float amplitude_ = 0.0f;   // 幅值 A
  float freq_hz_ = 0.0f;     // 频率 f（Hz）
  float phase_rad_ = 0.0f;   // 初相位 φ（弧度）
  uint32_t ticks_ = 0;       // 已经走过多少个 1 ms
  float value_ = 0.0f;       // 最近一次算出的值
};

}  // namespace rm
