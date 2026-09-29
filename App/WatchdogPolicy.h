// ============================================================================
//  WatchdogPolicy.h —— 看门狗喂狗策略（B5 关）
//
//  【什么是看门狗】
//      IWDG 是一个"倒计时炸弹"：约 0.8 秒内不去"喂"它，它就把单片机复位。
//      CubeMX 里已经默认开启了（Prescaler 64 / Reload 400，LSI≈32 kHz → 约 0.8 s）。
//      作用：**程序卡死时自动重启**。
//
//  【关键问题：什么时候喂？】
//      最糟的写法：在 1 ms 定时器中断里**无脑喂**。
//      那样只要中断还在跑，狗就一直被喂 —— 哪怕主循环已经死了、
//      CAN 早断了、电机早失联了。**保护形同虚设。**
//
//      正确写法：**条件喂狗** —— 只有"整个系统都健康"时才喂：
//        ① 1 kHz 控制任务：最近 5 ms 内跑过
//        ② CAN 反馈      ：最近 20 ms 内收到过
//        ③ 主循环        ：最近 200 ms 内跑过
//      任一环节卡死 → 停止喂狗 → 0.8 秒后硬件自动复位。
//      （这就是考核里"使用看门狗进行保护"的正解）
//
//  【为什么用"时间戳"而不是数拍子？】
//      控制任务和反馈有固定节拍（1 ms），数拍子也行；
//      但**主循环没有固定周期**（有时 1 ms 一圈，有时 50 ms），
//      只能问"现在几点、上次是几点" → 所以统一用毫秒时间戳 now_ms。
//      实际工程里传 HAL_GetTick()。
//
//  【为什么每个信号都要配一个 started_ 标志？】
//      和 B4 一模一样的陷阱：上电时 last_*_ms_ 都是 0，
//      如果只看"现在 − 上次 = 0 ≤ 超时"，就会把"**从没跑过**"误判成"**刚刚跑过**"。
//
//  【时间戳回绕】
//      now_ms 是 uint32_t，49.7 天回绕一次。判断超时必须用**无符号减法**：
//          (now_ms - last_ms) <= timeout
//      无符号减法在回绕处天然正确；用有符号比较、或先算绝对值，跨回绕就错。
// ============================================================================

#pragma once

#include <cstdint>

namespace rm {

class WatchdogPolicy {
 public:
  // 上电/重新开始：回到"什么都没跑过"的状态
  void reset();

  // 三个"我还在干活"的报到点（各自记下时间戳并置起对应的标志）
  void onControlTick(uint32_t now_ms);  // 1 kHz 控制中断末尾
  void onFeedback(uint32_t now_ms);     // CAN 接收中断里
  void onMainLoop(uint32_t now_ms);     // 主循环每轮

  // 三个信号各自是否健康
  bool controlOk(uint32_t now_ms) const;
  bool feedbackOk(uint32_t now_ms) const;
  bool loopOk(uint32_t now_ms) const;

  // 现在可以喂狗吗？（三个都健康才喂）
  bool shouldFeed(uint32_t now_ms) const;

 private:
  uint32_t last_control_ms_  = 0;
  uint32_t last_feedback_ms_ = 0;
  uint32_t last_loop_ms_     = 0;
  bool     started_control_  = false;
  bool     started_feedback_ = false;
  bool     started_loop_     = false;
};

}  // namespace rm
