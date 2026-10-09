// ============================================================================
//  RmConfig.h —— 全工程共用的常量（单一真相源）
//
//  为什么需要这个文件？
//  这些常量被好几个模块用到（SineRef、Pid、以后的 ControlTask...）。
//  如果每个头文件各写一遍，同一个名字在一个编译单元里就出现两次定义，
//  编译器会直接报"重定义"错误；就算能过，以后改的时候也容易漏改一处。
//
//  所以规矩是：**一个常量只在一个地方定义，别处一律 #include 它。**
// ============================================================================

#pragma once

#include <cstdint>

namespace rm {

// 控制周期：1 kHz，和 CubeMX 里 TIM6 的配置一致（Prescaler = 84-1, Period = 1000-1）
inline constexpr float kSampleTime = 0.001f;

// 2π：把"转了多少圈"换算成弧度（1 圈 = 2π 弧度）
inline constexpr float kTwoPi = 6.28318530718f;

// ---- GM6020 角度相关（B3 用）----
// 机械角度一圈 = 8192 个原始刻度（反馈帧 DATA[0..1] 给出 0 ~ 8191）
inline constexpr int32_t kRawPerTurn = 8192;

// 半圈刻度：判断"这次跳变到底是跨圈了，还是只是小位移"的阈值
inline constexpr int32_t kHalfTurnRaw = kRawPerTurn / 2;

// 原始刻度 → 弧度（2π / 8192）
inline constexpr float kRawToRad = kTwoPi / static_cast<float>(kRawPerTurn);

// ---- 安全保护（B4 用）----
// 反馈超时：电机每 1 ms 发一帧，超过这么久没收到就判定掉线 → 输出归零
inline constexpr uint32_t kFeedbackTimeoutMs = 20;

// 过温保护阈值（℃）：GM6020 反馈帧 DATA[6] 就是温度
inline constexpr float kTempLimitC = 70.0f;

// 迟滞（死区）：温度要降到 65℃ 以下才解除保护，避免在阈值附近反复开关
inline constexpr float kTempHysteresisC = 5.0f;

// 绝对电压上限（双保险）：就算 PID 的输出限幅被设错了，这里也会夹住
// 【为什么从 3000 改成 10000】协议满量程是 ±25000（≈满电压），
//   3000 只占 12%（约 3V）—— 连起步都不够，更追不上 60 rpm 的目标（现场实测确认过）。
//   10000 = 40%，够跟上 60 rpm，同时仍在"半程以内"，保留保守的安全余量。
inline constexpr float kSafeVoltageMax = 10000.0f;

// ---- 看门狗喂狗策略（B5 用）----
// 1 kHz 控制任务："多久没跑过就算卡死"
inline constexpr uint32_t kControlTimeoutMs = 5;

// 主循环："多久没跑过就算卡死"（主循环没有固定节拍，所以给得宽）
inline constexpr uint32_t kLoopTimeoutMs = 200;

}  // namespace rm
