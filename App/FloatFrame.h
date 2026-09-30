// ============================================================================
//  FloatFrame.h —— 串口波形帧编码（B6 关）
//
//  【为什么需要这个模块】
//      考核要交"正弦跟随"的波形证据。抓波形有两条路：
//        ① Ozone / J-Link —— 依赖 SEGGER 的调试器（克隆的 J-Link 有被拒的风险）
//        ② 串口 + 上位机  —— 只要一根 USB 转串口线，**完全不依赖 SEGGER**
//      这个模块负责第 ②  条路的"编码"部分：把几个浮点数打包成一帧字节，
//      丢给串口发出去，上位机（VOFA+）就能直接画成曲线。
//
//  【协议：VOFA+ 的 JustFloat】
//      一帧的布局（小端字节序）：
//
//          [float0 4字节][float1 4字节]...[floatN-1 4字节][帧尾 4字节]
//                                                        00 00 80 7F
//
//      帧尾 00 00 80 7F 小端解读就是 0x7F800000 = IEEE754 的 **+Infinity**。
//      协议选它当帧尾，是因为"正常数据不该出现无穷大" —— 上位机只要
//      "一直读到 00 00 80 7F 就切一刀"，就得到一帧 N 个浮点数。
//
//  【本模块的两个工程陷阱】
//      ① NaN / +Inf / -Inf 绝不允许进入数据区
//         否则它们内部就含有 0x7F800000 这个位模式 → 上位机**提前切帧** →
//         通道数错乱、波形全乱。所以 push() 必须**拒绝**它们。
//      ② 字节序必须**显式**写成小端
//         虽然 STM32（ARM）和 PC 都是小端，直接 memcpy 也能对，
//         但显式写字节才是"协议实现"，换个平台/换个人看代码都不会误解。
//
//  【不变量（invariant）—— 实现时时刻记住】
//      帧尾永远紧跟在已写入的数据后面，一个字节都不留空：
//          size() == count() * 4 + 4
//          data()[count()*4 ... count()*4+3] == 00 00 80 7F
//      好处：调用方 push 完直接 read(data(), size()) 就能发出去，不需要
//      再调一个"收尾"函数 —— 任何时候这一帧都是完整可解析的。
//
//  【为什么用 memcpy 取位，而不是 (uint32_t*)&v 强转】
//      强转指针会违反严格别名规则（strict aliasing），且若地址未对齐
//      在某些平台上直接崩 —— 属于未定义行为。取浮点数的二进制位，
//      标准做法就是 memcpy 到一个 uint32_t。
//
//  【和考核的关系】
//      D 阶段（基地上机）时，控制任务每 1 ms 把 (目标值, 反馈, 输出)
//      三个 float 打包一帧发给电脑，VOFA+ 上就能看到正弦跟随曲线。
// ============================================================================

#pragma once

#include <cstdint>

namespace rm {

class FloatFrame {
 public:
  // 缓冲区大小（字节）。够放 3 个通道 + 帧尾，还留了余量。
  static constexpr uint16_t kCapacity = 64;
  // 帧尾长度（字节）
  static constexpr uint16_t kTailSize = 4;
  // 一帧最多能装几个 float： (64 - 4) / 4 = 15
  static constexpr uint16_t kMaxFloats = (kCapacity - kTailSize) / 4;

  // 清空这一帧：count 归 0，缓冲区里只留下帧尾
  void reset();

  // 追加一个 float。成功返回 true；被拒绝（NaN/Inf、或已经装满）返回 false
  // 被拒绝时 **不得改动任何已有内容**
  bool push(float v);

  // 已经写入的 float 个数
  uint16_t count() const;

  // 当前这一帧的总字节数（含 4 字节帧尾）= count() * 4 + 4
  uint16_t size() const;

  // 一个 float 都没写
  bool empty() const;

  // 帧首地址（发给串口时用）
  const uint8_t* data() const;

 private:
  uint8_t  buf_[kCapacity] = {};
  uint16_t count_          = 0;
};

}  // namespace rm
