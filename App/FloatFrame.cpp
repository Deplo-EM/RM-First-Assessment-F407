/*
 * @Author: Redistuo 15869483306@163.com
 * @Date: 2026-09-29 19:22:16
 * @LastEditors: Redistuo 15869483306@163.com
 * @LastEditTime: 2026-09-30 19:41:35
 * @FilePath: \C_project\App\FloatFrame.cpp
 * @Description: B6 串口波形帧（VOFA+ JustFloat）：把若干 float 打包成
 *               "小端字节序 + 4 字节帧尾 0x7F800000" 的一帧，交给串口发送。
 */
// ============================================================================
//  FloatFrame.cpp —— 串口波形帧编码（B6 关）
//
//  协议（小端）：[float0 4B][float1 4B]...[floatN-1 4B][帧尾 00 00 80 7F]
//  帧尾 = 0x7F800000 = IEEE754 +Infinity；正常物理量取不到它 → 可安全当分隔符。
//
//  实现要点：
//    1) reset() 只做两件事：count_ 归 0 + 在开头写入帧尾。
//       不必清空整个缓冲区 —— size() 已经界定了有效长度，后面的旧字节不会被读/被发。
//    2) push() 必须**先检查、后动手**：NaN/±Inf 与"已装满"都在写入之前拒绝，
//       保证被拒绝时缓冲区一个字节都不变。
//    3) 字节序**显式**按小端写，不依赖"平台恰好是小端"这个巧合。
// ============================================================================

#include "FloatFrame.h"

#include <cstring>  // std::memcpy（取浮点数的二进制位必须用它，不能强转指针）

namespace rm {
namespace {

// 帧尾：0x7F800000（+Inf）的小端字节序，长度 = FloatFrame::kTailSize
constexpr uint8_t kFrameTail[4] = {0x00, 0x00, 0x80, 0x7F};

}  // namespace

// ---------------------------------------------------------------------------
//  reset()：把这一帧清空
//    ① count_ 归 0
//    ② 往缓冲区开头写入帧尾 → 清空后 size() == 4、count() == 0
//       （"只有帧尾"也是一帧合法数据，上位机解析出 0 个通道）
// ---------------------------------------------------------------------------
void FloatFrame::reset() {
  count_ = 0;
  std::memcpy(buf_, kFrameTail, kTailSize);
}

// ---------------------------------------------------------------------------
//  push()：追加一个 float
//
//  顺序不能颠倒（先检查、后动手）：
//    ① NaN 或 ±Inf   → return false（绝不能写进缓冲区）
//    ② 已经装满      → return false
//    ③ 按小端写入 4 个字节（低位字节在前）
//    ④ ++count_
//    ⑤ 在新的数据末尾重写帧尾（维持不变量 size() == count()*4 + 4）
//    ⑥ return true
//
//  ⚠️ 被拒绝时缓冲区内容一个字节都不能变。
// ---------------------------------------------------------------------------
bool FloatFrame::push(float v) {
  uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));  // 取二进制位（不是数值转换）

  // ① 指数位全 1 = NaN 或 ±Inf。
  //    掩码 0x7F800000 只保留指数位（屏蔽符号位）→ 正负 Inf 与所有 NaN 一次全拦。
  //    为什么必须拒绝：+Inf 的位模式**恰好等于帧尾**，混进数据区会让上位机提前切帧。
  if ((bits & 0x7F800000u) == 0x7F800000u) {
    return false;
  }

  // ② 装满就拒绝（拒绝时不改动任何字节）
  if (count_ >= kMaxFloats) {
    return false;
  }

  // ③ 显式小端写入：第 k 个字节 = 从最低位起的第 k 个 8 位
  const uint16_t base = static_cast<uint16_t>(count_ * 4);
  buf_[base + 0] = static_cast<uint8_t>(bits & 0xFFu);          // 最低 8 位 → 最低地址
  buf_[base + 1] = static_cast<uint8_t>((bits >> 8) & 0xFFu);   // 次低 8 位
  buf_[base + 2] = static_cast<uint8_t>((bits >> 16) & 0xFFu);  // 第 3 个 8 位
  buf_[base + 3] = static_cast<uint8_t>((bits >> 24) & 0xFFu);  // 最高 8 位

  // ④ 计数 +1
  ++count_;

  // ⑤ 帧尾跟着数据走，永远贴在最后
  std::memcpy(buf_ + count_ * 4, kFrameTail, kTailSize);

  // ⑥
  return true;
}

// ---------------------------------------------------------------------------
//  只读查询
// ---------------------------------------------------------------------------
uint16_t FloatFrame::count() const {
  return count_;
}

uint16_t FloatFrame::size() const {
  return static_cast<uint16_t>(count_ * 4 + kTailSize);
}

bool FloatFrame::empty() const {
  return count_ == 0;
}

const uint8_t* FloatFrame::data() const {
  return buf_;
}

}  // namespace rm
