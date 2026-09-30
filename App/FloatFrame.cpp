/*
 * @Author: Redistuo 15869483306@163.com
 * @Date: 2026-09-29 19:22:16
 * @LastEditors: Redistuo 15869483306@163.com
 * @LastEditTime: 2026-09-30 19:41:35
 * @FilePath: \undefinedd:\ForTest\C_project\App\FloatFrame.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// ============================================================================
//  FloatFrame.cpp —— 串口波形帧编码（B6 关，你来实现）
//
//  只填 TODO，不要改 FloatFrame.h。
//  规则和前面几关一样：判题器只检查**行为**，不检查你怎么写。
//
//  实现完自己先想清楚这三件事（说明文档里要写）：
//    1) 为什么帧尾是 4 个字节，而不能"发完就算"？
//    2) 为什么 push 必须显式按小端写字节？
//    3) 为什么 NaN / Inf 必须被拒绝（而不是原样发出去）？
// ============================================================================

#include "FloatFrame.h"

#include <cstring>  // memcpy（取浮点数的二进制位时必须用它，不能强转指针）

namespace rm {
namespace {
constexpr uint8_t kFrameTail[4] = {0x00, 0x00, 0x80, 0x7F};
} 
// ---------------------------------------------------------------------------
//  reset()：把这一帧清空
//
//  TODO(B6-1)  应该做两件事：
//                ① count_ 归 0
//                ② 往 buf_ 的开头写入帧尾 4 个字节： 00 00 80 7F
//              （清空后 size() 应该是 4，count() 是 0 —— 一帧"只有帧尾"也是合法帧）
//              提示：帧尾可以写成一个 constexpr uint8_t 数组再来 memcpy。
// ---------------------------------------------------------------------------

void FloatFrame::reset() {
  count_ = 0;
  constexpr uint8_t kFrameTail[4] = {0x00, 0x00, 0x80, 0x7F};
  memcpy(buf_, kFrameTail, 4);
}

// ---------------------------------------------------------------------------
//  push()：追加一个 float
//
//  TODO(B6-2)  按这个顺序写（顺序很重要，先判断再动手）：
//                ① 若是 NaN 或 ±Inf → 直接 return false（绝不能写进缓冲区）
//                   判断办法：把 float 用 memcpy 取成 uint32_t，
//                   看它的"指数位"（第 23~30 位）是不是全 1（0xFF）
//                   全 1 = NaN 或 Inf；否则是正常数
//                ② 若已装满（count_ >= kMaxFloats）→ return false
//                ③ 把 v 的 4 个字节**按小端**写进 buf_[count_ * 4]（低位字节在前）
//                ④ ++count_
//                ⑤ 在新的数据末尾**重新写一遍帧尾**（保持"帧尾紧跟数据"的不变量）
//                ⑥ return true
//
//              ⚠️ 要求：被拒绝时缓冲区内容一个字节都不能变。
// ---------------------------------------------------------------------------
bool FloatFrame::push(float v) {
  uint32_t bits;
  memcpy(&bits, &v, sizeof(bits));
  if ((bits & 0x7F800000) == 0x7F800000) {
    return false;
  }
  if (count_ >= kMaxFloats) {
    return false;
  }
  // bits 里已经是这个 float 的 32 位（上一行 memcpy 取出来的）
// 按小端写入：低位字节放在低地址
buf_[count_ * 4 + 0] = static_cast<uint8_t>(bits & 0xFFu);           // 最低 8 位
buf_[count_ * 4 + 1] = static_cast<uint8_t>((bits >> 8) & 0xFFu);    // 次低 8 位
buf_[count_ * 4 + 2] = static_cast<uint8_t>((bits >> 16) & 0xFFu);   // 第 3 个 8 位
buf_[count_ * 4 + 3] = static_cast<uint8_t>((bits >> 24) & 0xFFu);   // 最高 8 位
  ++count_;
  constexpr uint8_t kFrameTail[4] = {0x00, 0x00, 0x80, 0x7F};
  memcpy(buf_ + count_ * 4, kFrameTail, 4);
  return true;
}

// ---------------------------------------------------------------------------
//  count() / size() / empty() / data()
//
//  TODO(B6-3)  这四个是"只读查询"，一句话就能写完：
//                count()  → 已写入的 float 个数
//                size()   → 总字节数 = count * 4 + 帧尾 4
//                empty()  → 一个都没写吗
//                data()   → 返回缓冲区首地址
// ---------------------------------------------------------------------------
uint16_t FloatFrame::count() const {
  return count_;
}

uint16_t FloatFrame::size() const {
  return count_ * 4 + kTailSize;
}

bool FloatFrame::empty() const {
  return count_ == 0;
}

const uint8_t* FloatFrame::data() const {
  return buf_;
}

}  // namespace rm
