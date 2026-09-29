/*
 * @Author: Redistuo 15869483306@163.com
 * @Date: 2026-09-28 22:11:31
 * @LastEditors: Redistuo 15869483306@163.com
 * @LastEditTime: 2026-09-28 22:46:11
 * @FilePath: \undefinedd:\ForTest\C_project\App\WatchdogPolicy.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "WatchdogPolicy.h"

#include "RmConfig.h"

namespace rm {

void WatchdogPolicy::reset() {
  
  last_control_ms_ = 0;
  last_feedback_ms_ = 0;
  last_loop_ms_ = 0;
  started_control_ = false;
  started_feedback_ = false;
  started_loop_ = false;
}

void WatchdogPolicy::onControlTick(uint32_t now_ms) {
  
  last_control_ms_ = now_ms;
  started_control_ = true;
}

void WatchdogPolicy::onFeedback(uint32_t now_ms) {

  last_feedback_ms_ = now_ms;
  started_feedback_ = true;
}

void WatchdogPolicy::onMainLoop(uint32_t now_ms) {
  
  last_loop_ms_ = now_ms;
  started_loop_ = true;
}

bool WatchdogPolicy::controlOk(uint32_t now_ms) const {

  return started_control_ && (now_ms - last_control_ms_) <= kControlTimeoutMs;

}

bool WatchdogPolicy::feedbackOk(uint32_t now_ms) const {
  return started_feedback_ && (now_ms - last_feedback_ms_) <= kFeedbackTimeoutMs;
}

bool WatchdogPolicy::loopOk(uint32_t now_ms) const {
return started_loop_ && (now_ms - last_loop_ms_) <= kLoopTimeoutMs;
}

bool WatchdogPolicy::shouldFeed(uint32_t now_ms) const {
  if (feedbackOk(now_ms) && loopOk(now_ms) && controlOk(now_ms) ) 
    return true;
  return false;
}

}  // namespace rm
