/*
 * @Author: Redistuo 15869483306@163.com
 * @Date: 2026-10-04 10:31:12
 * @LastEditors: Redistuo 15869483306@163.com
 * @LastEditTime: 2026-10-04 14:42:12
 * @FilePath: \C_project\App\ControlTask.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H
#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

void ControlTaskInit(void);
void MainTask(void);
void CanFeedbackCallback(uint32_t std_id,const uint8_t *data);
void ControlTaskLoop(void);

#ifdef __cplusplus
}
#endif


#endif // CONTROL_TASK_H