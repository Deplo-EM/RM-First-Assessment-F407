// ============================================================================
//  ControlTask.cpp —— 把"外设"和"算法"接起来的那一层
//
//  这个文件是唯一允许出现 HAL 代码的地方（别的 App 模块都不许碰硬件）。
//  为什么要用 #if defined(USE_HAL_DRIVER)？
//    · 板子工程（F103 彩排 / F407 基地）：CubeMX 定义了这个宏 → 编"真身"
//    · 电脑判题器（B 阶段 145 项测试）：不定义这个宏 → 编"空壳"
// ============================================================================

#include "ControlTask.h"

#if defined(USE_HAL_DRIVER)     // ================= 板子世界 =================

// ---- 板子世界要用的"遥控器"（句柄）都声明在这些头文件里 ----
#include "can.h"        // hcan1
#include "tim.h"        // htim6
#include "iwdg.h"       // hiwdg
#include "usart.h"      // huart1
#include "main.h"       // Error_Handler()

// ---- 算法模块（都不碰硬件，电脑上也能编）----
#include "SineRef.h"
#include "Pid.h"
#include "AngleTrack.h"
#include "SafetyGuard.h"
#include "WatchdogPolicy.h"
#include "FloatFrame.h"
#include "RmConfig.h"

// ===========================================================================
//  一、模块对象：全文件共用，所以放在最外层（static = 只在本文件可见）
// ===========================================================================
static rm::SineRef        g_ref_speed;      // 速度环的正弦目标（rpm）
static rm::SineRef        g_ref_pos;        // 位置环的正弦目标（rad）
static rm::Pid            g_pid_speed(10.0f, 0.2f, 0.0f, 3000.0f, -3000.0f);
static rm::Pid            g_pid_pos(15.0f, 0.0f, 1.0f, 3000.0f, -3000.0f);
static rm::AngleTrack     g_angle;          // 多圈角度累计（只在 CAN 中断里被调用）
static rm::SafetyGuard    g_guard;          // 安全护栏
static rm::WatchdogPolicy g_wdog;           // 喂狗策略
static rm::FloatFrame     g_wave;           // 串口波形帧

// ===========================================================================
//  二、中断 ↔ 主循环 之间共享的数据（必须 volatile）
// ===========================================================================
static volatile float   g_fb_speed_rpm = 0.0f;   // 最近一次转速反馈（rpm）
static volatile float   g_fb_pos_rad   = 0.0f;   // 最近一次累计角度（rad）
static volatile float   g_fb_temp_c    = 0.0f;   // 最近一次温度（℃）
static volatile uint8_t g_mode         = 0;      // 0 = 速度环，1 = 位置环

// ===========================================================================
//  三、辅助函数
// ===========================================================================

// 配 CAN 过滤器：只把电机反馈帧放行给 CPU
// 规则：高 7 位必须等于 0100000、低 4 位随便 → 放行 0x200 ~ 0x20F
static void ConfigureCanFilter(void) {
    CAN_FilterTypeDef f = {0};

    f.FilterBank           = 0;                        // 0 号过滤器组
    f.FilterMode           = CAN_FILTERMODE_IDMASK;    // 掩码模式
    f.FilterScale          = CAN_FILTERSCALE_32BIT;    // 32 位
    f.FilterIdHigh         = (uint16_t)(0x200 << 5);   // 目标 ID（HAL 要求放在高 11 位）
    f.FilterIdLow          = 0;
    f.FilterMaskIdHigh     = (uint16_t)(0x7F0 << 5);   // 这 7 位必须相同
    f.FilterMaskIdLow      = 0;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;             // 放行的帧进 0 号筐
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;                       // F4 双 CAN：0~13 给 CAN1

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) {
        Error_Handler();
    }
}

// 把一路电压给定装进 0x1FF 帧发给 1 号电机
static void SendVoltage(float u) {
    if (u >  25000.0f) u =  25000.0f;              // 协议上限（双保险）
    if (u < -25000.0f) u = -25000.0f;

    const int16_t v = (int16_t)u;                  // 转成 16 位有符号整数
    uint8_t data[8] = {0};                         // 其余 3 个电机补 0（= 不给力，安全）
    data[0] = (uint8_t)(v & 0xFF);                 // 小端：低字节在前
    data[1] = (uint8_t)((v >> 8) & 0xFF);          //       高字节在后

    CAN_TxHeaderTypeDef tx = {0};
    tx.StdId = 0x1FF;                              // 1~4 号电机的控制帧
    tx.IDE   = CAN_ID_STD;                         // 标准帧
    tx.RTR   = CAN_RTR_DATA;                       // 数据帧
    tx.DLC   = 8;                                  // 8 字节

    uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox) != HAL_OK) {
        // 发送邮箱只有 3 个，总线忙时会 HAL_BUSY → 本拍不发，下一拍再发
        // 偶发丢一帧无害，所以这里**不**调用 Error_Handler
    }
}

// 把（目标, 反馈, 输出）打包成一帧发给电脑（VOFA+ 画曲线）
static void SendWave(float ref, float fdb, float u) {
    g_wave.reset();
    if (!g_wave.push(ref) || !g_wave.push(fdb) || !g_wave.push(u)) {
        return;                                    // 有 NaN/Inf 就整帧不发（B6 里验过的坑）
    }
    if (HAL_UART_Transmit(&huart1, g_wave.data(), g_wave.size(), 10) != HAL_OK) {
        // 串口忙/超时 → 丢这一帧波形即可（波形是给人看的，不影响控制）
    }
}

// ===========================================================================
//  四、对外接口（main.c 与中断会调用它们）
// ===========================================================================

// --- ① 上电初始化：main.c 里所有 MX_XXX_Init() 之后调用一次 ---
void ControlTaskInit(void) {
    ConfigureCanFilter();                              // 1. 先定 CAN 分拣规则

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {             // 2. 启动 CAN
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();                               // 3. 装门铃：0 号筐有帧就中断
    }

    g_ref_speed.configure(200.0f, 0.2f, 0.0f);         // 速度正弦：幅值 200 rpm、0.2 Hz
    g_ref_pos.configure(1.0f, 0.1f, 0.0f);             // 位置正弦：幅值 1.0 rad、0.1 Hz

    g_guard.reset();
    g_wdog.reset();

    HAL_TIM_Base_Start_IT(&htim6);                     // 4. 启动 1 kHz 节拍（漏了它控制逻辑永不执行）
}

// --- ② 1 kHz 控制主流程 ---
void MainTask(void) {
    const uint32_t now = HAL_GetTick();                // 当前毫秒时间戳

    g_wdog.onControlTick(now);                         // 报到①：控制任务还活着
    g_guard.tick(g_fb_temp_c);                         // 更新"反馈多久没来" + 温度判断

    float ref = 0.0f;
    float fdb = 0.0f;
    float u   = 0.0f;

    if (g_mode == 0) {                                 // 速度环
        ref = g_ref_speed.step();                      // 目标转速 rpm
        fdb = g_fb_speed_rpm;                          // 反馈转速 rpm
        u   = g_pid_speed.calc(ref, fdb);
    } else {                                           // 位置环
        ref = g_ref_pos.step();                        // 目标角度 rad
        fdb = g_fb_pos_rad;                            // 反馈角度 rad
        u   = g_pid_pos.calc(ref, fdb);
    }

    u = g_guard.limit(u);                              // 安全护栏：不允许就归零，否则夹进安全范围

    SendVoltage(u);                                    // 发给电机

    static uint32_t div = 0;                           // 波形降频：每 10 拍发一次（100 Hz）
    if (++div >= 10) {
        div = 0;
        SendWave(ref, fdb, u);
    }

    if (g_wdog.shouldFeed(now)) {                      // 条件喂狗：三路都健康才喂
        HAL_IWDG_Refresh(&hiwdg);
    }
}

// --- ③ CAN 接收回调（由下面的 HAL 弱回调转发进来）---
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    (void)std_id;                                      // 单电机版本先不用 ID（多电机时按 ID 分发）

    const uint16_t raw  = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));  // 机械角度 0~8191
    const int16_t  rpm  = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));   // 转速
    const uint8_t  temp = data[6];                                                   // 温度 ℃

    g_angle.update(raw);                               // 多圈角度累计（跨圈处理在 B3 验过）
    g_fb_speed_rpm = (float)rpm;
    g_fb_pos_rad   = g_angle.rad();
    g_fb_temp_c    = (float)temp;

    g_guard.onFeedback();                              // 告诉安全护栏"反馈来了"
    g_wdog.onFeedback(HAL_GetTick());                  // 报到②：CAN 这条路还活着

}

// --- ④ 主循环心跳 ---
void ControlTaskLoop(void) {
    g_wdog.onMainLoop(HAL_GetTick());                  // 报到③：主循环还活着
}

// ===========================================================================
//  五、覆盖 HAL 的"弱回调"：中断最终会调到这里
//     extern "C" 不能少 —— HAL 是 C 代码，它按原始名字找这两个函数
// ===========================================================================
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {                      // 别的定时器的回调也走这个函数，要筛
        MainTask();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {   // 把筐取空！
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) {
            break;
        }
        if (rx.IDE != CAN_ID_STD) {                    // 只认标准帧
            continue;
        }
        if (rx.StdId < 0x205 || rx.StdId > 0x20B) {     // 软件再筛一次（掩码会多收几个）
            continue;
        }
        CanFeedbackCallback(rx.StdId, data);
    }
}

#else                           // ================= 电脑世界 =================

// 空壳：名字、参数、返回值都跟真身一样，但什么都不做。
// 电脑上其实没有任何人调用它们，留着只是为了让"同一份源码在哪都能编过"。
void ControlTaskInit(void) {}
void MainTask(void) {}
void CanFeedbackCallback(uint32_t, const uint8_t *) {}
void ControlTaskLoop(void) {}

#endif