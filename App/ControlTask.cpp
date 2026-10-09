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
//  〇、编译开关（改这里就能切行为，不用翻遍代码）
// ===========================================================================

// 串口波形开关：
//   1 = 每 10 ms 通过 USART1 发一帧（目标 / 反馈 / 输出）给 VOFA+ 画曲线
//   0 = 完全不发 —— 走 Ozone 采样路线时用，这样 1 kHz 中断里没有任何串口阻塞
// 为什么需要这个开关？发一帧 16 字节要 0.6 ms（256000 波特率），
// 放在 1 kHz 中断里会让那一拍的耗时变长（控制节拍抖动）；不需要串口波形时关掉最干净。
#ifndef ENABLE_UART_WAVE
#define ENABLE_UART_WAVE 1
#endif

// ===========================================================================
//  一、模块对象：全文件共用，所以放在最外层（static = 只在本文件可见）
// ===========================================================================
static rm::SineRef        g_ref_speed;      // 速度环的正弦目标（rpm）
static rm::SineRef        g_ref_pos;        // 位置环的正弦目标（rad）

// ===========================================================================
//  一·补、现场调参开关（重新编译时用 -D 覆盖，例如 -DPID_KP_SPEED=300.0f）
// ===========================================================================
//  【这一组数字是什么】PID 的输出 = 发给电机的"电压给定值"，量程 ±25000（≈满电压）。
//    增益决定"每 1 rpm 误差 / 每 1 弧度误差，给多少输出"。
//  【2026-10-10 基地实测标定（GM6020 + 这块电池，空载）】
//    · 开环 1500 单位 → 约 15 rpm；8000 单位 → 约 50 rpm
//      → 大约"每 1000 单位 ≈ 6~10 rpm"（这就是这台电机的"脾气"）
//    · 速度环 kp 的稳定上限就在 60 附近：
//      kp=60 稳、kp=120 会让板子反复重启（环路失稳）、kp=150 直接爆转
//    · 只用比例(P)：实际值会"矮一截"（稳态误差），补上积分(I)才贴得紧
//    · 位置环只有 P 时推不动电机（齿槽力把它吸住）→ 表现为曲线"停—走—停"的方块波
//      解决靠积分(I)：积分会把过去的误差一直攒着，直到把电机顶过去
//  【怎么调】一次只改一个数 → 重新编译烧录 → 看曲线：
//    · 实际值矮一截、跟不上  → 加大 kp（或加 ki）
//    · 波峰附近出现尖刺/啸叫 → 减小 ki
//    · 越转越快 / 板子反复重启 → kp 太大，立刻减小
//    · 输出被削平成方波     → 是撞到 PID_VOLT_LIMIT 了，把上限调大
#ifndef PID_KP_SPEED
#define PID_KP_SPEED 60.0f             // 速度环比例（基地实测稳定值）
#endif
#ifndef PID_KI_SPEED
#define PID_KI_SPEED 150.0f            // 速度环积分（消除"矮一截"，再大波峰会起尖刺）
#endif
#ifndef PID_KP_POS
#define PID_KP_POS 6000.0f             // 位置环比例
#endif
#ifndef PID_KI_POS
#define PID_KI_POS 3000.0f             // 位置环积分（治"停—走—停"死区的关键）
#endif
#ifndef PID_KD_POS
#define PID_KD_POS 0.0f                // 位置环微分（角度量化噪声大，先不用）
#endif
#ifndef PID_VOLT_LIMIT
#define PID_VOLT_LIMIT 8000.0f         // PID 输出上限（满量程 25000 的 32%）；只是"天花板"
#endif

static rm::Pid            g_pid_speed(PID_KP_SPEED, PID_KI_SPEED, 0.0f, PID_VOLT_LIMIT, -PID_VOLT_LIMIT);
static rm::Pid            g_pid_pos(PID_KP_POS, PID_KI_POS, PID_KD_POS, PID_VOLT_LIMIT, -PID_VOLT_LIMIT);
static rm::AngleTrack     g_angle;          // 多圈角度累计（只在 CAN 中断里被调用）
static rm::SafetyGuard    g_guard;          // 安全护栏
static rm::WatchdogPolicy g_wdog;           // 喂狗策略
#if ENABLE_UART_WAVE
static rm::FloatFrame     g_wave;           // 串口波形帧（只在 ENABLE_UART_WAVE=1 时用）
#endif

// ===========================================================================
//  二、中断 ↔ 主循环 之间共享的数据（必须 volatile）
// ===========================================================================
static volatile float   g_fb_speed_rpm = 0.0f;   // 最近一次转速反馈（rpm）
static volatile float   g_fb_pos_rad   = 0.0f;   // 最近一次累计角度（rad）
static volatile float   g_fb_temp_c    = 0.0f;   // 最近一次温度（℃）
// 运行模式：0 = 速度环，1 = 位置环。
// 【为什么做成编译开关】位置版和速度版只差这一个数字，做成开关后一条命令就能出两个固件，
//   不用手工改代码，也就不会出现"忘了改回来"这种事故。
#ifndef RUN_MODE
#define RUN_MODE 0
#endif
static volatile uint8_t g_mode         = RUN_MODE;   // 0 = 速度环，1 = 位置环

// ---- 给上位机（Ozone / VOFA+）看的"每拍快照"：目标 / 反馈 / 输出 ----
//  为什么要单独存一份？因为 ref、fdb、u 原来都是 MainTask 里的**局部变量**，
//  函数一返回就消失了 —— Ozone 这类调试器**看不到局部变量**，
//  存成文件级变量（static）才能被采样、被画成曲线。
static volatile float   g_wave_ref     = 0.0f;   // 本拍目标值（速度环：rpm；位置环：rad）
static volatile float   g_wave_fdb     = 0.0f;   // 本拍反馈值（单位同上一行）
static volatile float   g_wave_u       = 0.0f;   // 本拍最终输出（限幅后的电压给定值）
static volatile float   g_out_raw      = 0.0f;   // 护栏之前的原始 PID 输出（区分"算法没出力"还是"护栏掐了"）
static volatile uint32_t g_can_rx_cnt  = 0;      // 收到过多少帧电机反馈（只增不减，用来确认 CAN 通不通）

// ===========================================================================
//  二·补、基地相关常量（一个数字只写一处，别处一律引用它）
// ===========================================================================

// 电机拨码 ID（1~7）：反馈帧标识符 = 0x204 + 本值；控制帧 1~4 号用 0x1FF、5~7 号用 0x2FF。
// 官方手册第 6 页：拨码开关第 1~3 位 = ID 的 Bit0~Bit2；第 4 位 = CAN 终端电阻。
// 现场若发现电机不是 1 号（判据：绿灯 1 秒闪 N 次 = ID 为 N），只改这一个数字即可。
constexpr uint8_t kMotorId = 1;

// ===== 基地上电阶梯（保守起见分三级，详见 notes\基地取数·前置准备与现场卡.md）=====
//   0 = 只观测：算出来的输出照常画在曲线上，但发给电机的是 0（电机零力矩，可自由手转轴）
//   1 = 小幅值试跟随：速度 30 rpm / 位置 0.2 rad
//   2 = 正式参数：速度 60 rpm / 位置 1.0 rad（**交付用的就是这一档**）
#ifndef BRINGUP_STAGE
#define BRINGUP_STAGE 2
#endif

// 正弦目标的"振幅"单独做成开关（不用去动上面的阶梯表）：
//   速度：单位 rpm    实测 60 rpm 大约就要 8000 单位电压，再往上容易顶到 PID_VOLT_LIMIT
//   位置：单位 弧度    0.5 ≈ ±28.6°，1.0 ≈ ±57.3°，1.5 ≈ ±86°
//   ⚠️ 把位置振幅改大之前，先确认转轴周围没有障碍物、线缆不会被卷进去！
#ifndef SPEED_AMP
#define SPEED_AMP 60.0f
#endif
#ifndef POS_AMP
#define POS_AMP 1.0f
#endif

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

// 把一路电压给定装进 0x1FF 帧发给 kMotorId 号电机
static void SendVoltage(float u) {
    if (u >  25000.0f) u =  25000.0f;              // 协议上限（双保险）
    if (u < -25000.0f) u = -25000.0f;

    const int16_t v = (int16_t)u;                  // 转成 16 位有符号整数
    uint8_t data[8] = {0};                         // 其余电机补 0（= 不给力，安全）
    const uint8_t idx = (uint8_t)((kMotorId - 1) * 2);  // 本电机在 8 字节里的起始下标：1 号→0、2 号→2 …

    // 官方手册第 7 页：电压给定值【高 8 位在前】
    data[idx]     = (uint8_t)((v >> 8) & 0xFF);    // 高字节在前（大端）
    data[idx + 1] = (uint8_t)(v & 0xFF);           // 低字节在后

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
#if ENABLE_UART_WAVE
static void SendWave(float ref, float fdb, float u) {
    g_wave.reset();
    if (!g_wave.push(ref) || !g_wave.push(fdb) || !g_wave.push(u)) {
        return;                                    // 有 NaN/Inf 就整帧不发（B6 里验过的坑）
    }
    if (HAL_UART_Transmit(&huart1, g_wave.data(), g_wave.size(), 10) != HAL_OK) {
        // 串口忙/超时 → 丢这一帧波形即可（波形是给人看的，不影响控制）
    }
}
#endif

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

#if BRINGUP_STAGE == 0
    g_ref_speed.configure(0.0f, 0.2f, 0.0f);           // 阶段0：只观测，目标恒 0
    g_ref_pos.configure(0.0f, 0.1f, 0.0f);
#elif BRINGUP_STAGE == 1
    g_ref_speed.configure(30.0f, 0.2f, 0.0f);          // 阶段1：小幅值试跟随
    g_ref_pos.configure(0.2f, 0.1f, 0.0f);
#else
    g_ref_speed.configure(SPEED_AMP, 0.2f, 0.0f);      // 阶段2：正式参数（默认速度 60 rpm、0.2 Hz）
    g_ref_pos.configure(POS_AMP, 0.1f, 0.0f);          //          位置（默认 1.0 rad、0.1 Hz）
#endif

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

#if defined(OPENLOOP_VOLT)
    // 【开环标定档】不做任何闭环计算，直接发一个固定的小电压。
    // 为什么需要它：万一"正电压"和"反馈转速的正负"方向相反，闭环就会变成正反馈 →
    // 电机越转越快（爆转）。先开环发一个小电压，看反馈是正还是负，就能确定方向。
    ref = 0.0f;
    fdb = g_fb_speed_rpm;
    u   = OPENLOOP_VOLT;
#else
    if (g_mode == 0) {                                 // 速度环
        ref = g_ref_speed.step();                      // 目标转速 rpm
        fdb = g_fb_speed_rpm;                          // 反馈转速 rpm
        u   = g_pid_speed.calc(ref, fdb);
    } else {                                           // 位置环
        ref = g_ref_pos.step();                        // 目标角度 rad
        fdb = g_fb_pos_rad;                            // 反馈角度 rad
        u   = g_pid_pos.calc(ref, fdb);
    }
#endif

    g_out_raw = u;                                     // 先存下"护栏之前"的值，方便判断是谁把输出弄没了
    u = g_guard.limit(u);                              // 安全护栏：不允许就归零，否则夹进安全范围

    // 把本拍三个量存成文件级变量：Ozone 采样和串口波形都从这里取
    g_wave_ref = ref;
    g_wave_fdb = fdb;
    g_wave_u   = u;

#if BRINGUP_STAGE == 0
    SendVoltage(0.0f);                                 // 阶段0：只观测，电机零力矩（可自由手转轴）
#else
    SendVoltage(u);                                    // 阶段1/2：正常发控制量
#endif

#if ENABLE_UART_WAVE
    static uint32_t div = 0;                           // 波形降频：每 10 拍发一次（100 Hz）
    if (++div >= 10) {
        div = 0;
        SendWave(g_wave_ref, g_wave_fdb, g_wave_u);
    }
#endif

#ifdef DIAG_ALWAYS_FEED
    // 【诊断档专用】无条件喂狗：就算收不到电机反馈，板子也不会重启，
    // 这样调试器能稳稳连住，我们才能看清"CAN 到底收到帧没有"。
    // 安全性：收不到反馈时 g_guard.feedbackOk() 为假 → 输出电压被强制归零，电机不会乱转。
    HAL_IWDG_Refresh(&hiwdg);
#else
    if (g_wdog.shouldFeed(now)) {                      // 条件喂狗：三路都健康才喂
        HAL_IWDG_Refresh(&hiwdg);
    }
#endif
}

// --- ③ CAN 接收回调（由下面的 HAL 弱回调转发进来）---
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id != (0x204u + kMotorId)) {               // 不是本电机（1 号 → 0x205）的帧就丢掉，
        return;                                        // 免得把总线上别的设备的数据当成自己的
    }

    // 官方手册第 8 页：机械角度、转速都是【高 8 位在前】
    const uint16_t raw = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);  // 角度原始值 0~8191
    const int16_t  rpm = (int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3]);   // 转速 rpm，高字节在前
    const uint8_t  temp = data[6];                                                  // 温度 ℃

    g_angle.update(raw);                               // 多圈角度累计（跨圈处理在 B3 验过）
    g_fb_speed_rpm = (float)rpm;
    g_fb_pos_rad   = g_angle.rad();
    g_fb_temp_c    = (float)temp;

    g_can_rx_cnt++;                                    // 收到一帧就 +1（诊断用：只要它在涨，就说明 CAN 通了）

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