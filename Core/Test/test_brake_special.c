/**
 * @file    test_brake_special.c
 * @brief   泄放从站 专项极限测试 — 串口遥测监视实现（跑真实 App + 只读观测）
 *
 * 设计（方案路线Y）：本模式不另写 BSP 直驱判据，而是正常跑真实固件的
 * App_Init/App_Run（含 MON/PROT/DCH/COMM/BBX 完整保护状态机），测试代码
 * 只额外挂一个**只读遥测钩子**：
 *   - 每 TELE_PERIOD_MS 打印一行定长遥测 [TELE]（时间/V/I/T/功率/占空比/状态/FB/on）；
 *   - 与上一轮快照比对，故障位/状态/占空比/计次发生边沿变化时打印事件行
 *     （[ALARM] 置位 / [CLEAR] 清除 / [STATE] 状态迁移 / [DCH] 起停 / [CNT] 计次）。
 * 台架加压/加热/加载时，从串口即可观察真实保护动作与炸板前的完整曲线。
 *
 * ⚠ 安全：本模式跑真实 App，母线过压(≥56V)会真实泄放！占空比受固件硬顶
 *   (硬顶90%) 与保护逻辑约束。复位即关断。CPU 被调试器 halt 时须靠硬件失效保护。
 *
 * 只读约束：本文件只调用 App / AppMon / AppProt / AppDch 的 Get* 接口，绝不写任何
 *   保护状态，保证观测到的是纯真实固件行为。
 *
 * @version 1.0.0
 * @date    2026-07-03
 */

#include "test_brake_special.h"

#include "app.h"
#include "app_monitor.h"
#include "app_protect.h"
#include "app_discharge.h"
#include "uart_log.h"

#define BST_TAG  "BST"

/* 遥测刷新周期（ms）：200ms 对极限爬升/炸板瞬间够密，又不刷屏。可 -DTELE_PERIOD_MS= 覆盖。 */
#ifndef TELE_PERIOD_MS
#define TELE_PERIOD_MS  200U
#endif

/* fault_bits 位名表（与 app_types.h 的 App_Fault_T 严格对应，低 8 位）。
 * bit7 原冷却闭锁已废弃，规划复用为 OVER_ENERGY（能量窗口，待标定；当前固件恒不置位）。 */
static const char *const FAULT_NAME[8] = {
    "OVER_CURRENT",   /* bit0 过流（仅置故障，不停泄放） */
    "OVER_TEMP",      /* bit1 过温≥95℃（停泄放，温度自恢复≤65℃清） */
    "UNDER_VOLT",     /* bit2 欠压/过泄放 */
    "LOOP_FB",        /* bit3 泄放回路失效（锁存） */
    "DCH_TIMEOUT",    /* bit4 泄放能力不足/封顶停机（锁存） */
    "OV_L1",          /* bit5 一级过压报警 */
    "OV_L2",          /* bit6 二级过压报警（计次） */
    "OVER_ENERGY",    /* bit7 能量窗口（规划，待标定；当前恒 0） */
};

/* 泄放级别名（App_DchLevel_T：0 停 / 1 一级爬升 / 2 二级100%） */
static const char *st_name(uint8_t st)
{
    switch (st) {
    case 0U: return "L0-OFF";
    case 1U: return "L1";
    case 2U: return "L2-FULL";
    default: return "?";
    }
}

/* 温度 m℃ → 整度（四舍五入），便于遥测行阅读 */
static int32_t temp_to_C(int32_t t_mC)
{
    return (t_mC + (t_mC >= 0 ? 500 : -500)) / 1000;
}

void Test_BrakeSpecial_Monitor(void)
{
    uint32_t n = 0U;
    uint32_t last_tele_ms;

    /* 上一轮快照（事件边沿检测用） */
    uint16_t prev_faults = 0U;
    uint8_t  prev_state  = 0U;
    uint8_t  prev_on     = 0U;
    uint16_t prev_otp    = 0U;
    uint16_t prev_ov2    = 0U;
    uint8_t  prev_level  = 0U;   /* 上轮泄放级别（0停/1一级/2二级），检测分级切换 */

    /* ===== 一次性横幅 + 安全警告 + 遥测格式说明 ===== */
    UartLog_Println("\r\n==== 泄放从站 专项极限测试 (真实App + 串口遥测) ====");
    UartLog_Println("[BST] uart ok, clk=%luHz — starting App_Init...",
                    (unsigned long)Bsp_Clock_GetSysclkFreq());
    UartLog_Println("!! 警告: 本模式跑真实固件, 母线过压(>=56V)会真实泄放! 占空比硬顶90%%, 20s超时兜底, 复位即关断 !!");
    UartLog_Println("!! 台架/负载仪器: 加压->过压泄放, 加热->过温停, 加载->过流, 持续高压->封顶停机 !!");
    UartLog_Println("遥测行 [TELE]: t=时间ms V=母线mV I=电流mA T=温度C Pi/Pk=瞬时/峰值功率W duty=占空比%% st=状态 FB=故障位hex on=泄放中 A9=Brake_FB原始电平(步骤1/2观测,极性见测试规程)");
    UartLog_Println("事件行: [ALARM]置位 [CLEAR]清除 [STATE]状态迁移 [DCH]起停 [CNT]计次 [LEVEL]分级切换(L0/L1/L2); grep [ALARM]/[LEVEL] 抓瞬间");
    UartLog_Println("状态 st: 0=NORMAL 1=OVER_V(过压泄放) 2=RECOVER; FB 位: 0过流 1过温 2欠压 3回路 4封顶 5一级 6二级 7能量(预留)");

    /* ===== 真实 App 初始化（BSP + MON + PROT + DCH + COMM + BBX，全程泄放关断，发 0xA5） ===== */
    App_Init();

    /* 测试夹具：把工作配置强制恢复为代码默认（仅 RAM，不写 FRAM），覆盖已烧录设备 FRAM 里的旧值
     * （如旧 duty_max=10 会让起步 20% 一上来就 ≥ 上限、500ms 即封顶停机 DCH_TIMEOUT）。
     * 每次 brake-special 启动都重新应用；不影响正式固件。 */
    Bsp_Config_SetDefault();
    UartLog_Println("[BST] 工作配置恢复代码默认(RAM): duty %u%%起步/%u%%封顶, 覆盖FRAM旧值",
                    (unsigned)Bsp_Config_GetPwmDutyStart(), (unsigned)Bsp_Config_GetPwmDutyMax());

    UartLog_Println("[BST] App_Init done, 进入真实主循环 + 遥测 (周期%ums)", (unsigned)TELE_PERIOD_MS);

    last_tele_ms = systick_get_ms();

    for (;;) {
        /* ---- 真实主循环单圈：量测→保护→泄放→CAN应答→黑匣子→安全→喂狗 ---- */
        App_Run();

        /* ---- 只读采集当前运行量 ---- */
        uint16_t faults = AppProt_GetFaults();
        uint8_t  state  = AppDch_GetLevel();   /* 泄放级别 L0/L1/L2（取代旧 PROT 施密特状态） */
        uint8_t  on     = AppDch_IsOn() ? 1U : 0U;
        uint16_t otp    = AppProt_GetOtpCount();
        uint16_t ov2    = AppProt_GetOv2Count();
        uint8_t  level  = AppDch_GetLevel();
        uint32_t now    = systick_get_ms();

#ifdef BST_WDG_NOFEED_MS
        /* 步骤1/2 看门狗→A9 观测：前 BST_WDG_NOFEED_MS 内不喂狗(AppSafe_Kick 门控)、之后自动喂狗。
         * A9 电平一变化立即打印 + 每100ms 打印一次，"喂狗"字段标当前阶段，供经验判定哪个电平=泄放有效。
         * 只读打印、仅主循环内，不进中断（符合 ISR 实时性纪律）。 */
        {
            static uint32_t s_a9_t0   = 0U;
            static int      s_a9_prev = -1;
            int a9  = (int)Bsp_Discharge_ReadFeedback();
            int fed = (now >= (uint32_t)(BST_WDG_NOFEED_MS)) ? 1 : 0;
            if (a9 != s_a9_prev || (uint32_t)(now - s_a9_t0) >= 100U) {
                UartLog_Println("[A9] t=%lums 喂狗=%d A9=%d (V=%ldmV)",
                                (unsigned long)now, fed, a9, (long)AppMon_GetVBus_mV());
                s_a9_t0   = now;
                s_a9_prev = a9;
            }
        }
#endif

        /* ---- 事件边沿检测（与上一轮比对，立即打印，不等遥测周期） ---- */
        if (faults != prev_faults) {
            uint16_t changed = (uint16_t)(faults ^ prev_faults);
            for (uint8_t b = 0U; b < 8U; b++) {
                uint16_t mask = (uint16_t)(1U << b);
                if ((changed & mask) == 0U) { continue; }
                if ((faults & mask) != 0U) {
                    UartLog_Println("[ALARM] t=%lums bit%u %s set   (V=%ldmV T=%ldC duty=%u%% on=%u)",
                                    (unsigned long)now, (unsigned)b, FAULT_NAME[b],
                                    (long)AppMon_GetVBus_mV(), (long)temp_to_C(AppMon_GetTemp_mDegC()),
                                    (unsigned)AppDch_GetDuty(), (unsigned)on);
                } else {
                    UartLog_Println("[CLEAR] t=%lums bit%u %s clear (V=%ldmV T=%ldC)",
                                    (unsigned long)now, (unsigned)b, FAULT_NAME[b],
                                    (long)AppMon_GetVBus_mV(), (long)temp_to_C(AppMon_GetTemp_mDegC()));
                }
            }
            prev_faults = faults;
        }
        if (state != prev_state) {
            UartLog_Println("[STATE] t=%lums %s -> %s (V=%ldmV)",
                            (unsigned long)now, st_name(prev_state), st_name(state),
                            (long)AppMon_GetVBus_mV());
            prev_state = state;
        }
        if (on != prev_on) {
            UartLog_Println("[DCH] t=%lums 泄放 %s (duty=%u%% V=%ldmV)",
                            (unsigned long)now, on ? "ON" : "OFF",
                            (unsigned)AppDch_GetDuty(), (long)AppMon_GetVBus_mV());
            prev_on = on;
        }
        if (otp != prev_otp) {
            UartLog_Println("[CNT] t=%lums otp_count=%u (过温异常次数, T=%ldC)",
                            (unsigned long)now, (unsigned)otp, (long)temp_to_C(AppMon_GetTemp_mDegC()));
            prev_otp = otp;
        }
        if (ov2 != prev_ov2) {
            UartLog_Println("[CNT] t=%lums ov2_count=%u (二级过压次数, V=%ldmV)",
                            (unsigned long)now, (unsigned)ov2, (long)AppMon_GetVBus_mV());
            prev_ov2 = ov2;
        }
        /* 分级泄放级别切换（L0停/L1一级20%起爬/L2二级100%），由 TIMER1 100us 级别机 StepIsr 维护。
         * 级别一变化立即打印，便于台架升/降压时观测三段迟滞的切换电压点是否符合预期。 */
        if (level != prev_level) {
            UartLog_Println("[LEVEL] t=%lums L%u -> L%u (V=%ldmV duty=%u%%)",
                            (unsigned long)now, (unsigned)prev_level, (unsigned)level,
                            (long)AppMon_GetVBus_mV(), (unsigned)AppDch_GetDuty());
            prev_level = level;
        }

        /* ---- 周期遥测行 ---- */
        if ((uint32_t)(now - last_tele_ms) >= TELE_PERIOD_MS) {
            App_RtData_T snap;
            AppMon_GetSnapshot(&snap);   /* 取功率统计 p_inst_w/p_peak_w */
            last_tele_ms = now;

            UartLog_Println("[TELE] #%lu t=%lums V=%ldmV I=%ldmA T=%ldC Traw=%u Pi=%uW Pk=%uW duty=%u%% st=%u(%s) FB=0x%02X on=%u A9=%d",
                            (unsigned long)(n++), (unsigned long)now,
                            (long)AppMon_GetVBus_mV(),
                            (long)AppMon_GetCurrent_mA(),
                            (long)temp_to_C(AppMon_GetTemp_mDegC()),
                            (unsigned)Bsp_Analog_ReadRawTemp(),
                            (unsigned)snap.p_inst_w, (unsigned)snap.p_peak_w,
                            (unsigned)AppDch_GetDuty(),
                            (unsigned)state, st_name(state),
                            (unsigned)(faults & 0xFFU), (unsigned)on, (int)Bsp_Discharge_ReadFeedback());
        }
    }
}
