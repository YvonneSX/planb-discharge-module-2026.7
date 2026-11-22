/**
 * @file    app_discharge.h
 * @brief   HLD-MOD-DCH 泄放控制模块 — PROT 请求的唯一执行者
 *
 * 职责：把 PROT 的“目标占空比/启停”转化为安全 PWM 执行，承载会话管理：
 *   - 占空比按固定斜率（dI/dt）爬升，工作区 20~90%、硬顶 90%
 *   - 一级泄放爬坡：20% 起步（56.5V 触发），每 100µs +2%，封顶 90%；二级 58V 直接 100%
 *   - 封顶后滞留 N 步（≈500ms）仍压不下母线 → 关断并置 DCH_TIMEOUT 锁存上报停机
 *   - 连续泄放超 20s（BSP_OVP_MAX_ON_MS）兜底：按单次连续会话计时，母线≤55V 后会话清零，下轮从 0 重计
 *   - 控制字 0x10 强制全开
 *   （过温不再由 DCH 关断/冷却：过温停泄放由 PROT 置 OVER_TEMP 故障挡住请求，
 *    温度自恢复；冷却闭锁已移除，见 app_protect.c）
 *
 * 关键对外接口（跨模块）：
 *   - AppDch_SetRequest()  ← PROT 占空比/启停请求入口（单一写者下行）
 *   - AppDch_ForceStop()   ← PROT/SAFE 紧急关断
 *   - AppDch_ForceFull()   ← COMM(0x10) 强制全开
 *   - AppDch_GetDuty()/IsOn()/GetTotalMs() → COMM(0x22)/BBX 只读
 *
 * 支撑需求：REQ-FUNC-012~018、REQ-PERF-004/007、REQ-SAFE-005。
 */

#ifndef APP_DISCHARGE_H
#define APP_DISCHARGE_H

#include "app.h"

#ifdef __cplusplus
extern "C" {
#endif

void    AppDch_Init(void);

/** @brief 接收 PROT 的目标占空比与启停请求（每圈刷新）。 */
void    AppDch_SetRequest(uint8_t duty_pct, bool enable);

/** @brief 主循环监控层（每圈调用）：累计时长、20s 连续泄放超时兜底。不再直接步进占空比。 */
void    AppDch_Update(void);

/** @brief 空闲态 A9(Brake_FB) 实时守护（主循环每圈调用）：空闲态(level==0)检测到 A9=高（等效
 *         100% 误泄放）即刻把 PA8 切普通 GPIO 强制拉低。A9 与泄放引脚输出状态一一对应。 */
void    AppDch_IdleFbGuard(void);

/** @brief 上电版本识别闪灯服务（主循环每圈调用，非阻塞）：首次上电从“亮”起每
 *         BSP_DCH_LED_PON_BLINK_MS 翻转 BSP_DCH_LED_PON_BLINK_COUNT 次（闪 2 下末次灭），
 *         用于区分固件版本；闪完自动让位 AppDch_ServiceDchLed。须在其之前调用。 */
void    AppDch_ServicePowerOnBlink(void);

/** @brief 泄放指示灯(PB0)服务（主循环每圈调用，非阻塞）：泄放中亮；停后至少再亮
 *         BSP_DCH_LED_MIN_ON_MS(1s) 才灭（泄放极快、保证可视）；另含母线持续<55V 达
 *         BSP_DCH_LED_OFF_FALLBACK_MS(2s) 兜底清标志。 */
void    AppDch_ServiceDchLed(void);

/** @brief 分级泄放爬坡/级别机（TIMER1 100µs 中断回调）：仅泄放开启后(level≠0) L1 每100µs+2%封顶90%、
 *         L1↔L2 级别切换；起/停归看门狗。ISR 上下文须极短。由 Bsp_StepTimer 注册驱动。 */
void    AppDch_StepIsr(void);

/** @brief ADC 模拟看门狗中断回调（µs 级）：泄放起/停唯一执行者。过压→起(20%,切泄放态窗口)、
 *         过泄压→停(切待机态窗口)。由 Bsp_Analog_EnableVBusWatchdog 注册、ADC0_1_IRQHandler 驱动。 */
void    AppDch_OvpIsr(void);

/** @brief 按当前泄放状态重设 ADC 看门狗窗口（阈值变更后调用，如 CAN 0x14/0x15 改过压启停）。 */
void    AppDch_RefreshWdgWindow(void);

/** @brief 紧急关断（过温停泄放/封顶停机/欠压/回路失效等调用），立即停 PWM。 */
void    AppDch_ForceStop(void);

/** @brief 控制字 0x10 强制全开。@return 是否被接受（冷却闭锁已移除，恒 true）。 */
bool    AppDch_ForceFull(bool enable);

/**
 * @brief 泄放总允许开关（0x30）：禁止时无论 PROT 请求/强制全开都不输出。
 *        默认上电允许。用于主站临时停用本机泄放。
 */
void    AppDch_SetAllowed(bool allowed);
/** @brief 当前是否允许泄放（0x30 状态）。 */
bool    AppDch_IsAllowed(void);

/** @brief 当前生效占空比（%）。 */
uint8_t AppDch_GetDuty(void);
/** @brief 当前是否在泄放。 */
bool    AppDch_IsOn(void);
/** @brief 当前泄放级别（0=停 1=一级爬升 2=二级100%）。 */
uint8_t AppDch_GetLevel(void);
/** @brief 累计泄放时长（ms）。 */
uint32_t AppDch_GetTotalMs(void);
/** @brief 是否处于冷却闭锁（冷却闭锁已移除，恒 false；保留接口兼容）。 */
bool    AppDch_InCooldown(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_DISCHARGE_H */
