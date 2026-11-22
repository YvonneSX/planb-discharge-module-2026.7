/**
 * @file    app_protect.h
 * @brief   HLD-MOD-PROT 保护判据/告警模块（★安全核心）
 *
 * 职责：多重保护判据 + 分级过压告警/计次 + 故障闭锁。**不再驱动泄放执行**
 * （起/停归 ADC 看门狗 AppDch_OvpIsr，爬坡/切级归 TIMER1 AppDch_StepIsr）：
 *   - 两级分级过压告警：一级 ≥56.5V(OV_L1)、二级 ≥58V(OV_L2，上报关电机使能+计次)；旧 60V 已并入 58V
 *   - 过温（≥95℃）→ 置 OVER_TEMP 故障(门控泄放) + 单次计次（温度自恢复，≤65℃自动解除，非锁存）
 *   - 关断类（Brake_FB 失效、泄放超时）→ 置故障 + 锁存（DCH 各 ISR 查故障位即停）
 *   - 仅置故障类（过流）→ 不改泄放动作、只置位
 *   从站不主动上报：故障置 fault_bits 由主站轮询读取（0x23）。
 *
 * 关键对外接口（跨模块）：
 *   - AppProt_GetFaults()/GetOv2Count()/GetOtpCount() → COMM(0x23)/BBX/SAFE 只读
 *   - AppProt_PollEvent()       → BBX 拉取故障/告警事件用于黑匣子记录
 *   - AppProt_ClearFaults()     → COMM(0x31) 清除可清故障锁存
 *   - AppProt_NotifyDchTimeout()→ DCH→PROT 回报封顶无效（置 DCH_TIMEOUT）
 *   注：泄放级别/状态由 AppDch_GetLevel() 提供（遥测 dch_state），不再由 PROT 出。
 *
 * 支撑需求：REQ-FUNC-010/011/020~029、REQ-SAFE-003、REQ-PERF-002。
 */

#ifndef APP_PROTECT_H
#define APP_PROTECT_H

#include "app.h"

#ifdef __cplusplus
extern "C" {
#endif

void           AppProt_Init(void);

/** @brief 【已废弃/保留符号】旧过压快路径回调，现为空操作（起/停已归 AppDch_OvpIsr）。 */
void           AppProt_FastCheck(void);

/** @brief 慢路径每圈调用：过温/过流/两级分级过压告警计次、故障置位与清除。不驱动泄放。 */
void           AppProt_Update(void);

/** @brief 慢路径：Brake_FB 去抖采样 + 泄放回路失效判定（失效则置 LOOP_FB + 锁存）。 */
void           AppProt_LoopSelfCheck(void);

/** @brief 当前故障位图（App_Fault_T），供 0x23/BBX/SAFE。 */
uint16_t       AppProt_GetFaults(void);

/** @brief 二级过压累计次数，供 0x23。 */
uint16_t       AppProt_GetOv2Count(void);
uint16_t       AppProt_GetOtpCount(void);   /* 过温(≥95℃)异常次数（单次计次，停泄放） */

/**
 * @brief 清除可清除的故障锁存（0x31）。
 * @return 清除后剩余的 fault_bits（未满足解除条件的锁存仍保留）。
 */
uint16_t       AppProt_ClearFaults(void);

/**
 * @brief DCH→PROT 故障回报：泄放封顶滞留仍无效时由 DCH 调用，置 DCH_TIMEOUT
 *        故障（锁存）并入事件队列，供 COMM(0x23)/BBX/主站感知"泄放能力不足需停机"。
 */
void           AppProt_NotifyDchTimeout(void);

/**
 * @brief 供 BBX 拉取一条待记录事件（无则返回 false）。PROT 为事件产生者，
 *        BBX 为消费者，保持 PROT 不反向依赖上层模块。
 * @param out_evt 输出事件类型
 * @return 是否取到事件
 */
bool           AppProt_PollEvent(App_EvtType_T *out_evt);

#ifdef __cplusplus
}
#endif

#endif /* APP_PROTECT_H */
