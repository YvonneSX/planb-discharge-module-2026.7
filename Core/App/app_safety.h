/**
 * @file    app_safety.h
 * @brief   HLD-MOD-SAFE 安全/喂狗模块
 *
 * 职责：周期翻转外部看门狗（PA15→TPS3820）；统一“安全态”进入（关泄放）。
 * 喂狗为“每圈喂一次”，主循环死锁即停止喂狗 → TPS3820 超时切断泄放通路（解耦后不复位 MCU，监护意义）。
 *
 * 关键对外接口（跨模块）：
 *   - AppSafe_Kick()           ← 主循环每圈喂狗（须在关键任务执行后）
 *   - AppSafe_EnterSafeState() ← PROT/任意模块触发进入安全态（关泄放）
 *
 * 支撑需求：REQ-FUNC-030~032、REQ-SAFE-001/002/004、REQ-PERF-006。
 */

#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include "app.h"

#ifdef __cplusplus
extern "C" {
#endif

void AppSafe_Init(void);

/** @brief 每圈喂狗一次（在确认本圈关键任务已执行后调用）。 */
void AppSafe_Kick(void);

/** @brief 进入安全态：立即关断泄放（MOSFET 截止）。可重复调用。 */
void AppSafe_EnterSafeState(void);

/** @brief 是否已进入安全态。 */
bool AppSafe_InSafeState(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_H */
