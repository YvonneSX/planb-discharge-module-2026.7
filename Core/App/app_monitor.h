/**
 * @file    app_monitor.h
 * @brief   HLD-MOD-MON 采样监测模块 — 系统量测快照的唯一产出者
 *
 * 职责：每圈从 bsp_analog 取母线电压/电流/温度与功率统计，推进过压判定
 * 用的 EMA 滤波，产出统一的“量测快照”（App_RtData_T 的量测字段）。
 *
 * 关键对外接口（跨模块）：
 *   - AppMon_GetSnapshot()        → PROT/COMM/BBX 只读消费量测快照
 *   - AppMon_GetVBusFiltered_mV() → PROT 状态机过压判定（抗毛刺）
 *
 * 支撑需求：REQ-FUNC-001~004、REQ-PERF-001、REQ-FUNC-028。
 */

#ifndef APP_MONITOR_H
#define APP_MONITOR_H

#include "app.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化监测模块：复位功率统计窗口与滤波器初值。
 * @note  须在 Bsp_Analog_Init() 之后调用（依赖 ADC/DMA 已在后台转换）。
 */
void AppMon_Init(void);

/**
 * @brief 慢路径每圈调用：推进滤波、更新功率统计、刷新内部量测快照。
 */
void AppMon_Update(void);

/**
 * @brief 读取当前量测快照（仅填充 V/I/T/P 量测字段；其余字段置 0）。
 * @param out 输出缓冲，不可为 NULL。
 * @note  duty/state/fault/ov2/total 等由 PROT/DCH 拥有，由 COMM/上层合成。
 */
void AppMon_GetSnapshot(App_RtData_T *out);

/**
 * @brief 滤波后母线电压（mV），供 PROT 过压判定使用。
 */
int32_t AppMon_GetVBusFiltered_mV(void);

/** @brief 瞬时母线电压（mV，未滤波）。 */
int32_t AppMon_GetVBus_mV(void);
/** @brief 瞬时母线电流（mA，带符号）。 */
int32_t AppMon_GetCurrent_mA(void);
/** @brief 泄放电阻温度（m℃）。 */
int32_t AppMon_GetTemp_mDegC(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MONITOR_H */
