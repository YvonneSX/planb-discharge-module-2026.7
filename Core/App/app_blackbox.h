/**
 * @file    app_blackbox.h
 * @brief   HLD-MOD-BBX 黑匣子模块 — 运行/故障数据分段持久化至 FRAM
 *
 * 职责：寿命统计/极值、故障事件环、故障前后趋势环（锚点 N+K）；断电不丢。
 * 时间轴仅存相对时间（pwr_seq + t_rel_ms）；本期不写 UTC。
 * 数据记录模块：保存运行统计、事件和趋势数据，供模块内部诊断逻辑使用。
 *
 * 关键对外接口（跨模块）：
 *   - AppBbx_LogEvent()      ← 由编排层从 AppProt_PollEvent() 取事件后调用
 *   - AppBbx_ShutdownClean() ← 0x34 软复位前调用，置正常关机标志
 *
 * 支撑需求：REQ-FUNC-050/051。
 */

#ifndef APP_BLACKBOX_H
#define APP_BLACKBOX_H

#include "app.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_BBX_OK            =  0,
    APP_BBX_ERR_PARAM     = -1,
    APP_BBX_ERR_FRAM      = -2,
    APP_BBX_NOT_IMPL      = -3,   /* 占位：FRAM 布局尚未落地 */
} App_Bbx_Status_T;

/**
 * @brief 上电初始化：读 RCU 复位原因→evt=7、判异常掉电（session_active）、
 *        递增 pwr_seq、校验各段 magic/CRC，损坏则重建。
 */
App_Bbx_Status_T AppBbx_Init(void);

/** @brief 正常关机清理（0x34 软复位前）：清 session_active，刷写统计段。 */
App_Bbx_Status_T AppBbx_ShutdownClean(void);

/** @brief 周期/限速记录运行快照到趋势环（消费 MON 快照）。 */
App_Bbx_Status_T AppBbx_LogPeriodic(const App_RtData_T *snap);

/** @brief 记录一条故障/告警事件（消费 PROT 事件 + 当时快照），并打趋势锚点。 */
App_Bbx_Status_T AppBbx_LogEvent(App_EvtType_T evt, const App_RtData_T *snap);

/** @brief 配置变更审计（evt=8）：记录来源与变更前后摘要。 */
App_Bbx_Status_T AppBbx_LogConfigChange(App_CfgSource_T src);

/* 诊断数据读取接口预留。 */
App_Bbx_Status_T AppBbx_ReadStats(void *buf, uint16_t len);
App_Bbx_Status_T AppBbx_ReadEvent(uint16_t index, void *buf, uint16_t len);
App_Bbx_Status_T AppBbx_ReadTrendPage(uint16_t page, void *buf, uint16_t len);

/** @brief 清空黑匣子全部分段并重新初始化记录区。 */
App_Bbx_Status_T AppBbx_ClearAll(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLACKBOX_H */
