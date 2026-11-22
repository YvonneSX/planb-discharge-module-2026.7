/**
 * @file    app_types.h
 * @brief   App 业务层共享数据契约（HLD-DS-001）
 *
 * 本文件只定义“模块之间传递的数据语言”，不依赖任何 BSP/外设头，
 * 任何 App 模块都可包含。跨模块单向数据流（HLD §4.1）：
 *   MON 产出量测快照 App_RtData_T → PROT/DCH/COMM/BBX 只读消费；
 *   PROT 产出 fault_bits / state / duty 请求 → DCH/COMM/BBX/SAFE 消费。
 *
 * 标度约定（与概要设计 §4.3.3 一致，便于 COMM 直接打包 CAN 应答）：
 *   电压 0.1V/LSB、电流 0.01A/LSB（有符号，正=母线回馈）、温度 0.1℃/LSB、
 *   功率 1W/LSB、占空比 1%/LSB。
 *
 * @version 0.1.0
 * @date    2026-06-16
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 * 泄放级别（HLD §5.2 两级分级泄放；与 0x22/0x23 应答 dch_state 字段一致）
 * 由 AppDch_GetLevel() 提供；取代旧 PROT 施密特状态（NORMAL/OVER_VOLTAGE/RECOVERY_DELAY 已移除）。
 * ==================================================================== */
typedef enum {
    APP_DCH_LEVEL_OFF   = 0U,  /* L0 停：泄放关断，待机 */
    APP_DCH_LEVEL_1     = 1U,  /* L1 一级：56.5V 起 20%，每100µs+2% 爬到封顶 90% */
    APP_DCH_LEVEL_2     = 2U,  /* L2 二级：58V 全速 100% */
} App_DchLevel_T;

/* ====================================================================
 * 故障/保护状态位图（fault_bits）。与概要设计 §4.3.6 App_Fault_T 对齐。
 * 由 PROT 置位/清除，COMM(0x23)/BBX/SAFE 只读消费。
 * ==================================================================== */
typedef enum {
    APP_FAULT_OVER_CURRENT = (1U << 0),  /* 过流（仅置故障，不改泄放动作） */
    APP_FAULT_OVER_TEMP    = (1U << 1),  /* 过温 ≥95℃：停泄放+单次计次（温度自恢复，≤65℃自动清位，非锁存） */
    APP_FAULT_UNDER_VOLT   = (1U << 2),  /* 过泄放/欠压（停泄放） */
    APP_FAULT_LOOP_FB      = (1U << 3),  /* 泄放回路失效（Brake_FB 高=异常，关泄放+锁存） */
    APP_FAULT_DCH_TIMEOUT  = (1U << 4),  /* 泄放无效：封顶滞留/连续泄放超时压不下母线（锁存） */
    APP_FAULT_OV_L1        = (1U << 5),  /* 一级过压 ≥56.5V（起泄放/爬坡；随泄放级别置位） */
    APP_FAULT_OV_L2        = (1U << 6),  /* 二级过压 ≥58V（100%全速+上报关电机使能+计次） */
    APP_FAULT_COOLDOWN     = (1U << 7),  /* 废弃保留兼容（冷却闭锁已移除；旧 60V 二级告警并入 58V L2） */
} App_Fault_T;

/* 锁存类故障掩码：须显式清除（0x31）方可解除。
 * 过温为温度自恢复类（≥95℃停泄放、≤65℃自动清位），不是锁存故障，故不含 OVER_TEMP。 */
#define APP_FAULT_LATCHED_MASK  (APP_FAULT_LOOP_FB | APP_FAULT_DCH_TIMEOUT)

/* ====================================================================
 * 运行数据快照：MON 统一产出，PROT/COMM/BBX 只读消费。
 * 全部为已按 CAN 标度换算的工程量，COMM 可直接小端打包。
 * ==================================================================== */
typedef struct {
    uint16_t vbus_0v1;     /* 母线电压 0.1V/LSB（0~980） */
    int16_t  ibus_0a01;    /* 母线电流 0.01A/LSB（±4000，正=回馈） */
    int16_t  temp_0c1;     /* 泄放电阻温度 0.1℃/LSB */
    uint16_t p_inst_w;     /* 瞬时功率 1W/LSB */
    uint16_t p_avg_w;      /* 平均功率 1W/LSB */
    uint16_t p_peak_w;     /* 峰值功率 1W/LSB */
    uint8_t  duty_pct;     /* 当前占空比 0~90 %（一级爬坡区间；二级=100） */
    uint8_t  dch_state;    /* 泄放级别 App_DchLevel_T: 0=L0停/1=L1一级/2=L2二级（AppDch_GetLevel） */
    uint8_t  dch_on;       /* 0=关断，1=泄放中 */
    uint16_t fault_bits;   /* App_Fault_T 位图 */
    uint16_t ov2_count;    /* 二级过压累计次数 */
    uint16_t otp_count;    /* 过温(≥95℃)异常次数（单次计次，停泄放） */
    uint32_t dch_total_ms; /* 累计泄放时长 ms */
} App_RtData_T;

/* ====================================================================
 * 设备信息（0x24 应答）。字段来自代码段宏 BSP_APP_*，不读 FRAM。
 * ==================================================================== */
typedef struct {
    uint8_t  fw_major;
    uint8_t  fw_minor;
    uint8_t  fw_patch;
    uint8_t  hw_ver;
    uint16_t dev_type;
} App_DevInfo_T;

/* ====================================================================
 * 黑匣子事件类型。此枚举用于固化记录类型编号。
 * ==================================================================== */
typedef enum {
    APP_EVT_NONE          = 0U,
    APP_EVT_OV_L1         = 1U,  /* 一级过压 */
    APP_EVT_OV_L2         = 2U,  /* 二级过压 */
    APP_EVT_OVER_TEMP     = 3U,  /* 过温关断 */
    APP_EVT_LOOP_FB       = 4U,  /* 泄放回路失效 */
    APP_EVT_DCH_TIMEOUT   = 5U,  /* 泄放超时 */
    APP_EVT_OVER_CURRENT  = 6U,  /* 过流 */
    APP_EVT_RESET         = 7U,  /* 复位原因（含异常掉电标志） */
    APP_EVT_CONFIG_CHANGE = 8U,  /* 配置变更审计 */
    /* 9/10 保留 */
} App_EvtType_T;

/* ====================================================================
 * 配置变更来源
 * ==================================================================== */
typedef enum {
    APP_CFG_SRC_LOCAL   = 0U,  /* 本地/上电默认 */
    APP_CFG_SRC_MASTER  = 1U,  /* 主站写入 */
    APP_CFG_SRC_RESTORE = 2U,  /* 恢复默认配置 */
    APP_CFG_SRC_SERVICE = 3U,  /* 维护操作 */
} App_CfgSource_T;

#ifdef __cplusplus
}
#endif

#endif /* APP_TYPES_H */
