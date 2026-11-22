/**
 * @file    app_can_slave.h
 * @brief   泄放 CAN 从站接入层（基于通用引擎 canx_proto 的从站范例）
 *
 * 用通用协议引擎 canx_proto 实现泄放模块 CAN 从站（被动应答），替代手写的
 * app_comm 收发框架，验证「主从共用同一协议栈」。本模块只写两个协议私有回调：
 *   - decode_rx ：主站 ID 过滤 + 命中本节点判定（写类命令就地落库）
 *   - on_request：按命令码组应答帧（读状态复用 AppComm_PackSnapshot）
 * 全部调度时序（收帧/派发/超时/故障）复用 canx_proto，不再重写。
 *
 * 与 app_comm 的关系：二者都注册 CAN RX 中断，**不可同时启用**——
 * 启用本模块时，App_Init 不应再调 AppComm_Init（见集成说明）。
 *
 * 数据流：经典 CAN 2.0B，按 CAN-ID 寻址（dispatch=CANX_DISPATCH_SINGLE，
 * 主站 ID 过滤在 decode_rx 内完成）。
 *
 * @version 0.1.0
 * @date    2026-06-29
 */

#ifndef APP_CAN_SLAVE_H
#define APP_CAN_SLAVE_H

#include "app.h"
#include "canx_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化泄放 CAN 从站：复位接收桥接 + 以从站角色初始化 canx_proto 引擎。
 * @note  须在 Bsp_Can_Init() 之后调用（CAN 控制器就绪）。与 AppComm_Init 互斥。
 *        内部完成：CanxPort_Init()（注册 RX 中断回调）+ CanxProto_InitSlave()。
 */
void AppCanSlave_Init(void);

/**
 * @brief 主循环每圈调用：驱动引擎收帧/派发/被动应答/主站掉线超时检测。
 * @param now_ms 当前毫秒时间戳（systick_get_ms()）。
 */
void AppCanSlave_Task(uint32_t now_ms);

/**
 * @brief 发送上电完成通告帧（A5 5A 5A FF FF FF FF FF），无条件一次。
 */
void AppCanSlave_SendBootAnnounce(void);

#if defined(BSP_UNIT_TEST)
/** @brief 测试钩子：返回从站 Ops 表（供单测用 mock bsp 驱动 decode_rx/on_request）。 */
const CanxOps_T *AppCanSlave_TestGetOps(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_CAN_SLAVE_H */
