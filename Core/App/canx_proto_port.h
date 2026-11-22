/**
 * @file    canx_proto_port.h
 * @brief   canx_proto 引擎平台适配契约（移植时按平台改本文件 + canx_proto_port.c）
 *
 * canx_proto 引擎本体(canx_proto.c/.h)平台中立、只认中性帧 CanxFrame_T，
 * 不 #include 任何 bsp.h。引擎与底层 CAN/CAN-FD BSP 的连接全部经由本 port：
 *   - 参数宏：CANX_CH_NUM / CANX_MAX_DATA / CANX_RXQ_DEPTH（移植时改这几个数）
 *   - 收发适配：在 canx_proto_port.c 实现 4 个 BspOps，并提供 CanxProto_DefaultBsp()
 *
 * 本工程(泄放模块)：GD32F303CCT6 经典 CAN 2.0B（Bsp_Can_*），中断回调式接收，
 * port.c 补软件接收环形缓冲把"中断入帧"桥接成引擎期望的"recv 出队"。
 *
 * @version 0.1.0
 * @date    2026-06-29
 */

#ifndef CANX_PROTO_PORT_H
#define CANX_PROTO_PORT_H

#include "canx_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 * 平台参数（移植时按平台改这几个数）
 *   CANX_MAX_DATA / CANX_CH_NUM 有默认值，可在 canx_proto.h 或 -D 覆盖；
 *   此处显式重申本工程取值，便于移植者一眼看到。
 * ==================================================================== */
/* 经典 CAN 最大 8 字节；CAN-FD 平台改 64（同时 -DCANX_MAX_DATA=64） */
/* 本工程仅 CAN0 一个通道 */
#ifndef CANX_RXQ_DEPTH
#define CANX_RXQ_DEPTH      16U     /* 软件接收环形缓冲深度（命令低频，16 足够） */
#endif

/* ====================================================================
 * 接收桥接初始化
 *   把 BSP 的「中断回调式接收」桥接成引擎期望的「recv 出队」：
 *   内部清空软件接收环形缓冲并把内部 ISR 回调注册到 Bsp_Can_EnableRxIt。
 *   须在 Bsp_Can_Init() 之后调用（CAN 控制器就绪后才能使能 RX 中断）。
 * ==================================================================== */

/**
 * @brief 初始化接收桥接：复位环形缓冲 + 注册 CAN RX 中断回调。
 * @note  调用顺序：Bsp_Can_Init() → CanxPort_Init()。
 *        引擎 recv 即从本缓冲出队，引擎本体不感知底层为中断驱动。
 */
void CanxPort_Init(void);

/* ====================================================================
 * 测试钩子（仅 BSP_UNIT_TEST）：注入 RX 帧 + 复位缓冲，替代真实 CAN 中断
 * ==================================================================== */
#if defined(BSP_UNIT_TEST)
/** @brief 复位接收环形缓冲与丢帧计数（不注册中断）。 */
void CanxPort_TestReset(void);
/** @brief 以原始字段注入一帧到接收环形缓冲（模拟 CAN RX 中断）。 */
void CanxPort_TestInjectRx(uint32_t id, uint8_t ide, uint8_t dlc, const uint8_t *data);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CANX_PROTO_PORT_H */
