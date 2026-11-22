/**
 * @file    test_canx_proto.h
 * @brief   canx_proto 通用 CAN 协议引擎 单元测试入口（模块1：引擎本体）
 *
 * 用 mock BspOps + mock Ops 回调驱动，不碰真实 CAN 硬件，结果确定可复现。
 * 覆盖主站时序（Init/注册/PollStep/握手/Tick 故障派生）与从站被动应答
 * （InitSlave/请求-应答/静音/超时）以及派发模式（DATA_BYTE/CAN_ID/SINGLE）。
 *
 * @version 0.1.0
 * @date    2026-06-29
 */

#ifndef TEST_CANX_PROTO_H
#define TEST_CANX_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

/** @brief 运行 canx_proto 引擎全部单元用例，统计累加到 *_stats。 */
void Test_CanxProto_RunAll(Test_Stats_T *_stats);

#ifdef __cplusplus
}
#endif

#endif /* TEST_CANX_PROTO_H */
