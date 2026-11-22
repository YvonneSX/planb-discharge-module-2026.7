/**
 * @file    test_canx_port.h
 * @brief   canx_proto port 适配层 单元测试入口（模块2：帧互转 + 接收环形缓冲）
 *
 * 验证 canx_proto_port.c 的纯软件部分（与 CAN 硬件无关）：
 *   - 接收环形缓冲：注入→出队顺序、空队列返回 EMPTY、溢出丢帧计 drop；
 *   - 帧互转：经典 BSP 帧 → 中性帧字段映射（id/ide/dlc→len/data）。
 * 发送到总线、Bsp_Can_EnableRxIt 等硬件路径属集成测试，不在本模块。
 *
 * @version 0.1.0
 * @date    2026-06-29
 */

#ifndef TEST_CANX_PORT_H
#define TEST_CANX_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

void Test_CanxPort_RunAll(Test_Stats_T *_stats);

#ifdef __cplusplus
}
#endif

#endif /* TEST_CANX_PORT_H */
