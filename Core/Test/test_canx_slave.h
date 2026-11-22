/**
 * @file    test_canx_slave.h
 * @brief   泄放从站接入层 app_can_slave 单元测试入口（模块3）
 *
 * 用 mock bsp + 真实从站 Ops（AppCanSlave_TestGetOps）经引擎驱动，验证：
 *   - decode_rx 主站 ID 过滤（绑定主站受理 / 非绑定丢弃）；
 *   - on_request 读状态应答组帧（0x20~0x24）；
 *   - 控制命令业务联动（0x31 清故障经 PROT，0x30 允许门经 DCH）。
 *
 * @version 0.1.0
 * @date    2026-06-29
 */

#ifndef TEST_CANX_SLAVE_H
#define TEST_CANX_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

void Test_CanxSlave_RunAll(Test_Stats_T *_stats);

#ifdef __cplusplus
}
#endif

#endif /* TEST_CANX_SLAVE_H */
