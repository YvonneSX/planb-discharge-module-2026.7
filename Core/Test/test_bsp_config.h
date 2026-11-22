/**
 * @file    test_bsp_config.h
 * @brief   设备配置（CAN 参数持久化）单元测试 头文件
 * @version 1.0.0
 * @date    2026-06-13
 */

#ifndef TEST_BSP_CONFIG_H
#define TEST_BSP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

/* 运行设备配置模块全部测试, 统计累加到 stats */
void Test_BspConfig_RunAll(Test_Stats_T *stats);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BSP_CONFIG_H */
