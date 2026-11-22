/**
 * @file    test_bsp_wdg.h
 * @brief   外部看门狗单元测试 头文件
 * @version 1.0.0
 * @date    2026-06-13
 */

#ifndef TEST_BSP_WDG_H
#define TEST_BSP_WDG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

/* 运行外部看门狗模块全部测试, 统计累加到 stats */
void Test_BspWdg_RunAll(Test_Stats_T *stats);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BSP_WDG_H */
