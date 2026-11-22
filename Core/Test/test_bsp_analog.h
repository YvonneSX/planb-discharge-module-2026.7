/**
 * @file    test_bsp_analog.h
 * @brief   模拟采样单元测试 头文件
 * @version 1.0.0
 * @date    2026-06-12
 */

#ifndef TEST_BSP_ANALOG_H
#define TEST_BSP_ANALOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

/* 运行模拟采样模块全部测试, 统计累加到 stats */
void Test_BspAnalog_RunAll(Test_Stats_T *stats);

/* 实时监视模式: 循环按 TEST_MONITOR_PERIOD_MS 打印电压/电流/温度/功率/滤波值,
 * 供改变母线电压/加热 NTC 时随动观测并用仪表交叉核对标定。按 q/ESC 退出。 */
void Test_BspAnalog_Monitor(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BSP_ANALOG_H */
