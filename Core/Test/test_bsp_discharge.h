/**
 * @file    test_bsp_discharge.h
 * @brief   PWM泄放控制单元测试 头文件
 * @version 1.0.0
 * @date    2026-06-12
 */

#ifndef TEST_BSP_DISCHARGE_H
#define TEST_BSP_DISCHARGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

void Test_BspDischarge_RunAll(Test_Stats_T *stats);

/* 实时监视模式: PWM 保持关断(安全), 循环打印 Brake_FB(PA9) 反馈电平与当前占空比,
 * 供改变反馈输入时随动观测。按 q/ESC 退出。 */
void Test_BspDischarge_Monitor(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BSP_DISCHARGE_H */
