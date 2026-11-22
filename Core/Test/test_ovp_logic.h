/**
 * @file    test_ovp_logic.h
 * @brief   过压保护业务逻辑单元测试 头文件
 *
 * 本测试在单元测试层实现并验证《过压保护设计思路梳理》定义的完整过压保护
 * 业务逻辑（施密特状态机 + 分级阈值 + 20s/8min 计时 + 温度保护 + 控制字），
 * 作为后续 App 层 OvpProtect 模块的参考实现与验收基线。
 *
 * 依赖 BSP: Bsp_Analog_*（采样/滤波）、Bsp_Discharge_*（PWM 斜坡/停止）。
 *
 * @version 1.0.0
 * @date    2026-06-13
 */

#ifndef TEST_OVP_LOGIC_H
#define TEST_OVP_LOGIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

void Test_OvpLogic_RunAll(Test_Stats_T *stats);

#ifdef __cplusplus
}
#endif

#endif /* TEST_OVP_LOGIC_H */
