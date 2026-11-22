/**
 * @file    test_bsp_wdg.c
 * @brief   外部看门狗单元测试 实现
 *
 * 覆盖接口: Init / IsInitialized / Feed
 *
 * 真机说明: Feed 翻转 PA15(WDI) 电平喂狗。本测试验证初始化与喂狗调用不崩溃、
 *   幂等性, 并连续喂狗多次 (真机配合逻辑分析仪可观测 PA15 方波翻转)。
 *   ⚠ 注意: 若 TPS3820 已使能, 测试期间须保证喂狗周期 < 超时窗口, 否则超时会切断泄放通路(解耦后不复位 MCU)。
 *
 * @version 1.0.0
 * @date    2026-06-13
 */

#include "test_bsp_wdg.h"

#define BSP_USE_WDG
#include "bsp.h"

void Test_BspWdg_RunAll(Test_Stats_T *_stats)
{
    int i;

    TEST_CASE("Wdg: 初始化");
    TEST_ASSERT_EQ(Bsp_Wdg_Init(), BSP_WDG_OK);
    TEST_ASSERT(Bsp_Wdg_IsInitialized() != 0);

    TEST_CASE("Wdg: 重复初始化幂等");
    TEST_ASSERT_EQ(Bsp_Wdg_Init(), BSP_WDG_OK);
    TEST_ASSERT(Bsp_Wdg_IsInitialized() != 0);

    TEST_CASE("Wdg: 连续喂狗不崩溃 (PA15 电平翻转)");
    /* 连续喂狗 20 次, 真机配合逻辑分析仪可观测 PA15 方波;
     * 此处验证函数可重复安全调用 (无返回值, 不崩溃即通过路径断言) */
    for (i = 0; i < 20; i++) {
        Bsp_Wdg_Feed();
    }
    TEST_LOG_VAL("已连续喂狗 %d 次 (PA15 应有方波翻转, 逻辑分析仪观测)", i);
    TEST_ASSERT(Bsp_Wdg_IsInitialized() != 0);   /* 喂狗后状态保持 */
}
