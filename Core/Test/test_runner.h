/**
 * @file    test_runner.h
 * @brief   BSP 单元测试总控 头文件 — 宏开关 + 统一入口
 *
 * main.c 只需:
 *   #if defined(BSP_UNIT_TEST)
 *       Test_Runner_RunAll();
 *   #endif
 *
 * 各模块测试开关 TEST_ENABLE_xxx 默认全开, 可在编译时 -D 覆盖单独裁剪。
 *
 * @version 1.0.0
 * @date    2026-06-12
 */

#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

/* ----------------------------------------------------------------
 * 各 BSP 模块单元测试开关 (默认全开, 可用 -DTEST_ENABLE_xxx=0 关闭)
 * ---------------------------------------------------------------- */
#ifndef TEST_ENABLE_UART_LOG
#define TEST_ENABLE_UART_LOG    1
#endif
#ifndef TEST_ENABLE_ANALOG
#define TEST_ENABLE_ANALOG      1
#endif
#ifndef TEST_ENABLE_DISCHARGE
#define TEST_ENABLE_DISCHARGE   1
#endif
#ifndef TEST_ENABLE_CAN
#define TEST_ENABLE_CAN         1
#endif
#ifndef TEST_ENABLE_SPI
#define TEST_ENABLE_SPI         1
#endif
#ifndef TEST_ENABLE_FRAM
#define TEST_ENABLE_FRAM        1
#endif
#ifndef TEST_ENABLE_WDG
#define TEST_ENABLE_WDG         1
#endif
#ifndef TEST_ENABLE_OVP
#define TEST_ENABLE_OVP         1
#endif
/* FLASH 暂时停用（配置改存 FRAM）；如需启用片上 Flash 测试，恢复 Makefile 源文件并设回 1 */
#ifndef TEST_ENABLE_FLASH
#define TEST_ENABLE_FLASH       0
#endif
#ifndef TEST_ENABLE_CONFIG
#define TEST_ENABLE_CONFIG      1
#endif
#ifndef TEST_ENABLE_CANX
#define TEST_ENABLE_CANX        1
#endif
#ifndef TEST_ENABLE_CANX_PORT
#define TEST_ENABLE_CANX_PORT   1
#endif
#ifndef TEST_ENABLE_CANX_SLAVE
#define TEST_ENABLE_CANX_SLAVE  1
#endif

/* ----------------------------------------------------------------
 * 公共接口
 * ---------------------------------------------------------------- */

/* 测试统一初始化 (日志等公共准备); RunAll 内部会自动调用 */
void Test_Runner_Init(void);

/* 运行所有已启用的 BSP 单元测试, 返回失败断言数 (0 = 全通过) */
uint32_t Test_Runner_RunAll(void);

/* ----------------------------------------------------------------
 * 实时监视模式 (输入类模块): 循环采集并打印, 供改变物理输入时随动观测。
 *   用编译宏选定监视对象, 经 make 目标自动注入:
 *     MONITOR_ANALOG     - 模拟量(电压/电流/温度/功率)   make analog-live
 *     MONITOR_DISCHARGE  - 泄放 Brake_FB 反馈电平         make discharge-live
 *     MONITOR_UART       - 串口 RX 回显                   make uart-live
 *     CAN_INTERACTIVE_TEST - CAN 主站命令交互应答         make can-loop
 *   main.c: 定义 BSP_MONITOR 时改调 Test_Runner_RunMonitor() 取代 RunAll()。
 * ---------------------------------------------------------------- */
void Test_Runner_RunMonitor(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_RUNNER_H */
