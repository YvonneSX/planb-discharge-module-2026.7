/**
 * @file    test_bsp_uart_log.h
 * @brief   调试串口单元测试 头文件
 * @version 1.0.0
 * @date    2026-06-12
 */

#ifndef TEST_BSP_UART_LOG_H
#define TEST_BSP_UART_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

void Test_BspUartLog_RunAll(Test_Stats_T *stats);

/* 实时监视模式: 回显串口接收到的每个字节(十六进制+可见字符),
 * 在终端键入即时验证 RX 通路。发送 ESC(0x1B) 退出。 */
void Test_BspUartLog_Monitor(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BSP_UART_LOG_H */
