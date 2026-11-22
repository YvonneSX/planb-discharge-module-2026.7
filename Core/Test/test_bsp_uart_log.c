/**
 * @file    test_bsp_uart_log.c
 * @brief   调试串口单元测试 实现
 *
 * 覆盖接口: Init / Deinit / IsInitialized / Write / Putc / ReadByte
 *
 * 真机说明: 测试日志本身经该串口输出。Deinit 会关闭串口, 故 Deinit 测试后
 *   立即重新 Init 恢复, 避免后续测试无日志输出。ReadByte 为非阻塞, 无输入时返回 0。
 *
 * @version 1.1.0
 * @date    2026-06-13
 */

#include "test_bsp_uart_log.h"

#define BSP_USE_UART_LOG
#include "bsp.h"
#include "uart_log.h"

void Test_BspUartLog_RunAll(Test_Stats_T *_stats)
{
    uint8_t ch = 0U;
    int r;

    /* ---- 初始化 ---- */
    TEST_CASE("UartLog: 初始化");
    TEST_ASSERT_EQ(Bsp_UartLog_Init(), BSP_UART_LOG_OK);
    TEST_ASSERT(Bsp_UartLog_IsInitialized() != 0);

    TEST_CASE("UartLog: 重复初始化幂等");
    TEST_ASSERT_EQ(Bsp_UartLog_Init(), BSP_UART_LOG_OK);

    /* ---- 写参数校验 ---- */
    TEST_CASE("UartLog: 写参数校验");
    TEST_ASSERT_EQ_INT(Bsp_UartLog_Write(NULL, 4U), 0);
    TEST_ASSERT_EQ_INT(Bsp_UartLog_Write("x", 0U), 0);

    /* ---- 正常写返回发送长度 ---- */
    TEST_CASE("UartLog: 正常写返回长度");
    TEST_ASSERT_EQ_INT(Bsp_UartLog_Write("hello", 5U), 5);

    /* ---- Putc 已初始化时返回 OK ---- */
    TEST_CASE("UartLog: Putc 发送单字节");
    TEST_ASSERT_EQ(Bsp_UartLog_Putc('A'), BSP_UART_LOG_OK);

    /* ---- ReadByte 非阻塞: 无输入返回 0 (实时 RX 验证用 make uart-live) ---- */
    TEST_CASE("UartLog: ReadByte 非阻塞 (无输入返回 0)");
    r = Bsp_UartLog_ReadByte(&ch);
    TEST_ASSERT_IN_RANGE(r, 0, 1);        /* 0=无数据, 1=恰好有输入 */
    /* NULL 入参应返回参数错 */
    TEST_ASSERT_EQ(Bsp_UartLog_ReadByte(NULL), BSP_UART_LOG_ERR_PARAM);

    /* ---- Deinit 后状态为未初始化, 随即恢复 ---- */
    TEST_CASE("UartLog: Deinit 关闭后重新初始化恢复");
    Bsp_UartLog_Deinit();
    TEST_ASSERT_EQ_INT(Bsp_UartLog_IsInitialized(), 0);
    /* 此时串口已关, 未初始化下 Putc 应返回 ERR_INIT */
    TEST_ASSERT_EQ(Bsp_UartLog_Putc('B'), BSP_UART_LOG_ERR_INIT);
    /* 恢复串口, 后续测试日志才能继续输出 */
    TEST_ASSERT_EQ(Bsp_UartLog_Init(), BSP_UART_LOG_OK);
    TEST_ASSERT(Bsp_UartLog_IsInitialized() != 0);
}

/* ================================================================
 * 串口 RX 回显实时监视模式 (make uart-live)
 * ----------------------------------------------------------------
 * RunAll 中 ReadByte 仅能断言"无输入返回 0", 无法验证真正收到数据。
 * 本模式循环非阻塞读取 RX: 每收到一个字节即原样回显其十六进制与可见
 * 字符, 在串口终端键入即可即时确认 RX 通路工作正常。
 * 退出: 串口发送 ESC(0x1B), 或直接复位。
 * ================================================================ */
void Test_BspUartLog_Monitor(void)
{
    uint8_t  ch;
    uint32_t cnt = 0U;

    (void)Bsp_UartLog_Init();

    UartLog_Println("\r\n=== UART RX 回显实时监视 ===");
    UartLog_Println("在串口终端键入字符, 应即时回显 (十六进制 + 可见字符)");
    UartLog_Println("退出: 发送 ESC(0x1B), 或复位");

    for (;;) {
        if (Bsp_UartLog_ReadByte(&ch) == 1) {
            if (ch == 0x1BU) {
                break;
            }
            UartLog_Println("RX[%lu] = 0x%02X '%c'",
                            (unsigned long)(cnt++), ch,
                            (ch >= 0x20U && ch < 0x7FU) ? (char)ch : '.');
        }
    }
    UartLog_Println("=== UART 监视结束 ===\r\n");
}
