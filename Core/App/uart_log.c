/**
 * @file    uart_log.c
 * @brief   UART 日志模块实现 (GD32F303CCT6)
 *
 * 基于 PlanbPro GD32H75E uart_log.c 适配。
 * 提供分级日志、HEX 转储、newlib _write() 重定向。
 *
 * @version 1.0.0
 * @date    2026-06-11
 */

#include "uart_log.h"

#define BSP_USE_UART_LOG
#include "bsp.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ----------------------------------------------------------------
 * 模块状态
 * ---------------------------------------------------------------- */

#if UART_LOG_ENABLE
static volatile uint8_t s_log_initialized = 0U;
static volatile uint8_t s_runtime_level   = (uint8_t)UART_LOG_LEVEL;
#endif

/* ----------------------------------------------------------------
 * 公共 API
 * ---------------------------------------------------------------- */

/**
 * 功能：初始化日志模块（置初始化标志，幂等）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void UartLog_Init(void)
{
#if UART_LOG_ENABLE
    if (s_log_initialized) {
        return;
    }
    s_log_initialized = 1U;
#endif
}

/**
 * 功能：反初始化日志模块（清初始化标志）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void UartLog_Deinit(void)
{
#if UART_LOG_ENABLE
    s_log_initialized = 0U;
#endif
}

/**
 * 功能：查询日志模块是否已初始化。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，非 0=已初始化，0=未初始化或日志被禁用。
 */
int UartLog_IsInitialized(void)
{
#if UART_LOG_ENABLE
    return (int)s_log_initialized;
#else
    return 0;
#endif
}

/**
 * 功能：按格式化串与可变参数表输出日志（不自动换行）。
 * 输入参数：fmt —— 格式化字符串；ap —— 可变参数列表。
 * 输出参数：无
 * 返回值：int，实际写出的字节数（未初始化/禁用返回 0）。
 */
int UartLog_Vprintf(const char *fmt, va_list ap)
{
#if UART_LOG_ENABLE
    if (!s_log_initialized) {
        return 0;
    }
    char buf[UART_LOG_PRINT_BUF];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n <= 0) {
        return 0;
    }
    if ((unsigned)n > sizeof(buf) - 1U) {
        n = (int)(sizeof(buf) - 1U);
    }
    return Bsp_UartLog_Write(buf, (uint32_t)n);
#else
    (void)fmt; (void)ap;
    return 0;
#endif
}

/**
 * 功能：按格式化串与变参输出日志（不自动换行）。
 * 输入参数：fmt —— 格式化字符串；... —— 可变参数。
 * 输出参数：无
 * 返回值：int，实际写出的字节数（未初始化/禁用返回 0）。
 */
int UartLog_Printf(const char *fmt, ...)
{
#if UART_LOG_ENABLE
    va_list ap;
    va_start(ap, fmt);
    int n = UartLog_Vprintf(fmt, ap);
    va_end(ap);
    return n;
#else
    (void)fmt;
    return 0;
#endif
}

/**
 * 功能：按格式化串与可变参数表输出日志并补足 CRLF 换行。
 * 输入参数：fmt —— 格式化字符串；ap —— 可变参数列表。
 * 输出参数：无
 * 返回值：int，实际写出的字节数（未初始化/禁用返回 0）。
 */
int UartLog_Vprintln(const char *fmt, va_list ap)
{
#if UART_LOG_ENABLE
    if (!s_log_initialized) {
        return 0;
    }

    char buf[UART_LOG_PRINT_BUF];
    const uint32_t cap = (uint32_t)sizeof(buf);

    int n = vsnprintf(buf, cap - 2U, fmt, ap);
    if (n < 0) return 0;
    if ((uint32_t)n > cap - 3U) {
        n = (int)(cap - 3U);
    }

    const int has_crlf = (n >= 2 && buf[n - 2] == '\r' && buf[n - 1] == '\n');
    const int has_lf   = (n >= 1 && buf[n - 1] == '\n');

    if (!has_crlf) {
        if (has_lf) {
            buf[n - 1] = '\r';
            buf[n++]   = '\n';
        } else {
            buf[n++] = '\r';
            buf[n++] = '\n';
        }
    }

    return Bsp_UartLog_Write(buf, (uint32_t)n);
#else
    (void)fmt; (void)ap;
    return 0;
#endif
}

/**
 * 功能：按格式化串与变参输出日志并补足 CRLF 换行。
 * 输入参数：fmt —— 格式化字符串；... —— 可变参数。
 * 输出参数：无
 * 返回值：int，实际写出的字节数（未初始化/禁用返回 0）。
 */
int UartLog_Println(const char *fmt, ...)
{
#if UART_LOG_ENABLE
    va_list ap;
    va_start(ap, fmt);
    int n = UartLog_Vprintln(fmt, ap);
    va_end(ap);
    return n;
#else
    (void)fmt;
    return 0;
#endif
}

/**
 * 功能：以十六进制+ASCII 行格式转储一段数据到日志。
 * 输入参数：tag —— 标题（可空）；data —— 数据指针；len —— 数据字节数。
 * 输出参数：无
 * 返回值：无
 */
void UartLog_Hexdump(const char *tag, const void *data, uint32_t len)
{
#if UART_LOG_ENABLE
    if (!s_log_initialized || data == NULL || len == 0U) {
        return;
    }

    const uint8_t *p = (const uint8_t *)data;
    uint32_t i, j;
    char line[80];

    if (tag != NULL) {
        UartLog_Println("--- %s (%lu bytes) ---", tag, (unsigned long)len);
    }

    for (i = 0U; i < len; i += 16U) {
        int pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "%04lX: ", (unsigned long)i);

        for (j = 0U; j < 16U; j++) {
            if (i + j < len) {
                pos += snprintf(line + pos, sizeof(line) - pos,
                                "%02X ", p[i + j]);
            } else {
                pos += snprintf(line + pos, sizeof(line) - pos, "   ");
            }
        }

        pos += snprintf(line + pos, sizeof(line) - pos, " ");
        for (j = 0U; j < 16U && (i + j) < len; j++) {
            uint8_t c = p[i + j];
            line[pos++] = (c >= 32U && c < 127U) ? (char)c : '.';
        }
        line[pos] = '\0';

        UartLog_Println("%s", line);
    }
#else
    (void)tag; (void)data; (void)len;
#endif
}

/**
 * 功能：设置运行期日志输出等级阈值。
 * 输入参数：level —— 新的日志等级。
 * 输出参数：无
 * 返回值：无
 */
void UartLog_SetRuntimeLevel(uint8_t level)
{
#if UART_LOG_ENABLE
    s_runtime_level = level;
#else
    (void)level;
#endif
}

/**
 * 功能：获取当前运行期日志输出等级阈值。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint8_t，当前日志等级（禁用时返回 0）。
 */
uint8_t UartLog_GetRuntimeLevel(void)
{
#if UART_LOG_ENABLE
    return s_runtime_level;
#else
    return 0;
#endif
}

/**
 * 功能：自测各级日志与 HEX 转储输出（调试/验证用）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void UartLog_TestAll(void)
{
#if UART_LOG_ENABLE
    UART_LOGE("TEST", "error level message");
    UART_LOGW("TEST", "warning level message");
    UART_LOGI("TEST", "info level message");
    UART_LOGD("TEST", "debug level message");
    UART_LOGV("TEST", "verbose level message");

    static const uint8_t test_data[] = {
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
    };
    UartLog_Hexdump("TEST_HEX", test_data, sizeof(test_data));
#else
    (void)0;
#endif
}
