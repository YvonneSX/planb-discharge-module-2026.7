/**
 * @file    uart_log.h
 * @brief   UART 日志模块头文件 — 分级打印 + newlib _write 重定向
 *
 * 基于 PlanbPro GD32H75E uart_log.h 适配 GD32F303CCT6。
 *
 * 调用链:
 *   UART_LOG* 宏 / UartLog_Println()
 *       → Bsp_UartLog_Write()  (BSP 层)
 *       → USART0 硬件发送      (硬件层)
 *
 * 设计要点:
 *   - 零堆分配: 所有格式化使用栈上静态缓冲区 (UART_LOG_PRINT_BUF);
 *   - 编译时裁剪: UART_LOG_ENABLE=0 时所有宏展开为 ((void)0);
 *   - newlib 重定向: 通过 _write() 让传统 printf() 也走 UART。
 *
 * @version 1.0.0
 * @date    2026-06-11
 */

#ifndef UART_LOG_H
#define UART_LOG_H

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 * 主开关与日志级别
 * ==================================================================== */

#ifndef UART_LOG_ENABLE
#define UART_LOG_ENABLE          1
#endif

#define UART_LOG_LEVEL_NONE      0
#define UART_LOG_LEVEL_ERROR     1
#define UART_LOG_LEVEL_WARN      2
#define UART_LOG_LEVEL_INFO      3
#define UART_LOG_LEVEL_DEBUG     4
#define UART_LOG_LEVEL_VERBOSE   5

/* 编译时最大详细程度 */
#ifndef UART_LOG_LEVEL
#define UART_LOG_LEVEL           UART_LOG_LEVEL_INFO
#endif

/* ANSI 颜色开关 (默认关闭，sscom/多数串口助手不支持 ANSI) */
#ifndef UART_LOG_COLOR
#define UART_LOG_COLOR           0
#endif

/* 内部 vsnprintf 缓冲区大小 */
#ifndef UART_LOG_PRINT_BUF
#define UART_LOG_PRINT_BUF       192U
#endif

/* ====================================================================
 * 核心 API
 * ==================================================================== */

void    UartLog_Init(void);
void    UartLog_Deinit(void);
int     UartLog_IsInitialized(void);

int     UartLog_Printf(const char *fmt, ...);
int     UartLog_Vprintf(const char *fmt, va_list ap);

int     UartLog_Println(const char *fmt, ...);
int     UartLog_Vprintln(const char *fmt, va_list ap);

void    UartLog_Hexdump(const char *tag, const void *data, uint32_t len);

void    UartLog_SetRuntimeLevel(uint8_t level);
uint8_t UartLog_GetRuntimeLevel(void);

void    UartLog_TestAll(void);

/* ====================================================================
 * 分级日志宏
 * ==================================================================== */

#if UART_LOG_ENABLE

#if UART_LOG_COLOR
#define UART_LOG_CLR_E   "\033[31m"   /* 红色 */
#define UART_LOG_CLR_W   "\033[33m"   /* 黄色 */
#define UART_LOG_CLR_I   "\033[32m"   /* 绿色 */
#define UART_LOG_CLR_D   "\033[36m"   /* 青色 */
#define UART_LOG_CLR_V   "\033[37m"   /* 白色 */
#define UART_LOG_CLR_RST "\033[0m"
#else
#define UART_LOG_CLR_E   ""
#define UART_LOG_CLR_W   ""
#define UART_LOG_CLR_I   ""
#define UART_LOG_CLR_D   ""
#define UART_LOG_CLR_V   ""
#define UART_LOG_CLR_RST ""
#endif

#if UART_LOG_LEVEL >= UART_LOG_LEVEL_ERROR
#define UART_LOGE(tag, fmt, ...) \
    UartLog_Println(UART_LOG_CLR_E "[%s] E: " fmt UART_LOG_CLR_RST, tag, ##__VA_ARGS__)
#else
#define UART_LOGE(tag, fmt, ...) ((void)0)
#endif

#if UART_LOG_LEVEL >= UART_LOG_LEVEL_WARN
#define UART_LOGW(tag, fmt, ...) \
    UartLog_Println(UART_LOG_CLR_W "[%s] W: " fmt UART_LOG_CLR_RST, tag, ##__VA_ARGS__)
#else
#define UART_LOGW(tag, fmt, ...) ((void)0)
#endif

#if UART_LOG_LEVEL >= UART_LOG_LEVEL_INFO
#define UART_LOGI(tag, fmt, ...) \
    UartLog_Println(UART_LOG_CLR_I "[%s] I: " fmt UART_LOG_CLR_RST, tag, ##__VA_ARGS__)
#else
#define UART_LOGI(tag, fmt, ...) ((void)0)
#endif

#if UART_LOG_LEVEL >= UART_LOG_LEVEL_DEBUG
#define UART_LOGD(tag, fmt, ...) \
    UartLog_Println(UART_LOG_CLR_D "[%s] D: " fmt UART_LOG_CLR_RST, tag, ##__VA_ARGS__)
#else
#define UART_LOGD(tag, fmt, ...) ((void)0)
#endif

#if UART_LOG_LEVEL >= UART_LOG_LEVEL_VERBOSE
#define UART_LOGV(tag, fmt, ...) \
    UartLog_Println(UART_LOG_CLR_V "[%s] V: " fmt UART_LOG_CLR_RST, tag, ##__VA_ARGS__)
#else
#define UART_LOGV(tag, fmt, ...) ((void)0)
#endif

#else /* !UART_LOG_ENABLE */

#define UART_LOGE(tag, fmt, ...) ((void)0)
#define UART_LOGW(tag, fmt, ...) ((void)0)
#define UART_LOGI(tag, fmt, ...) ((void)0)
#define UART_LOGD(tag, fmt, ...) ((void)0)
#define UART_LOGV(tag, fmt, ...) ((void)0)

#endif /* UART_LOG_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* UART_LOG_H */
