/**
 * @file    test_framework.h
 * @brief   轻量单元测试框架 (无动态分配, 适用 MCU 在线自测与宿主机测试)
 *
 * 用法:
 *   TEST_CASE("名称") { TEST_ASSERT(cond); ... }  -- 在 RunAll 内组织
 *   每个模块提供 Test_BspXxx_RunAll(Test_Stats_T *), 累加统计。
 *
 * @version 1.0.0
 * @date    2026-06-12
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 测试统计 */
typedef struct {
    uint32_t total;     /* 断言总数 */
    uint32_t passed;    /* 通过数 */
    uint32_t failed;    /* 失败数 */
} Test_Stats_T;

/* 日志钩子: 默认走 UART 日志, 宿主机测试可重定义为 printf */
#ifndef TEST_LOG
#include "uart_log.h"
#define TEST_LOG(...)   UartLog_Println(__VA_ARGS__)
#endif

/* 断言宏: 需要当前作用域有 Test_Stats_T *_stats 指针 */
#define TEST_ASSERT(cond)                                                  \
    do {                                                                   \
        (_stats)->total++;                                                 \
        if (cond) {                                                        \
            (_stats)->passed++;                                            \
        } else {                                                           \
            (_stats)->failed++;                                            \
            TEST_LOG("[FAIL] %s:%d  %s", __FILE__, __LINE__, #cond);       \
        }                                                                  \
    } while (0)

#define TEST_ASSERT_EQ(a, b)    TEST_ASSERT((a) == (b))

/* 带实际值打印的整数相等断言: 失败时打印期望值与实际值, 便于真机定位 */
#define TEST_ASSERT_EQ_INT(actual, expect)                                     \
    do {                                                                       \
        long _a = (long)(actual);                                              \
        long _e = (long)(expect);                                              \
        (_stats)->total++;                                                     \
        if (_a == _e) {                                                        \
            (_stats)->passed++;                                                \
        } else {                                                               \
            (_stats)->failed++;                                                \
            TEST_LOG("[FAIL] %s:%d  %s  expect=%ld actual=%ld",                \
                     __FILE__, __LINE__, #actual, _e, _a);                     \
        }                                                                      \
    } while (0)

/* 范围断言: 验证 lo <= val <= hi (闭区间), 失败打印实际值与范围 */
#define TEST_ASSERT_IN_RANGE(val, lo, hi)                                      \
    do {                                                                       \
        long _v = (long)(val);                                                 \
        long _lo = (long)(lo);                                                 \
        long _hi = (long)(hi);                                                 \
        (_stats)->total++;                                                     \
        if (_v >= _lo && _v <= _hi) {                                          \
            (_stats)->passed++;                                                \
        } else {                                                               \
            (_stats)->failed++;                                                \
            TEST_LOG("[FAIL] %s:%d  %s=%ld not in [%ld, %ld]",                 \
                     __FILE__, __LINE__, #val, _v, _lo, _hi);                  \
        }                                                                      \
    } while (0)

/* 内存块相等断言: 比较 len 字节, 失败打印首个不一致的偏移 */
#define TEST_ASSERT_MEM_EQ(buf_a, buf_b, len)                                  \
    do {                                                                       \
        const uint8_t *_pa = (const uint8_t *)(buf_a);                         \
        const uint8_t *_pb = (const uint8_t *)(buf_b);                         \
        uint32_t _n = (uint32_t)(len);                                         \
        uint32_t _k;                                                           \
        int _ok = 1;                                                           \
        (_stats)->total++;                                                     \
        for (_k = 0U; _k < _n; _k++) {                                         \
            if (_pa[_k] != _pb[_k]) { _ok = 0; break; }                        \
        }                                                                      \
        if (_ok) {                                                             \
            (_stats)->passed++;                                                \
        } else {                                                               \
            (_stats)->failed++;                                                \
            TEST_LOG("[FAIL] %s:%d  mem diff @off=%lu a=0x%02X b=0x%02X",      \
                     __FILE__, __LINE__, (unsigned long)_k,                    \
                     _pa[_k], _pb[_k]);                                        \
        }                                                                      \
    } while (0)

/* 子测试分隔日志 */
#define TEST_CASE(name)         TEST_LOG("[CASE] " name)

/* 实时监视模式（输入类模块）循环刷新周期（ms）。
 * 输入采集（ADC 电压/电流/温度、Brake_FB 反馈、串口 RX）需实时观测，
 * 一次性快照无参考价值；监视模式按此周期循环打印，供改变物理输入时随动核对。 */
#ifndef TEST_MONITOR_PERIOD_MS
#define TEST_MONITOR_PERIOD_MS  1000U
#endif

/* 监测值打印: 无论断言成败都打印实测值, 供台架仪器交叉核对。
 * 用法: TEST_LOG_VAL("VBus(mV)=%ld", (long)v); 输出前缀 [VAL] 便于过滤 */
#define TEST_LOG_VAL(fmt, ...)  TEST_LOG("[VAL] " fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* TEST_FRAMEWORK_H */
