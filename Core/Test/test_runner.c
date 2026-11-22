/**
 * @file    test_runner.c
 * @brief   BSP 单元测试总控 — 统一 init + 各 BSP 测试回调(宏开关管理)
 *
 * 设计:
 *   - 每个 BSP 模块的单元测试入口(Test_BspXxx_RunAll)视为一个"测试回调";
 *   - 本文件用 TEST_ENABLE_xxx 宏开关管理各回调是否纳入执行;
 *   - 统一的测试初始化(日志等)集中在 Test_Runner_Init();
 *   - main.c 只需调用 Test_Runner_RunAll() 一个总入口。
 *
 * 使用 (main.c 中用编译宏隔离):
 *   #if defined(BSP_UNIT_TEST)
 *       Test_Runner_RunAll();
 *   #endif
 * 编译: make all EXTRA_CFLAGS="-DBSP_UNIT_TEST -DUART_LOG_ENABLE=1"
 * 单独测某模块: 追加 -DTEST_ENABLE_ANALOG=1 -DTEST_ENABLE_CAN=0 等覆盖默认。
 *
 * @version 1.0.0
 * @date    2026-06-12
 */

#include "test_runner.h"

/* 各 BSP 模块测试回调声明 */
#include "test_bsp_uart_log.h"
#include "test_bsp_analog.h"
#include "test_bsp_discharge.h"
#include "test_bsp_can.h"
#include "test_bsp_spi.h"
#include "test_bsp_fram.h"
#include "test_bsp_wdg.h"
#include "test_bsp_flash.h"
#include "test_bsp_config.h"
#include "test_ovp_logic.h"
#include "test_canx_proto.h"
#include "test_canx_port.h"
#include "test_canx_slave.h"
#include "test_brake_special.h"

/* 统一初始化用到的 BSP */
#define BSP_USE_UART_LOG
#include "bsp.h"
#include "uart_log.h"

/* ================================================================
 * 测试回调表 — 通过 TEST_ENABLE_xxx 宏开关纳入/排除每个模块
 * 默认全开(=1); 编译时可用 -DTEST_ENABLE_xxx=0 单独关闭
 * ================================================================ */
typedef void (*Test_Callback_T)(Test_Stats_T *);

typedef struct {
    const char     *name;       /* 模块名(日志用) */
    Test_Callback_T run;        /* 测试回调 */
} Test_Entry_T;

static const Test_Entry_T s_test_table[] = {
#if TEST_ENABLE_UART_LOG
    { "UART_LOG",  Test_BspUartLog_RunAll  },
#endif
#if TEST_ENABLE_ANALOG
    { "ANALOG",    Test_BspAnalog_RunAll   },
#endif
#if TEST_ENABLE_DISCHARGE
    { "DISCHARGE", Test_BspDischarge_RunAll },
#endif
#if TEST_ENABLE_CAN
    { "CAN",       Test_BspCan_RunAll      },
#endif
#if TEST_ENABLE_SPI
    { "SPI",       Test_BspSpi_RunAll      },
#endif
#if TEST_ENABLE_FRAM
    { "FRAM",      Test_BspFram_RunAll     },
#endif
#if TEST_ENABLE_WDG
    { "WDG",       Test_BspWdg_RunAll      },
#endif
#if TEST_ENABLE_FLASH
    { "FLASH",     Test_BspFlash_RunAll    },
#endif
#if TEST_ENABLE_CONFIG
    { "CONFIG",    Test_BspConfig_RunAll   },
#endif
#if TEST_ENABLE_OVP
    { "OVP_LOGIC", Test_OvpLogic_RunAll    },
#endif
#if TEST_ENABLE_CANX
    { "CANX_PROTO", Test_CanxProto_RunAll  },
#endif
#if TEST_ENABLE_CANX_PORT
    { "CANX_PORT",  Test_CanxPort_RunAll   },
#endif
#if TEST_ENABLE_CANX_SLAVE
    { "CANX_SLAVE", Test_CanxSlave_RunAll  },
#endif
};

#define TEST_TABLE_LEN  (sizeof(s_test_table) / sizeof(s_test_table[0]))

/* ================================================================
 * 统一测试初始化 — 测试运行前的公共准备(日志等)
 * ================================================================ */
void Test_Runner_Init(void)
{
    /* 调试串口/日志初始化, 供所有测试输出结果 */
    (void)Bsp_UartLog_Init();
    UartLog_Init();
}

/* ================================================================
 * 测试总入口 — 遍历回调表执行, 汇总统计
 * ================================================================ */
uint32_t Test_Runner_RunAll(void)
{
    Test_Stats_T stats = {0U, 0U, 0U};
    uint32_t i;

    Test_Runner_Init();

    TEST_LOG("==== 泄放模块 BSP 单元测试开始 (共 %lu 模块) ====",
             (unsigned long)TEST_TABLE_LEN);

    for (i = 0U; i < TEST_TABLE_LEN; i++) {
        TEST_LOG("---- [%lu/%lu] %s ----",
                 (unsigned long)(i + 1U), (unsigned long)TEST_TABLE_LEN,
                 s_test_table[i].name);
        s_test_table[i].run(&stats);
    }

    TEST_LOG("==== 测试结束: 总计 %lu, 通过 %lu, 失败 %lu ====",
             (unsigned long)stats.total,
             (unsigned long)stats.passed,
             (unsigned long)stats.failed);

    return stats.failed;
}

/* ================================================================
 * 实时监视模式入口 — 按编译宏分发到对应模块的连续采集循环
 * ================================================================ */
void Test_Runner_RunMonitor(void)
{
    Test_Runner_Init();

#if defined(MONITOR_ANALOG)
    Test_BspAnalog_Monitor();
#elif defined(MONITOR_DISCHARGE)
    Test_BspDischarge_Monitor();
#elif defined(MONITOR_UART)
    Test_BspUartLog_Monitor();
#elif defined(CAN_INTERACTIVE_TEST)
    Test_BspCan_Interactive();
#elif defined(MONITOR_BRAKE_SPECIAL)
    Test_BrakeSpecial_Monitor();
#else
    TEST_LOG("未指定监视对象, 请用 make analog-live / discharge-live / uart-live / can-loop / brake-special");
#endif
}
