/*!
    \file    main.c
    \brief   USART printf

    \version 2025-08-20, V3.0.2, demo for GD32F30x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"

#define BSP_USE_UART_LOG
#include "bsp.h"
#include "uart_log.h"
#include "systick.h"
#include "app.h"
#include <stdio.h>

#if defined(BSP_UNIT_TEST)
#include "test_runner.h"
#endif

/* ===================================================================
 * 时钟/主循环诊断探针 (PB7)
 * -------------------------------------------------------------------
 * 把 PB7 (原 USART0 RX) 配成普通推挽输出, 主循环用 systick_get_ms()
 * 非阻塞状态机每 100ms 翻转一次 => PB7 输出周期 200ms (100ms高+100ms低)
 * 的方波。用示波器/逻辑分析仪测量:
 *   - 能测到方波  => 主循环在跑、SysTick(系统时钟)正常计数;
 *   - 周期 = 200ms => 系统时钟频率与预期(100MHz)一致;
 *     若周期偏大/偏小, 说明实际主频与 SystemCoreClock 设定不符。
 * 注意: 开启探针会占用 PB7, 此时 UART 接收(RX)失效, 但发送(TX=PB6)
 *       与日志打印不受影响。诊断完成后置 0 即可恢复 RX。
 * =================================================================== */
#define PROBE_PB7_ENABLE        0

#define PROBE_PORT              GPIOB
#define PROBE_PIN               GPIO_PIN_7
#define PROBE_RCU               RCU_GPIOB
#define PROBE_TOGGLE_MS         100U     /* 翻转周期 (ms) */

#if PROBE_PB7_ENABLE
/* 探针 GPIO 初始化: PB7 推挽输出 50MHz, 初始低电平 */
static void Probe_Init(void)
{
    rcu_periph_clock_enable(PROBE_RCU);
    gpio_init(PROBE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, PROBE_PIN);
    gpio_bit_reset(PROBE_PORT, PROBE_PIN);
}
#endif

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* ===== 早期裸探针 (在任何时钟/外设配置之前) =====
     * CPU 上电默认跑内部 IRC8M(8MHz), 不依赖外部晶振、不依赖PLL。
     * 直接配 PB7 推挽输出并用软件延时(nop)翻转 20 次快速方波。
     * 只要固件成功烧入并复位运行, 这段就一定能让 PB7 抖动,
     * 与时钟配置、晶振、串口、SysTick 全部无关。
     *   - 示波器看到这串早期快速跳变 => 固件确实在跑, 问题在后面(时钟/晶振);
     *   - 完全没有                   => 固件没烧进 / 没复位 / 复位即挂(硬件)。 */
    /* ===== 最早安全动作: 强制泄放栅极(PA8)为低 =====
     * 复位/上电后第一时间把 PA8 配成推挽输出并拉低, 命令栅驱动关断(MOSFET 截止),
     * 缩小"复位→Bsp_Discharge_Init"之间 PA8 浮空的危险窗口。
     * 注意: CPU 被调试器 halt 时本代码不执行, 该状态下的安全须由硬件
     *       (栅极下拉 / TIMER 刹车 BKIN / 外部看门狗 TPS3820→SGM2554 门控)保证。 */
    {
        rcu_periph_clock_enable(RCU_GPIOA);
        gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
        gpio_bit_reset(GPIOA, GPIO_PIN_8);
    }

    {
        volatile uint32_t i;
        uint32_t n;
        rcu_periph_clock_enable(RCU_GPIOB);
        gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
        for (n = 0U; n < 20U; n++) {
            gpio_bit_set(GPIOB, GPIO_PIN_7);
            for (i = 0U; i < 100000U; i++) { __asm volatile("nop"); }
            gpio_bit_reset(GPIOB, GPIO_PIN_7);
            for (i = 0U; i < 100000U; i++) { __asm volatile("nop"); }
        }
    }

    /* BSP 系统初始化 (时钟 → Systick → 调试串口) */
    Bsp_Init();

    /* 日志系统初始化 */
    UartLog_Init();

#if UART_LOG_ENABLE
    UartLog_Println("[MAIN] bsp+uart ready, sysclk=%luHz",
                    (unsigned long)Bsp_Clock_GetSysclkFreq());
#endif
#if defined(BSP_UNIT_TEST)
#if defined(BSP_MONITOR)
    /* 实时监视模式: 循环采集输入(模拟量/反馈/RX/CAN), 供改变物理输入时随动观测;
     * 监视对象由编译宏 MONITOR_xxx / CAN_INTERACTIVE_TEST 选定 (见 make *-live 目标) */
    UartLog_Println("[MAIN] monitor mode enter");
    Test_Runner_RunMonitor();
#else
    /* 单元测试模式: 运行全部已启用的 BSP 单元测试, 结束后停在此处 */
    (void)Test_Runner_RunAll();
#endif
    while (1) {
    }
#endif

    /* 冒烟测试: 输出各日志级别 */
    UartLog_TestAll();

    /* ===== App 业务层集成 ===== */
    /* 按依赖序初始化全部 App 与 BSP 模块（全程泄放关断），完成后发上电通告帧 */
    App_Init();

    /* 主循环：单圈非阻塞任务序（量测→保护→泄放→通信→黑匣子→安全→喂狗）。
     * CAN 从站(app_comm)在此响应主站命令(0x01~0x33)：主站轮询 0x20~0x24 读状态、
     * 0x30/0x10/0x31 控制等，从站按泄放协议应答。 */
    while (1) {
        App_Run();
    }
}
