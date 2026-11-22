/**
 * @file    bsp.c
 * @brief   BSP 系统初始化入口（GD32F303CCT6 泄放模块）
 *
 * 依次初始化：时钟确认 → Systick → 调试串口 → 外部看门狗。
 * main.c 仅需调用 Bsp_Init() 即可完成全部硬件初始化。
 *
 * 基于 PlanbPro GD32H75E bsp.c 适配 GD32F30x。
 *
 * @version 1.0.0
 * @date    2026-06-11
 */

#define BSP_USE_UART_LOG
#define BSP_USE_WDG
#include "bsp.h"

#include "systick.h"

/* -------------------------------------------------------------------
 * 系统时钟参数
 * GD32F303CCT6（HD 系列）：默认 25MHz HXTAL → PLL(÷1×4) → 100MHz SYSCLK
 * 时钟方案由 system_gd32f30x.c 顶部的 BSP_CLK_PLAN 开关选择：
 *   BSP_CLK_PLAN_25M_100M  ：外部 25M  ÷1×4  = 100MHz（当前默认）
 *   BSP_CLK_PLAN_8M_120M   ：外部 8M   ÷2×30 = 120MHz（换 8M 晶振后启用，
 *                            同时把 Makefile 的 HXTAL_VALUE 改为 8000000U）
 *   BSP_CLK_PLAN_IRC8M_120M：内部 IRC8M÷2×30 = 120MHz（无外部晶振兜底）
 * 实际主频以 SystemCoreClock 为准（由 SystemCoreClockUpdate 从寄存器回算）。
 * 注：HD 系列 PREDV0 仅 ÷1/÷2，外部 25M 经单级 PLL 配不出精确 120MHz，故先用 100MHz。
 * ------------------------------------------------------------------- */

/* 外部晶振频率（Hz）—— 须与 Makefile 的 -DHXTAL_VALUE 保持一致 */
#define BSP_HXTAL_FREQ_HZ       8000000U

/**
 * 功能：简单忙等软件延时（非精确，仅用于上电稳定等场景）。
 * 输入参数：loops —— 空循环次数。
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_SoftDelay(uint32_t loops)
{
    while (loops > 0U) {
        loops--;
    }
}

/**
 * 功能：BSP 系统初始化总入口，依次完成时钟稳定等待、Systick、调试串口、外部看门狗初始化。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Init(void)
{
    /* 1. 时钟：已由 startup → SystemInit() 完成，此处仅做软延时等待稳定 */
    Bsp_SoftDelay(10000U);

    /* 2. Systick：提供 delay_1ms() */
    systick_config();

    /* 3. 调试串口：必须先初始化，后续所有日志依赖它 */
#if defined(BSP_USE_UART_LOG)
    (void)Bsp_UartLog_Init();
#endif

    /* 4. 外部看门狗：初始化 WDI 喂狗 GPIO（喂狗周期由应用层管理） */
#if defined(BSP_USE_WDG)
    (void)Bsp_Wdg_Init();
#endif
}

/**
 * 功能：获取当前系统主频。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint32_t，系统时钟频率（Hz）。
 */
uint32_t Bsp_Clock_GetSysclkFreq(void)
{
    return SystemCoreClock;
}
