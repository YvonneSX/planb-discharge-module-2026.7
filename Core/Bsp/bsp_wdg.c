/**
 * @file    bsp_wdg.c
 * @brief   外部看门狗喂狗 —— 【已改由硬件 PWM 接管，本模块降级为空操作】
 *
 * PA15 — MCU_Watchdog_WDI → TPS3820 (外部硬件看门狗)
 *
 * ★V2 变更（用户定案）：喂狗信号改由 TIMER1_CH0 硬件 PWM 直接输出到 PA15
 *   （见 bsp_timer.c / Bsp_StepTimer_Init），50% / 10kHz 方波自主喂狗，
 *   使泄放模块保持"正常使能"状态。故本 GPIO 软件翻转喂狗模块**不再使用**：
 *     - Bsp_Wdg_Init()：空操作（不再配置 PA15，避免与 TIMER1 AF 复用冲突）；
 *     - Bsp_Wdg_Feed()：空操作（喂狗由硬件 PWM 自主产生）。
 *   保留符号以兼容 bsp.c 的初始化调用序，后续可整体移除。
 *
 * 语义说明：喂狗=保持泄放模块正常使能（硬件 PWM 心跳），停喂=泄放功能失效；
 *   与"软件关泄放(占空比0 的正常控制)"是两回事。喂狗停/开的测试控制见
 *   Bsp_StepTimer_WdgFeed()（测试固件 -DBST_WDG_NOFEED_MS 用）。
 *
 * @version 2.0.0
 * @date    2026-07-08
 */

#define BSP_USE_WDG
#include "bsp.h"

static volatile uint8_t s_wdg_initialized = 0U;

/**
 * 功能：【已废弃/空操作】外部看门狗喂狗初始化。喂狗已改由 TIMER1_CH0 硬件 PWM(bsp_timer.c)接管，
 *       本函数不再配置 PA15（避免与 TIMER1 AF 复用冲突）。保留符号。幂等。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Wdg_Status_T，恒 BSP_WDG_OK。
 */
Bsp_Wdg_Status_T Bsp_Wdg_Init(void)
{
    s_wdg_initialized = 1U;
    return BSP_WDG_OK;
}

/**
 * 功能：查询（占位）看门狗驱动是否已"初始化"。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，非 0=已调用过 Init。
 */
int Bsp_Wdg_IsInitialized(void)
{
    return (int)s_wdg_initialized;
}

/**
 * 功能：【已废弃/空操作】喂狗。喂狗由 TIMER1_CH0 硬件 PWM 自主产生，无需软件翻转。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Wdg_Feed(void)
{
    /* 空操作：喂狗信号由 TIMER1_CH0 硬件 PWM(PA15) 自主产生。 */
}
