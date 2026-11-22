/**
 * @file    app_safety.c
 * @brief   HLD-MOD-SAFE 安全/喂狗模块实现
 *
 * 设计要点：安全态直接调用 BSP 原语（Bsp_Discharge_Stop），不经 DCH，
 * 以保证即使 DCH 状态机异常也能强制关断（安全兜底独立于业务层）。
 */

#include "app_safety.h"
#include "uart_log.h"
#include "bsp_timer.h"   /* Bsp_StepTimer_WdgFeed：喂狗 PWM 通断（测试步骤1/2 停喂/开喂） */

#define SAFE_TAG "SAFE"

static bool s_inited = false;
static bool s_in_safe = false;

/**
 * 功能：初始化安全/喂狗模块，清安全态标志并置就绪。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppSafe_Init(void)
{
    s_in_safe = false;
    s_inited  = true;
    UART_LOGI(SAFE_TAG, "init done");
}

/**
 * 功能：喂狗心跳维护（每控制圈调用）。
 *       V2：正式固件喂狗由 TIMER1_CH0 硬件 PWM(PA15) 自主产生（Bsp_StepTimer_Init 已启），
 *       本函数在正式固件中无喂狗动作（喂狗=保持泄放模块使能，不随主循环停）。
 *       测试专用：定义 BST_WDG_NOFEED_MS 时，上电前该毫秒数内停喂（PA15 静止低）供台架观测
 *       "停喂→泄放功能失效→A9(Brake_FB) 电平"（步骤1），到时开 PWM 喂狗（步骤2）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppSafe_Kick(void)
{
    if (!s_inited) { return; }
#ifdef BST_WDG_NOFEED_MS
    /* 测试(步骤1/2)：上电前 N ms 停喂(PA15 静止低)观测 A9；到时开硬件 PWM 喂狗。 */
    Bsp_StepTimer_WdgFeed((systick_get_ms() >= (uint32_t)(BST_WDG_NOFEED_MS)) ? 1U : 0U);
#endif
    /* 正式固件：喂狗由 TIMER1_CH0 硬件 PWM 自主产生，本函数无需喂狗动作。 */
}

/**
 * 功能：进入安全态，立即关断泄放（直接调 BSP，独立于 DCH 状态机），可重复调用。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppSafe_EnterSafeState(void)
{
    Bsp_Discharge_Stop();              /* 直接关断，独立于 DCH 状态机 */
    Bsp_Analog_DisableVBusWatchdog();  /* 关过压看门狗：防安全态下过压中断(AppDch_OvpIsr)又把泄放拉起 */
    if (!s_in_safe) {
        s_in_safe = true;
        UART_LOGW(SAFE_TAG, "enter safe state");
    }
}

/**
 * 功能：查询当前是否已进入安全态。
 * 输入参数：无
 * 输出参数：无
 * 返回值：bool，true=已进入安全态，false=正常。
 */
bool AppSafe_InSafeState(void)
{
    return s_in_safe;
}
