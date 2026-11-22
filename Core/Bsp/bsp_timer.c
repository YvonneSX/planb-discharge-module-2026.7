/**
 * @file    bsp_timer.c
 * @brief   步进节拍定时器 + 外部看门狗喂狗 PWM（双用 TIMER1）
 *
 * 双用 TIMER1（周期 = period_us，默认 100µs）：
 *   ① 更新中断：分级泄放 100µs 步进节拍（Bsp_StepTimer_Irq → 注册回调）。
 *   ② CH0 PWM 输出到 PA15（TIMER1_CH0 重映射）：外部看门狗 TPS3820 喂狗信号，
 *      50% 占空比、频率=1/period（100µs→10kHz），硬件自主产生，边沿远密于 TPS3820 超时窗。
 *
 * ★看门狗喂狗语义（用户定案，V2）：喂狗 = 硬件 PWM 心跳，让泄放模块保持"正常使能"状态
 *   （TPS3820 得到边沿→不超时→SGM2554 门控 VCC_Safety_3V3 常通→栅驱动有电→泄放可用）。
 *   停喂（PA15 静止）= 泄放功能失效（硬件通路被禁）。这与"软件关泄放(PWM 占空比=0 的正常控制)"
 *   是两回事。本喂狗为硬件 PWM 自主产生，不承担"软件死机检测"（旧 REQ-SAFE-002 语义已按此调整）。
 *
 * PA15 默认为 JTDI，须先 SWJ 重映射释放，再 TIMER1 重映射到 CH0=PA15。
 * TIMER1 挂 APB1；GD32F30x 当 APB1 分频≠1 时定时器时钟=APB1×2，据分频判定 ×2。
 *
 * @version 2.0.0
 * @date    2026-07-08
 */

#define BSP_USE_TIMER
#include "bsp.h"
#include "bsp_timer.h"

#ifndef BSP_STEP_TIMER
#define BSP_STEP_TIMER          TIMER1
#endif
#ifndef BSP_STEP_TIMER_RCU
#define BSP_STEP_TIMER_RCU      RCU_TIMER1
#endif
#ifndef BSP_STEP_TIMER_IRQN
#define BSP_STEP_TIMER_IRQN     TIMER1_IRQn
#endif
#ifndef BSP_STEP_TIMER_TICK_HZ
#define BSP_STEP_TIMER_TICK_HZ  1000000U   /* 计数时钟 1MHz → 1µs/tick */
#endif
/* 看门狗喂狗 PWM 输出脚：PA15 = TIMER1_CH0（部分重映射0：CH0/ETR→PA15） */
#ifndef BSP_WDG_PWM_PORT
#define BSP_WDG_PWM_PORT        GPIOA
#endif
#ifndef BSP_WDG_PWM_PIN
#define BSP_WDG_PWM_PIN         GPIO_PIN_15
#endif

static Bsp_StepTimer_Cb_T s_step_cb = NULL;
static volatile uint8_t   s_step_inited = 0U;

/**
 * 功能：初始化双用 TIMER1——period_us 周期更新中断（步进节拍）+ CH0 PWM(50%) 输出到 PA15(喂狗)。
 *       上电默认开启喂狗 PWM（泄放模块使能）。幂等。
 * 输入参数：period_us —— 节拍/PWM 周期（µs，如 100→10kHz）；cb —— 更新中断回调（可空）。
 * 输出参数：无
 * 返回值：无
 */
void Bsp_StepTimer_Init(uint32_t period_us, Bsp_StepTimer_Cb_T cb)
{
    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocintpara;
    uint32_t timer_clk;
    uint32_t prescaler;
    uint32_t period_ticks;
    uint32_t pulse;

    s_step_cb = cb;

    rcu_periph_clock_enable(BSP_STEP_TIMER_RCU);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    timer_deinit(BSP_STEP_TIMER);

    /* PA15 默认 JTDI：SWJ 重映射(保留 SWDIO/SWCLK)释放 JTDI，再 TIMER1 重映射 CH0→PA15 */
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP0, ENABLE);

    /* TIMER1 实际时钟（APB1 分频≠1 时为 APB1×2） */
    timer_clk = rcu_clock_freq_get(CK_APB1);
    {
        uint32_t apb1_psc = (RCU_CFG0 & RCU_CFG0_APB1PSC) >> 8;
        if ((apb1_psc & 0x4U) != 0U) {     /* 分频≠1 → 定时器时钟 ×2 */
            timer_clk <<= 1;
        }
    }

    /* 预分频到 1MHz 计数时钟：psc = timer_clk/1MHz - 1 */
    prescaler = (timer_clk / BSP_STEP_TIMER_TICK_HZ);
    if (prescaler > 0U) { prescaler -= 1U; }

    /* period_us 个 1µs tick 一个周期 */
    period_ticks = period_us;
    if (period_ticks > 0U) { period_ticks -= 1U; }

    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = (uint16_t)prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period_ticks;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0U;
    timer_init(BSP_STEP_TIMER, &timer_initpara);

    /* —— CH0 PWM：50% 占空比，输出到 PA15 作看门狗喂狗方波 —— */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(BSP_STEP_TIMER, TIMER_CH_0, &timer_ocintpara);

    pulse = (period_ticks + 1U) / 2U;   /* 50% 占空比 */
    timer_channel_output_pulse_value_config(BSP_STEP_TIMER, TIMER_CH_0, pulse);
    timer_channel_output_mode_config(BSP_STEP_TIMER, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(BSP_STEP_TIMER, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_auto_reload_shadow_enable(BSP_STEP_TIMER);

    /* —— 更新中断：步进节拍 —— */
    timer_interrupt_flag_clear(BSP_STEP_TIMER, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(BSP_STEP_TIMER, TIMER_INT_UP);
    /* 步进中断优先级：低于过压看门狗(更急)，高于一般外设 */
    nvic_irq_enable(BSP_STEP_TIMER_IRQN, 2U, 0U);

    timer_enable(BSP_STEP_TIMER);

    /* 上电默认开启喂狗 PWM（PA15 作 AF 输出）——泄放模块使能 */
    Bsp_StepTimer_WdgFeed(1U);

    s_step_inited = 1U;
}

/**
 * 功能：喂狗 PWM 通断控制（PA15）。1=PA15 作 TIMER1_CH0 AF 输出 PWM（喂狗、泄放使能）；
 *       0=PA15 改普通输出并拉低静止（停喂、泄放功能失效，测试步骤1观测 A9 用）。
 *       正式固件恒开（Init 已置 1）；测试固件（-DBST_WDG_NOFEED_MS）上电前段调 0、到时调 1。
 * 输入参数：enable —— 非0=喂狗，0=停喂。
 * 输出参数：无
 * 返回值：无
 */
void Bsp_StepTimer_WdgFeed(uint8_t enable)
{
    if (enable != 0U) {
        gpio_init(BSP_WDG_PWM_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BSP_WDG_PWM_PIN);
    } else {
        gpio_init(BSP_WDG_PWM_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BSP_WDG_PWM_PIN);
        gpio_bit_reset(BSP_WDG_PWM_PORT, BSP_WDG_PWM_PIN);
    }
}

/**
 * 功能：停用步进节拍中断（关更新中断，保留 PWM 输出与配置）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_StepTimer_Stop(void)
{
    timer_interrupt_disable(BSP_STEP_TIMER, TIMER_INT_UP);
}

/**
 * 功能：TIMER1 更新中断服务（由 it.c TIMER1_IRQHandler 调用）：清标志 + 转调回调。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_StepTimer_Irq(void)
{
    if (timer_interrupt_flag_get(BSP_STEP_TIMER, TIMER_INT_FLAG_UP) == SET) {
        timer_interrupt_flag_clear(BSP_STEP_TIMER, TIMER_INT_FLAG_UP);
        if (s_step_cb != NULL) { s_step_cb(); }
    }
}
