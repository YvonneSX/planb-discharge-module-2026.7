/**
 * @file    bsp_timer.h
 * @brief   步进节拍定时器 + 看门狗喂狗 PWM 驱动（双用 TIMER1）
 *
 * 双用 TIMER1（周期 100µs）：
 *   ① 更新中断 → 分级泄放"一级 100µs 步进 + 二级判定"节拍（回调在 ISR，须极短不阻塞不打印）。
 *   ② CH0 PWM → PA15，作外部看门狗 TPS3820 喂狗方波（50%，10kHz，硬件自主）。
 * TIMER0 已占用于泄放 PWM，PA15 唯一定时器映射为 TIMER1_CH0，故双用 TIMER1。
 *
 * 调用链：TIMER1 更新中断 → it.c TIMER1_IRQHandler → Bsp_StepTimer_Irq() → 注册的回调。
 * 喂狗语义（用户定案）：喂狗=硬件PWM心跳保持泄放模块使能；停喂=泄放功能失效（详见 .c 头注释）。
 *
 * @version 2.0.0
 * @date    2026-07-08
 */

#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*Bsp_StepTimer_Cb_T)(void);   /* 节拍回调（ISR 上下文，须极短不阻塞） */

/**
 * @brief 初始化双用 TIMER1：period_us 周期更新中断（步进节拍）+ CH0 PWM(50%) 到 PA15(喂狗)。
 *        上电默认开启喂狗 PWM。
 * @param period_us 节拍/PWM 周期（µs），如 100（→10kHz 喂狗方波）。
 * @param cb        每个节拍在 ISR 中调用的回调（可空则仅计数不回调）。
 */
void Bsp_StepTimer_Init(uint32_t period_us, Bsp_StepTimer_Cb_T cb);

/**
 * @brief 喂狗 PWM 通断（PA15）。1=输出 PWM 喂狗(泄放使能)；0=拉低静止停喂(泄放功能失效)。
 *        正式固件恒开；测试固件(-DBST_WDG_NOFEED_MS)用于步骤1"停喂观测 A9"。
 */
void Bsp_StepTimer_WdgFeed(uint8_t enable);

/** @brief 停用步进节拍中断（关中断，保留 PWM 输出）。 */
void Bsp_StepTimer_Stop(void);

/** @brief 由 it.c 的 TIMER1_IRQHandler 调用：清更新标志并转调回调。 */
void Bsp_StepTimer_Irq(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIMER_H */
