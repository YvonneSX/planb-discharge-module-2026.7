/**
 * @file    app_discharge.c
 * @brief   HLD-MOD-DCH 泄放控制模块实现（首版骨架）
 *
 * 分级泄放架构（用户定案，见《过泄压中断动态启停与分级泄放执行设计说明》）：
 *   - AppDch_OvpIsr()：ADC 模拟看门狗“双边窗口”中断回调（µs 级）——起/停/升二级/退一级的唯一执行者。
 *     用瞬时电压(去滤波)判方向；每个态的看门狗窗口边界=该态“下一个要抓的阈值”，故级别切换全部中断化：
 *       L0 待机[0,56.5V]：越上限→ ≥58V 一步到位 L2 100% / 否则 L1 50% 起步；
 *       L1 一级[55V,58V]：越下限<55V→停 / 越上限≥58V→立即升 L2 100%；
 *       L2 二级[56.5V,满量程]：越下限——<55V→停 / 55~56.5V→退回 L1。
 *   - AppDch_StepIsr()：TIMER1 50µs 中断回调，L1 做占空比爬坡（每50µs +10% 封顶100%）+ 上升趋势升级
 *     （连升2步仍高于上一步→直接跳二级100%）；停/退级/58V升级仍归看门狗。写占空比前重读 level 防竞写。L2 静态100%。
 *   - AppDch_Update()：主循环监控层——20s兜底、累计时长；force_full(0x10)/allowed(0x30) 门由两个 ISR 消费。
 * 判据链路全部用瞬时值 AppMon_GetVBus_mV()（不经 EMA 滤波），消除软件滞后。见《泄放中断即时响应优化设计说明》。
 * 阈值/占空比/步进周期均为暂定宏，测试按串口遥测重标。冷却闭锁已移除。
 */

#include "app_discharge.h"
#include "app_monitor.h"
#include "app_protect.h"   /* AppProt_GetFaults：StepIsr 关断类故障门控 */
#include "app_safety.h"    /* AppSafe_InSafeState：StepIsr 安全态门控 */
#include "bsp_timer.h"     /* TIMER1 50µs 节拍（StepIsr 由其驱动） */
#include "uart_log.h"

#define DCH_TAG "DCH"

/* 20s 连续泄放超时兜底开关：正式固件默认开(=1)。
 * 单元测试(brake-special)可 -DAPP_DCH_MAXON_TIMEOUT_ENABLE=0 关闭——测温升需持续给 56V+
 * 远超 20s，兜底会打断。已验证 20s 兜底逻辑正常，正式代码保持开启。 */
#ifndef APP_DCH_MAXON_TIMEOUT_ENABLE
#define APP_DCH_MAXON_TIMEOUT_ENABLE  1
#endif

typedef struct {
    uint8_t  target_duty;   /* 控制字请求的目标占空比（force_full 用；PROT 过压请求 duty 恒 0，占空比 DCH 自管） */
    bool     req_enable;    /* 请求泄放 */
    bool     force_full;    /* 0x10 强制全开 */
    bool     allowed;       /* 0x30 泄放总允许（默认 true） */
    volatile bool on;       /* 当前是否输出（看门狗ISR起停 + TIMER1/主循环共享，volatile 防缓存） */
    volatile uint8_t level; /* 泄放级别：0=停 1=一级(爬升) 2=二级(100%)。看门狗ISR写起停(0↔非0)、StepIsr写爬/切级，遥测/主循环只读 */
    volatile uint32_t on_t0;/* 本次连续开通起始时间戳（看门狗起泄放时写，主循环20s判定读） */
    volatile bool timed_out;/* 20s 连续泄放超时已触发（主循环置位，StepIsr/OvpIsr 读；母线跌到起泄放阈值56.5V以下才清） */
    uint32_t total_ms;      /* 累计泄放时长 */
    uint32_t last_tick;     /* 上次 Update 时间戳（累计用） */
    uint32_t step_t0;       /* 上次步进检测时间戳（100ms 闭环步进） */
    int32_t  l1_v_prev;     /* L1 趋势判断：上一步进节拍的瞬时母线电压(mV)。进 L1 时置当前值 */
    uint8_t  l1_rise_cnt;   /* L1 趋势判断：连续"较上一步上升"的节拍数；达 N 步升二级 100% */
    bool     inited;
} Dch_Ctx_T;

static Dch_Ctx_T s_ctx;

/* —— 泄放指示灯(PB0)标志（= 用户“泄放完成标志位”）——
 *   语义：1=泄放中(灯亮) / 0=停止或完成(灯灭)。唯一写点：dch_apply_duty(随起/停)、ForceStop、
 *   以及 AppDch_ServiceDchLed 的 2s<55V 兜底。ISR 写、主循环读/兜底写，故 volatile。 */
static volatile uint8_t s_dch_led_active = 0U;
static uint32_t         s_dch_led_c_ms   = 0U;   /* 最小亮灯/延时熄灭基准：最后一次“泄放中”的时刻（仅主循环访问） */
static uint32_t         s_dch_led_d_ms   = 0U;   /* 2s<55V 兜底清标志基准：连续<55V 起点（仅主循环访问） */

/* —— 上电版本识别闪灯（仅主循环访问）——：cnt>0 期间由 AppDch_ServicePowerOnBlink 独占 PB0，闪完让位泄放灯。 */
static uint8_t          s_pon_cnt        = 0U;   /* 剩余翻转次数（初始 BSP_DCH_LED_PON_BLINK_COUNT，减到0结束） */
static uint32_t         s_pon_e_ms       = 0U;   /* 上次翻转时间戳 */
static uint8_t          s_pon_led_on     = 1U;   /* 本次要设置的灯状态：1=亮/0=灭（从“亮”起） */

static void dch_session_reset(void);
static void dch_session_begin(void);
static void dch_apply_duty(uint8_t duty, uint8_t level);
static void dch_set_wdg_window_for_level(uint8_t level);
static bool dch_start_blocked(void);
static void dch_l1_trend_reset(int32_t v);

/**
 * 功能：初始化泄放控制模块，关断输出并复位会话上下文（允许态默认开）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppDch_Init(void)
{
    Bsp_Discharge_Stop();
    s_ctx.target_duty = 0U;
    s_ctx.req_enable  = false;
    s_ctx.force_full  = false;
    s_ctx.allowed     = true;
    s_ctx.on          = false;
    s_ctx.level       = 0U;
    s_ctx.on_t0       = 0U;
    s_ctx.timed_out   = false;
    s_ctx.total_ms    = 0U;
    s_ctx.last_tick   = systick_get_ms();
    s_ctx.step_t0     = 0U;
    s_ctx.l1_v_prev   = 0;
    s_ctx.l1_rise_cnt = 0U;
    s_dch_led_active  = 0U;                 /* 泄放灯标志上电=0(灭) */
    /* c_ms 置为“已过最小亮灯窗”，保证上电即灭、不因初值导致开机亮 1s */
    s_dch_led_c_ms    = systick_get_ms() - (uint32_t)BSP_DCH_LED_MIN_ON_MS;
    s_dch_led_d_ms    = systick_get_ms();
    /* 上电版本识别闪灯：从“亮”起翻转 N 次，200ms 一次，末次停在灭 */
    s_pon_cnt         = (uint8_t)BSP_DCH_LED_PON_BLINK_COUNT;
    s_pon_e_ms        = systick_get_ms();
    s_pon_led_on      = 1U;
    s_ctx.inited      = true;
    UART_LOGI(DCH_TAG, "init done");
}

/**
 * 功能：接收 PROT/控制字下发的泄放请求（目标占空比与使能）。
 * 输入参数：duty_pct —— 目标占空比（%）；enable —— 是否请求泄放。
 * 输出参数：无
 * 返回值：无
 */
void AppDch_SetRequest(uint8_t duty_pct, bool enable)
{
    s_ctx.target_duty = duty_pct;
    s_ctx.req_enable  = enable;
}

/**
 * 功能：主循环监控层（每控制圈调用）——不再直接步进占空比（步进已移入 StepIsr）。
 *       职责：累计泄放时长、20s 连续泄放超时兜底（非锁存）、force_full/allowed 门由 StepIsr 消费。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppDch_Update(void)
{
	//uint16_t itmp = 0;
    if (!s_ctx.inited) { return; }

    const uint32_t now = systick_get_ms();
    const uint32_t dt  = (uint32_t)(now - s_ctx.last_tick);
    s_ctx.last_tick = now;

    /* 泄放中累计时长（level!=0 即在泄放） */
    if (s_ctx.level != 0U) {
        s_ctx.total_ms += dt;
    }
	//itmp = Bsp_Analog_ReadRawTemp();
	
	//UART_LOGI(DCH_TAG,"ntc adc = %d", itmp) ;
#if APP_DCH_MAXON_TIMEOUT_ENABLE
    /* —— 20s 连续泄放超时兜底（只统计“母线持续 ≥56.5V 压不下来的连续泄放”）——
     * 清零/解锁判据：只要母线【瞬时】电压 < 起泄放阈值(56.5V, BSP_DCH_L1_ON_MV) 就立即
     *   dch_session_reset()（on_t0=0 计时清零 + 解除 timed_out 闭锁），且不看 level。
     *   含义：母线跌到起泄放阈值以下＝过压已消除、不再需要泄放＝本次连续泄放会话结束。
     *   ——用【起泄放阈值】而非 55V 停泄放阈值作解锁线：开关电源等硬源把电压夹在 55~57V 之间时，
     *     旧的“≤55V 才解锁”永远等不到，20s 超时闭锁后一直卡死（56V→57V 也不再起泄放）；
     *     改用 <56.5V 后，只要降到起泄放阈值以下就解锁，再冲高才重新起泄放。
     *   用瞬时值(AppMon_GetVBus_mV)而非滤波值：ping-pong 快抖时 EMA 会被平均、抓不到瞬时跌破。
     * 计时：on_t0 仅由 dch_session_begin() 在起泄放时写；L1↔L2 切换不重置。
     * 触发：只有母线连续 ≥56.5V（真的 20s 都压不到起泄放阈值以下）才判超时停并闭锁。 */
    if (AppMon_GetVBus_mV() < (int32_t)BSP_DCH_L1_ON_MV) {
        dch_session_reset();
    } else if (s_ctx.level != 0U && !s_ctx.timed_out && s_ctx.on_t0 != 0U &&
        (uint32_t)(now - s_ctx.on_t0) >= BSP_OVP_MAX_ON_MS) {
        s_ctx.timed_out = true;
        /* 打印实际计时毫秒数(now-on_t0)：应≈BSP_OVP_MAX_ON_MS(20000)。
         * 若壁钟只有~15s 但此处仍打印~20000ms → 是时钟；若壁钟20s此处才~20000 → 计时准，
         * 那么之前"15s停"是别的原因(过温/复位/看门狗)而非本兜底。 */
        UART_LOGW(DCH_TAG, "MAXON timeout: elapsed=%lums (limit=%lums, L%u) stop",
                  (unsigned long)(uint32_t)(now - s_ctx.on_t0),
                  (unsigned long)BSP_OVP_MAX_ON_MS,
                  (unsigned)s_ctx.level);
    }
#endif
}

/**
 * 功能：空闲态 A9(Brake_FB) 实时守护——主循环每圈调用（App_Run），实时监测 A9 输入。
 *   A9 电平与泄放引脚 PA8 输出状态一一对应（硬件组确认）：空闲态(未泄放 level==0)时 PA8 应为低、
 *   A9 应为低；若 A9 检测为高(=1)，说明 PA8 被异常拉高，空闲态等效 100% 占空比误泄放（危险）→
 *   立即把 PA8 由 TIMER0 复用切成普通 GPIO 推挽并拉低(Bsp_Discharge_ForcePinLow)，强制 MOSFET 截止。
 *   （泄放态 level!=0 时 PA8 为 20kHz PWM，A9 随之翻转，"高"属正常，故本守护仅空闲态生效。
 *    下一次正常起泄放时 Bsp_Discharge_SetDuty 会自动把 PA8 恢复为 AF PWM 输出。）
 * 输入参数：无
 * 输出参数：无（异常时改写 PA8 引脚模式/电平）
 * 返回值：无
 * 说明：TIMER1 StepIsr 亦有等效的空闲防误泄放守护(50µs 快路径)；本函数为主循环实时兜底。
 */
void AppDch_IdleFbGuard(void)
{
    if (!s_ctx.inited) { return; }
    /* 仅空闲态(level==0)判据；泄放中 A9 随 PWM 翻转，"高"正常，跳过避免误拉低 */
    if (s_ctx.level == 0U && Bsp_Discharge_ReadFeedback() != 0) {
        Bsp_Discharge_ForcePinLow();
    }
}

/**
 * 功能：泄放指示灯(PB0)服务——主循环每圈调用（非阻塞）。
 *   泄放极快(灯一闪难辨)，故用两级计时保证可视：
 *     · 泄放中(标志=1)：灯亮，持续刷新最小亮灯基准 c_ms；母线高时刷新 2s 兜底基准 d_ms。
 *     · 停/完成(标志=0)：自最后一次泄放(c_ms) 起【至少再亮 BSP_DCH_LED_MIN_ON_MS(1s)】才熄灭。
 *   另附 2s 兜底：标志=1 但母线已持续 <55V 达 BSP_DCH_LED_OFF_FALLBACK_MS(停泄放中断偶发丢失)→清标志。
 * 输入参数：无
 * 输出参数：无（改写 PB0 电平 / 兜底清 s_dch_led_active）
 * 返回值：无
 */
/**
 * 功能：上电版本识别闪灯服务——主循环每圈调用（非阻塞）。首次上电从“亮”起每 BSP_DCH_LED_PON_BLINK_MS
 *       翻转一次、共 BSP_DCH_LED_PON_BLINK_COUNT 次（4 次=闪 2 下），末次停在灭，用于区分固件版本。
 *       cnt 减到 0 后本函数不再动灯，PB0 交回 AppDch_ServiceDchLed。
 * 输入参数：无
 * 输出参数：无（闪灯期间独占改写 PB0）
 * 返回值：无
 */
void AppDch_ServicePowerOnBlink(void)
{
    if (!s_ctx.inited) { return; }
    if (s_pon_cnt == 0U) { return; }           /* 已闪完：让位泄放灯服务 */

    const uint32_t now = systick_get_ms();
    if ((uint32_t)(now - s_pon_e_ms) >= (uint32_t)BSP_DCH_LED_PON_BLINK_MS) {
        s_pon_e_ms = now;
        s_pon_cnt--;
        Bsp_DchLed_Set(s_pon_led_on);          /* 先按当前灯状态设置 */
        s_pon_led_on = (uint8_t)(!s_pon_led_on); /* 再翻转，供下次 */
    }
}

void AppDch_ServiceDchLed(void)
{
    if (!s_ctx.inited) { return; }
    if (s_pon_cnt != 0U) { return; }           /* 上电闪灯进行中：PB0 由闪灯服务独占，本服务让位 */

    const uint32_t now = systick_get_ms();

    if (s_dch_led_active != 0U) {
        /* —— 泄放中：灯亮，刷新最小亮灯基准；母线<55V 持续 2s 则兜底清标志 —— */
        Bsp_DchLed_Set(1U);
        s_dch_led_c_ms = now;
        if (AppMon_GetVBus_mV() < (int32_t)BSP_DCH_OFF_MV) {
            if ((uint32_t)(now - s_dch_led_d_ms) >= (uint32_t)BSP_DCH_LED_OFF_FALLBACK_MS) {
                s_dch_led_active = 0U;      /* 母线已低却仍标记泄放中：兜底清标志（下圈进入延时熄灭） */
            }
        } else {
            s_dch_led_d_ms = now;           /* 母线≥55V：复位 2s 基准（只计连续<55V 时长） */
        }
    } else {
        /* —— 停/完成：延时熄灭。自最后一次泄放(c_ms)起至少再亮 MIN_ON_MS 才灭，保证肉眼可辨 —— */
        if ((uint32_t)(now - s_dch_led_c_ms) >= (uint32_t)BSP_DCH_LED_MIN_ON_MS) {
            Bsp_DchLed_Set(0U);             /* 已过最小亮灯窗 → 熄灭 */
        } else {
            Bsp_DchLed_Set(1U);            /* 仍在最小亮灯窗内 → 保持亮 */
        }
        s_dch_led_d_ms = now;               /* 未泄放：2s 基准跟随，避免下次起泄放立即误触发兜底 */
    }
}

/* —— 20s 连续泄放会话计时复位：母线跌到起泄放阈值(56.5V)以下即清零并解锁，下轮泄放从 0 重计 —— */
static void dch_session_reset(void)
{
    s_ctx.on_t0     = 0U;
    s_ctx.timed_out = false;
}

/* —— 新连续泄放会话起始：仅看门狗起泄放/手动 force_full 从 L0 起时调用，on_t0 从 0 重新计 20s —— */
static void dch_session_begin(void)
{
    s_ctx.on_t0 = systick_get_ms();
}

/* —— 进入 L1 时复位上升趋势判断：以当前瞬时电压为基准、连升计数清零 —— */
static void dch_l1_trend_reset(int32_t v)
{
    s_ctx.l1_v_prev   = v;
    s_ctx.l1_rise_cnt = 0U;
}

/* —— StepIsr 内部：设置占空比并同步 on 标志（唯一 PWM 占空比写者，TIMER1 ISR 上下文）—— */
static void dch_apply_duty(uint8_t duty, uint8_t level)
{
    if (duty == 0U) {
        Bsp_Discharge_Stop();
        s_ctx.on = false;
        s_dch_led_active = 0U;      /* 停泄放 → 泄放灯标志清0(灭)。对应“低压停止泄放→标志=0” */
    } else {
        Bsp_Discharge_SetDuty(duty);
        s_ctx.on = true;
        s_dch_led_active = 1U;      /* 起/维持泄放 → 泄放灯标志置1(亮)。对应“过压启动泄放→标志=1” */
    }
    s_ctx.level = level;
}

/* —— 看门狗窗口切换：按当前泄放级别设 ADC 模拟看门狗“双边”窗口（阈值取 FRAM 配置，支持在线标定）——
 *   级别切换全部中断化：每个态的窗口边界 = 该态“下一个要抓的阈值”，越窗即 WDE 中断，ISR 读瞬时码判方向。
 *   L0 待机：[0, 56.5V]        只盯上限——冲过 56.5V 起泄放；48V 待机(下限0)不空触发。
 *   L1 一级：[55V, 58V]        双边——跌破 55V 停；冲过 58V 立即升二级。
 *   L2 二级：[56.5V, 满量程]   盯下限——跌破 56.5V 退回一级、跌破 55V 停。
 * 见《泄放中断即时响应优化设计说明》§3。ISR 与主循环均可调用。 */
static void dch_set_wdg_window_for_level(uint8_t level)
{
    switch (level) {
    case 0U:   /* 待机：只盯上限 [0, 一级触发码 56.5V] */
        Bsp_Analog_SetVBusWindow(0U, Bsp_Analog_VBusMvToCode(BSP_DCH_L1_ON_MV));
        break;
    case 1U:   /* 一级：双边 [停止码 55V, 二级触发码 58V] */
        Bsp_Analog_SetVBusWindow(Bsp_Analog_VBusMvToCode((int32_t)Bsp_Config_GetOvpOff() * 100),
                                 Bsp_Analog_VBusMvToCode(BSP_DCH_L2_ON_MV));
        break;
    default:   /* 二级：盯下限 [一级触发码 56.5V, 满量程] */
        Bsp_Analog_SetVBusWindow(Bsp_Analog_VBusMvToCode(BSP_DCH_L1_ON_MV),
                                 (uint16_t)(BSP_ANALOG_ADC_FULL - 1U));
        break;
    }
}

/**
 * 功能：泄放总允许门当前是否处于禁止/兜底状态（起泄放前的门控条件汇总）。
 * 返回：true=禁止起泄放（allowed=false / 20s超时 / 关断类故障 / 安全态）。
 */
static bool dch_start_blocked(void)
{
    return (!s_ctx.allowed) || s_ctx.timed_out
         || ((AppProt_GetFaults() & (APP_FAULT_OVER_TEMP | APP_FAULT_LOOP_FB | APP_FAULT_DCH_TIMEOUT)) != 0U)
         || AppSafe_InSafeState();
}

/**
 * 功能：ADC 模拟看门狗“双边窗口”中断回调（µs 级）——起/停/升二级/退一级的唯一执行者。
 *       用瞬时电压 AppMon_GetVBus_mV()(去滤波) 判越限方向，按当前级别做三态迁移，并切到目标态窗口：
 *       L0：越上限→ ≥58V 直接 L2 100% / 否则 L1 50% 起步；
 *       L1：越下限<55V→停 / 越上限≥58V→立即升 L2 100%；
 *       L2：越下限——<55V→停 / 55~56.5V→退回 L1（90%封顶继续 L1 闭环）。
 * 输入参数：无
 * 输出参数：无（写 Bsp_Discharge 占空比 + s_ctx.level/on/on_t0 + 看门狗窗口）
 * 返回值：无
 * 说明：由 Bsp_Analog_WatchdogIrq()(it.c ADC0_1_IRQHandler) 调用。须极短、不阻塞、不打印。
 *       抢占优先级高于 TIMER1，保证起停/升降级能抢占正在爬坡的 StepIsr。
 */
void AppDch_OvpIsr(void)
{
    if (!s_ctx.inited) { return; }

    /* ★瞬时值（去滤波）：看门狗已按原始码越窗触发，此处读瞬时电压判方向，无软件滞后 */
    const int32_t v      = AppMon_GetVBus_mV();
    const int32_t v_stop = (int32_t)Bsp_Config_GetOvpOff() * 100;   /* 停泄放阈值（默认 55V） */

    switch (s_ctx.level) {
    case 0U:
        /* —— 待机态：越上限＝母线冲过一级触发(56.5V) → 起泄放 —— */
        if (dch_start_blocked()) { return; }              /* 过温/20s/安全态等：禁止起泄放 */
        if (v < (int32_t)BSP_DCH_L1_ON_MV) { return; }    /* 毛刺兜底：确认确实过 56.5V 再起 */
        dch_session_begin();
        if (v >= (int32_t)BSP_DCH_L2_ON_MV) {             /* 已 ≥58V：一步到位二级 100% */
            dch_apply_duty(BSP_DISCHARGE_DUTY_L2_FULL, 2U);
            dch_set_wdg_window_for_level(2U);
        } else {                                          /* 56.5~58V：一级 50% 起步（TIMER1 +10% 爬坡） */
            dch_apply_duty(BSP_DISCHARGE_DUTY_START, 1U);
            dch_set_wdg_window_for_level(1U);
            dch_l1_trend_reset(v);                        /* 起 L1：以起步瞬时电压为趋势基准 */
        }
        break;

    case 1U:
        /* —— 一级态：越下限<55V 停 / 越上限≥58V 立即升二级 —— */
        if (v < v_stop) {
            dch_apply_duty(0U, 0U);
            dch_session_reset();                          /* 母线已泄到≤55V：会话结束，计时清零 */
            dch_set_wdg_window_for_level(0U);
        } else if (v >= (int32_t)BSP_DCH_L2_ON_MV) {
            dch_apply_duty(BSP_DISCHARGE_DUTY_L2_FULL, 2U);   /* 冲过 58V → 立即升二级 100% */
            dch_set_wdg_window_for_level(2U);
        }
        break;

    default:   /* level == 2U 二级 */
        /* —— 二级态：越下限——<55V 停 / 55~56.5V 退回一级 —— */
        if (v < v_stop) {
            dch_apply_duty(0U, 0U);
            dch_session_reset();
            dch_set_wdg_window_for_level(0U);
        } else if (v < (int32_t)BSP_DCH_L1_ON_MV) {
            dch_apply_duty(BSP_DISCHARGE_DUTY_MAX, 1U);       /* 退回一级：从封顶继续 L1 闭环 */
            dch_set_wdg_window_for_level(1U);
            dch_l1_trend_reset(v);                            /* 退回 L1：重置趋势基准 */
        }
        /* 56.5~58V 之间维持二级 100% */
        break;
    }
}

/**
 * 功能：一级泄放占空比爬坡（TIMER1 50µs 中断回调）。仅在 L1(level==1)：每 50µs +10% 爬到封顶 100%，
 *       并做"上升趋势升级"——连升 BSP_DCH_L1_RISE_ESCALATE_STEPS(2) 步仍高于上一步则直接跳二级 100%。
 *       起/停/58V升级/退一级仍由 ADC 看门狗中断 AppDch_OvpIsr 处理；L2 静态 100% 不步进。
 * 输入参数：无
 * 输出参数：无（写 Bsp_Discharge 占空比）
 * 返回值：无
 * 说明：ISR 上下文，须极短、不阻塞、不打印。写占空比前重读 level，防与看门狗竞写。判据用瞬时值，不滤波。
 */
void AppDch_StepIsr(void)
{
    if (!s_ctx.inited) { return; }

    /* —— 门控：禁止泄放的条件（allowed=false / 20s超时 / 关断类故障 / 安全态）—— */
    bool blocked = (!s_ctx.allowed) || s_ctx.timed_out
                 || ((AppProt_GetFaults() & (APP_FAULT_OVER_TEMP | APP_FAULT_LOOP_FB | APP_FAULT_DCH_TIMEOUT)) != 0U)
                 || AppSafe_InSafeState();

    /* —— 手动全功率控制：仅未被门控时生效，不走级别机 —— */
    if (s_ctx.force_full && !blocked) {
        if (s_ctx.level == 0U) {
            dch_set_wdg_window_for_level(2U);   /* 手动起：切二级窗口，仍受跌破55V看门狗停 */
            dch_session_begin();                /* 新会话：20s 从 0 重计 */
        }
        dch_apply_duty(BSP_DISCHARGE_DUTY_L2_FULL, 2U);
        return;
    }

    /* —— 被门控且在泄放：立即停并归 L0，切回待机态窗口 —— */
    if (blocked && s_ctx.level != 0U) {
        dch_apply_duty(0U, 0U);
        s_ctx.on_t0 = 0U;   /* 会话中断；timed_out 保持至母线跌破起泄放阈值(56.5V)才由 Update 复位 */
        dch_set_wdg_window_for_level(0U);
    }

    /* —— 未泄放(level=0，含被门控停后)：空闲防误泄放守护（硬件组策略）——
     * A9(=泄放引脚 PA8 状态) 高 表示 PA8 被异常拉高，空闲态等效 100% 占空比误泄放，很危险。
     * 故：母线 < 56.5V（非正常起泄放区间）且 A9 检测为高 → 立即经普通 GPIO 把 PA8 拉低，杜绝误泄放。
     * （母线 ≥56.5V 属正常起泄放区间，交 ADC 看门狗 OvpIsr 起泄放，不在此拉低。） */
    if (s_ctx.level == 0U) {
        /* 空闲防误泄放守护（不在此清 timed_out/on_t0：20s 超时后须保持闭锁直至母线跌破起泄放阈值56.5V；
         * 亦不在此清 on_t0——OvpIsr 刚起 L1/L2 时可能与本次 ISR 竞态，误清会导致 20s 永不触发） */
        if (AppMon_GetVBus_mV() < (int32_t)BSP_DCH_L1_ON_MV
            && Bsp_Discharge_ReadFeedback() != 0) {
            Bsp_Discharge_ForcePinLow();
        }
        return;   /* 未泄放：起泄放归看门狗，本 ISR 不做级别推进 */
    }

    /* —— 仅 L1：先判上升趋势升级，否则常规占空比爬坡（每 50µs +10%，封顶 100%）——
     * 停(<55V)/退一级(<56.5V)/58V 升二级 仍由看门狗 OvpIsr 处理；本 ISR 额外做"趋势升级"：
     * 趋势升级（用户定案）：L1 期间用瞬时值逐步比较，若连续 BSP_DCH_L1_RISE_ESCALATE_STEPS(2) 步
     *   母线都比上一步更高(泄放压不住、还在冲高)——即便没到 58V、也不等爬坡到顶——直接跳二级 100%，
     *   并切 L2 窗口。防"电源夹在 56.5~58V、58V 永不触发、只能慢爬 ~250µs 到满"的场景，升级是安全方向。
     * 重读 level==1：确认未被看门狗抢占改为停(0)/升二级(2)，再处理。L2 静态 100% 不步进。 */
    if (s_ctx.level == 1U) {
        const int32_t vnow = AppMon_GetVBus_mV();
        if (vnow > (s_ctx.l1_v_prev + (int32_t)BSP_DCH_L1_RISE_MARGIN_MV)) {
            s_ctx.l1_rise_cnt++;                            /* 较上一步继续上升 */
        } else {
            s_ctx.l1_rise_cnt = 0U;                         /* 掉头/持平：趋势中断，重新计 */
        }
        s_ctx.l1_v_prev = vnow;

        if (s_ctx.l1_rise_cnt >= BSP_DCH_L1_RISE_ESCALATE_STEPS) {
            /* 连升达标 → 直接升二级 100%（不再等爬坡/58V），切 L2 窗口交看门狗管退级/停 */
            dch_apply_duty(BSP_DISCHARGE_DUTY_L2_FULL, 2U);
            dch_set_wdg_window_for_level(2U);
            s_ctx.l1_rise_cnt = 0U;
            return;
        }

        uint8_t cur = Bsp_Discharge_GetDuty();
        uint8_t nxt = (uint8_t)(cur + BSP_DISCHARGE_DUTY_STEP);
        if (nxt > BSP_DISCHARGE_DUTY_MAX) { nxt = BSP_DISCHARGE_DUTY_MAX; }
        if (nxt != cur) { Bsp_Discharge_SetDuty(nxt); }    /* 常规爬升一步（封顶 100%） */
    }
}

/**
 * 功能：立即强制关断泄放并清除输出/强制全开标志（紧急停/安全用）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppDch_ForceStop(void)
{
    Bsp_Discharge_Stop();
    s_ctx.on         = false;
    s_ctx.level      = 0U;      /* 归 L0，避免残留级别让 StepIsr 继续爬坡 */
    s_ctx.force_full = false;
    s_dch_led_active = 0U;      /* 紧急停 → 泄放灯灭 */
    dch_session_reset();
    if (Bsp_Analog_IsInitialized()) {
        dch_set_wdg_window_for_level(0U);   /* 切回待机态窗口：只盯过压再起 */
    }
}

/**
 * 功能：设置/清除强制全开标志（0x10）。
 * 输入参数：enable —— true=请求强制全开，false=取消。
 * 输出参数：无
 * 返回值：bool，恒 true（冷却闭锁已移除，不再拒绝）。
 */
bool AppDch_ForceFull(bool enable)
{
    s_ctx.force_full = enable;
    return true;
}

/**
 * 功能：设置泄放总允许门（0x30），禁止时立即关断而不等下一圈。
 * 输入参数：allowed —— true=允许泄放，false=禁止并即刻关断。
 * 输出参数：无
 * 返回值：无
 */
void AppDch_SetAllowed(bool allowed)
{
    s_ctx.allowed = allowed;
    if (!allowed) {
        AppDch_ForceStop();   /* 禁止即刻关断，不等下一圈 */
    }
}

/**
 * 功能：按当前泄放级别重设 ADC 看门狗窗口（阈值变更后调用，如 CAN 0x14/0x15 改过压启停）。
 *       L0 [0,56.5V] 只盯上限；L1 [55V,58V] 双边；L2 [56.5V,满量程] 盯下限。
 *       替代 COMM 里"无脑设固定窗口"——泄放中改阈值也能即时生效且不误开监测边。
 * 输入参数：无
 * 输出参数：无（写 ADC 看门狗窗口）
 * 返回值：无
 */
void AppDch_RefreshWdgWindow(void)
{
    dch_set_wdg_window_for_level(s_ctx.level);
}

/**
 * 功能：查询泄放总允许门当前状态。
 * 输入参数：无
 * 输出参数：无
 * 返回值：bool，true=允许，false=禁止。
 */
bool AppDch_IsAllowed(void)       { return s_ctx.allowed; }

/**
 * 功能：获取当前实际输出占空比。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint8_t，当前占空比（%）。
 */
uint8_t  AppDch_GetDuty(void)     { return Bsp_Discharge_GetDuty(); }

/**
 * 功能：查询当前是否正在泄放输出。
 * 输入参数：无
 * 输出参数：无
 * 返回值：bool，true=输出中，false=停止。
 */
bool     AppDch_IsOn(void)        { return s_ctx.on; }

/**
 * 功能：获取当前泄放级别（0=停 1=一级爬升 2=二级100%），供遥测/上报观测。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint8_t，泄放级别。
 */
uint8_t  AppDch_GetLevel(void)    { return s_ctx.level; }

/**
 * 功能：获取累计泄放时长。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint32_t，累计泄放时长（ms）。
 */
uint32_t AppDch_GetTotalMs(void)  { return s_ctx.total_ms; }

/**
 * 功能：查询是否处于冷却闭锁中。
 * 输入参数：无
 * 输出参数：无
 * 返回值：bool，恒 false（冷却闭锁已移除，策略 V1.6；保留接口兼容）。
 */
bool     AppDch_InCooldown(void)  { return false; }
