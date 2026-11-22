/**
 * @file    app_protect.c
 * @brief   HLD-MOD-PROT 保护状态机模块实现（首版骨架，安全策略后续细化）
 *
 * 本版已落地：过压施密特状态机骨架、分级过压(L1/L2)迟滞、二级过压计次、
 * 过温停泄放+单次计次（温度自恢复：≥95℃停泄放，≤65℃自动解除）、
 * 事件队列（供 BBX 拉取）、DCH 请求输出（单写者）。
 *
 * 待细化（标注 TODO）：过流阈值（CAL 标定）、占空比比例/斜率策略、
 * 恢复延时常数、Brake_FB 去抖完整实现。
 */

#include "app_protect.h"
#include "app_monitor.h"
#include "app_safety.h"    /* AppSafe_InSafeState：ISR 快路径互斥 */
#include "app_discharge.h" /* AppDch_IsOn：过泄压检测门控 */
#include "uart_log.h"

#define PROT_TAG "PROT"

/* 恢复延时（过压回落到 SW_OFF 后，维持低压多久才回 NORMAL）。★CAL 待定。 */
#ifndef APP_PROT_RECOVER_MS
#define APP_PROT_RECOVER_MS     1000U
#endif

/* Brake_FB 泄放态去抖次数（PA9 20kHz 翻转，单采样不可靠，见 PROT §3.2）。 */
#ifndef APP_PROT_FB_DEBOUNCE
#define APP_PROT_FB_DEBOUNCE    30U
#endif

typedef struct {
    uint16_t       faults;
    uint16_t       ov2_count;
    uint16_t       otp_count;       /* 过温(≥95℃)异常次数（单次计次，停泄放） */
    bool           prev_l2;        /* 上次是否处于二级过压（计次边沿检测） */
    bool           prev_otp;       /* 上次是否处于过温（单次计次边沿检测，迟滞防抖） */
    uint16_t       fb_high_cnt;    /* 泄放态 Brake_FB 连续高计数 */
    bool           inited;
} Prot_Ctx_T;

static Prot_Ctx_T s_ctx;

/* —— 事件队列（PROT 产生、BBX 消费）—— */
#define PROT_EVT_Q_LEN  8U
static App_EvtType_T s_evt_q[PROT_EVT_Q_LEN];
static uint8_t       s_evt_head;   /* 取出位置 */
static uint8_t       s_evt_tail;   /* 写入位置 */

/**
 * 功能：向事件环形队列写入一个事件（队满则丢弃新事件，不拖累保护）。
 * 输入参数：evt —— 待入队的事件类型。
 * 输出参数：无
 * 返回值：无
 */
static void prot_push_evt(App_EvtType_T evt)
{
    uint8_t next = (uint8_t)((s_evt_tail + 1U) % PROT_EVT_Q_LEN);
    if (next == s_evt_head) { return; }   /* 满则丢弃最旧之外的新事件（保护不被记录拖累） */
    s_evt_q[s_evt_tail] = evt;
    s_evt_tail = next;
}

/**
 * 功能：置位指定故障；仅当该位为新置位且事件有效时推入事件队列。
 * 输入参数：bit —— 故障位掩码；evt —— 关联事件类型（APP_EVT_NONE 表示不记录）。
 * 输出参数：无
 * 返回值：无
 */
static void prot_set_fault(uint16_t bit, App_EvtType_T evt)
{
    if ((s_ctx.faults & bit) == 0U) {
        s_ctx.faults |= bit;
        if (evt != APP_EVT_NONE) { prot_push_evt(evt); }
    }
}

/**
 * 功能：初始化保护判据模块，复位故障/计次/去抖与事件队列。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppProt_Init(void)
{
    s_ctx.faults      = 0U;
    s_ctx.ov2_count   = 0U;
    s_ctx.otp_count   = 0U;
    s_ctx.prev_l2     = false;
    s_ctx.prev_otp    = false;
    s_ctx.fb_high_cnt = 0U;
    s_ctx.inited      = true;
    s_evt_head = 0U;
    s_evt_tail = 0U;
    UART_LOGI(PROT_TAG, "init done");
}

/**
 * 功能：【已废弃/保留符号】过压/欠压 ADC 看门狗中断回调（旧单级实现）。
 *   分级泄放重构后，泄放起/停已改由 AppDch_OvpIsr()（app_discharge.c）承担，看门狗回调
 *   在 App_Init 里注册的是 AppDch_OvpIsr 而非本函数。本函数不再被注册/调用，改为空操作，
 *   仅保留符号避免外部引用编译失败。PROT 现职责为纯告警/计次/故障闭锁，不再写 PWM。
 *   （见《过泄压中断动态启停与分级泄放执行设计说明》§7：PROT 降级为纯告警/计次。）
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppProt_FastCheck(void)
{
    /* 已废弃：起/停执行权归 AppDch_OvpIsr，本函数不再动作（不写 PWM、不改 req_enable）。 */
}

/**
 * 功能：保护判据慢路径主体——按阈值判过温/过流/两级分级过压（带迟滞计次），置/清故障位。须每控制圈调用。不驱动泄放执行。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppProt_Update(void)
{
    if (!s_ctx.inited) { return; }

    const int32_t vbus = AppMon_GetVBus_mV();   /* ★瞬时值(去滤波)：告警/计次带迟滞，不再用 EMA */
    const int32_t temp = AppMon_GetTemp_mDegC();

    /* 阈值：过温/过流/分级过压告警(L1/L2) 均取可标定配置（FRAM，经 CAN 0x15/0x17 整定）。
     * L1/L2 报警缺省 565/580(56.5/58V)、恢复 550/565(55/56.5V)，与泄放执行阈值 BSP_DCH_* 同值；
     * 上位机可经 0x17 重标报警动作/恢复值（越界由 Bsp_Config 校验、迟滞派生恢复值）。
     * 换算：0.1V→mV(×100)、0.1℃→m℃(×100)、0.01A→mA(×10)。 */
    const int32_t otp_off_mc = (int32_t)Bsp_Config_GetOtpOff() * 100;
    const int32_t ovc_ma     = (int32_t)Bsp_Config_GetOvc()    * 10;

    /* —— 过温：≥95℃ 主动停泄放 + 单次计次（温度自恢复，非锁存）——
     * 到过温关断阈值即停止泄放并记一次过温异常；计次为单次（上升沿 +1），
     * 持续过温不重复累加，须降到恢复阈值(65℃)及以下才允许再次计次（迟滞防抖）。
     * 过温=泄放功能失效：置 OVER_TEMP 故障位，OvpIsr/StepIsr 门控挡住泄放——过温期间即使 60V
     * 也不起泄放（非停当前泄放，是整个泄放功能禁用）。须冷却到恢复阈值(65℃)及以下才自动清位、
     * 重新“支持”泄放（不自动起：仅清 OVER_TEMP 解除门控，仍须母线 ≥56.5V 看门狗+软件确认才起）。
     * 温度自恢复类：不进 0x31 清除掩码、不升级安全态。 */
    {
        int32_t otp_recover_mc = (int32_t)Bsp_Config_GetOtpRecover() * 100;
        if (temp >= otp_off_mc) {
            if (!s_ctx.prev_otp) { s_ctx.otp_count++; }   /* 上升沿 +1（单次计次） */
            s_ctx.prev_otp = true;
            prot_set_fault(APP_FAULT_OVER_TEMP, APP_EVT_OVER_TEMP);   /* 置位→停泄放（边沿入队） */
        } else if (temp <= otp_recover_mc) {
            s_ctx.prev_otp = false;                       /* 降到恢复阈值以下，允许再计 */
            s_ctx.faults &= (uint16_t)~APP_FAULT_OVER_TEMP;           /* 温度自恢复，清过温位 */
        }
    }

    /* —— 仅置故障类：过流（不改泄放动作，非锁存，低于阈值即清）—— */
    int32_t cur = AppMon_GetCurrent_mA();
    if (cur < 0) { cur = -cur; }
    if (cur >= ovc_ma) {
        prot_set_fault(APP_FAULT_OVER_CURRENT, APP_EVT_OVER_CURRENT);
    } else {
        s_ctx.faults &= (uint16_t)~APP_FAULT_OVER_CURRENT;
    }

    /* —— 两级分级过压告警/计次（带迟滞，阈值取可标定配置）——
     * 一级 OV_L1：≥ovp_l1 置位，≤ovp_l1_off 清除；二级 OV_L2：≥ovp_l2 置位并上升沿计次，<ovp_l2_off 清除。
     * 缺省 L1=56.5V/清55V、L2=58V/清56.5V；纯告警/计次，供主站 0x23 轮询（决定停电机使能），
     * 不驱动泄放（泄放执行归看门狗+TIMER1，用 BSP_DCH_* 宏）。 */
    const int32_t ovp_l1_mv     = (int32_t)Bsp_Config_GetOvpL1()    * 100;
    const int32_t ovp_l1_off_mv = (int32_t)Bsp_Config_GetOvpL1Off() * 100;
    const int32_t ovp_l2_mv     = (int32_t)Bsp_Config_GetOvpL2()    * 100;
    const int32_t ovp_l2_off_mv = (int32_t)Bsp_Config_GetOvpL2Off() * 100;

    if (vbus >= ovp_l1_mv) {
        prot_set_fault(APP_FAULT_OV_L1, APP_EVT_OV_L1);
    } else if (vbus <= ovp_l1_off_mv) {
        s_ctx.faults &= (uint16_t)~APP_FAULT_OV_L1;
    }

    bool l2_now = (vbus >= ovp_l2_mv);
    if (l2_now) {
        prot_set_fault(APP_FAULT_OV_L2, APP_EVT_OV_L2);
        if (!s_ctx.prev_l2) { s_ctx.ov2_count++; }   /* 上升沿计次（二级触发次数） */
    } else if (vbus < ovp_l2_off_mv) {
        s_ctx.faults &= (uint16_t)~APP_FAULT_OV_L2;
    }
    s_ctx.prev_l2 = l2_now;

    /* 注：过压起/停泄放的执行已移交 ADC 看门狗中断(AppDch_OvpIsr)与 TIMER1(AppDch_StepIsr)，
     * PROT 不再运行施密特状态机、不再产出占空比/启停请求（原 state/req_enable/req_duty 已移除）。
     * 关断类故障(过温/回路失效/泄放超时)经故障位由 DCH 各 ISR 查 AppProt_GetFaults() 门控关断。 */
}

/**
 * 功能：回路自检——对 Brake_FB 高电平去抖（连续超阈值判泄放回路失效并锁存故障）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppProt_LoopSelfCheck(void)
{
    if (!s_ctx.inited) { return; }

    /* A9(Brake_FB) = 泄放引脚(PA8)状态一一对应（硬件组确认，非看门狗反馈）：
     *   空闲态 PA8 低→A9 应为低；若持续高=泄放通路异常(开路/短路)，锁存 LOOP_FB。
     *   泄放态 PA8 为 20kHz PWM→A9 随之翻转(低占空比时多为高)，"高"是正常的，
     *   故本"高=异常"判据**仅在空闲态(level==0)有效**；泄放中跳过（清计数）。
     *   泄放是否有效由 DCH 封顶/20s 兜底(DCH_TIMEOUT)覆盖。 */
    if (AppDch_GetLevel() != 0U) {   /* 泄放中：A9 为 PWM，不做空闲判据，避免误锁存 LOOP_FB */
        s_ctx.fb_high_cnt = 0U;
        return;
    }
    int fb = Bsp_Discharge_ReadFeedback();
    if (fb != 0) {
        if (s_ctx.fb_high_cnt < 0xFFFFU) { s_ctx.fb_high_cnt++; }
        if (s_ctx.fb_high_cnt >= APP_PROT_FB_DEBOUNCE) {
            prot_set_fault(APP_FAULT_LOOP_FB, APP_EVT_LOOP_FB);
        }
    } else {
        s_ctx.fb_high_cnt = 0U;
    }
}

/**
 * 功能：获取当前故障位图。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，故障位掩码（按 App_Fault_T 定义）。
 */
uint16_t       AppProt_GetFaults(void)  { return s_ctx.faults; }

/**
 * 功能：获取二级过压触发累计次数。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，二级过压计次。
 */
uint16_t       AppProt_GetOv2Count(void){ return s_ctx.ov2_count; }
uint16_t       AppProt_GetOtpCount(void){ return s_ctx.otp_count; }

/**
 * 功能：清除可清除故障位（锁存/冷却闭锁类需满足解除条件后由上层另清），并复位回路去抖计数。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，清除后的剩余故障位图。
 */
uint16_t AppProt_ClearFaults(void)
{
    /* 仅清除可清除位；锁存类须满足解除条件（冷却由 DCH 管理）后才允许清。
     * 首版：清除非冷却闭锁类的全部位（过温/回路失效在温度恢复后由上层再清）。 */
    s_ctx.faults &= (uint16_t)~(APP_FAULT_OVER_CURRENT | APP_FAULT_UNDER_VOLT |
                                APP_FAULT_DCH_TIMEOUT | APP_FAULT_OV_L1 | APP_FAULT_OV_L2);
    s_ctx.fb_high_cnt = 0U;
    return s_ctx.faults;
}

/**
 * 功能：DCH→PROT 回报——泄放封顶滞留仍无效（泄放能力不足）时由 DCH 调用，
 *       置 DCH_TIMEOUT 故障（锁存）并入事件队列，供 COMM 0x23 读取 / BBX 记录 / 主站停机决策。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 * 说明：这是 DCH→PROT 唯一的故障回报通道（占空比请求仍 PROT→DCH 单向，不破坏单写者）。
 */
void AppProt_NotifyDchTimeout(void)
{
    if (!s_ctx.inited) { return; }
    prot_set_fault(APP_FAULT_DCH_TIMEOUT, APP_EVT_DCH_TIMEOUT);   /* 边沿置位+入队（去重） */
}

/**
 * 功能：从事件队列取出一个待记录事件（供 BBX 消费）。
 * 输入参数：out_evt —— 接收事件类型的指针（可空）。
 * 输出参数：*out_evt —— 写入取出的事件类型。
 * 返回值：bool，true=取到事件，false=队列为空。
 */
bool AppProt_PollEvent(App_EvtType_T *out_evt)
{
    if (s_evt_head == s_evt_tail) { return false; }   /* 空 */
    if (out_evt != NULL) { *out_evt = s_evt_q[s_evt_head]; }
    s_evt_head = (uint8_t)((s_evt_head + 1U) % PROT_EVT_Q_LEN);
    return true;
}
