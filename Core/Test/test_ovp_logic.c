/**
 * @file    test_ovp_logic.c
 * @brief   过压保护业务逻辑单元测试 实现（兼 App 层 OvpProtect 参考实现）
 *
 * 实现《过压保护设计思路梳理》的施密特触发状态机与分级保护策略，并用注入式
 * 输入（电压/温度/控制字）驱动验证全部分支。占空比经 Bsp_Discharge_RampToDuty
 * 斜坡控制，验证 BSP 与业务逻辑衔接。阈值参数全部取自 bsp.h 的 BSP_OVP_* 宏。
 *
 * @version 1.0.0
 * @date    2026-06-13
 */

#include "test_ovp_logic.h"

#define BSP_USE_ANALOG
#define BSP_USE_DISCHARGE
#include "bsp.h"

/* 多次确认次数（抗瞬态毛刺，REQ-FUNC-028） */
#define OVP_CONFIRM_N   5U

/* ---- 过压保护状态机（App 参考实现） ---- */
typedef enum {
    OVP_NORMAL = 0,
    OVP_OVER_VOLTAGE,
    OVP_RECOVERY_DELAY,
    OVP_LOCKED_COOL,        /* 过温/超时锁定，须冷却 */
} Ovp_State_T;

typedef struct {
    Ovp_State_T state;
    uint8_t  duty;          /* 当前占空比（与 BSP 同步） */
    uint8_t  l1_alarm;      /* 一级过压报警 */
    uint8_t  l2_alarm;      /* 二级过压报警 */
    uint32_t l2_count;      /* 二级过压次数 */
    uint32_t otp_count;     /* 过温(≥95℃)异常次数（单次计次，停泄放） */
    uint8_t  prev_otp;      /* 过温边沿检测 */
    uint8_t  otp_active;    /* 过温停泄放态（≥95℃置位，≤65℃恢复清除，温度自恢复） */
    uint32_t on_ms;         /* 连续开通计时 */
    uint32_t cool_ms;       /* 冷却计时 */
    uint16_t cap_hold_cnt;  /* 封顶(DUTY_MAX)后滞留步数（未达停止电压）→ 达阈值停机 */
    uint8_t  motor_disabled;/* 关闭电机使能标志 */
    uint8_t  report_stop;   /* 上报主站并停机标志 */
    uint16_t confirm_cnt;   /* 过压多次确认计数 */
} Ovp_Ctx_T;

static void Ovp_Init(Ovp_Ctx_T *c)
{
    c->state = OVP_NORMAL; c->duty = 0U;
    c->l1_alarm = 0U; c->l2_alarm = 0U; c->l2_count = 0U;
    c->otp_count = 0U; c->prev_otp = 0U; c->otp_active = 0U;
    c->on_ms = 0U; c->cool_ms = 0U; c->cap_hold_cnt = 0U;
    c->motor_disabled = 0U; c->report_stop = 0U; c->confirm_cnt = 0U;
    Bsp_Discharge_Stop();
}

/* 状态机单步：vbus_mV=滤波后母线电压，temp_mC=电阻温度，ctrl=控制字，dt_ms=步进周期 */
static void Ovp_Step(Ovp_Ctx_T *c, int32_t vbus_mV, int32_t temp_mC, uint8_t ctrl, uint32_t dt_ms)
{
    /* 分级报警（带迟滞） */
    if (vbus_mV >= BSP_OVP_L1_MV)        { c->l1_alarm = 1U; }
    else if (vbus_mV < BSP_OVP_L1_OFF_MV){ c->l1_alarm = 0U; }
    if (vbus_mV >= BSP_OVP_L2_MV) {
        if (!c->l2_alarm) { c->l2_count++; }
        c->l2_alarm = 1U; c->motor_disabled = 1U; c->report_stop = 1U;
    } else if (vbus_mV < BSP_OVP_L2_OFF_MV) {
        c->l2_alarm = 0U;
    }

    /* 过温：≥95℃ 主动停泄放 + 单次计次（温度自恢复，非锁定）。
     * ≥95℃ 上升沿 otp_count+1 且置 otp_active（停泄放）；持续过温不重复计次；
     * 降到恢复阈值(65℃)及以下才清 otp_active 并允许再计（迟滞防抖）。 */
    if (temp_mC >= BSP_OTP_OFF_MDEGC) {
        if (!c->prev_otp) { c->otp_count++; }
        c->prev_otp = 1U;
        c->otp_active = 1U;
    } else if (temp_mC <= BSP_OTP_RECOVER_MDEGC) {
        c->prev_otp = 0U;
        c->otp_active = 0U;
    }

    /* 过温停泄放门：过温期间强制关断并退出泄放态（优先于过压/控制字），
     * 温度回落到恢复阈值以下自动解除后，若仍过压会重新进入泄放。 */
    if (c->otp_active) {
        Bsp_Discharge_Stop(); c->duty = 0U;
        c->state = OVP_NORMAL; c->on_ms = 0U; c->confirm_cnt = 0U; c->cap_hold_cnt = 0U;
        return;
    }

    /* 控制字强制全开（未锁定时）：直接开到上限，滞留 N 步兜底停机 */
    if (ctrl == BSP_OVP_CTRL_FORCE_FULL && c->state != OVP_LOCKED_COOL) {
        c->duty = Bsp_Discharge_RampToDuty(BSP_DISCHARGE_DUTY_MAX);
        c->on_ms += dt_ms;
        if (Bsp_Discharge_GetDuty() >= BSP_DISCHARGE_DUTY_MAX) {
            c->cap_hold_cnt++;
            if (c->cap_hold_cnt >= BSP_DISCHARGE_DUTY_CAP_HOLD_STEPS) {
                Bsp_Discharge_Stop(); c->duty = 0U;
                c->state = OVP_LOCKED_COOL; c->cool_ms = 0U; c->report_stop = 1U;
            }
        }
        return;
    }

    switch (c->state) {
    case OVP_NORMAL:
        Bsp_Discharge_Stop(); c->duty = 0U; c->on_ms = 0U;
        if (vbus_mV >= BSP_OVP_SW_ON_MV) {
            if (++c->confirm_cnt >= OVP_CONFIRM_N) {
                c->state = OVP_OVER_VOLTAGE; c->confirm_cnt = 0U;
                c->cap_hold_cnt = 0U;
                (void)Bsp_Discharge_SetDuty(BSP_DISCHARGE_DUTY_START);  /* 从启动占空比 5% 起步 */
                c->duty = BSP_DISCHARGE_DUTY_START;
            }
        } else {
            c->confirm_cnt = 0U;
        }
        break;
    case OVP_OVER_VOLTAGE: {
        /* 泄放效果闭环步进：未降到停止电压则占空比朝上限 +1 步；封顶后滞留 N 步仍无效 → 停机上报。
         * （每次 Ovp_Step 视作一个步进周期，与固件 DCH 的 100ms 步进等价） */
        if (vbus_mV < BSP_OVP_SW_OFF_MV) {          /* 回落到停止电压以下 → 泄放见效 */
            c->cap_hold_cnt = 0U;
            c->state = OVP_RECOVERY_DELAY;
        } else {
            uint8_t cur = Bsp_Discharge_GetDuty();
            if (cur < BSP_DISCHARGE_DUTY_MAX) {
                c->cap_hold_cnt = 0U;
                c->duty = Bsp_Discharge_RampToDuty(BSP_DISCHARGE_DUTY_MAX);  /* 朝上限走一步 */
            } else {
                c->cap_hold_cnt++;                  /* 已封顶仍压不下 */
                if (c->cap_hold_cnt >= BSP_DISCHARGE_DUTY_CAP_HOLD_STEPS) {
                    Bsp_Discharge_Stop(); c->duty = 0U;
                    c->state = OVP_LOCKED_COOL; c->cool_ms = 0U; c->report_stop = 1U;
                }
            }
        }
        c->on_ms += dt_ms;
        break;
    }
    case OVP_RECOVERY_DELAY:
        c->duty = Bsp_Discharge_RampToDuty(0U);
        if (c->duty == 0U) { c->state = OVP_NORMAL; c->on_ms = 0U; }
        if (vbus_mV >= BSP_OVP_SW_ON_MV) { c->state = OVP_OVER_VOLTAGE; }
        break;
    case OVP_LOCKED_COOL:
        Bsp_Discharge_Stop(); c->duty = 0U; c->cool_ms += dt_ms;
        if (temp_mC < BSP_OTP_RECOVER_MDEGC && c->cool_ms >= BSP_OVP_COOL_MS) {
            c->state = OVP_NORMAL; c->on_ms = 0U;
        }
        break;
    default: break;
    }
}

/* <!--TESTCASES--> */

void Test_OvpLogic_RunAll(Test_Stats_T *_stats)
{
    Ovp_Ctx_T c;
    int i;
    uint32_t t;

    /* 依赖 BSP 泄放已初始化（提供 RampToDuty/Stop/GetDuty） */
    (void)Bsp_Discharge_Init();

    /* 1) 正常无过压：占空比恒 0 */
    TEST_CASE("OVP: 正常态无泄放");
    Ovp_Init(&c);
    for (i = 0; i < 10; i++) { Ovp_Step(&c, 48000, 30000, 0U, 1U); }
    TEST_ASSERT_EQ_INT(c.state, OVP_NORMAL);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), 0);

    /* 2) 多次确认：单次 57V 不触发，连续 5 次才进 OVER_VOLTAGE */
    TEST_CASE("OVP: 多次确认抗毛刺 (N=5)");
    Ovp_Init(&c);
    Ovp_Step(&c, 57000, 30000, 0U, 1U);
    TEST_ASSERT_EQ_INT(c.state, OVP_NORMAL);          /* 单次不触发 */
    for (i = 0; i < 4; i++) { Ovp_Step(&c, 57000, 30000, 0U, 1U); }
    TEST_ASSERT_EQ_INT(c.state, OVP_OVER_VOLTAGE);    /* 连续 5 次触发 */

    /* 3) 泄放占空比斜坡上升，不超过 90% */
    TEST_CASE("OVP: 泄放占空比斜坡 (<=90%)");
    for (i = 0; i < 300; i++) { Ovp_Step(&c, 58000, 30000, 0U, 1U); }
    TEST_LOG_VAL("过压58V泄放 300步后: state=%d duty=%u%%", (int)c.state, Bsp_Discharge_GetDuty());
    TEST_ASSERT(Bsp_Discharge_GetDuty() > 0);
    TEST_ASSERT_IN_RANGE(Bsp_Discharge_GetDuty(), 1, BSP_DISCHARGE_DUTY_MAX);

    /* 4) 电压回落 → RECOVERY → NORMAL，占空比归零 */
    TEST_CASE("OVP: 回落恢复至 NORMAL");
    for (i = 0; i < 300; i++) { Ovp_Step(&c, 53000, 30000, 0U, 1U); }
    TEST_ASSERT_EQ_INT(c.state, OVP_NORMAL);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), 0);

    /* 5) 持续过压压不下：占空比步进至封顶后滞留 N 步 → 强制关断+上报停机 */
    TEST_CASE("OVP: 封顶滞留N步无效强制关断并上报停机");
    Ovp_Init(&c);
    for (i = 0; i < 5; i++) { Ovp_Step(&c, 58000, 30000, 0U, 1U); }   /* 进 OVER，起步 5% */
    /* 持续 58V(>55V停止值)：每步 +1% 到封顶(需约 DUTY_MAX-DUTY_START 步)，再滞留 CAP_HOLD_STEPS 步停机 */
    for (t = 0; t < (uint32_t)(BSP_DISCHARGE_DUTY_MAX + BSP_DISCHARGE_DUTY_CAP_HOLD_STEPS + 5U); t++) {
        Ovp_Step(&c, 58000, 30000, 0U, BSP_DISCHARGE_DUTY_STEP_PERIOD_MS);
    }
    TEST_ASSERT_EQ_INT(c.state, OVP_LOCKED_COOL);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), 0);
    TEST_ASSERT_EQ_INT(c.report_stop, 1);

    /* 6) 冷却：温度未降/未满 8min 不恢复；满足后恢复 */
    TEST_CASE("OVP: 冷却50℃+8min后恢复");
    Ovp_Step(&c, 48000, 60000, 0U, 1U);              /* 60℃>50，不恢复 */
    TEST_ASSERT_EQ_INT(c.state, OVP_LOCKED_COOL);
    for (t = 0; t < BSP_OVP_COOL_MS + 10U; t++) { Ovp_Step(&c, 48000, 40000, 0U, 1U); }
    TEST_ASSERT_EQ_INT(c.state, OVP_NORMAL);

    /* 7) 过温 95℃ 主动停泄放 + 单次计次，降到 65℃ 以下恢复（温度自恢复） */
    TEST_CASE("OVP: 过温95℃停泄放+单次计次+65℃恢复");
    Ovp_Init(&c);
    for (i = 0; i < 5; i++) { Ovp_Step(&c, 58000, 30000, 0U, 1U); }  /* 进 OVER，泄放中 */
    TEST_ASSERT_EQ_INT(c.state, OVP_OVER_VOLTAGE);
    TEST_ASSERT(Bsp_Discharge_GetDuty() > 0);
    Ovp_Step(&c, 58000, 105000, 0U, 1U);        /* 105℃ 过温 → 停泄放 + 计次 */
    TEST_ASSERT_EQ_INT(c.otp_count, 1);         /* 单次计次 +1 */
    TEST_ASSERT_EQ_INT(c.otp_active, 1);        /* 过温停泄放态 */
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), 0);  /* 已停泄放 */
    Ovp_Step(&c, 58000, 105000, 0U, 1U);        /* 持续105℃ 不重复计次、维持停泄放 */
    TEST_ASSERT_EQ_INT(c.otp_count, 1);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), 0);
    Ovp_Step(&c, 58000, 80000, 0U, 1U);         /* 降到80℃(>65恢复阈值) 仍过温停泄放、不清计次 */
    TEST_ASSERT_EQ_INT(c.otp_active, 1);
    TEST_ASSERT_EQ_INT(c.otp_count, 1);
    Ovp_Step(&c, 58000, 60000, 0U, 1U);         /* 降到60℃(≤65恢复) 解除过温，仍过压→重新泄放 */
    TEST_ASSERT_EQ_INT(c.otp_active, 0);
    for (i = 0; i < 5; i++) { Ovp_Step(&c, 58000, 60000, 0U, 1U); }  /* 重新确认进 OVER 泄放 */
    TEST_ASSERT_EQ_INT(c.state, OVP_OVER_VOLTAGE);
    TEST_ASSERT(Bsp_Discharge_GetDuty() > 0);
    Ovp_Step(&c, 58000, 105000, 0U, 1U);        /* 再过温 → 计次+1（恢复后可再计） */
    TEST_ASSERT_EQ_INT(c.otp_count, 2);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), 0);

    /* 8) 二级过压：关电机使能 + 计次 */
    TEST_CASE("OVP: 二级过压关电机使能并计次");
    Ovp_Init(&c);
    Ovp_Step(&c, 61000, 30000, 0U, 1U);
    TEST_LOG_VAL("61V过压: l2_alarm=%u motor_disabled=%u l2_count=%lu",
                 c.l2_alarm, c.motor_disabled, (unsigned long)c.l2_count);
    TEST_ASSERT_EQ_INT(c.l2_alarm, 1);
    TEST_ASSERT_EQ_INT(c.motor_disabled, 1);
    TEST_ASSERT_EQ_INT(c.l2_count, 1);

    /* 9) 一级过压：报警 + 泄放（58V 触发 L1，仅记录不急停） */
    TEST_CASE("OVP: 一级过压报警 (58V)");
    Ovp_Init(&c);
    Ovp_Step(&c, 58500, 30000, 0U, 1U);
    TEST_ASSERT_EQ_INT(c.l1_alarm, 1);

    /* 10) 控制字 0x10 强制全开 */
    TEST_CASE("OVP: 控制字0x10强制全开");
    Ovp_Init(&c);
    for (i = 0; i < 300; i++) { Ovp_Step(&c, 50000, 30000, BSP_OVP_CTRL_FORCE_FULL, 1U); }
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), BSP_DISCHARGE_DUTY_MAX);

    /* 收尾：确保关断 */
    Bsp_Discharge_Stop();
}

