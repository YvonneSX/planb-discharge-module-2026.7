/**
 * @file    test_bsp_discharge.c
 * @brief   PWM泄放控制单元测试 实现
 *
 * 覆盖接口: Init / IsInitialized / SetDuty / GetDuty / GetFreq / SetFreq /
 *           RampToDuty / Stop / ReadFeedback
 *
 * 验证手段: 通过读 TIMER0 CH0 比较值寄存器 TIMER_CH0CV 反算实际占空比。
 *   安全约束验证(重点): ① 上限硬钳制 ≤ DUTY_MAX(90%); ② 绝不出现 100%/常高电平
 *   (即任意设定下 pulse < period+1, 始终保留低电平段); ③ 停止=0; ④ 设值/斜坡正确。
 *   操作型占空比测试点取安全值 BSP_DISCHARGE_TEST_DUTY(5%), 仅在钳制安全测试里触及 90% 上限边界
 *   (⚠ 该断言会瞬间输出 90% PWM, 真机跑 make test/RunAll 须确认泄放负载与散热)。
 *   实测减压功能请用 make discharge-live (48V 供电 + 泄放负载, 占空比自动扫描:
 *   4% 起每档最多 20 秒、每次 +1%、上限 25%; 母线降到 <45V 即判减压成功并关断;
 *   到 25% 仍 20 秒降不到 45V 则报错; 每 200ms 精简打印电压/电流/温度/占空比/反馈)。
 *
 * @version 1.8.0
 * @date    2026-06-15
 */

#include "test_bsp_discharge.h"

#define BSP_USE_DISCHARGE
#define BSP_USE_UART_LOG        /* 监视模式用 UartLog 调试打印 (不使用串口接收) */
#define BSP_USE_ANALOG          /* PWM 验证时同屏实时打印母线电流 */
#include "bsp.h"
#include "uart_log.h"
#include "systick.h"

/* PWM 计数时钟与 bsp_discharge.c 内部一致 (10MHz), 用于反算期望比较值 */
#define TEST_DISCHARGE_TIMER_CK_HZ   10000000U
#define TEST_DISCHARGE_PERIOD        ((TEST_DISCHARGE_TIMER_CK_HZ / BSP_DISCHARGE_PWM_FREQ_HZ) - 1U)

static uint32_t Test_Discharge_GetPulse(void)
{
    return (uint32_t)TIMER_CH0CV(BSP_DISCHARGE_TIMER);
}

static uint32_t Test_Discharge_GetPeriod(void)
{
    return (uint32_t)TIMER_CAR(BSP_DISCHARGE_TIMER);   /* 自动重装值寄存器 */
}

void Test_BspDischarge_RunAll(Test_Stats_T *_stats)
{
    uint32_t pulse, period_plus1, duty_max_pulse;
    int fb;

    period_plus1   = TEST_DISCHARGE_PERIOD + 1U;
    duty_max_pulse = period_plus1 * (uint32_t)BSP_DISCHARGE_DUTY_MAX / 100U;
    TEST_LOG_VAL("PWM: ck=10MHz period=%lu freq=%uHz DUTY_MAX=%u%%",
                 (unsigned long)TEST_DISCHARGE_PERIOD, BSP_DISCHARGE_PWM_FREQ_HZ, BSP_DISCHARGE_DUTY_MAX);

    /* ---- 初始化默认关断 (安全态) ---- */
    TEST_CASE("Discharge: 初始化默认关断");
    TEST_ASSERT_EQ(Bsp_Discharge_Init(), BSP_DISCHARGE_OK);
    TEST_ASSERT(Bsp_Discharge_IsInitialized() != 0);
    TEST_LOG_VAL("上电 pulse=%lu (应为0)", (unsigned long)Test_Discharge_GetPulse());
    TEST_ASSERT_EQ_INT(Test_Discharge_GetPulse(), 0);

    TEST_CASE("Discharge: 重复初始化幂等");
    TEST_ASSERT_EQ(Bsp_Discharge_Init(), BSP_DISCHARGE_OK);

    /* ---- 正常占空比设值: 安全测试占空比 (BSP_DISCHARGE_TEST_DUTY) ---- */
    TEST_CASE("Discharge: 设安全测试占空比");
    Bsp_Discharge_SetDuty(BSP_DISCHARGE_TEST_DUTY);
    pulse = Test_Discharge_GetPulse();
    TEST_LOG_VAL("SetDuty(%u): pulse=%lu (期望~%lu) GetDuty=%u%%",
                 BSP_DISCHARGE_TEST_DUTY, (unsigned long)pulse,
                 (unsigned long)(period_plus1 * BSP_DISCHARGE_TEST_DUTY / 100U), Bsp_Discharge_GetDuty());
    TEST_ASSERT_IN_RANGE(pulse, (period_plus1 * BSP_DISCHARGE_TEST_DUTY / 100U) - 1U,
                                (period_plus1 * BSP_DISCHARGE_TEST_DUTY / 100U) + 1U);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), BSP_DISCHARGE_TEST_DUTY);

    /* ---- 安全: 上限钳制 设 100% 应被钳到 DUTY_MAX(90%) ---- */
    TEST_CASE("Discharge: 占空比上限钳制 (100->10)");
    Bsp_Discharge_SetDuty(100U);
    pulse = Test_Discharge_GetPulse();
    TEST_LOG_VAL("SetDuty(100): GetDuty=%u%% (应钳到%u) pulse=%lu",
                 Bsp_Discharge_GetDuty(), BSP_DISCHARGE_DUTY_MAX, (unsigned long)pulse);
    TEST_ASSERT_IN_RANGE(pulse, duty_max_pulse - 1U, duty_max_pulse + 1U);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), BSP_DISCHARGE_DUTY_MAX);

    /* ---- 安全(关键): 即使设 100%/255%, 输出也绝不能是常高/100% ----
     * PWM0 模式下 CNT∈[0,period], pulse 必须 < period+1 才能保留低电平段;
     * 90% 钳制 → pulse=duty_max_pulse < period+1, 故不可能常高。 */
    TEST_CASE("Discharge: 绝不出现 100%/常高电平");
    Bsp_Discharge_SetDuty(255U);    /* 极端输入 */
    pulse = Test_Discharge_GetPulse();
    TEST_LOG_VAL("SetDuty(255): pulse=%lu 必须 < period+1=%lu (有低电平段=非常高)",
                 (unsigned long)pulse, (unsigned long)period_plus1);
    TEST_ASSERT(pulse < period_plus1);
    TEST_ASSERT(Bsp_Discharge_GetDuty() <= BSP_DISCHARGE_DUTY_MAX);

    /* ---- 斜坡: 从 0 朝 45% 步进 ---- */
    TEST_CASE("Discharge: 斜坡步进 RampToDuty");
    Bsp_Discharge_Stop();
    Bsp_Discharge_RampToDuty(BSP_DISCHARGE_TEST_DUTY);
    TEST_LOG_VAL("Ramp 第1步: GetDuty=%u%% (应=DUTY_STEP=%u)",
                 Bsp_Discharge_GetDuty(), BSP_DISCHARGE_DUTY_STEP);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), BSP_DISCHARGE_DUTY_STEP);

    /* ---- 停止: 占空比归零 ---- */
    TEST_CASE("Discharge: 停止进入安全态");
    Bsp_Discharge_Stop();
    TEST_LOG_VAL("Stop 后 pulse=%lu GetDuty=%u%% (应为0)",
                 (unsigned long)Test_Discharge_GetPulse(), Bsp_Discharge_GetDuty());
    TEST_ASSERT_EQ_INT(Test_Discharge_GetPulse(), 0);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetDuty(), 0);

    /* ---- PWM 频率: 回读 + 运行时配置 (支撑“频率回读 + 从 FRAM 配置初始化”) ---- */
    TEST_CASE("Discharge: 频率回读默认值");
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetFreq(), BSP_DISCHARGE_PWM_FREQ_HZ);

    TEST_CASE("Discharge: SetFreq 越界拒绝 (不改原值)");
    TEST_ASSERT_EQ(Bsp_Discharge_SetFreq(BSP_DISCHARGE_PWM_FREQ_MIN_HZ - 1U), BSP_DISCHARGE_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Discharge_SetFreq(BSP_DISCHARGE_PWM_FREQ_MAX_HZ + 1U), BSP_DISCHARGE_ERR_PARAM);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetFreq(), BSP_DISCHARGE_PWM_FREQ_HZ);

    TEST_CASE("Discharge: SetFreq 合法即时生效 (period 随频率变化)");
    TEST_ASSERT_EQ(Bsp_Discharge_SetFreq(10000U), BSP_DISCHARGE_OK);
    TEST_ASSERT_EQ_INT(Bsp_Discharge_GetFreq(), 10000U);
    /* 10MHz/10kHz - 1 = 999 */
    TEST_LOG_VAL("SetFreq(10kHz): GetFreq=%uHz period=%lu (期望 999)",
                 Bsp_Discharge_GetFreq(), (unsigned long)Test_Discharge_GetPeriod());
    TEST_ASSERT_EQ_INT(Test_Discharge_GetPeriod(), 999U);
    /* 恢复默认频率, 避免影响后续/真机演示 */
    TEST_ASSERT_EQ(Bsp_Discharge_SetFreq(BSP_DISCHARGE_PWM_FREQ_HZ), BSP_DISCHARGE_OK);
    TEST_ASSERT_EQ_INT(Test_Discharge_GetPeriod(), TEST_DISCHARGE_PERIOD);

    /* ---- 反馈读取 (单次快照仅自检电平合法; 实时观测用 make discharge-live) ---- */
    TEST_CASE("Discharge: Brake_FB 反馈电平读取");
    fb = Bsp_Discharge_ReadFeedback();
    TEST_LOG_VAL("Brake_FB(PA9) = %d (单次快照, 实时观测用 make discharge-live)", fb);
    TEST_ASSERT_IN_RANGE(fb, 0, 1);

    Bsp_Discharge_Stop();
}

/* ================================================================
 * 泄放减压功能验证模式 — 占空比自动扫描 (make discharge-live)
 * ----------------------------------------------------------------
 * 用途: 在 48V 供电 + 泄放功率级(MOSFET/泄放电阻)接好的前提下, 从 4% 起逐档
 *   提升 PWM 占空比, 验证泄放(减压)功能: 把母线从 48V 泄放到 45V 以下。每 200ms
 *   实时打印母线电压/电流/温度/占空比/反馈。
 * 判定 (绝对阈值, 基于 48V 供电, 目标减压到 <45V):
 *   - 起始 BSP_DISCHARGE_TEST_DUTY_START(4%), 每档最多持续 BSP_DISCHARGE_TEST_STEP_MS(20秒);
 *   - 任一时刻母线降到 <BSP_DISCHARGE_TEST_VBUS_OFF_MV(45V) 并连续 OFF_CONFIRM 次(=1s)
 *     -> 减压成功, 关断 PWM, 测试通过(连续确认可滤掉瞬时跌落毛刺);
 *   - 本档 20 秒仍未降到 <45V -> 占空比 +BSP_DISCHARGE_TEST_DUTY_STEP(1%) 进入下一档;
 *   - 占空比上限 BSP_DISCHARGE_TEST_DUTY_MAX(10%, 本冒烟测试的保守本地上限, 低于硬钳 DUTY_MAX=90%);
 *   - 已达本地上限 10% 且再持续 20 秒仍降不到 <45V -> 报错(提示检查功率级/负载/栅驱动), 关断结束。
 *   一次性单元测试: 结束(通过或报错)后保持关断, 不再重开。
 * 交互: 纯调试打印, 不使用串口接收。上电先倒计时 BSP_DISCHARGE_START_DELAY_MS,
 *   期间 PA8 保持低; 非阻塞: 仅靠 systick_get_ms() 判周期, 无阻塞延时。
 * ⚠ 安全: 本模式会真实输出 PWM 并真实泄放! 占空比硬钳 <=DUTY_MAX, 绝不 100%/常高。
 *   复位即关断。CPU 被调试器 halt 时固件无法关断, 须靠硬件失效保护。
 * 备注: 若用台式"硬"稳压源(会补流把母线钳在 48V 不降), 则会一路扫到上限 10% 报错;
 *   要能降到 45V 须用限流电源(进 CC)或电容供电。
 * ================================================================ */
#define BSP_DISCHARGE_START_DELAY_MS    3000U   /* 起 PWM 前的安全倒计时窗口 */
#define BSP_DISCHARGE_MON_PERIOD_MS     200U    /* 本模式电压/电流刷新周期 (看降压曲线更细) */
#define BSP_DISCHARGE_TEST_DUTY_START   4U      /* 扫描起始占空比 (%) */
#define BSP_DISCHARGE_TEST_DUTY_MAX     10U     /* 扫描占空比上限 (%, =硬钳 DUTY_MAX=10) */
#define BSP_DISCHARGE_TEST_DUTY_STEP    1U      /* 每档占空比步进 (%) */
#define BSP_DISCHARGE_TEST_STEP_MS      20000U  /* 每档持续时长 (20 秒) */
#define BSP_DISCHARGE_TEST_VBUS_OFF_MV  45000   /* 减压目标: 母线降到此值以下即判减压成功 (45V) */
#define BSP_DISCHARGE_TEST_OFF_CONFIRM  5U      /* 连续多少次采样确认"已低于目标" (5*200ms=1s, 抗瞬时毛刺) */

/* 扫描阶段 */
#define DCH_PHASE_COUNTDOWN  0U
#define DCH_PHASE_SWEEP      1U
#define DCH_PHASE_PASS       2U
#define DCH_PHASE_FAIL       3U

void Test_BspDischarge_Monitor(void)
{
    uint32_t n = 0U;
    uint32_t last_print_ms;
    uint32_t start_ms;
    uint32_t step_start_ms = 0U;
    uint32_t sweep_start_ms = 0U;   /* 起 PWM(开始泄放)的时刻 */
    uint32_t reach_ms = 0U;         /* 母线首次跌破目标(本次持续段起点)的时刻 */
    uint32_t pass_elapsed_ms = 0U;  /* 起 PWM -> 降到<目标 的耗时 */
    uint8_t  phase = DCH_PHASE_COUNTDOWN;
    uint8_t  duty = BSP_DISCHARGE_TEST_DUTY_START;
    uint8_t  off_confirm = 0U;

    (void)Bsp_Discharge_Init();   /* 初始化即关断 (PA8 低) */
    Bsp_Discharge_Stop();
    (void)Bsp_Analog_Init();      /* 电压/电流采样 (ADC+DMA 后台刷新) */

    UartLog_Println("\r\n=== Discharge 减压验证 占空比自动扫描 (48V 供电, 纯打印, 刷新 %ums) ===",
                    (unsigned int)BSP_DISCHARGE_MON_PERIOD_MS);
    UartLog_Println("!! 警告: %us 后将在 PA8 起 %u%% PWM 真实泄放, 请先确认 48V 供电与泄放负载接好 !!",
                    (unsigned int)(BSP_DISCHARGE_START_DELAY_MS / 1000U), BSP_DISCHARGE_TEST_DUTY_START);
    UartLog_Println("扫描: 起%u%% 每+%u%% 每档%us 上限%u%%; 母线降到<%dmV(45V) 即判减压成功并关 PWM; 频率=%uHz 硬钳<=%u%%",
                    BSP_DISCHARGE_TEST_DUTY_START, BSP_DISCHARGE_TEST_DUTY_STEP,
                    (unsigned int)(BSP_DISCHARGE_TEST_STEP_MS / 1000U), BSP_DISCHARGE_TEST_DUTY_MAX,
                    BSP_DISCHARGE_TEST_VBUS_OFF_MV, BSP_DISCHARGE_PWM_FREQ_HZ, BSP_DISCHARGE_DUTY_MAX);

    start_ms = systick_get_ms();
    last_print_ms = start_ms;

    for (;;) {
        uint32_t now = systick_get_ms();

        /* 倒计时结束: 进入扫描, 起第一档 PWM */
        if (phase == DCH_PHASE_COUNTDOWN &&
            (uint32_t)(now - start_ms) >= BSP_DISCHARGE_START_DELAY_MS) {
            duty = BSP_DISCHARGE_TEST_DUTY_START;
            Bsp_Discharge_SetDuty(duty);
            off_confirm = 0U;
            step_start_ms = now;
            sweep_start_ms = now;
            phase = DCH_PHASE_SWEEP;
            UartLog_Println(">>> 起 PWM 扫描: %u%% @ %uHz, 目标减压到 <%dmV", duty,
                            BSP_DISCHARGE_PWM_FREQ_HZ, BSP_DISCHARGE_TEST_VBUS_OFF_MV);
        }

        if ((uint32_t)(now - last_print_ms) >= BSP_DISCHARGE_MON_PERIOD_MS) {
            int32_t v_mV = Bsp_Analog_GetVBus_mV();
            int32_t i_mA = Bsp_Analog_GetCurrent_mA();
            int32_t t_mC = Bsp_Analog_GetTemp_mDegC();
            int32_t t_C  = (t_mC + (t_mC >= 0 ? 500 : -500)) / 1000;  /* 四舍五入到整度 */
            last_print_ms = now;

            if (phase == DCH_PHASE_SWEEP) {
                /* 减压目标确认: 连续低于 45V 才累计, 一旦回升即清零(滤掉瞬时毛刺) */
                if (v_mV < BSP_DISCHARGE_TEST_VBUS_OFF_MV) {
                    if (off_confirm == 0U) {
                        reach_ms = now;   /* 本次持续低于目标的起点 = 首次跌破目标时刻 */
                    }
                    off_confirm++;
                } else {
                    off_confirm = 0U;
                }

                if (off_confirm >= BSP_DISCHARGE_TEST_OFF_CONFIRM) {
                    /* 母线已稳定降到 <45V -> 减压成功, 关 PWM, 测试通过 */
                    Bsp_Discharge_Stop();
                    phase = DCH_PHASE_PASS;
                    pass_elapsed_ms = (uint32_t)(reach_ms - sweep_start_ms);  /* 起PWM->降到目标耗时 */
                    UartLog_Println(">>> 减压成功: %u%% 下母线降到 %ldmV (<%dmV 目标), 耗时 %lu.%lus (起PWM->达标), 关断 PWM, 测试通过 <<<",
                                    duty, (long)v_mV, BSP_DISCHARGE_TEST_VBUS_OFF_MV,
                                    (unsigned long)(pass_elapsed_ms / 1000U),
                                    (unsigned long)((pass_elapsed_ms % 1000U) / 100U));
                } else if ((uint32_t)(now - step_start_ms) >= BSP_DISCHARGE_TEST_STEP_MS) {
                    /* 本档 20s 仍未降到 45V 以下 */
                    if ((uint32_t)duty + BSP_DISCHARGE_TEST_DUTY_STEP <= (uint32_t)BSP_DISCHARGE_TEST_DUTY_MAX) {
                        uint8_t old = duty;
                        duty = (uint8_t)(duty + BSP_DISCHARGE_TEST_DUTY_STEP);
                        Bsp_Discharge_SetDuty(duty);
                        off_confirm = 0U;
                        step_start_ms = now;
                        UartLog_Println(">>> 升档: %u%%->%u%% (上一档 %us 内母线未降到 <%dmV)",
                                        old, duty, (unsigned int)(BSP_DISCHARGE_TEST_STEP_MS / 1000U),
                                        BSP_DISCHARGE_TEST_VBUS_OFF_MV);
                    } else {
                        /* 已在上限 25%, 仍降不到 45V 以下 -> 报错 */
                        Bsp_Discharge_Stop();
                        phase = DCH_PHASE_FAIL;
                        UartLog_Println(">>> 错误: 占空比已达上限 %u%% 且持续 %us 仍降不到 <%dmV! 请检查泄放功率级/负载/栅驱动副边电源/电流传感器 <<<",
                                        duty, (unsigned int)(BSP_DISCHARGE_TEST_STEP_MS / 1000U),
                                        BSP_DISCHARGE_TEST_VBUS_OFF_MV);
                    }
                }
            }

            /* 打印 (精简: 序号 状态 V(相对基准d) I T FB) */
            if (phase == DCH_PHASE_COUNTDOWN) {
                UartLog_Println("[%lu] D0%% 倒计时%lus V=%ldmV I=%ldmA T=%ldC F=%d",
                                (unsigned long)(n++),
                                (unsigned long)((BSP_DISCHARGE_START_DELAY_MS - (now - start_ms)) / 1000U + 1U),
                                (long)v_mV, (long)i_mA, (long)t_C, Bsp_Discharge_ReadFeedback());
            } else if (phase == DCH_PHASE_SWEEP) {
                UartLog_Println("[%lu] D%u%% %lu/%us V=%ldmV(目标<%d) I=%ldmA T=%ldC F=%d",
                                (unsigned long)(n++), duty,
                                (unsigned long)((now - step_start_ms) / 1000U),
                                (unsigned int)(BSP_DISCHARGE_TEST_STEP_MS / 1000U),
                                (long)v_mV, BSP_DISCHARGE_TEST_VBUS_OFF_MV,
                                (long)i_mA, (long)t_C, Bsp_Discharge_ReadFeedback());
            } else if (phase == DCH_PHASE_PASS) {
                UartLog_Println("[%lu] PASS D%u%% 达标耗时%lu.%lus V=%ldmV I=%ldmA T=%ldC F=%d",
                                (unsigned long)(n++), duty,
                                (unsigned long)(pass_elapsed_ms / 1000U),
                                (unsigned long)((pass_elapsed_ms % 1000U) / 100U),
                                (long)v_mV, (long)i_mA, (long)t_C, Bsp_Discharge_ReadFeedback());
            } else { /* DCH_PHASE_FAIL */
                UartLog_Println("[%lu] FAIL D%u%% %lus V=%ldmV I=%ldmA T=%ldC F=%d",
                                (unsigned long)(n++), duty,
                                (unsigned long)((now - step_start_ms) / 1000U),
                                (long)v_mV, (long)i_mA, (long)t_C, Bsp_Discharge_ReadFeedback());
            }
        }
    }
}
