/**
 * @file    bsp_discharge.c
 * @brief   PWM泄放控制 BSP 驱动 (GD32F303CCT6)
 *
 * PA8  — TIMER0 CH0 PWM 输出 → NSi6601B 隔离栅驱动 → CSD19532Q5BT MOSFET
 * PA9  — Brake_FB 反馈监测 (泄放电阻组电压反馈)
 *
 * 控制频率 20kHz, 占空比工作区 5~90%(硬顶 90%, ★暂定值测试按串口数据重标),
 *   过压期间以固定斜率(dI/dt)增加占空比以限制 MOS+电阻热冲击。
 *
 * 对应需求: REQ-FUNC-010~014 (PWM自适应泄放), REQ-FUNC-024 (回路故障诊断),
 *           REQ-FUNC-014/REQ-SAFE-001 (上电默认关断安全态), REQ-SAFE-005 (禁止超额定功率)
 *
 * @version 1.2.0
 * @date    2026-06-13
 */

#define BSP_USE_DISCHARGE
#include "bsp.h"

/* PWM 计数时钟 (定时器分频后的计数频率), 取 10MHz:
 *   period = 计数时钟 / PWM频率; 20kHz 时 period=500, 占空比分辨率 0.2% */
#define BSP_DISCHARGE_TIMER_CK_HZ   10000000U

/* ---------------------------------------------------------------- */
static volatile uint8_t s_discharge_initialized = 0U;
static uint16_t s_pwm_period = 0U;          /* 自动重装值 (period), Init 时按主频算出 */
static uint8_t  s_cur_duty   = 0U;          /* 当前生效占空比 (%), 用于读回与斜坡控制 */
static uint8_t  s_pin_af     = 1U;          /* PA8 引脚模式: 1=AF(TIMER0 PWM), 0=GPIO强制低(空闲防误泄放) */
static uint32_t s_pwm_freq_hz = BSP_DISCHARGE_PWM_FREQ_HZ; /* 当前 PWM 频率(Hz), 默认宏值; 可经 SetFreq 配置(供 App 从 FRAM 配置初始化), GetFreq 读回 */

/* ---------------------------------------------------------------- */
/**
 * 功能：配置 PWM 输出 GPIO（PA8 复用推挽）与 Brake_FB 反馈输入 GPIO（PA9 浮空）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Discharge_GpioConfig(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /* PA8 = TIMER0_CH0, 复用推挽输出 */
    gpio_init(BSP_DISCHARGE_PWM_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              BSP_DISCHARGE_PWM_PIN);

    /* PA9 = Brake_FB, 浮空输入 (泄放回路状态反馈) */
    gpio_init(BSP_DISCHARGE_FB_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,
              BSP_DISCHARGE_FB_PIN);

    /* PB0 = 泄放指示灯(低有效), 推挽输出, 上电默认拉高(灭, 未泄放);
     * 泄放中由主循环 AppDch_ServiceDchLed → Bsp_DchLed_Set 点亮, 停止/完成灭。 */
    rcu_periph_clock_enable(BSP_DCH_LED_RCU);
    gpio_init(BSP_DCH_LED_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,
              BSP_DCH_LED_PIN);
    gpio_bit_set(BSP_DCH_LED_PORT, BSP_DCH_LED_PIN);   /* 高 = 灭（上电未泄放） */
}

/**
 * 功能：泄放指示灯（PB0，低有效）亮/灭控制。由主循环 AppDch_ServiceDchLed 按泄放标志位调用。
 * 输入参数：on —— 非0=点亮(拉低)，0=熄灭(拉高)。
 * 输出参数：无
 * 返回值：无
 */
void Bsp_DchLed_Set(uint8_t on)
{
    if (on) {
        gpio_bit_reset(BSP_DCH_LED_PORT, BSP_DCH_LED_PIN);   /* 低电平 = 点亮 */
    } else {
        gpio_bit_set(BSP_DCH_LED_PORT, BSP_DCH_LED_PIN);     /* 高电平 = 熄灭 */
    }
}

/**
 * 功能：配置 TIMER0 CH0 PWM（按主频与目标频率算分频/period，初始占空比 0%，失驱默认低电平安全态）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Discharge_TimerConfig(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct    timer_initpara;
    uint32_t prescaler;

    rcu_periph_clock_enable(RCU_TIMER0);
    timer_deinit(BSP_DISCHARGE_TIMER);

    /* 安全: 调试器 halt core 时不冻结 TIMER0(清 DBG_TIMER0_HOLD)。
     * 若冻结(=1), 计数器停在随机相位会把 PA8 锁存为随机高/低(常高即危险);
     * 不冻结(=0)则 halt 时计数器继续跑, 配合默认 duty=0 -> PA8 确定保持低。
     * 注意: 此位可能被 IDE「halt 停外设」选项重置, 须同时关闭该 IDE 选项;
     * 且 CPU 真正 halt 时固件无法关断, 最终安全仍须硬件失效保护保证。 */
    dbg_periph_disable(DBG_TIMER0_HOLD);

    /* 计数时钟 = SystemCoreClock / (prescaler+1) = BSP_DISCHARGE_TIMER_CK_HZ
     * 100MHz / 10MHz = 10 -> prescaler = 9 (按主频自适应, 不写死) */
    prescaler   = (SystemCoreClock / BSP_DISCHARGE_TIMER_CK_HZ);
    if (prescaler > 0U) {
        prescaler -= 1U;
    }
    /* period = 计数时钟 / PWM频率 - 1; 10MHz/20kHz=500 -> period=499
     * 频率取运行时值 s_pwm_freq_hz(默认宏, 可由 SetFreq 配置), 便于上电从 FRAM 配置初始化 */
    s_pwm_period = (uint16_t)(BSP_DISCHARGE_TIMER_CK_HZ / s_pwm_freq_hz);
    if (s_pwm_period > 0U) {
        s_pwm_period -= 1U;
    }

    timer_initpara.prescaler         = (uint16_t)prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = s_pwm_period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0U;
    timer_init(BSP_DISCHARGE_TIMER, &timer_initpara);

    /* CH0 PWM 配置; ocidlestate 低 -> 失驱默认低电平, 与硬件栅极下拉一致(安全态) */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(BSP_DISCHARGE_TIMER, TIMER_CH_0, &timer_ocintpara);

    /* 初始比较值 = 0 -> 占空比 0% = 默认关断 (安全态) */
    timer_channel_output_pulse_value_config(BSP_DISCHARGE_TIMER, TIMER_CH_0, 0U);
    timer_channel_output_mode_config(BSP_DISCHARGE_TIMER, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    /* ★比较寄存器开影子(preload): 写 CH0CV 缓存到更新事件(周期边界)才原子生效, 而非立即异步生效。
     * 消除步进改占空比时的中途杂散跳变沿(毛刺)——原 SHADOW_DISABLE 下, TIMER1(50µs)与本 PWM(50µs)
     * 不同步, 几乎每个周期在任意相位改比较值会让输出中途翻低又翻高(notch), MOS 抖动→母线密集毛刺。
     * 代价: 占空比变更最多晚 1 个 PWM 周期(≤50µs)生效(相对 <1ms 响应可忽略); 硬关断走 ForcePinLow(GPIO)不受影响。 */
    timer_channel_output_shadow_config(BSP_DISCHARGE_TIMER, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);

    timer_primary_output_config(BSP_DISCHARGE_TIMER, ENABLE);
    timer_auto_reload_shadow_enable(BSP_DISCHARGE_TIMER);
    timer_enable(BSP_DISCHARGE_TIMER);
}

/* ---------------------------------------------------------------- */
/**
 * 功能：初始化泄放 PWM 驱动（GPIO + 定时器），确保上电默认关断。幂等。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Discharge_Status_T，BSP_DISCHARGE_OK 表示成功。
 */
Bsp_Discharge_Status_T Bsp_Discharge_Init(void)
{
    if (s_discharge_initialized) {
        return BSP_DISCHARGE_OK;
    }
    Bsp_Discharge_GpioConfig();
    Bsp_Discharge_TimerConfig();
    Bsp_Discharge_Stop();           /* 确保上电默认关断 */
    s_discharge_initialized = 1U;
    return BSP_DISCHARGE_OK;
}

/**
 * 功能：查询泄放 PWM 驱动是否已初始化。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，非 0=已初始化，0=未初始化。
 */
int Bsp_Discharge_IsInitialized(void)
{
    return (int)s_discharge_initialized;
}

/**
 * 功能：设置 PWM 占空比（超安全硬顶 BSP_DISCHARGE_DUTY_MAX 则钳制）。
 * 输入参数：duty_percent —— 目标占空比（%）。
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Discharge_SetDuty(uint8_t duty_percent)
{
    uint32_t pulse;

    /* 若 PA8 曾被空闲防误泄放守护强制为 GPIO 低，起泄放前恢复为 TIMER0 AF 输出，PWM 才生效 */
    if (s_pin_af == 0U) {
        gpio_init(BSP_DISCHARGE_PWM_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BSP_DISCHARGE_PWM_PIN);
        s_pin_af = 1U;
    }

    /* 物理绝对钳制 100%（占空比不可超 100）。一级封顶 90% 由上层级别机 StepIsr 控制,
     * 二级泄放需 100% 全开,故此处钳制放到物理上限而非 DUTY_MAX(90,一级封顶)。 */
    if (duty_percent > 100U) {
        duty_percent = 100U;
    }
    /* 比较值 = (period+1) × duty% / 100 */
    pulse = ((uint32_t)s_pwm_period + 1U) * (uint32_t)duty_percent / 100U;
    timer_channel_output_pulse_value_config(BSP_DISCHARGE_TIMER, TIMER_CH_0, pulse);
    s_cur_duty = duty_percent;
}

/**
 * 功能：读回当前生效占空比。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint8_t，当前占空比（%）。
 */
uint8_t Bsp_Discharge_GetDuty(void)
{
    return s_cur_duty;
}

/**
 * 功能：读回当前 PWM 频率。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint32_t，当前 PWM 频率（Hz）。
 */
uint32_t Bsp_Discharge_GetFreq(void)
{
    return s_pwm_freq_hz;
}

/**
 * 功能：设置 PWM 频率（越界拒绝；已初始化时立即重算 period 并保持占空比生效）。
 * 输入参数：freq_hz —— 目标频率（Hz）。
 * 输出参数：无
 * 返回值：Bsp_Discharge_Status_T，OK 成功，ERR_PARAM 频率越界。
 */
Bsp_Discharge_Status_T Bsp_Discharge_SetFreq(uint32_t freq_hz)
{
    /* 频率合法范围校验; 越界拒绝, 保持原频率(供上层非法→默认回退判定) */
    if (freq_hz < BSP_DISCHARGE_PWM_FREQ_MIN_HZ ||
        freq_hz > BSP_DISCHARGE_PWM_FREQ_MAX_HZ) {
        return BSP_DISCHARGE_ERR_PARAM;
    }
    s_pwm_freq_hz = freq_hz;

    /* 若已初始化, 立即重算 period 并按当前占空比重配, 使新频率即时生效 */
    if (s_discharge_initialized) {
        uint8_t duty = s_cur_duty;
        s_pwm_period = (uint16_t)(BSP_DISCHARGE_TIMER_CK_HZ / s_pwm_freq_hz);
        if (s_pwm_period > 0U) {
            s_pwm_period -= 1U;
        }
        timer_autoreload_value_config(BSP_DISCHARGE_TIMER, s_pwm_period);
        Bsp_Discharge_SetDuty(duty);    /* 以新 period 重算比较值, 保持占空比 */
    }
    return BSP_DISCHARGE_OK;
}

/**
 * 功能：朝目标占空比按固定步进 BSP_DISCHARGE_DUTY_STEP 斜坡一步（限制 dI/dt 热冲击）。
 * 输入参数：target_percent —— 目标占空比（%，超上限自动钳制）。
 * 输出参数：无
 * 返回值：uint8_t，本步后实际生效占空比（%）。
 */
uint8_t Bsp_Discharge_RampToDuty(uint8_t target_percent)
{
    uint8_t next;

    /* 目标先钳制到上限 */
    if (target_percent > BSP_DISCHARGE_DUTY_MAX) {
        target_percent = BSP_DISCHARGE_DUTY_MAX;
    }
    /* 朝目标按 DUTY_STEP 步进一步 (固定斜率, 限制 dI/dt 热冲击) */
    if (s_cur_duty < target_percent) {
        next = s_cur_duty + BSP_DISCHARGE_DUTY_STEP;
        if (next > target_percent) { next = target_percent; }
    } else if (s_cur_duty > target_percent) {
        next = (s_cur_duty > BSP_DISCHARGE_DUTY_STEP)
             ? (uint8_t)(s_cur_duty - BSP_DISCHARGE_DUTY_STEP) : 0U;
        if (next < target_percent) { next = target_percent; }
    } else {
        next = s_cur_duty;
    }
    Bsp_Discharge_SetDuty(next);
    return s_cur_duty;
}

/**
 * 功能：停止泄放（占空比置 0%、MOSFET 截止，进入安全态）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Discharge_Stop(void)
{
    /* 比较值置 0 -> 占空比 0%, MOSFET 截止 (安全态) */
    timer_channel_output_pulse_value_config(BSP_DISCHARGE_TIMER, TIMER_CH_0, 0U);
    s_cur_duty = 0U;
}

/**
 * 功能：空闲防误泄放——把泄放引脚 PA8 由 TIMER0 AF 切为普通 GPIO 推挽输出并拉低，
 *       强制 MOSFET 截止（比 duty=0 更硬：即便 PWM 通路异常也确保引脚为低）。
 *       起泄放时 Bsp_Discharge_SetDuty 会自动恢复 AF。幂等。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 * 说明：由 AppDch_StepIsr 空闲守护调用（母线<56.5V 且 A9=PA8 状态检测为高时）。
 */
void Bsp_Discharge_ForcePinLow(void)
{
    if (s_pin_af != 0U) {
        gpio_init(BSP_DISCHARGE_PWM_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, BSP_DISCHARGE_PWM_PIN);
        s_pin_af = 0U;
    }
    gpio_bit_reset(BSP_DISCHARGE_PWM_PORT, BSP_DISCHARGE_PWM_PIN);
    s_cur_duty = 0U;
}

/**
 * 功能：读取 Brake_FB（PA9）电平，用于泄放回路开/短路诊断。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，PA9 当前电平（0/1，高=异常）。
 */
int Bsp_Discharge_ReadFeedback(void)
{
    /* 读取 PA9 Brake_FB 电平, 用于回路开/短路诊断 */
    return (int)gpio_input_bit_get(BSP_DISCHARGE_FB_PORT, BSP_DISCHARGE_FB_PIN);
}
