/**
 * @file    bsp_analog.c
 * @brief   模拟采样 BSP 驱动 (GD32F303CCT6)
 *
 * PA0  — MCU_ADC_Current (CC6920BSO-40A 霍尔电流传感器, ±40A)
 * PA2  — MCU_ADC_VBUS    (AMC1311 隔离电压采样, 0~98V, 分压比 49:1)
 * PA4  — MCU_BRAKE_TEMP  (B57861S0103F040 10K NTC 温度传感器)
 *
 * 对应需求: REQ-FUNC-001/002/003/004 (母线电压/电流/温度采样与功率计算)
 *
 * @version 1.2.0
 * @date    2026-06-13
 */

#define BSP_USE_ANALOG
#include "bsp.h"

/* ---------------------------------------------------------------- */
static volatile uint8_t s_analog_initialized = 0U;
/* DMA 目标缓冲: 规则组三通道转换结果由 DMA 循环搬运至此 (顺序=转换顺序) */
static volatile uint16_t s_adc_raw[BSP_ANALOG_CH_NUM] = {0U, 0U, 0U};

/* 功率统计窗口状态 (REQ-FUNC-004) */
static int32_t  s_pwr_peak_mW = 0;          /* 峰值功率 (按绝对值取峰) */
static int64_t  s_pwr_acc_mW  = 0;          /* 平均累加器 */
static uint32_t s_pwr_cnt     = 0U;         /* 采样计数 */

/* ADC 滑动滤波状态 (过压判定抗误触发): filtered = (filtered*(2^n-1) + new) / 2^n */
static uint32_t s_filt_vbus = 0U;           /* 电压通道滤波累积值 (定点, 实际码=本值) */
static uint8_t  s_filt_inited = 0U;         /* 滤波器是否已用初值装载 */

/* ---------------------------------------------------------------- */
/**
 * 功能：配置三路采样 GPIO（PA0/PA2/PA4）为模拟输入。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Analog_GpioConfig(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(BSP_ANALOG_CURRENT_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, BSP_ANALOG_CURRENT_PIN);
    gpio_init(BSP_ANALOG_VBUS_PORT,    GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, BSP_ANALOG_VBUS_PIN);
    gpio_init(BSP_ANALOG_TEMP_PORT,    GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, BSP_ANALOG_TEMP_PIN);
}

/**
 * 功能：配置 DMA0 通道循环搬运 ADC0 规则组转换结果到 s_adc_raw[]。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Analog_DmaConfig(void)
{
    dma_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(RCU_DMA0);
    dma_deinit(BSP_ANALOG_DMA, BSP_ANALOG_DMA_CH);

    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.periph_addr  = (uint32_t)(&ADC_RDATA(BSP_ANALOG_ADC_INSTANCE));
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_addr  = (uint32_t)s_adc_raw;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.number       = BSP_ANALOG_CH_NUM;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init(BSP_ANALOG_DMA, BSP_ANALOG_DMA_CH, &dma_init_struct);

    dma_circulation_enable(BSP_ANALOG_DMA, BSP_ANALOG_DMA_CH);   /* 循环: 自动反复刷新 */
    dma_channel_enable(BSP_ANALOG_DMA, BSP_ANALOG_DMA_CH);
}

/**
 * 功能：配置 ADC0 规则组扫描三通道（电流/电压/温度），连续转换 + DMA + 软触发启动。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Analog_AdcConfig(void)
{
    rcu_periph_clock_enable(RCU_ADC0);
    /* ADC 时钟: APB2(120MHz)/12 = 10MHz ≤ 14MHz 上限（8M/120MHz方案；旧100MHz/8已不适用） */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV12);

    adc_deinit(BSP_ANALOG_ADC_INSTANCE);
    adc_mode_config(ADC_MODE_FREE);
    adc_data_alignment_config(BSP_ANALOG_ADC_INSTANCE, ADC_DATAALIGN_RIGHT);
    /* 扫描模式 + 连续转换: 三通道按序连续刷新 */
    adc_special_function_config(BSP_ANALOG_ADC_INSTANCE, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(BSP_ANALOG_ADC_INSTANCE, ADC_CONTINUOUS_MODE, ENABLE);

    adc_channel_length_config(BSP_ANALOG_ADC_INSTANCE, ADC_REGULAR_CHANNEL, BSP_ANALOG_CH_NUM);
    /* 转换顺序: 0=电流(CH0), 1=电压(CH2), 2=温度(CH4); 采样时间取长提高稳定性 */
    adc_regular_channel_config(BSP_ANALOG_ADC_INSTANCE, BSP_ANALOG_IDX_CURRENT, BSP_ANALOG_CURRENT_CH, ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(BSP_ANALOG_ADC_INSTANCE, BSP_ANALOG_IDX_VBUS,    BSP_ANALOG_VBUS_CH,    ADC_SAMPLETIME_239POINT5);
    adc_regular_channel_config(BSP_ANALOG_ADC_INSTANCE, BSP_ANALOG_IDX_TEMP,    BSP_ANALOG_TEMP_CH,    ADC_SAMPLETIME_239POINT5);

    /* 软件触发 (连续模式下触发一次即自动持续转换) */
    adc_external_trigger_source_config(BSP_ANALOG_ADC_INSTANCE, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
    adc_external_trigger_config(BSP_ANALOG_ADC_INSTANCE, ADC_REGULAR_CHANNEL, ENABLE);

    adc_enable(BSP_ANALOG_ADC_INSTANCE);
    adc_calibration_enable(BSP_ANALOG_ADC_INSTANCE);
    adc_dma_mode_enable(BSP_ANALOG_ADC_INSTANCE);
    adc_software_trigger_enable(BSP_ANALOG_ADC_INSTANCE, ADC_REGULAR_CHANNEL);
}


/* ================================================================
 * NTC 温度查表 (B57861S0103F040, 10K@25℃, B25/85≈3988K)
 * ----------------------------------------------------------------
 * 采样链路: NTC 与上拉电阻分压 → 运放调理(BRAKE_TEMP=0.62×VTemp_IN) → ADC。
 * 直接以「温度 → 期望 ADC 原始码」建表, 运行时反查 + 线性插值得到温度。
 * 表中 ADC 码随温度升高而下降 (NTC 负温度系数, 阻值降低→分压点变化)。
 *
 * ★实测标定说明 (CAL-12): 下表为基于器件标称 B 值 + 标称分压网络的设计计算值,
 *   样机阶段须以恒温源实测若干温度点校正本表 (尤其 0.62 调理系数与上拉阻值)。
 *   表项: { 温度(℃), 对应 ADC 原始码(12bit) }, 温度从低到高, ADC 码单调递减。
 * ================================================================ */
typedef struct {
    int16_t  temp_c;        /* 温度 (℃) */
    uint16_t adc_code;      /* 该温度对应的 ADC 原始码 (12bit, 0~4095) */
} Bsp_NtcPoint_T;

/* ★基于电路实测: Rpu=10kΩ, ÷0.62还原VTemp_IN, 官方R/T表(B57861S/8016)重算 (CAL-12-V3) */
static const Bsp_NtcPoint_T s_ntc_table[] = {
    { -20, 3712 }, { -10, 3468 }, {   0, 3134 }, {  10, 2726 },
    {  20, 2274 }, {  25, 2048 }, {  30, 1827 }, {  40, 1423 },
    {  50, 1085 }, {  60,  816 }, {  70,  611 }, {  80,  458 },
    {  90,  344 }, { 100,  261 }, { 110,  199 }, { 120,  153 },
};
#define BSP_NTC_TABLE_LEN   (sizeof(s_ntc_table) / sizeof(s_ntc_table[0]))

/**
 * 功能：由 NTC ADC 原始码反查温度（表外钳位、表内线性插值）。
 * 输入参数：code —— ADC 原始码（12bit）。
 * 输出参数：无
 * 返回值：int32_t，温度（m℃，即 0.001℃）。
 */
static int32_t Bsp_Analog_NtcCodeToTemp_mDegC(uint16_t code)
{
    uint32_t i;

    /* 表按温度升序、ADC 码降序排列 */
    if (code >= s_ntc_table[0].adc_code) {
        return (int32_t)s_ntc_table[0].temp_c * 1000;          /* 低于最低温, 钳到表头 */
    }
    if (code <= s_ntc_table[BSP_NTC_TABLE_LEN - 1U].adc_code) {
        return (int32_t)s_ntc_table[BSP_NTC_TABLE_LEN - 1U].temp_c * 1000; /* 高于最高温, 钳到表尾 */
    }
    for (i = 0U; i < BSP_NTC_TABLE_LEN - 1U; i++) {
        uint16_t hi_code = s_ntc_table[i].adc_code;            /* 较低温对应较高码 */
        uint16_t lo_code = s_ntc_table[i + 1U].adc_code;       /* 较高温对应较低码 */
        if (code <= hi_code && code > lo_code) {
            int32_t t0 = (int32_t)s_ntc_table[i].temp_c * 1000;
            int32_t t1 = (int32_t)s_ntc_table[i + 1U].temp_c * 1000;
            /* 线性插值: code 在 [lo_code, hi_code], 温度在 [t0, t1] */
            int32_t span = (int32_t)hi_code - (int32_t)lo_code;
            int32_t pos  = (int32_t)hi_code - (int32_t)code;
            return t0 + (t1 - t0) * pos / span;
        }
    }
    return 0;   /* 理论不可达 */
}


/* ---------------------------------------------------------------- */
/**
 * 功能：初始化模拟采样（GPIO + DMA + ADC 连续转换，复位功率统计与滤波器）。幂等。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Analog_Status_T，BSP_ANALOG_OK 表示成功。
 */
Bsp_Analog_Status_T Bsp_Analog_Init(void)
{
    if (s_analog_initialized) {
        return BSP_ANALOG_OK;
    }
    Bsp_Analog_GpioConfig();
    Bsp_Analog_DmaConfig();     /* DMA 须在 ADC 使能前配好 */
    Bsp_Analog_AdcConfig();
    Bsp_Analog_PowerStatReset();
    Bsp_Analog_FilterReset();   /* 用首个采样初始化滤波器 */
    s_analog_initialized = 1U;
    return BSP_ANALOG_OK;
}

/**
 * 功能：查询模拟采样模块是否已初始化。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，非 0=已初始化，0=未初始化。
 */
int Bsp_Analog_IsInitialized(void)
{
    return (int)s_analog_initialized;
}

/**
 * 功能：读取电流通道 ADC 原始码（DMA 已自动搬运）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，电流通道原始码（12bit）。
 */
uint16_t Bsp_Analog_ReadRawCurrent(void)
{
    return s_adc_raw[BSP_ANALOG_IDX_CURRENT];   /* DMA 已自动搬运, 直接读 */
}

/**
 * 功能：读取电压通道 ADC 原始码。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，电压通道原始码（12bit）。
 */
uint16_t Bsp_Analog_ReadRawVBus(void)
{
    return s_adc_raw[BSP_ANALOG_IDX_VBUS];
}

/**
 * 功能：读取温度通道 ADC 原始码。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，温度通道原始码（12bit）。
 */
uint16_t Bsp_Analog_ReadRawTemp(void)
{
    return s_adc_raw[BSP_ANALOG_IDX_TEMP];
}

/**
 * 功能：换算母线电压（原始码 → 分压还原工程量）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，母线电压（mV，★线性系数待 CAL-10 标定）。
 */
int32_t Bsp_Analog_GetVBus_mV(void)
{
    /* raw → ADC输入电压(mV) → ×分压比 → 母线 mV
     * V_bus = raw/4096 × VREF × 49  (母线经 49:1 分压，ADC 读小电压，还原乘 49)
     * 注：曾按硬件同事要求试 ÷49，实测 48V 读成 ~19mV 明显错，已还原为 ×49。★线性系数 CAL-10 精标。 */
    int32_t raw = (int32_t)s_adc_raw[BSP_ANALOG_IDX_VBUS];
    return raw * BSP_ANALOG_VREF_MV / (int32_t)BSP_ANALOG_ADC_FULL * (int32_t)BSP_ANALOG_VBUS_DIV_RATIO;
}

/**
 * 功能：换算母线电流（霍尔传感器，零点偏移 + 灵敏度反算，带符号表示方向）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，母线电流（mA，带符号；★零点/灵敏度待 CAL-11 标定）。
 */
int32_t Bsp_Analog_GetCurrent_mA(void)
{
    /* 双向电流 (CC6920BSO-40A: 灵敏度 33mV/A, 零点=0.5×VCC)。
     * 零点码以上=正向、以下=负向 (正负对应的物理方向取决于母线穿过 IP+/IP- 的朝向, 符号约定见 CAL-11)。
     *   ΔV(µV) = (raw - zero) × VREF(mV) × 1000 / 4096
     *   I(mA)  = ΔV(µV) / 灵敏度(mV/A)           [∵ µV/(mV/A)=mA]
     * (用 int64 防溢出; ★零点与灵敏度需 CAL-11 实测微调) */
    int32_t raw  = (int32_t)s_adc_raw[BSP_ANALOG_IDX_CURRENT];
    int32_t zero = (int32_t)BSP_ANALOG_CURRENT_ZERO_CODE;
    int64_t dv_uV = (int64_t)(raw - zero) * BSP_ANALOG_VREF_MV * 1000 / (int64_t)BSP_ANALOG_ADC_FULL;
    return (int32_t)(dv_uV / (int64_t)BSP_ANALOG_CURRENT_SENS_MV_PER_A);
}

/**
 * 功能：换算温度（NTC 查表 + 线性插值）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，温度（m℃；★表数据待 CAL-12 标定）。
 */
int32_t Bsp_Analog_GetTemp_mDegC(void)
{
    /* 直接查 NTC 表得温度。★表数据待 CAL-12 实测标定校正。 */
    return Bsp_Analog_NtcCodeToTemp_mDegC(s_adc_raw[BSP_ANALOG_IDX_TEMP]);
}

/* ================================================================
 * 功率统计 (REQ-FUNC-004)
 * ================================================================ */
/**
 * 功能：计算瞬时功率（母线电压 × 电流，带符号表示回馈/泄放方向）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，瞬时功率（mW，带符号）。
 */
int32_t Bsp_Analog_GetPower_mW(void)
{
    /* 瞬时功率 P(mW) = U(mV) × I(mA) / 1000; 带符号 (回馈/泄放方向) */
    int64_t u = (int64_t)Bsp_Analog_GetVBus_mV();
    int64_t i = (int64_t)Bsp_Analog_GetCurrent_mA();
    return (int32_t)(u * i / 1000);
}

/**
 * 功能：复位功率统计窗口（峰值/累加器/计数清零）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Analog_PowerStatReset(void)
{
    s_pwr_peak_mW = 0;
    s_pwr_acc_mW  = 0;
    s_pwr_cnt     = 0U;
}

/**
 * 功能：采一次瞬时功率并更新平均累加器与峰值（按绝对值取峰）。须周期调用。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Analog_PowerUpdate(void)
{
    /* 采一次瞬时功率, 更新平均累加器与峰值(按绝对值取峰) */
    int32_t p = Bsp_Analog_GetPower_mW();
    int32_t pa = (p < 0) ? -p : p;

    s_pwr_acc_mW += (int64_t)p;
    s_pwr_cnt++;
    if (pa > s_pwr_peak_mW) {
        s_pwr_peak_mW = pa;
    }
}

/**
 * 功能：获取统计窗口内平均功率。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，平均功率（mW，带符号；无采样时返回 0）。
 */
int32_t Bsp_Analog_GetAvgPower_mW(void)
{
    if (s_pwr_cnt == 0U) {
        return 0;
    }
    return (int32_t)(s_pwr_acc_mW / (int64_t)s_pwr_cnt);
}

/**
 * 功能：获取统计窗口内峰值功率（绝对值）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，峰值功率（mW）。
 */
int32_t Bsp_Analog_GetPeakPower_mW(void)
{
    return s_pwr_peak_mW;
}

/* ================================================================
 * ADC 滑动滤波 (过压判定抗误触发, 依据《过压保护设计思路梳理》)
 *   filtered = (filtered * (2^n - 1) + new) / 2^n
 *   当前仅对电压通道滤波 (过压判定用); 须周期调用 FilterUpdate 推进。
 * ================================================================ */
/**
 * 功能：复位电压通道滑动滤波器（用当前原始码装载初值，避免上电缓爬）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Analog_FilterReset(void)
{
    /* 用当前原始码装载滤波初值, 避免上电瞬间从 0 缓慢爬升导致判定滞后 */
    s_filt_vbus   = (uint32_t)s_adc_raw[BSP_ANALOG_IDX_VBUS];
    s_filt_inited = 1U;
}

/**
 * 功能：推进电压通道一阶滑动滤波（filtered = (filtered*(2^n-1)+new)>>n）。须周期调用。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Analog_FilterUpdate(void)
{
    uint32_t newv = (uint32_t)s_adc_raw[BSP_ANALOG_IDX_VBUS];
    if (!s_filt_inited) {
        s_filt_vbus = newv;
        s_filt_inited = 1U;
        return;
    }
    /* filtered = (filtered*(2^n-1) + new) >> n  等价 (filtered*7+new)/8 (n=3) */
    s_filt_vbus = ((s_filt_vbus * (((uint32_t)1U << BSP_ANALOG_FILTER_SHIFT) - 1U)) + newv)
                  >> BSP_ANALOG_FILTER_SHIFT;
}

/**
 * 功能：获取电压通道滤波后的 ADC 码值。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，滤波后电压原始码。
 */
uint16_t Bsp_Analog_GetFilteredVBus(void)
{
    return (uint16_t)s_filt_vbus;
}

/**
 * 功能：换算滤波后的母线电压（用滤波码值，供过压判定抗误触发）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，滤波母线电压（mV）。
 */
int32_t Bsp_Analog_GetVBusFiltered_mV(void)
{
    /* 与 GetVBus_mV 同换算, 输入用滤波后码值 */
    int32_t raw = (int32_t)s_filt_vbus;
    return raw * BSP_ANALOG_VREF_MV / (int32_t)BSP_ANALOG_ADC_FULL * (int32_t)BSP_ANALOG_VBUS_DIV_RATIO;
}

/* ================================================================
 * 母线电压 ADC 模拟看门狗 (过压/欠压中断快速响应)
 *   越 [low, high] 窗口 → 硬件置 WDE → ADC0_1_IRQHandler → 本模块回调 app 层。
 *   见《过压欠压ADC看门狗中断快速响应设计》。
 * ================================================================ */
static Bsp_Analog_WdgCb_T s_wdg_cb = NULL;   /* 越限回调 (ISR 上下文调用) */

/**
 * 功能：母线电压(mV) → ADC 原始码 (GetVBus_mV 的逆运算，供看门狗阈值配置)。
 * 输入参数：mv —— 母线电压 (mV)。
 * 输出参数：无
 * 返回值：uint16_t，对应 ADC 12bit 原始码 (0~4095，越界钳位)。
 */
uint16_t Bsp_Analog_VBusMvToCode(int32_t mv)
{
    /* code = mv / 分压比(49) × ADC_FULL / VREF  (GetVBus_mV: mv = code×VREF/FULL×49 的逆) */
    if (mv < 0) { mv = 0; }
    int32_t code = mv * (int32_t)BSP_ANALOG_ADC_FULL
                 / (int32_t)BSP_ANALOG_VBUS_DIV_RATIO / (int32_t)BSP_ANALOG_VREF_MV;
    if (code < 0) { code = 0; }
    if (code > (int32_t)(BSP_ANALOG_ADC_FULL - 1U)) { code = (int32_t)(BSP_ANALOG_ADC_FULL - 1U); }
    return (uint16_t)code;
}

/**
 * 功能：设置看门狗低/高阈值码 (阈值变更时调，如 CAN 0x14 改过压阈值后同步)。
 * 输入参数：code_lo —— 低阈值 (欠压/过泄压)；code_hi —— 高阈值 (过压)。
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Analog_SetVBusWindow(uint16_t code_lo, uint16_t code_hi)
{
    adc_watchdog_threshold_config(BSP_ANALOG_ADC_INSTANCE, code_lo, code_hi);
}

/**
 * 功能：挂电压通道单通道模拟看门狗 + 使能 WDE 中断 + 注册越限回调。
 * 输入参数：cb —— 越限回调 (ISR 上下文，须极短不阻塞；可空)。
 * 输出参数：无
 * 返回值：无
 * 说明：单通道看门狗只对电压通道 (ADC_CHANNEL_2) 生效，不误触发电流/温度通道。
 *       阈值须先经 Bsp_Analog_SetVBusWindow 设定 (或本函数后调用)。
 */
void Bsp_Analog_EnableVBusWatchdog(Bsp_Analog_WdgCb_T cb)
{
    s_wdg_cb = cb;
    /* 单通道看门狗: 扫描模式下仅对指定通道 (电压) 生效 (库内置 WDSC+WDCHSEL+RWDEN) */
    adc_watchdog_single_channel_enable(BSP_ANALOG_ADC_INSTANCE, BSP_ANALOG_VBUS_CH);
    adc_interrupt_flag_clear(BSP_ANALOG_ADC_INSTANCE, ADC_INT_FLAG_WDE);
    adc_interrupt_enable(BSP_ANALOG_ADC_INSTANCE, ADC_INT_WDE);
    /* 过压响应优先级高于一般外设 (低于 SysTick)；抢占组按工程惯例，此处给较高抢占 */
    nvic_irq_enable(ADC0_1_IRQn, 1U, 0U);
}

/**
 * 功能：关闭看门狗中断 (安全态/测试用；防安全态下被 ISR 重新起泄放)。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Analog_DisableVBusWatchdog(void)
{
    adc_interrupt_disable(BSP_ANALOG_ADC_INSTANCE, ADC_INT_WDE);
    adc_watchdog_disable(BSP_ANALOG_ADC_INSTANCE);
}

/**
 * 功能：看门狗中断服务 (由 it.c 的 ADC0_1_IRQHandler 调用)：清标志 + 转调回调。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Analog_WatchdogIrq(void)
{
    if (adc_interrupt_flag_get(BSP_ANALOG_ADC_INSTANCE, ADC_INT_FLAG_WDE) == SET) {
        adc_interrupt_flag_clear(BSP_ANALOG_ADC_INSTANCE, ADC_INT_FLAG_WDE);
        if (s_wdg_cb != NULL) { s_wdg_cb(); }
    }
}
