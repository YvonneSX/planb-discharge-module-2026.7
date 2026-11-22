/**
 * @file    app_monitor.c
 * @brief   HLD-MOD-MON 采样监测模块实现
 */

#include "app_monitor.h"
#include "uart_log.h"

#define MON_TAG "MON"

/* 模块私有量测快照：仅 MON 写入量测字段，外部经 GetSnapshot 只读拷贝。 */
static App_RtData_T s_snapshot;
static bool         s_inited = false;

/**
 * 功能：将带符号工程量钳制到 uint16 范围（负值取 0，超 65535 取上限）。
 * 输入参数：v —— 待钳制的有符号整数值。
 * 输出参数：无
 * 返回值：uint16_t，钳制后的无符号结果。
 */
static uint16_t mon_clamp_u16(int32_t v)
{
    if (v < 0)       { return 0U; }
    if (v > 65535)   { return 65535U; }
    return (uint16_t)v;
}

/**
 * 功能：将带符号工程量钳制到 int16 范围（±32767/−32768 饱和）。
 * 输入参数：v —— 待钳制的有符号整数值。
 * 输出参数：无
 * 返回值：int16_t，钳制后的有符号结果。
 */
static int16_t mon_clamp_i16(int32_t v)
{
    if (v >  32767)  { return  32767; }
    if (v < -32768)  { return -32768; }
    return (int16_t)v;
}

/**
 * 功能：初始化采样监测模块，复位功率统计与滤波器并清零量测快照。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppMon_Init(void)
{
    Bsp_Analog_PowerStatReset();
    Bsp_Analog_FilterReset();   /* 用当前原始值定初值，避免滤波从 0 缓爬 */

    for (uint32_t i = 0U; i < sizeof(s_snapshot); i++) {
        ((uint8_t *)&s_snapshot)[i] = 0U;
    }
    s_inited = true;
    UART_LOGI(MON_TAG, "init done");
}

/**
 * 功能：推进滤波/功率统计并刷新量测快照（电压/电流/温度/功率，按 CAN 标度换算）。须每控制圈调用。
 * 输入参数：无
 * 输出参数：无（结果写入模块私有快照）
 * 返回值：无
 */
void AppMon_Update(void)
{
    if (!s_inited) { return; }

    /* 功率统计推进（须周期调用）。
     * ★去滤波：判据链路(DCH 起停/升退、PROT 告警)已全部改用瞬时值 AppMon_GetVBus_mV()，
     *   不再在运行态推进 EMA 滤波（消除软件滞后，见《泄放中断即时响应优化设计说明》）。
     *   Bsp_Analog_Filter* 仍保留供 BSP 单元测试与可选诊断，但不在本主循环推进。 */
    Bsp_Analog_PowerUpdate();

    /* mV → 0.1V/LSB；mA → 0.01A/LSB；m℃ → 0.1℃/LSB；mW → 1W/LSB */
    s_snapshot.vbus_0v1 = mon_clamp_u16(Bsp_Analog_GetVBus_mV() / 100);
    s_snapshot.ibus_0a01 = mon_clamp_i16(Bsp_Analog_GetCurrent_mA() / 10);
    s_snapshot.temp_0c1  = mon_clamp_i16(Bsp_Analog_GetTemp_mDegC() / 100);

    int32_t p_inst = Bsp_Analog_GetPower_mW();
    int32_t p_avg  = Bsp_Analog_GetAvgPower_mW();
    int32_t p_peak = Bsp_Analog_GetPeakPower_mW();
    if (p_inst < 0) { p_inst = -p_inst; }
    if (p_avg  < 0) { p_avg  = -p_avg;  }
    if (p_peak < 0) { p_peak = -p_peak; }
    s_snapshot.p_inst_w = mon_clamp_u16(p_inst / 1000);
    s_snapshot.p_avg_w  = mon_clamp_u16(p_avg  / 1000);
    s_snapshot.p_peak_w = mon_clamp_u16(p_peak / 1000);
}

/**
 * 功能：拷贝当前量测快照给调用方（只读获取，空指针则忽略）。
 * 输入参数：out —— 接收快照的缓冲区指针。
 * 输出参数：*out —— 写入最新量测快照。
 * 返回值：无
 */
void AppMon_GetSnapshot(App_RtData_T *out)
{
    if (out == NULL) { return; }
    *out = s_snapshot;
}

/**
 * 功能：获取滤波后的母线电压（供保护判定使用）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，滤波母线电压（mV）。
 */
int32_t AppMon_GetVBusFiltered_mV(void)
{
    return Bsp_Analog_GetVBusFiltered_mV();
}

/**
 * 功能：获取当前母线电压原始量测值。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，母线电压（mV）。
 */
int32_t AppMon_GetVBus_mV(void)      { return Bsp_Analog_GetVBus_mV(); }

/**
 * 功能：获取当前母线电流量测值（带符号，正/负表示方向）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，母线电流（mA，带符号）。
 */
int32_t AppMon_GetCurrent_mA(void)   { return Bsp_Analog_GetCurrent_mA(); }

/**
 * 功能：获取当前温度量测值。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int32_t，温度（m℃，即 0.001℃）。
 */
int32_t AppMon_GetTemp_mDegC(void)   { return Bsp_Analog_GetTemp_mDegC(); }
