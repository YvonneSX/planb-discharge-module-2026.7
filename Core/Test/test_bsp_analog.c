/**
 * @file    test_bsp_analog.c
 * @brief   模拟采样单元测试 实现
 *
 * 覆盖接口: Init / IsInitialized / ReadRawCurrent / ReadRawVBus / ReadRawTemp /
 *           GetVBus_mV / GetCurrent_mA / GetTemp_mDegC /
 *           GetPower_mW / PowerUpdate / PowerStatReset / GetAvgPower_mW / GetPeakPower_mW /
 *           FilterUpdate / FilterReset / GetFilteredVBus / GetVBusFiltered_mV
 *
 * 真机说明: ADC+DMA 后台连续刷新, 读到的是当前母线实际值。
 *   RunAll 为一次性"自检": 仅用范围断言验证读数落在物理合理区间(防硬件/通道接错),
 *   单次快照值不作标定依据。实时数据获取的交叉核对与标定(CAL-10/11/12)
 *   请用"实时监视模式" Test_BspAnalog_Monitor (make analog-live):
 *   循环刷新打印, 改变母线电压/加热 NTC 时观察读数随动, 与仪表逐点核对。
 *
 * @version 1.3.0
 * @date    2026-06-13
 */

#include "test_bsp_analog.h"

#define BSP_USE_ANALOG
#define BSP_USE_UART_LOG        /* 监视模式用 ReadByte 检测退出键 */
#include "bsp.h"
#include "uart_log.h"
#include "systick.h"

void Test_BspAnalog_RunAll(Test_Stats_T *_stats)
{
    uint16_t raw_i, raw_v, raw_t;
    int32_t  vbus, curr, temp, p_inst, p_avg, p_peak, vbus_f;
    int k;

    /* ---- 初始化 ---- */
    TEST_CASE("Analog: 初始化");
    TEST_ASSERT_EQ(Bsp_Analog_Init(), BSP_ANALOG_OK);
    TEST_ASSERT(Bsp_Analog_IsInitialized() != 0);

    TEST_CASE("Analog: 重复初始化幂等");
    TEST_ASSERT_EQ(Bsp_Analog_Init(), BSP_ANALOG_OK);

    /* ---- 原始 ADC 读取: 12bit, 必在 0~4095 ---- */
    TEST_CASE("Analog: 原始 ADC 值范围 (0~4095)");
    raw_i = Bsp_Analog_ReadRawCurrent();
    raw_v = Bsp_Analog_ReadRawVBus();
    raw_t = Bsp_Analog_ReadRawTemp();
    TEST_LOG_VAL("raw: I=%u V=%u T=%u", raw_i, raw_v, raw_t);
    TEST_ASSERT_IN_RANGE(raw_i, 0, 4095);
    TEST_ASSERT_IN_RANGE(raw_v, 0, 4095);
    TEST_ASSERT_IN_RANGE(raw_t, 0, 4095);

    /* ---- 电压换算: 0~98V 量程 (仅范围自检, 标定见 analog-live 监视模式) ---- */
    TEST_CASE("Analog: 母线电压换算 (0~98000 mV)");
    vbus = Bsp_Analog_GetVBus_mV();
    TEST_LOG_VAL("VBus = %ld mV  (单次快照, 标定用 make analog-live)", (long)vbus);
    TEST_ASSERT_IN_RANGE(vbus, 0, 98000);

    /* ---- 电流换算: ±40A 双向 ---- */
    TEST_CASE("Analog: 母线电流换算 (+-40000 mA)");
    curr = Bsp_Analog_GetCurrent_mA();
    TEST_LOG_VAL("Current = %ld mA  (单次快照, 标定用 make analog-live)", (long)curr);
    TEST_ASSERT_IN_RANGE(curr, -40000, 40000);

    /* ---- 温度查表换算: -20~+120℃ ---- */
    TEST_CASE("Analog: 温度查表换算 (-20~120 du)");
    temp = Bsp_Analog_GetTemp_mDegC();
    TEST_LOG_VAL("Temp = %ld mC (= %ld.%03ld C, 标定用 make analog-live)",
                 (long)temp, (long)(temp/1000),
                 (long)((temp%1000<0)?-(temp%1000):(temp%1000)));
    TEST_ASSERT_IN_RANGE(temp, -20000, 120000);

    /* ---- 瞬时功率 ---- */
    TEST_CASE("Analog: 瞬时功率换算");
    p_inst = Bsp_Analog_GetPower_mW();
    TEST_LOG_VAL("Power = %ld mW", (long)p_inst);
    TEST_ASSERT_IN_RANGE(p_inst, -3920000, 3920000);

    /* ---- 功率统计 ---- */
    TEST_CASE("Analog: 功率统计 复位->更新->读取");
    Bsp_Analog_PowerStatReset();
    TEST_ASSERT_EQ_INT(Bsp_Analog_GetAvgPower_mW(), 0);
    TEST_ASSERT_EQ_INT(Bsp_Analog_GetPeakPower_mW(), 0);
    for (k = 0; k < 10; k++) { Bsp_Analog_PowerUpdate(); }
    p_avg  = Bsp_Analog_GetAvgPower_mW();
    p_peak = Bsp_Analog_GetPeakPower_mW();
    TEST_LOG_VAL("Pavg = %ld mW, Ppeak = %ld mW (10 次采样)", (long)p_avg, (long)p_peak);
    TEST_ASSERT_IN_RANGE(p_avg, -3920000, 3920000);
    TEST_ASSERT(p_peak >= 0);
    TEST_ASSERT(p_peak >= (p_avg < 0 ? -p_avg : p_avg));

    /* ---- ADC 滤波 ---- */
    TEST_CASE("Analog: ADC 滤波收敛");
    Bsp_Analog_FilterReset();
    for (k = 0; k < 20; k++) { Bsp_Analog_FilterUpdate(); }
    vbus_f = Bsp_Analog_GetVBusFiltered_mV();
    TEST_LOG_VAL("VBus_filtered = %ld mV (滤波后, 过压判定用)", (long)vbus_f);
    TEST_ASSERT_IN_RANGE(vbus_f, 0, 98000);

    TEST_CASE("Analog: 再次复位功率统计清零");
    Bsp_Analog_PowerStatReset();
    TEST_ASSERT_EQ_INT(Bsp_Analog_GetPeakPower_mW(), 0);
}

/* ================================================================
 * 模拟量实时监视模式 (make analog-live)
 * ----------------------------------------------------------------
 * 一次性 RunAll 的快照对实时采集无参考价值; 标定/核对需在改变物理
 * 输入(可调电源调压、恒温源/热风枪给 NTC 升温、电子负载拉流)的同时
 * 连续观测读数随动。本模式按 TEST_MONITOR_PERIOD_MS 周期循环打印:
 *   raw(I/V/T) 原始码 + VBus(mV) + 电流(mA) + 温度(℃) + 功率(mW) + 滤波后电压
 * 每周期推进滤波/功率统计, 使滤波值/瞬时功率实时有效。
 * 退出: 串口发送 'q' 或 ESC(0x1B), 或直接复位。
 * ================================================================ */
void Test_BspAnalog_Monitor(void)
{
    uint8_t  key;
    uint32_t n = 0U;
    uint32_t last_tick_ms, last_print_ms;

    (void)Bsp_Analog_Init();
    Bsp_Analog_FilterReset();
    Bsp_Analog_PowerStatReset();
    /* 注: 不做阻塞等待。即使首次 FilterReset 装载到 0, 在第一帧打印(300ms)前
     * 已按 ~1kHz 推进约 300 次, 滤波后电压必已收敛, 故第一帧即有效。 */

    UartLog_Println("\r\n=== Analog 实时监视模式 (刷新 %ums) ===",
                    (unsigned int)TEST_MONITOR_PERIOD_MS);
    UartLog_Println("调电源/加热NTC/拉流时观察读数随动, 用万用表交叉核对标定 CAL-10/11/12");
    UartLog_Println("退出: 发送 q 或 ESC, 或复位");

    last_tick_ms  = systick_get_ms();
    last_print_ms = last_tick_ms;

    /* 非阻塞实时循环: 滤波/功率按真实时间以 ~1kHz 节奏推进(不阻塞、不拖慢显示),
     * 主显示字段取瞬时实时值, 退出键每轮即时响应。滤波只作用于电压(过压判定用),
     * 电流不额外滤波以保实时。 */
    for (;;) {
        uint32_t now = systick_get_ms();

        /* ① ~1kHz 推进电压 EMA 滤波与功率统计(贴近真机控制环节奏) */
        if ((uint32_t)(now - last_tick_ms) >= 1U) {
            last_tick_ms = now;
            Bsp_Analog_FilterUpdate();
            Bsp_Analog_PowerUpdate();
        }

        /* ② 周期打印一帧实时快照 */
        if ((uint32_t)(now - last_print_ms) >= TEST_MONITOR_PERIOD_MS) {
            uint16_t ri, rv, rt;
            int32_t  vbus, curr, temp, pavg, vf, tdeg, tfrac;
            const char *tflag;

            last_print_ms = now;

            ri = Bsp_Analog_ReadRawCurrent();
            rv = Bsp_Analog_ReadRawVBus();
            rt = Bsp_Analog_ReadRawTemp();
            vbus = Bsp_Analog_GetVBus_mV();          /* 瞬时实时值 */
            curr = Bsp_Analog_GetCurrent_mA();        /* 瞬时实时值(不滤波) */
            temp = Bsp_Analog_GetTemp_mDegC();        /* 瞬时实时值 */
            pavg = Bsp_Analog_GetAvgPower_mW();        /* 窗口平均功率, 比瞬时稳定 */
            vf   = Bsp_Analog_GetVBusFiltered_mV();    /* 滤波后电压(过压判定用) */
            tdeg  = temp / 1000;
            tfrac = temp % 1000;
            if (tfrac < 0) { tfrac = -tfrac; }

            /* 温度越界提示: 查表在 [-20,120]℃ 外会钳位, 钳位值不是真实温度
             * (常见原因: NTC 未接/开路→码偏高; 短路→码偏低; 或表未标定 CAL-12)。 */
            if (temp <= -20000) {
                tflag = " <CLAMP! raw偏高,疑NTC开路/未接或未标定>";
            } else if (temp >= 120000) {
                tflag = " <CLAMP! raw偏低,疑NTC短路或未标定>";
            } else {
                tflag = "";
            }

            UartLog_Println("[%lu] raw I/V/T=%u/%u/%u | VBus=%ldmV I=%ldmA T=%ld.%03ldC%s Pavg=%ldmW Vf=%ldmV",
                            (unsigned long)(n++), ri, rv, rt,
                            (long)vbus, (long)curr, (long)tdeg, (long)tfrac, tflag,
                            (long)pavg, (long)vf);

            Bsp_Analog_PowerStatReset();   /* 下一窗口重新统计平均功率 */
        }

        /* ③ 退出键: 每轮都查, 实时响应 */
        if (Bsp_UartLog_ReadByte(&key) == 1 && (key == 'q' || key == 0x1BU)) {
            break;
        }
    }
    UartLog_Println("=== Analog 监视结束 ===\r\n");
}
