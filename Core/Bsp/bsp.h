/**
 * @file    bsp.h
 * @brief   BSP 板级支持包统一头文件（GD32F303CCT6 泄放模块）
 *
 * 硬件引脚映射：
 *   PB6  — 调试串口 TX（USART0，Remap）
 *   PB7  — 调试串口 RX（USART0，Remap）
 *   PA8  — 泄放 PWM（TIMER0 CH0）
 *   PA0  — 电流 ADC
 *   PA2  — 电压 ADC
 *   PA4  — 温度 ADC（NTC）
 *   PA9  — Brake_FB 泄放回路反馈
 *   PA15 — 外部看门狗 WDI
 *   PA11 — CAN0 RX
 *   PA12 — CAN0 TX
 *   PB10~PB15 — SPI1（FRAM MB85RS64）
 *
 * 使用方式：在 #include "bsp.h" 之前定义所需模块宏，例如：
 *   #define BSP_USE_UART_LOG
 *   #include "bsp.h"
 *
 * @version 1.0.0
 * @date    2026-06-11
 */

#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "gd32f30x.h"

/* ====================================================================
 * 模块：BSP 系统初始化
 * ==================================================================== */

/** @brief BSP 系统级初始化总入口。依次完成：时钟确认 → Systick → 调试串口 → 各外设。 */
void     Bsp_Init(void);
/** @brief 获取当前系统主频（Hz）。 */
uint32_t Bsp_Clock_GetSysclkFreq(void);

/* ====================================================================
 * 模块：调试串口 UART LOG（BSP_USE_UART_LOG）
 * ==================================================================== */
#if defined(BSP_USE_UART_LOG)

/* 硬件配置（可通过 -D 覆盖） */
#ifndef BSP_UART_LOG_INSTANCE
#define BSP_UART_LOG_INSTANCE       USART0
#endif
#ifndef BSP_UART_LOG_RCU_UART
#define BSP_UART_LOG_RCU_UART       RCU_USART0
#endif
#ifndef BSP_UART_LOG_BAUD
#define BSP_UART_LOG_BAUD           115200U
#endif
/* GD32F303CCT6：USART0 Remap → PB6(TX) / PB7(RX) */
#ifndef BSP_UART_LOG_TX_PORT
#define BSP_UART_LOG_TX_PORT        GPIOB
#define BSP_UART_LOG_TX_PIN         GPIO_PIN_6
#define BSP_UART_LOG_RCU_TX         RCU_GPIOB
#endif
#ifndef BSP_UART_LOG_RX_PORT
#define BSP_UART_LOG_RX_PORT        GPIOB
#define BSP_UART_LOG_RX_PIN         GPIO_PIN_7
#define BSP_UART_LOG_RCU_RX         RCU_GPIOB
#endif
/* 轮询模式 TX 超时（循环次数） */
#ifndef BSP_UART_LOG_TX_TIMEOUT
#define BSP_UART_LOG_TX_TIMEOUT     200000U
#endif

typedef enum {
    BSP_UART_LOG_OK          =  0,
    BSP_UART_LOG_ERR_PARAM   = -1,
    BSP_UART_LOG_ERR_INIT    = -2,
    BSP_UART_LOG_ERR_TIMEOUT = -3,
} Bsp_UartLog_Status_T;

Bsp_UartLog_Status_T Bsp_UartLog_Init(void);
void Bsp_UartLog_Deinit(void);
int  Bsp_UartLog_IsInitialized(void);
int  Bsp_UartLog_Write(const void *buf, uint32_t len);
int  Bsp_UartLog_Putc(uint8_t ch);
int  Bsp_UartLog_ReadByte(uint8_t *out);

#endif /* BSP_USE_UART_LOG */

/* ====================================================================
 * 模块：模拟采样 ANALOG（BSP_USE_ANALOG）
 *   PA0 电流（CC6920 ±40A）/ PA2 电压（AMC1311 0~98V，49:1）/ PA4 温度（NTC）
 * ==================================================================== */
#if defined(BSP_USE_ANALOG)

#ifndef BSP_ANALOG_ADC_INSTANCE
#define BSP_ANALOG_ADC_INSTANCE     ADC0
#endif
/* 电流采样：PA0 = ADC 通道 0 */
#ifndef BSP_ANALOG_CURRENT_PORT
#define BSP_ANALOG_CURRENT_PORT     GPIOA
#define BSP_ANALOG_CURRENT_PIN      GPIO_PIN_0
#define BSP_ANALOG_CURRENT_CH       ADC_CHANNEL_0
#endif
/* 电压采样：PA2 = ADC 通道 2 */
#ifndef BSP_ANALOG_VBUS_PORT
#define BSP_ANALOG_VBUS_PORT        GPIOA
#define BSP_ANALOG_VBUS_PIN         GPIO_PIN_2
#define BSP_ANALOG_VBUS_CH          ADC_CHANNEL_2
#endif
/* 温度采样：PA4 = ADC 通道 4 */
#ifndef BSP_ANALOG_TEMP_PORT
#define BSP_ANALOG_TEMP_PORT        GPIOA
#define BSP_ANALOG_TEMP_PIN         GPIO_PIN_4
#define BSP_ANALOG_TEMP_CH          ADC_CHANNEL_4
#endif
/* 标定常量（★实测标定，来自硬件规格说明书） */
#ifndef BSP_ANALOG_VBUS_DIV_RATIO
#define BSP_ANALOG_VBUS_DIV_RATIO   49U     /* 母线分压比 49:1 */
#endif
#ifndef BSP_ANALOG_CURRENT_RANGE_A
#define BSP_ANALOG_CURRENT_RANGE_A  40      /* ±40A 量程（仅作上下限钳位参考，非换算系数） */
#endif
/* CC6920BSO-40A 霍尔传感器实参（数据手册）：灵敏度 33mV/A，零电流输出=0.5×VCC。
 * 传感器 VCC 与 MCU VDDA 同为 3.3V（比例式），故零点落在 ADC 满量程一半。
 * 换算按实际灵敏度，不再用「±量程=±半满量程」的近似。★零点/灵敏度仍需 CAL-11 实测微调。 */
#ifndef BSP_ANALOG_CURRENT_SENS_MV_PER_A
#define BSP_ANALOG_CURRENT_SENS_MV_PER_A   33      /* 灵敏度 33 mV/A（CC6920BSO-40A） */
#endif
#ifndef BSP_ANALOG_CURRENT_ZERO_CODE
#define BSP_ANALOG_CURRENT_ZERO_CODE       2048    /* 零电流 ADC 码≈2048（实测反推：3103 时零电流读成 -25.8A，
                                                    * 反算真实零点码≈2048；采样前端运放/分压把 CC6920 的 2.5V 零点缩放回 ~1.65V）。
                                                    * ★CAL-11 仍须实测零电流码精确替换 + 核对灵敏度/符号。 */
#endif
#ifndef BSP_ANALOG_VREF_MV
#define BSP_ANALOG_VREF_MV          3300    /* ADC 基准 3.3V */
#endif
#ifndef BSP_ANALOG_ADC_FULL
#define BSP_ANALOG_ADC_FULL         4096U   /* 12bit 满量程 */
#endif
/* 温度调理运放增益：ADC 读到的是 BRAKE_TEMP = 0.62 × VTemp_IN（原理图 sheet2）。
 * 查官方 NTC R/T 表前须先 ÷0.62 把 ADC 码还原成 VTemp_IN 对应的码（用整数 100/62 避免浮点）。 */
#ifndef BSP_ANALOG_TEMP_GAIN_NUM
#define BSP_ANALOG_TEMP_GAIN_NUM    100     /* 0.62 的倒数用 100/62 表示 */
#endif
#ifndef BSP_ANALOG_TEMP_GAIN_DEN
#define BSP_ANALOG_TEMP_GAIN_DEN    62
#endif
/* DMA 通道（ADC0 规则组对应 DMA0_CH0） */
#ifndef BSP_ANALOG_DMA
#define BSP_ANALOG_DMA              DMA0
#define BSP_ANALOG_DMA_CH           DMA_CH0
#endif
/* 规则组通道数及 DMA 缓冲索引 */
#define BSP_ANALOG_CH_NUM           3U
#define BSP_ANALOG_IDX_CURRENT      0U
#define BSP_ANALOG_IDX_VBUS         1U
#define BSP_ANALOG_IDX_TEMP         2U

typedef enum {
    BSP_ANALOG_OK          =  0,
    BSP_ANALOG_ERR_PARAM   = -1,
    BSP_ANALOG_ERR_INIT    = -2,
    BSP_ANALOG_ERR_TIMEOUT = -3,
} Bsp_Analog_Status_T;

Bsp_Analog_Status_T Bsp_Analog_Init(void);
int      Bsp_Analog_IsInitialized(void);
uint16_t Bsp_Analog_ReadRawCurrent(void);   /* 电流通道原始 ADC 值 */
uint16_t Bsp_Analog_ReadRawVBus(void);      /* 电压通道原始 ADC 值 */
uint16_t Bsp_Analog_ReadRawTemp(void);      /* 温度通道原始 ADC 值 */
int32_t  Bsp_Analog_GetVBus_mV(void);       /* 换算后母线电压（mV） */
int32_t  Bsp_Analog_GetCurrent_mA(void);    /* 换算后母线电流（mA，带符号） */
int32_t  Bsp_Analog_GetTemp_mDegC(void);    /* 换算后温度（m°C，NTC 查表插值） */

/* 功率统计（REQ-FUNC-004）：瞬时 P=U×I；平均/峰值由 BSP 内部状态维护，
 * 须周期调用 Bsp_Analog_PowerUpdate() 累计，Reset 清零统计窗口 */
int32_t  Bsp_Analog_GetPower_mW(void);      /* 瞬时功率（mW，= 电压 × 电流，带符号） */
void     Bsp_Analog_PowerUpdate(void);      /* 采一次瞬时功率并更新平均/峰值统计，须周期调用 */
void     Bsp_Analog_PowerStatReset(void);   /* 复位平均/峰值统计窗口 */
int32_t  Bsp_Analog_GetAvgPower_mW(void);   /* 统计窗口内平均功率（mW） */
int32_t  Bsp_Analog_GetPeakPower_mW(void);  /* 统计窗口内峰值功率（mW，按绝对值取峰） */

/* ADC 滑动滤波（依据《过压保护设计思路梳理》抗误判：adc_filtered=(adc_filtered*7+adc_new)/8）。
 * 须周期调用 FilterUpdate 推进滤波；GetVBusFiltered_mV 返回滤波后母线电压，用于过压判定，抗瞬态毛刺误触发 */
#ifndef BSP_ANALOG_FILTER_SHIFT
#define BSP_ANALOG_FILTER_SHIFT     3U          /* 滤波系数：新值权重 1/8（移位 3），旧值 7/8 */
#endif
void     Bsp_Analog_FilterUpdate(void);     /* 推进三通道滑动滤波一次，须周期调用 */
void     Bsp_Analog_FilterReset(void);      /* 用当前原始值重置滤波器初值 */
uint16_t Bsp_Analog_GetFilteredVBus(void);  /* 滤波后电压通道原始码 */
int32_t  Bsp_Analog_GetVBusFiltered_mV(void); /* 滤波后母线电压（mV），过压判定用 */

/* —— 母线电压 ADC 模拟看门狗（过压/欠压中断快速响应，见《过压欠压ADC看门狗中断快速响应设计》）——
 * 看门狗盯电压通道单通道，越 [low, high] 窗口硬件立即触发 WDE 中断（µs 级，确定性上界）。
 * ISR 只做最小动作（起/停 PWM + 置标志），精细判定仍由主循环 AppProt_Update 接管。 */
typedef void (*Bsp_Analog_WdgCb_T)(void);   /* 看门狗越限回调（ISR 上下文，须极短、不阻塞） */

uint16_t Bsp_Analog_VBusMvToCode(int32_t mv);                      /* 母线电压(mV) → ADC 原始码（阈值配置用，×49 逆运算） */
void     Bsp_Analog_SetVBusWindow(uint16_t code_lo, uint16_t code_hi); /* 设看门狗低/高阈值码（阈值变更时调） */
void     Bsp_Analog_EnableVBusWatchdog(Bsp_Analog_WdgCb_T cb);     /* 挂电压通道单通道看门狗 + 使能 WDE 中断 + 注册回调 */
void     Bsp_Analog_DisableVBusWatchdog(void);                     /* 关看门狗中断（安全态/测试用） */
void     Bsp_Analog_WatchdogIrq(void);                             /* 由 it.c 的 ADC0_1_IRQHandler 调用，转调回调 */

#endif /* BSP_USE_ANALOG */

/* ====================================================================
 * 模块：PWM 泄放控制 DISCHARGE（BSP_USE_DISCHARGE）
 *   PA8 PWM（TIMER0 CH0）→ 隔离栅驱动 → MOSFET；PA9 Brake_FB 反馈
 * ==================================================================== */
#if defined(BSP_USE_DISCHARGE)

#ifndef BSP_DISCHARGE_TIMER
#define BSP_DISCHARGE_TIMER         TIMER0
#endif
#ifndef BSP_DISCHARGE_PWM_PORT
#define BSP_DISCHARGE_PWM_PORT      GPIOA
#define BSP_DISCHARGE_PWM_PIN       GPIO_PIN_8       /* TIMER0_CH0 */
#endif
#ifndef BSP_DISCHARGE_FB_PORT
#define BSP_DISCHARGE_FB_PORT       GPIOA
#define BSP_DISCHARGE_FB_PIN        GPIO_PIN_9       /* Brake_FB 反馈输入 */
#endif
/* 泄放指示灯（PB0 推挽输出，★低电平点亮/低有效）：泄放中亮、停止/完成灭。
 *   由主循环 AppDch_ServiceDchLed() 按"泄放标志位"驱动：标志=1(泄放中)亮 / =0(停或完成)灭；
 *   ★最小亮灯：泄放极快(灯一闪难辨)，故每次起泄放后至少再亮 BSP_DCH_LED_MIN_ON_MS(1s) 才允许熄灭；
 *   并在母线持续 <55V 达 BSP_DCH_LED_OFF_FALLBACK_MS 时兜底清标志（防停泄放中断偶发丢失致常亮）。 */
#ifndef BSP_DCH_LED_PORT
#define BSP_DCH_LED_PORT            GPIOB
#define BSP_DCH_LED_PIN             GPIO_PIN_0
#define BSP_DCH_LED_RCU             RCU_GPIOB
#endif
#ifndef BSP_DCH_LED_MIN_ON_MS
#define BSP_DCH_LED_MIN_ON_MS       1000U   /* 最小亮灯：自最后一次泄放起至少再亮此时长才熄灭（便于肉眼观测） */
#endif
/* 上电版本识别闪灯：首次上电先按 BSP_DCH_LED_PON_BLINK_MS 周期翻转 BSP_DCH_LED_PON_BLINK_COUNT 次
 *   （从“亮”起，4 次=亮/灭/亮/灭=闪 2 下，末次保证停在灭），用于与上板程序/旧固件区分。闪灯期间泄放灯服务让位。 */
#ifndef BSP_DCH_LED_PON_BLINK_MS
#define BSP_DCH_LED_PON_BLINK_MS    200U    /* 上电闪灯翻转周期 200ms */
#endif
#ifndef BSP_DCH_LED_PON_BLINK_COUNT
#define BSP_DCH_LED_PON_BLINK_COUNT 4U      /* 上电翻转次数 4（=闪 2 下；偶数保证末次为灭） */
#endif
#ifndef BSP_DCH_LED_OFF_FALLBACK_MS
#define BSP_DCH_LED_OFF_FALLBACK_MS 2000U   /* 兜底清标志：母线持续<55V 达此时长即强制清泄放标志 */
#endif
/* PWM 频率与占空比范围（20kHz）。占空比工作区 5%~90%（★负载实测重标）。
 * 空载 5% 已能泄压（48V/0.2s 约降 15V），但大电流实载电机持续灌能量，同占空比净压降小得多，
 * 需爬到更高占空比才压得住：P=V²/R×duty，5Ω 下 60V/100W≈14%、56V/200W≈32%、90%@60V≈648W(仅短时脉冲)。
 * ★安全：高占空比稳态功率远超电阻额定 200W，靠三道闸兜底——过流/过压(电压电流触发) →
 *   过温 95℃ 停泄放(高功率时先触发) → 连续泄放 20s 超时停(中低功率长时间兜底，见 BSP_OVP_MAX_ON_MS)。
 * 注：DUTY_MAX 为安全硬顶，运行占空比由控制环在 [DUTY_MIN, DUTY_MAX] 内按泄放效果自适应爬升。 */
#ifndef BSP_DISCHARGE_PWM_FREQ_HZ
#define BSP_DISCHARGE_PWM_FREQ_HZ   20000U           /* 控制频率 20kHz */
#endif
#ifndef BSP_DISCHARGE_DUTY_MAX
#define BSP_DISCHARGE_DUTY_MAX      100U             /* 一级泄放占空比上限 100%（★用户定案：允许 L1 自身爬到满占空比，
                                                      * 不依赖 L2 触发即可全力泄放——防母线被开关电源过压保护夹住、始终冲不到 58V 的场景）。
                                                      * 满占空比稳态超电阻额定，与 L2 同级，靠 20s 超时+95℃过温兜底。 */
#endif
#ifndef BSP_DISCHARGE_DUTY_MIN
#define BSP_DISCHARGE_DUTY_MIN      5U               /* 占空比下限 5%（低于此泄压无意义；0=关断除外） */
#endif
#ifndef BSP_DISCHARGE_DUTY_START
#define BSP_DISCHARGE_DUTY_START    50U              /* 一级泄放起步占空比 50%（56.5V 起泄放，之后每步 +5% 爬到 90% 封顶）
                                                      * ★起步值提高：原 20% 沉降功率不足、母线压不动，爬坡到临界占空比需 ~1ms（响应慢主因）；
                                                      *   50% 起步使沉降功率一上来即足够，母线几乎立即掉头，见《泄放中断即时响应优化设计说明》。 */
#endif
#ifndef BSP_DISCHARGE_DUTY_L2_FULL
#define BSP_DISCHARGE_DUTY_L2_FULL  100U             /* 二级泄放占空比 100%（58V 全力泄放，MOSFET 常通；靠 20s+95℃ 兜底） */
#endif
#ifndef BSP_DISCHARGE_L1_STEP_PERIOD_US
#define BSP_DISCHARGE_L1_STEP_PERIOD_US  50U         /* 一级步进周期 50µs（TIMER1 节拍；★宏可调，实测冲太快可增大）
                                                      * 注：本宏亦决定 TIMER1→PA15 外部看门狗喂狗 PWM 频率(=1/period)，
                                                      *     50µs→20kHz，边沿更密于 TPS3820 超时窗，安全无影响。 */
#endif
/* 占空比斜坡步进（每次 RampStep 调用的增量）。
 * 设计要求过压期间以固定斜率 dI/dt 增加占空比，slope=0.2%/?us（★待定 CAL-17）。
 * 此处给"每步增量"，步进调用周期由上层按 slope 时间常数控制 */
#ifndef BSP_DISCHARGE_TEST_DUTY
#define BSP_DISCHARGE_TEST_DUTY     5U               /* PWM 验证/演示用安全占空比（≤DUTY_MAX，避开高占空比危险区） */
#endif
#ifndef BSP_DISCHARGE_DUTY_STEP
#define BSP_DISCHARGE_DUTY_STEP     10U              /* 一级每步 +10%（★用户定案；配合 50µs TIMER1 节拍）
                                                      * 50%→100% 封顶仅需 (100-50)/10×50µs=250µs（原 +2%/100µs 从 20% 起需 3.5ms） */
#endif
/* —— 泄放闭环步进控制参数（DCH 按泄放效果自适应调占空比）——
 * 逻辑：过压泄放从 DUTY_START(50%) 起，每 STEP_PERIOD 检测母线是否已降到停止电压(55V)；
 *       未降则占空比 +DUTY_STEP(10%)，直至封顶 DUTY_MAX(100%)；封顶后再滞留 CAP_HOLD_STEPS 步
 *       仍压不下 → 置 DCH_TIMEOUT 故障+上报停机。另有 BSP_OVP_MAX_ON_MS(20s) 连续泄放兜底超时。
 * ★标定警示：DUTY_START/STEP/MAX 均为暂定值，测试时按串口遥测数据重标（起步值、斜率、封顶）。
 *   高占空比稳态功率远超电阻额定 200W（90%@60V≈648W），仅短时脉冲可承受，靠 20s 超时+95℃过温兜底；
 *   负载仪模拟大电流有炸板风险，须限流+足额电阻+散热+值守，从保守值逐步放开。 */
#ifndef BSP_DISCHARGE_DUTY_STEP_PERIOD_MS
#define BSP_DISCHARGE_DUTY_STEP_PERIOD_MS  100U      /* 步进检测周期 100ms（★标定） */
#endif
#ifndef BSP_DISCHARGE_DUTY_CAP_HOLD_STEPS
#define BSP_DISCHARGE_DUTY_CAP_HOLD_STEPS  5U        /* 封顶后滞留步数(×周期)未达停止电压→停机上报（★标定，5步≈500ms） */
#endif
/* PWM 频率合法范围（SetFreq 越界拒绝；上层据此回落默认宏值） */
#ifndef BSP_DISCHARGE_PWM_FREQ_MIN_HZ
#define BSP_DISCHARGE_PWM_FREQ_MIN_HZ  1000U
#endif
#ifndef BSP_DISCHARGE_PWM_FREQ_MAX_HZ
#define BSP_DISCHARGE_PWM_FREQ_MAX_HZ  50000U
#endif

typedef enum {
    BSP_DISCHARGE_OK         =  0,
    BSP_DISCHARGE_ERR_PARAM  = -1,
    BSP_DISCHARGE_ERR_INIT   = -2,
} Bsp_Discharge_Status_T;

Bsp_Discharge_Status_T Bsp_Discharge_Init(void);        /* 初始化 PWM 与反馈 GPIO，默认关断 */
int  Bsp_Discharge_IsInitialized(void);
void Bsp_Discharge_SetDuty(uint8_t duty_percent);       /* 设置占空比 0~DUTY_MAX（%），超限自动钳制 */
uint8_t Bsp_Discharge_GetDuty(void);                    /* 读回当前生效占空比（%） */
uint32_t Bsp_Discharge_GetFreq(void);                   /* 读回当前 PWM 频率（Hz） */
Bsp_Discharge_Status_T Bsp_Discharge_SetFreq(uint32_t freq_hz); /* 设置 PWM 频率（Hz）；越界返回 ERR_PARAM；已初始化则即时重配生效 */
uint8_t Bsp_Discharge_RampToDuty(uint8_t target_percent); /* 朝目标占空比斜坡一步（dI/dt 限制热冲击），返回步进后占空比 */
void Bsp_Discharge_Stop(void);                          /* 立即关断泄放（安全态） */
void Bsp_Discharge_ForcePinLow(void);                   /* 空闲防误泄放：PA8 切GPIO强制拉低（起泄放时SetDuty自动恢复AF） */
int  Bsp_Discharge_ReadFeedback(void);                  /* 读 Brake_FB 反馈电平 */
void Bsp_DchLed_Set(uint8_t on);                        /* 泄放指示灯（PB0，低有效）：on=1 亮 / on=0 灭 */

#endif /* BSP_USE_DISCHARGE */

/* ====================================================================
 * 过压保护策略参数（依据《过压保护设计思路梳理》）
 *   集中定义阈值/温度/时间常数宏，供上层保护状态机（App/单元测试）引用。
 *   BSP 不实现状态机，仅提供这些可标定参数与采样/PWM/温度原语。
 *   阈值均带迟滞（动作值 > 恢复值），防抖动。单位：电压 mV，温度 m℃，时间 ms。
 * ==================================================================== */

/* —— 电压阈值（mV，★样机标定 CAL-01~03） —— */
#ifndef BSP_OVP_SW_ON_MV
#define BSP_OVP_SW_ON_MV        56000   /* 泄放软件开启动作值 56V */
#endif
#ifndef BSP_OVP_SW_OFF_MV
#define BSP_OVP_SW_OFF_MV       55000   /* 泄放软件关闭恢复值 55V（迟滞） */
#endif
#ifndef BSP_OVP_HW_ON_MV
#define BSP_OVP_HW_ON_MV        58000   /* 泄放硬件比较器开启 58V（兜底，仅参考） */
#endif
#ifndef BSP_OVP_HW_OFF_MV
#define BSP_OVP_HW_OFF_MV       56000   /* 泄放硬件恢复 56V */
#endif
/* ⚠ 遗留参考值（旧施密特设计，58V/60V），仅 test_ovp_logic.c 早期参考实现引用；
 *   量产报警阈值 OV_L1/OV_L2 由 app_protect.c 用 BSP_DCH_L1_ON_MV(56.5V)/BSP_DCH_L2_ON_MV(58V)
 *   判定（无 60V）。可标定报警配置默认见 BSP_CFG_DEFAULT_OVP_L1/L2_0V1(565/580)。勿在量产逻辑引用以下四宏。 */
#ifndef BSP_OVP_L1_MV
#define BSP_OVP_L1_MV           58000   /* 【遗留/参考】旧一级报警 58V（量产用 BSP_DCH_L1_ON_MV=56.5V） */
#endif
#ifndef BSP_OVP_L1_OFF_MV
#define BSP_OVP_L1_OFF_MV       56000   /* 【遗留/参考】旧一级恢复 56V */
#endif
#ifndef BSP_OVP_L2_MV
#define BSP_OVP_L2_MV           60000   /* 【遗留/参考】旧二级报警 60V（量产已作废，OV_L2=BSP_DCH_L2_ON_MV=58V） */
#endif
#ifndef BSP_OVP_L2_OFF_MV
#define BSP_OVP_L2_OFF_MV       58000   /* 【遗留/参考】旧二级恢复 58V */
#endif

/* —— 分级泄放阈值（mV，★用户定案；单一来源编译期宏，便于调参，不走 FRAM）——
 * 泄放动作分两级 + 停止，带迟滞：
 *   停止/恢复 55V  ── 母线跌破即停泄放（L2/L1→L0）
 *   一级 56.5V     ── 起 20% 起步、每 100µs +2% 爬升、封顶 90%（L0→L1）
 *   二级 58V       ── 直接 100% 全力泄放（L1→L2）；跌破 56.5V 退回一级（L2→L1）
 * 由 ADC 看门狗(起/停触发) + TIMER1 100µs(一级步进/二级判定) 共同执行。 */
#ifndef BSP_DCH_L1_ON_MV
#define BSP_DCH_L1_ON_MV        56500   /* 一级泄放触发 56.5V */
#endif
#ifndef BSP_DCH_L2_ON_MV
#define BSP_DCH_L2_ON_MV        58000   /* 二级泄放触发 58V（→100%） */
#endif
/* —— L1 上升趋势升级（用户定案）——：L1 泄放中若母线连升 N 步(每步=一个 TIMER1 节拍)仍高于上一步
 *   电压(泄放压不住、还在往上冲) → 不等 +10% 爬坡到顶、也不等 58V 看门狗，直接跳二级 100%。
 *   防"电源把母线夹在 56.5~58V、58V 永不触发、只能靠 250µs 慢爬到满"的场景。升级到 100% 是安全方向。 */
#ifndef BSP_DCH_L1_RISE_ESCALATE_STEPS
#define BSP_DCH_L1_RISE_ESCALATE_STEPS  2U      /* 连升 2 步即升二级（★可调；对应"两次步进后电压仍比之前大"） */
#endif
#ifndef BSP_DCH_L1_RISE_MARGIN_MV
#define BSP_DCH_L1_RISE_MARGIN_MV       100     /* 判"上升"的最小增量(mV)，滤 ADC 抖动；本步 > 上步 + 该值 才算升 */
#endif
#ifndef BSP_DCH_OFF_MV
#define BSP_DCH_OFF_MV          55000   /* 泄放停止/恢复 55V（跌破即停） */
#endif

/* —— 温度保护（m℃，★散热测试与样机标定 CAL-04） —— */
#ifndef BSP_OTP_OFF_MDEGC
#define BSP_OTP_OFF_MDEGC       95000  /* 泄放电阻过温关闭阈值 95℃ */
#endif
#ifndef BSP_OTP_RECOVER_MDEGC
#define BSP_OTP_RECOVER_MDEGC   65000   /* 过温恢复阈值 65℃（≤此值才自动解除过温、重新“支持”泄放；用户定案，非自动起泄放） */
#endif

/* —— 时间常数（ms） —— */
#ifndef BSP_OVP_MAX_ON_MS
#define BSP_OVP_MAX_ON_MS       20000U  /* 连续泄放最长 20s：母线持续 ≥ 起泄放阈值(56.5V, BSP_DCH_L1_ON_MV)
                                         * 连续泄放超此值即停(兜底防烧)——即"20s 都压不到起泄放阈值以下"。
                                         * 计时按"会话"：起泄放时起算，L1↔L2 切换不重置；母线【瞬时】跌破 56.5V 即清零并解锁。
                                         * 一般用不到——若 20s 还压不下去，多半泄放功能已失效(另有 Brake_FB 回路失效检测)。
                                         * 当前非锁存(自恢复：母线跌破 56.5V 自动解锁，再过压可重开)；后续可改锁存(须 0x31 清)。 */
#endif
#ifndef BSP_OVP_COOL_MS
#define BSP_OVP_COOL_MS         480000U /* 封顶滞留停机后冷却等待 8min（仅 OVP 参考实现用；固件冷却闭锁已移除） */
#endif

/* —— 控制字 —— */
#ifndef BSP_OVP_CTRL_FORCE_FULL
#define BSP_OVP_CTRL_FORCE_FULL 0x10U   /* 控制字 0x10：强制全开 */
#endif


/* ====================================================================
 * 模块：CAN 通信（BSP_USE_CAN）
 *   PA11 CAN0_RX / PA12 CAN0_TX；收发器 TCAN3414
 *   经典 CAN（CAN2.0B，≤8字节/帧）。
 *   注：GD32F303 片内 CAN 控制器仅支持经典 CAN 2.0B，不支持 CAN-FD；
 *       TCAN3414 虽为 CAN-FD 收发器，但仅是物理层，无法弥补 MCU 控制器能力缺失。
 *   对应需求：REQ-FUNC-040/042，REQ-IF-001/002
 * ==================================================================== */
#if defined(BSP_USE_CAN)

#ifndef BSP_CAN_INSTANCE
#define BSP_CAN_INSTANCE            CAN0
#endif
#ifndef BSP_CAN_RX_PORT
#define BSP_CAN_RX_PORT             GPIOA
#define BSP_CAN_RX_PIN              GPIO_PIN_11
#endif
#ifndef BSP_CAN_TX_PORT
#define BSP_CAN_TX_PORT             GPIOA
#define BSP_CAN_TX_PIN              GPIO_PIN_12
#endif
/* 仲裁段波特率（★待定，默认 500kbps） */
#ifndef BSP_CAN_BAUD_NOMINAL
#define BSP_CAN_BAUD_NOMINAL        500000U
#endif
/* RX FIFO0 接收中断向量（CAN0 RX0 与 USBD_LP 共用，见官方 demo 17_CAN_Network） */
#ifndef BSP_CAN_RX0_IRQN
#define BSP_CAN_RX0_IRQN            USBD_LP_CAN0_RX0_IRQn
#endif
/* RX 中断抢占/子优先级（默认组 PRE2_SUB2；留 0/1 给将来更高优先级的过压快路径） */
#ifndef BSP_CAN_RX_IRQ_PRE_PRIO
#define BSP_CAN_RX_IRQ_PRE_PRIO     2U
#endif
#ifndef BSP_CAN_RX_IRQ_SUB_PRIO
#define BSP_CAN_RX_IRQ_SUB_PRIO     0U
#endif

typedef enum {
    BSP_CAN_OK          =  0,
    BSP_CAN_ERR_PARAM   = -1,
    BSP_CAN_ERR_INIT    = -2,
    BSP_CAN_ERR_TIMEOUT = -3,
    BSP_CAN_ERR_NOMSG   = -4,
} Bsp_Can_Status_T;

/* 报文结构（经典 CAN，数据段最大 8 字节） */
typedef struct {
    uint32_t id;        /* 帧 ID */
    uint8_t  ide;       /* 0=标准帧，1=扩展帧 */
    uint8_t  dlc;       /* 数据长度（≤8） */
    uint8_t  data[8];   /* 经典 CAN 最大 8 字节 */
} Bsp_Can_Frame_T;

/* RX 接收回调：在 CAN0 RX0 中断上下文调用，参数为收到的一帧（只读，调用后失效）。
 * 约定：回调应保持极短（搬运/置标志），不得在其中做阻塞操作或 CAN 发送。 */
typedef void (*Bsp_Can_RxCallback_T)(const Bsp_Can_Frame_T *frame);

Bsp_Can_Status_T Bsp_Can_Init(void);
int  Bsp_Can_IsInitialized(void);
Bsp_Can_Status_T Bsp_Can_Send(const Bsp_Can_Frame_T *frame);   /* 发送一帧 */
int  Bsp_Can_Receive(Bsp_Can_Frame_T *frame);                  /* 轮询收一帧，无报文返回 0 */

/* 中断接收（从站事件驱动，HLD-MOD-COMM）：注册回调并使能 RX FIFO0 非空中断 + NVIC。
 * cb 为 NULL 时返回 BSP_CAN_ERR_PARAM；未初始化返回 BSP_CAN_ERR_INIT。 */
Bsp_Can_Status_T Bsp_Can_EnableRxIt(Bsp_Can_RxCallback_T cb);
/* 关闭 RX FIFO0 中断并注销回调 */
void Bsp_Can_DisableRxIt(void);
/* RX FIFO0 中断处理：必须由 USBD_LP_CAN0_RX0_IRQHandler 调用（见 gd32f30x_it.c）。
 * 取出 FIFO0 中所有待收报文，逐帧回调；无回调时丢弃以清空 FIFO。 */
void Bsp_Can_Rx0IrqHandler(void);

#endif /* BSP_USE_CAN */

/* ====================================================================
 * 模块：外部看门狗 WDG（BSP_USE_WDG）
 *   PA15 — MCU_Watchdog_WDI → TPS3820（外部硬件看门狗）
 *   MCU 须周期翻转 WDI 喂狗；超时 TPS3820 拉低 /RESET。
 *   ★硬件改版(解耦)：/RESET 不再接 MCU_NRST，改接泄放通路门控 —— 超时只切断泄放
 *   (强制 MOSFET 关断)，MCU 不复位、继续运行。喂狗防的是"泄放失控"而非"MCU 死机"。
 *   注：外部硬件看门狗，非 MCU 内部 FWDGT/WWDGT。
 *   对应需求：REQ-FUNC-030，REQ-SAFE-002
 * ==================================================================== */
#if defined(BSP_USE_WDG)

#ifndef BSP_WDG_WDI_PORT
#define BSP_WDG_WDI_PORT            GPIOA
#define BSP_WDG_WDI_PIN             GPIO_PIN_15      /* MCU_Watchdog_WDI */
#endif
#ifndef BSP_WDG_RCU_GPIO
#define BSP_WDG_RCU_GPIO            RCU_GPIOA
#endif

typedef enum {
    BSP_WDG_OK          =  0,
    BSP_WDG_ERR_PARAM   = -1,
    BSP_WDG_ERR_INIT    = -2,
} Bsp_Wdg_Status_T;

Bsp_Wdg_Status_T Bsp_Wdg_Init(void);       /* 初始化 WDI GPIO（解除 PA15 JTAG 复用） */
int  Bsp_Wdg_IsInitialized(void);
void Bsp_Wdg_Feed(void);                    /* 翻转 WDI 喂狗，须在超时窗口内周期调用 */

#endif /* BSP_USE_WDG */

/* ====================================================================
 * 模块：SPI1 通用总线驱动（BSP_USE_SPI）
 *   PB13 SCK / PB14 MISO / PB15 MOSI；主模式 Mode0
 *   通用总线层，不含器件语义；片选（CS）由各器件驱动自行管理。
 * ==================================================================== */
#if defined(BSP_USE_SPI)

#ifndef BSP_SPI_INSTANCE
#define BSP_SPI_INSTANCE            SPI1
#endif
#ifndef BSP_SPI_PORT
#define BSP_SPI_PORT                GPIOB
#define BSP_SPI_SCK_PIN             GPIO_PIN_13
#define BSP_SPI_MISO_PIN            GPIO_PIN_14
#define BSP_SPI_MOSI_PIN            GPIO_PIN_15
#endif
/* FRAM MB85RS64 支持 Mode0（CPOL=0，CPHA=0），最高 20MHz */
#ifndef BSP_SPI_PRESCALER
#define BSP_SPI_PRESCALER           SPI_PSC_8        /* APB1=50M/8=6.25MHz ≤20MHz */
#endif

typedef enum {
    BSP_SPI_OK          =  0,
    BSP_SPI_ERR_PARAM   = -1,
    BSP_SPI_ERR_INIT    = -2,
    BSP_SPI_ERR_TIMEOUT = -3,
} Bsp_Spi_Status_T;

Bsp_Spi_Status_T Bsp_Spi_Init(void);
int     Bsp_Spi_IsInitialized(void);
uint8_t Bsp_Spi_TransferByte(uint8_t tx);                                         /* 全双工收发一字节 */
int     Bsp_Spi_XferFailed(void);                                                 /* 最近一次 TransferByte 是否超时失败 */
Bsp_Spi_Status_T Bsp_Spi_TransferBuffer(const uint8_t *tx, uint8_t *rx, uint16_t len); /* 块收发 */

#endif /* BSP_USE_SPI */

/* ====================================================================
 * 模块：FRAM 黑匣子存储（BSP_USE_FRAM）
 *   SPI1：PB13 SCK / PB14 MISO / PB15 MOSI / PB12 CS；PB10 WP / PB11 HOLD
 *   器件 MB85RS64（64Kbit = 8KB）
 * ==================================================================== */
#if defined(BSP_USE_FRAM)

#ifndef BSP_FRAM_SPI_INSTANCE
#define BSP_FRAM_SPI_INSTANCE       SPI1
#endif
#ifndef BSP_FRAM_PORT
#define BSP_FRAM_PORT               GPIOB
#define BSP_FRAM_WP_PIN             GPIO_PIN_10
#define BSP_FRAM_HOLD_PIN           GPIO_PIN_11
#define BSP_FRAM_CS_PIN             GPIO_PIN_12
#define BSP_FRAM_SCK_PIN            GPIO_PIN_13
#define BSP_FRAM_MISO_PIN           GPIO_PIN_14
#define BSP_FRAM_MOSI_PIN           GPIO_PIN_15
#endif
#ifndef BSP_FRAM_SIZE_BYTES
#define BSP_FRAM_SIZE_BYTES         8192U            /* 64Kbit = 8KB */
#endif

typedef enum {
    BSP_FRAM_OK          =  0,
    BSP_FRAM_ERR_PARAM   = -1,
    BSP_FRAM_ERR_INIT    = -2,
    BSP_FRAM_ERR_RANGE   = -3,
    BSP_FRAM_ERR_TIMEOUT = -4,
} Bsp_Fram_Status_T;

Bsp_Fram_Status_T Bsp_Fram_Init(void);
int  Bsp_Fram_IsInitialized(void);
Bsp_Fram_Status_T Bsp_Fram_Read(uint16_t addr, void *buf, uint16_t len);
Bsp_Fram_Status_T Bsp_Fram_Write(uint16_t addr, const void *buf, uint16_t len);

#endif /* BSP_USE_FRAM */

/* ====================================================================
 * 模块：片上 Flash（BSP_USE_FLASH）
 *   HD 系列 2KB/页，最后一页 0x0803F800 用于配置参数存储
 *   操作规则：必须先擦除（页为单位）后才能写（32bit 字为单位）
 * ==================================================================== */
#if defined(BSP_USE_FLASH)

/* GD32F303CC: 256KB Flash, 2KB/页 */
#ifndef BSP_FLASH_PAGE_SIZE
#define BSP_FLASH_PAGE_SIZE     2048U
#endif
#ifndef BSP_FLASH_CFG_ADDR
#define BSP_FLASH_CFG_ADDR      0x0803F800U   /* 最后一页起始地址 */
#endif

typedef enum {
    BSP_FLASH_OK          =  0,
    BSP_FLASH_ERR_PARAM   = -1,
    BSP_FLASH_ERR_RANGE   = -2,
    BSP_FLASH_ERR_ERASE   = -3,
    BSP_FLASH_ERR_WRITE   = -4,
} Bsp_Flash_Status_T;

Bsp_Flash_Status_T Bsp_Flash_ErasePage(uint32_t page_addr);   /* 擦除配置页（仅允许 CFG_ADDR）*/
Bsp_Flash_Status_T Bsp_Flash_Write(uint32_t addr, const void *buf, uint32_t len); /* 写（4B 对齐，先擦后用）*/
Bsp_Flash_Status_T Bsp_Flash_Read(uint32_t addr, void *buf, uint32_t len);        /* 读 */

#endif /* BSP_USE_FLASH */

/* ====================================================================
 * 只读设备信息（代码段宏，不存 FRAM；CAN 0x24 由 App 打包应答）
 * ==================================================================== */
#ifndef BSP_APP_FW_MAJOR
#define BSP_APP_FW_MAJOR            1U
#endif
#ifndef BSP_APP_FW_MINOR
#define BSP_APP_FW_MINOR            0U
#endif
#ifndef BSP_APP_FW_PATCH
#define BSP_APP_FW_PATCH            0U
#endif
#ifndef BSP_APP_HW_VER
#define BSP_APP_HW_VER              1U      /* 硬件版本，随 PCB/BOM 定 */
#endif
#ifndef BSP_APP_DEV_TYPE
#define BSP_APP_DEV_TYPE            0x0001U /* 设备型号/类别码 */
#endif

/* ====================================================================
 * 模块：设备配置（BSP_USE_CONFIG）
 *   存储自身 CAN ID、绑定主站 CAN ID、CAN 波特率档位
 *   存于 FRAM 末尾 16 字节（BSP_CFG_FRAM_ADDR），带 magic + CRC16 校验，
 *   读出全 0/0xFF 或非法值回落默认值（依赖 BSP_USE_FRAM）
 *   设置后调用 Bsp_Config_Save() 保存，重启后由 Bsp_Can_Init() 取最新值生效
 *   注：片上 Flash 方案（bsp_flash）暂时停用，配置改存 FRAM
 * ==================================================================== */
#if defined(BSP_USE_CONFIG)

/* 配置在 FRAM 中的存储地址：工作配置区 @ 0x1F80（128B 区，见 FRAM 文档配置区布局）。
 * 含 CAN 参数 + 可标定保护阈值，blob 约 34B；旧 16B@0x1FF0 布局已废弃（magic 版本升级后自动回落默认）。 */
#ifndef BSP_CFG_FRAM_ADDR
#define BSP_CFG_FRAM_ADDR           0x1F80U
#endif
/* 出厂默认配置备份（代码段 const；恢复出厂/Load 回落唯一来源，见 FRAM 文档 §3.0）。
 * 阈值单位贴合 CAN 标度：电压 0.1V/LSB，温度 0.1℃/LSB，电流 0.01A/LSB。 */
typedef struct {
    uint16_t self_id;
    uint16_t master_id;
    uint8_t  baud_idx;
    uint8_t  pwm_duty_max;
    uint8_t  pwm_duty_step;
    uint8_t  pwm_duty_start;
    uint16_t pwm_freq_hz;
    /* —— 可标定保护阈值（0x04/0x05/0x07/0x14/0x15/0x17）—— */
    uint16_t ovp_on_0v1;        /* 泄放启动电压 0.1V */
    uint16_t ovp_off_0v1;       /* 泄放停止/欠压停止电压 0.1V */
    uint16_t ovp_l1_0v1;        /* 一级报警动作 0.1V */
    uint16_t ovp_l1_off_0v1;    /* 一级报警恢复 0.1V */
    uint16_t ovp_l2_0v1;        /* 二级报警动作 0.1V */
    uint16_t ovp_l2_off_0v1;    /* 二级报警恢复 0.1V */
    uint16_t otp_off_0c1;       /* 过温关断 0.1℃ */
    uint16_t otp_recover_0c1;   /* 过温冷却恢复 0.1℃ */
    uint16_t ovc_0a01;          /* 过流阈值 0.01A */
} Bsp_Cfg_FactoryBackup_T;

extern const Bsp_Cfg_FactoryBackup_T g_bsp_cfg_factory_backup;

/* 默认参数（可通过 -D 覆盖） */
#ifndef BSP_CFG_DEFAULT_SELF_ID
#define BSP_CFG_DEFAULT_SELF_ID     1U      /* 从站自身 CAN ID */
#endif
#ifndef BSP_CFG_DEFAULT_MASTER_ID
#define BSP_CFG_DEFAULT_MASTER_ID   100U    /* 绑定主站 CAN ID */
#endif
#ifndef BSP_CFG_DEFAULT_BAUD
#define BSP_CFG_DEFAULT_BAUD        1U      /* BSP_CFG_BAUD_500K */
#endif

/* PWM 泄放参数默认值（与 discharge 宏一致，配置模块独立可见，可 -D 覆盖）。
 * 上电从 FRAM 读取配置；若该字段非法/越界，回落到这些默认宏值。 */
#ifndef BSP_CFG_DEFAULT_PWM_FREQ_HZ
#define BSP_CFG_DEFAULT_PWM_FREQ_HZ 20000U  /* PWM 频率 20kHz */
#endif
#ifndef BSP_CFG_DEFAULT_DUTY_MAX
#define BSP_CFG_DEFAULT_DUTY_MAX    90U     /* 占空比上限默认 90%（=安全硬顶；★暂定，测试重标） */
#endif
#ifndef BSP_CFG_DEFAULT_DUTY_START
#define BSP_CFG_DEFAULT_DUTY_START  50U     /* 默认启动占空比 50%（与 BSP_DISCHARGE_DUTY_START 对齐；★负载实测重标）
                                             * 注：一级起泄放实际用宏 BSP_DISCHARGE_DUTY_START，本配置仅供 CAN 遥测/整定上报 */
#endif
#ifndef BSP_CFG_DEFAULT_DUTY_STEP
#define BSP_CFG_DEFAULT_DUTY_STEP   5U      /* 斜坡步进 5%/步（与 BSP_DISCHARGE_DUTY_STEP 对齐） */
#endif
/* PWM 参数合法范围（越界即回落默认） */
#ifndef BSP_CFG_PWM_FREQ_MIN_HZ
#define BSP_CFG_PWM_FREQ_MIN_HZ     1000U
#endif
#ifndef BSP_CFG_PWM_FREQ_MAX_HZ
#define BSP_CFG_PWM_FREQ_MAX_HZ     50000U
#endif
#ifndef BSP_CFG_DUTY_MAX_CEIL
#define BSP_CFG_DUTY_MAX_CEIL       90U     /* 占空比上限不得超过 90%（安全硬顶） */
#endif
#ifndef BSP_CFG_DUTY_MIN
#define BSP_CFG_DUTY_MIN            5U      /* 占空比（上限/启动值）下限 5% */
#endif
#ifndef BSP_CFG_DUTY_STEP_MAX
#define BSP_CFG_DUTY_STEP_MAX       50U
#endif

/* —— 可标定保护阈值默认值（与 BSP_OVP/BSP_OTP 宏一致，单位换为 0.1V / 0.1℃ / 0.01A）—— */
#ifndef BSP_CFG_DEFAULT_OVP_ON_0V1
#define BSP_CFG_DEFAULT_OVP_ON_0V1      565U    /* 56.5V（与 BSP_DCH_L1_ON_MV 一级泄放触发对齐） */
#endif
#ifndef BSP_CFG_DEFAULT_OVP_OFF_0V1
#define BSP_CFG_DEFAULT_OVP_OFF_0V1     550U    /* 55.0V（兼作欠压停止） */
#endif
#ifndef BSP_CFG_DEFAULT_OVP_L1_0V1
#define BSP_CFG_DEFAULT_OVP_L1_0V1      565U    /* 一级报警 56.5V（与 OV_L1/一级泄放同阈值 BSP_DCH_L1_ON_MV；旧 58V 作废） */
#endif
#ifndef BSP_CFG_DEFAULT_OVP_L1_OFF_0V1
#define BSP_CFG_DEFAULT_OVP_L1_OFF_0V1  550U    /* 一级恢复 55.0V（OV_L1 清阈 BSP_DCH_OFF_MV） */
#endif
#ifndef BSP_CFG_DEFAULT_OVP_L2_0V1
#define BSP_CFG_DEFAULT_OVP_L2_0V1      580U    /* 二级报警 58.0V（与 OV_L2/二级泄放同阈值 BSP_DCH_L2_ON_MV；旧 60V 作废） */
#endif
#ifndef BSP_CFG_DEFAULT_OVP_L2_OFF_0V1
#define BSP_CFG_DEFAULT_OVP_L2_OFF_0V1  565U    /* 二级恢复 56.5V（OV_L2 清阈 BSP_DCH_L1_ON_MV） */
#endif
#ifndef BSP_CFG_DEFAULT_OTP_OFF_0C1
#define BSP_CFG_DEFAULT_OTP_OFF_0C1     950U    /* 95.0℃（过温关断触发） */
#endif
#ifndef BSP_CFG_DEFAULT_OTP_RECOVER_0C1
#define BSP_CFG_DEFAULT_OTP_RECOVER_0C1 650U    /* 65.0℃（冷却到此才重新支持泄放） */
#endif
#ifndef BSP_CFG_DEFAULT_OVC_0A01
#define BSP_CFG_DEFAULT_OVC_0A01        1200U   /* 12.00A（★CAL 待实测细化） */
#endif

/* 报警恢复迟滞（0x17 仅写动作值，恢复值 = 动作值 - 迟滞，单位 0.1V） */
#ifndef BSP_CFG_ALARM_HYST_0V1
#define BSP_CFG_ALARM_HYST_0V1          20U     /* 2.0V */
#endif

/* 阈值合法范围（越界回落默认/拒绝写入） */
#ifndef BSP_CFG_VOLT_MIN_0V1
#define BSP_CFG_VOLT_MIN_0V1            100U    /* 10.0V */
#endif
#ifndef BSP_CFG_VOLT_MAX_0V1
#define BSP_CFG_VOLT_MAX_0V1            980U    /* 98.0V（采样上限） */
#endif
#ifndef BSP_CFG_TEMP_MIN_0C1
#define BSP_CFG_TEMP_MIN_0C1            200U    /* 20.0℃ */
#endif
#ifndef BSP_CFG_TEMP_MAX_0C1
#define BSP_CFG_TEMP_MAX_0C1            1100U   /* 110.0℃ */
#endif
#ifndef BSP_CFG_CURR_MIN_0A01
#define BSP_CFG_CURR_MIN_0A01          100U     /* 1.00A */
#endif
#ifndef BSP_CFG_CURR_MAX_0A01
#define BSP_CFG_CURR_MAX_0A01          4000U    /* 40.00A（量程上限） */
#endif

/* CAN 波特率档位（APB1=50MHz，SJW=1，BS1=7，BS2=2，共 10TQ） */
typedef enum {
    BSP_CFG_BAUD_250K  = 0U,   /* prescaler=20, 250kbps */
    BSP_CFG_BAUD_500K  = 1U,   /* prescaler=10, 500kbps */
    BSP_CFG_BAUD_1M    = 2U,   /* prescaler=5,  1Mbps */
} Bsp_Cfg_Baud_T;

typedef enum {
    BSP_CFG_OK          =  0,
    BSP_CFG_ERR_PARAM   = -1,
    BSP_CFG_ERR_SAVE    = -2,
    BSP_CFG_ERR_CRC     = -3,   /* Flash 内容 CRC 校验失败（已回落默认）*/
} Bsp_Cfg_Status_T;

/* 加载（自动 CRC 校验，失败则写入默认值到 RAM，不触写 Flash）*/
Bsp_Cfg_Status_T  Bsp_Config_Load(void);

/* 保存当前 RAM 配置到 Flash（先擦后写）*/
Bsp_Cfg_Status_T  Bsp_Config_Save(void);

/* 恢复出厂默认值到 RAM（不自动写 FRAM，需手动 Save 或调 RestoreFromFactory）*/
void              Bsp_Config_SetDefault(void);

/* 恢复默认配置：代码段 g_bsp_cfg_factory_backup → RAM + FRAM。 */
Bsp_Cfg_Status_T  Bsp_Config_RestoreFromFactory(void);

/* 自身 CAN ID（标准帧 11bit，0~0x7FF）*/
uint16_t          Bsp_Config_GetSelfId(void);
Bsp_Cfg_Status_T  Bsp_Config_SetSelfId(uint16_t id);

/* 绑定主站 CAN ID（标准帧 11bit，0~0x7FF）*/
uint16_t          Bsp_Config_GetMasterId(void);
Bsp_Cfg_Status_T  Bsp_Config_SetMasterId(uint16_t id);

/* CAN 波特率档位（设置后调 Save 并重启生效）*/
Bsp_Cfg_Baud_T    Bsp_Config_GetBaud(void);
Bsp_Cfg_Status_T  Bsp_Config_SetBaud(Bsp_Cfg_Baud_T baud);

/* 波特率档位对应的 prescaler（供 bsp_can.c Init 查表使用）*/
uint16_t          Bsp_Config_BaudToPrescaler(Bsp_Cfg_Baud_T baud);

/* ---- PWM 泄放参数（持久化于 FRAM；越界自动回落默认宏值）----
 * 由上电流程读出后传入 BSP 初始化（如 Bsp_Discharge_SetFreq(Bsp_Config_GetPwmFreq())），
 * 实现“重启读取配置→初始化各模块；非法值用宏默认”。Set 后调 Save() 落 FRAM。 */
uint16_t          Bsp_Config_GetPwmFreq(void);            /* PWM 频率 Hz */
Bsp_Cfg_Status_T  Bsp_Config_SetPwmFreq(uint16_t hz);     /* 越界返回 ERR_PARAM */
uint8_t           Bsp_Config_GetPwmDutyMax(void);         /* 占空比上限 %（5~90，硬顶 90） */
Bsp_Cfg_Status_T  Bsp_Config_SetPwmDutyMax(uint8_t pct);
uint8_t           Bsp_Config_GetPwmDutyStart(void);       /* 默认启动占空比 %（4~duty_max） */
Bsp_Cfg_Status_T  Bsp_Config_SetPwmDutyStart(uint8_t pct);
uint8_t           Bsp_Config_GetPwmDutyStep(void);        /* 斜坡步进 %/步 */
Bsp_Cfg_Status_T  Bsp_Config_SetPwmDutyStep(uint8_t pct);

/* ---- 可标定保护阈值（持久化于 FRAM；单位 0.1V / 0.1℃ / 0.01A）----
 * 由 App 保护状态机（PROT/DCH）按 Get* 取用，CAN 0x04/0x05/0x07/0x14/0x15/0x17 读写。
 * Set 后须调 Save() 落 FRAM；越界返回 ERR_PARAM。迟滞关系（off < on）由 Set 校验。 */
uint16_t          Bsp_Config_GetOvpOn(void);              /* 泄放启动 0.1V */
Bsp_Cfg_Status_T  Bsp_Config_SetOvpOn(uint16_t v0v1);
uint16_t          Bsp_Config_GetOvpOff(void);             /* 泄放停止/欠压 0.1V */
Bsp_Cfg_Status_T  Bsp_Config_SetOvpOff(uint16_t v0v1);
uint16_t          Bsp_Config_GetOvpL1(void);              /* 一级动作 0.1V */
uint16_t          Bsp_Config_GetOvpL1Off(void);           /* 一级恢复 0.1V */
Bsp_Cfg_Status_T  Bsp_Config_SetOvpL1(uint16_t v0v1);     /* 写动作值，恢复值按固定迟滞派生 */
uint16_t          Bsp_Config_GetOvpL2(void);              /* 二级动作 0.1V */
uint16_t          Bsp_Config_GetOvpL2Off(void);           /* 二级恢复 0.1V */
Bsp_Cfg_Status_T  Bsp_Config_SetOvpL2(uint16_t v0v1);     /* 写动作值，恢复值按固定迟滞派生 */
uint16_t          Bsp_Config_GetOtpOff(void);             /* 过温关断 0.1℃ */
Bsp_Cfg_Status_T  Bsp_Config_SetOtpOff(uint16_t t0c1);
uint16_t          Bsp_Config_GetOtpRecover(void);         /* 过温恢复 0.1℃ */
uint16_t          Bsp_Config_GetOvc(void);                /* 过流 0.01A */
Bsp_Cfg_Status_T  Bsp_Config_SetOvc(uint16_t i0a01);

#endif /* BSP_USE_CONFIG */

#ifdef __cplusplus
}
#endif

#endif /* BSP_H */
