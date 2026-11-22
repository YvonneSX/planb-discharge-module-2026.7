/**
 * @file    app.c
 * @brief   App 业务层顶层编排（HLD §5.1 初始化序 / §2.2 主循环任务序）
 *
 * App_Init：按依赖序初始化 BSP 外设与各 App 模块，全程保持泄放关断，
 *           完成后发送上电通告帧。
 * App_Run ：主循环单圈非阻塞任务序，定时一律用 systick 时间戳增量。
 */

#include "app.h"
#include "app_features.h"  /* APP_CAN_ENABLE 编译开关 */
#include "app_monitor.h"
#include "app_protect.h"
#include "app_discharge.h"
#include "app_safety.h"
#include "app_blackbox.h"
#include "app_comm.h"
#include "bsp_timer.h"     /* Bsp_StepTimer_Init：分级泄放 100us 节拍 */
#include "uart_log.h"

#define APP_TAG "APP"

/**
 * 功能：App 业务层初始化编排，按依赖序初始化 BSP 外设与各 App 模块（全程保持泄放关断），完成后发送上电通告帧。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void App_Init(void)
{
    /* 1. 总线/存储：SPI → FRAM → 配置（配置存于 FRAM，须先于读取就绪） */
    UART_LOGI(APP_TAG, "init: spi+fram+config...");
    (void)Bsp_Spi_Init();
    (void)Bsp_Fram_Init();
    (void)Bsp_Config_Load();

    /* 2. 采样与执行：ANALOG（DMA 后台连续转换）→ DISCHARGE（上电默认关断） */
    UART_LOGI(APP_TAG, "init: analog+discharge...");
    (void)Bsp_Analog_Init();
    (void)Bsp_Discharge_Init();
    Bsp_Discharge_Stop();

    /* 3. 看门狗 GPIO 由 Bsp_Init() 已初始化；此处确保安全态模块就绪 */
    AppSafe_Init();

    /* 4. 黑匣子：复位原因/异常掉电判定（须 FRAM 就绪） */
    (void)AppBbx_Init();

    /* 5. 业务模块：MON（数据源）→ PROT（仲裁）→ DCH（执行） */
    UART_LOGI(APP_TAG, "init: mon+prot+dch...");
    AppMon_Init();
    AppProt_Init();
    AppDch_Init();

    /* 5b. 分级泄放执行——看门狗(起/停) + TIMER1(爬坡/切级) 解耦并存（见《过泄压中断动态启停…》）：
     *
     *   ① ADC 模拟看门狗：过压起泄放 / 过泄压停泄放的 µs 级快速响应通道（安全关键）。
     *      上电为待机态窗口 [0, 一级触发码56.5V]——只盯上限，母线冲过 56.5V 即中断起泄放；
     *      48V 待机低于 55V 也不空触发（下限=0）。起泄放后由 AppDch_OvpIsr 动态切到泄放态窗口
     *      [停止码55V, 满量程] 只盯下限，泄到 55V 以下即中断停泄放。须在 PROT/DCH 就绪后使能。 */
    Bsp_Analog_SetVBusWindow(0U, Bsp_Analog_VBusMvToCode(BSP_DCH_L1_ON_MV));
    Bsp_Analog_EnableVBusWatchdog(AppDch_OvpIsr);

    /*   ② TIMER1 100us 节拍：仅在泄放开启后(level≠0)驱动 AppDch_StepIsr 做 L1 每100µs+2% 爬坡、
     *      L1↔L2 级别切换；不做起/停。与看门狗完全解耦、互不触发。须在 DCH 就绪后启动。 */
    UART_LOGI(APP_TAG, "init: step-timer...");
    Bsp_StepTimer_Init(BSP_DISCHARGE_L1_STEP_PERIOD_US, AppDch_StepIsr);

#if APP_CAN_ENABLE
    /* 6. 通信：CAN 控制器（取配置波特率/ID）→ COMM（注册 RX 中断回调） */
    UART_LOGI(APP_TAG, "init: can+comm...");
    (void)Bsp_Can_Init();
    AppComm_Init();

    /* 7. 上电完成通告（无条件一次） */
    AppComm_SendBootAnnounce();
#else
    /* CAN 未启用（APP_CAN_ENABLE=0）：跳过 CAN/COMM 初始化与上电通告，不注册 RX 中断。 */
    UART_LOGI(APP_TAG, "can disabled (APP_CAN_ENABLE=0)");
#endif
    UART_LOGI(APP_TAG, "App_Init done");
}

/**
 * 功能：主循环单圈非阻塞任务序——量测刷新→保护仲裁/自检→下发 DCH 执行→通信分发→黑匣子记录→安全态判定→喂狗。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void App_Run(void)
{
    /* 注：泄放起/停快路径由 ADC 看门狗中断(AppDch_OvpIsr, µs 级)执行，L1 爬坡/切级由 TIMER1
     * 中断(AppDch_StepIsr, 100µs)执行；均不在本慢路径主循环中。本循环只做量测/仲裁/记录/喂狗。 */

    /* 0. 空闲态 A9(Brake_FB) 实时守护：A9 与泄放引脚 PA8 输出状态一一对应；空闲态(未泄放)若
     *    A9=高＝PA8 被异常拉高，等效 100% 占空比误泄放（危险）→ 立即把 PA8 切普通 GPIO 强制拉低。
     *    放在主循环最前，每圈实时监测（TIMER1 StepIsr 亦有 100µs 快路径等效守护）。 */
    AppDch_IdleFbGuard();

    /* 1. 量测刷新（数据源头） */
    AppMon_Update();

    /* 2. 保护判据/告警 + 回路自检（不驱动泄放；泄放执行在看门狗/TIMER1 中断中） */
    AppProt_Update();
    AppProt_LoopSelfCheck();

    /* 3. 泄放会话监控（累计时长、20s 兜底）。起/停/爬坡由 OvpIsr/StepIsr 中断执行，非此处 */
    AppDch_Update();

    /* 4. 通信：处理已收命令并应答（CAN 未启用时整体跳过） */
#if APP_CAN_ENABLE
    AppComm_Dispatch();
#endif

    /* 5. 黑匣子：拉取 PROT 事件并记录（消费者，PROT 不反向依赖） */
    App_EvtType_T evt;
    while (AppProt_PollEvent(&evt)) {
        App_RtData_T snap;
        AppMon_GetSnapshot(&snap);
        snap.fault_bits = AppProt_GetFaults();
        snap.dch_state  = AppDch_GetLevel();   /* 泄放级别 L0/L1/L2 */
        (void)AppBbx_LogEvent(evt, &snap);
    }

    /* 6. 关断类锁存 → 进入安全态（独立于 DCH，强制关断兜底）。
     *    过温为温度自恢复类（≥95℃停泄放、≤65℃自动解除），不锁存不进安全态，故不含 OVER_TEMP。
     *    进安全态时 AppSafe_EnterSafeState 已关 ADC 看门狗，杜绝过压中断(AppDch_OvpIsr)重新起泄放；
     *    TIMER1 的 StepIsr 也查 AppSafe_InSafeState 门控。 */
    if ((AppProt_GetFaults() & (APP_FAULT_LOOP_FB | APP_FAULT_DCH_TIMEOUT)) != 0U) {
        AppSafe_EnterSafeState();
    }

    /* 7. 喂狗（本圈关键任务已执行后） */
    AppSafe_Kick();

    /* 8. PB0 指示灯（非阻塞）：先跑上电版本识别闪灯（首次上电闪 2 下），闪完让位；
     *    再跑泄放指示灯（泄放中亮 / 停后至少亮 1s / 持续<55V 达 2s 兜底）。 */
    AppDch_ServicePowerOnBlink();
    AppDch_ServiceDchLed();
}
