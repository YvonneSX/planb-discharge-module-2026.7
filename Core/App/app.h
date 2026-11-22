/**
 * @file    app.h
 * @brief   App 业务层总伞头 + 顶层编排接口（HLD-MOD-SCHED 协作面）
 *
 * 作用：
 *   1) 集中开启本项目 App 层所需的 BSP 模块宏，并 #include "bsp.h"，
 *      使各 App 源文件只需 `#include "app.h"` 即可拿到 BSP 接口与 App 类型；
 *   2) 暴露顶层编排接口 App_Init() / App_Run()，供 main.c（调度层）调用。
 *
 * 包含顺序约定（重要）：
 *   App 模块的 .c/.h 必须把 "app.h" 作为**第一个项目头文件**包含，
 *   以保证 BSP_USE_* 宏在 bsp.h 之前生效（bsp.h 用 include guard，
 *   若先被裸包含会导致这些宏失效）。
 *
 * @version 0.1.0
 * @date    2026-06-16
 */

#ifndef APP_H
#define APP_H

/* ---- 集中开启 App 层依赖的 BSP 模块（须在 bsp.h 之前定义）---- */
#define BSP_USE_UART_LOG
#define BSP_USE_ANALOG
#define BSP_USE_DISCHARGE
#define BSP_USE_CAN
#define BSP_USE_SPI
#define BSP_USE_FRAM
#define BSP_USE_WDG
#define BSP_USE_CONFIG

#include "bsp.h"
#include "systick.h"
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 * 顶层编排（实现见 app.c，由 main.c 调用）
 * ==================================================================== */

/**
 * @brief 按依赖序初始化全部 App 与其依赖的 BSP 模块。
 *
 * 次序（HLD §5.1）：CFG(Load) → ANALOG → DISCHARGE(默认关断) → WDG →
 * SPI/FRAM → BBX(异常掉电/复位判定) → MON → PROT → DCH → CAN/COMM →
 * 最后发送上电完成通告帧 0xA5。初始化完成前泄放保持关断（REQ-SAFE-001）。
 */
void App_Init(void);

/**
 * @brief 主循环单圈任务序（非阻塞）。由 main 的 while(1) 每圈调用一次。
 *
 * 顺序：MON 刷新快照 → PROT 慢路径仲裁 → DCH 斜坡/计时 →
 * COMM 处理命令并应答（事件驱动）→ BBX 记录（事件/限速）→ SAFE 喂狗。
 * 一切定时用 systick 时间戳增量内联判断，不设固定周期门。
 */
void App_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
