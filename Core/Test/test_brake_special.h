/**
 * @file    test_brake_special.h
 * @brief   泄放从站 专项极限测试 — 串口遥测监视入口
 *
 * 与 BSP 单元测试不同：本模式**跑真实 App**（App_Init/App_Run 完整保护
 * 状态机），测试代码只挂一个**只读串口遥测钩子**周期打印运行量与故障位，
 * 并在故障位/状态/占空比边沿变化时打印事件标记行（[ALARM]/[CLEAR]/…）。
 *
 * 用途：负载仪器（可编程电源/电子负载/功率台架）逐保护功能极限测试，
 * 台架加压/加热/加载时直接从串口观察**真实固件**的保护反应；EC+CAN 前期
 * 仅作数据上报/转发，报警与实时状态先走串口。
 *
 * 编译/烧录：make brake-special（见 Makefile）。
 *
 * @version 1.0.0
 * @date    2026-07-03
 */

#ifndef TEST_BRAKE_SPECIAL_H
#define TEST_BRAKE_SPECIAL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 专项极限测试串口遥测监视循环（不返回）。
 *
 * 内部：App_Init() 后进入 for(;;){ App_Run(); 周期遥测; 事件边沿检测 }。
 * 只调用 App / AppMon / AppProt / AppDch 的 Get* 只读接口，不改任何保护状态，
 * 保证观测的是纯真实固件行为。由 Test_Runner_RunMonitor() 在
 * -DMONITOR_BRAKE_SPECIAL 下分发调用。
 */
void Test_BrakeSpecial_Monitor(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BRAKE_SPECIAL_H */
