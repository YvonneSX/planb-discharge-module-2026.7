/**
 * @file    app_features.h
 * @brief   项目级功能编译开关（无依赖头，供 app.c / gd32f30x_it.c / 各模块共用）
 *
 * 说明：本头**不得** #include 任何重量级头（尤其 bsp.h），以便中断文件 gd32f30x_it.c
 *       也能直接包含，保持一个统一的编译开关来源，避免多处定义不一致。
 */

#ifndef APP_FEATURES_H
#define APP_FEATURES_H

/* —— CAN 应用 + 中断 总开关 ——
 *   0 = 屏蔽：不初始化 CAN 控制器/COMM、不发上电通告、主循环不分派命令、不注册 RX 中断处理。
 *   1 = 启用：CAN 从站通信全功能。
 * 当前项目暂未用到 CAN，默认关闭（用户定案）。需要时置 1 即可整体恢复。 */
#ifndef APP_CAN_ENABLE
#define APP_CAN_ENABLE   0
#endif

#endif /* APP_FEATURES_H */
