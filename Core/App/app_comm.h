/**
 * @file    app_comm.h
 * @brief   HLD-MOD-COMM 通信模块（★CAN 从站，事件驱动）
 *
 * 模型：除上电通告帧（0xA5，一次）外，从站全程被动应答（请求-应答）。
 *   - 接收：CAN0 RX FIFO0 中断 → OnRx 入缓存 + 置标志（ISR 不做业务）
 *   - 处理：主循环 Dispatch 检测标志 → switch-case 分派 → 组应答帧 TX
 *
 * 数据边界：运行状态应答来自 App_RtData_T（MON/PROT/DCH 的运行时数据）。
 *
 * 关键对外接口（跨模块）：
 *   - AppComm_OnRx()    ← 由 Bsp_Can RX 中断回调（入缓存+置标志）
 *   - AppComm_Dispatch()← 主循环驱动命令处理与应答
 *   命令处理结果下发 PROT/DCH/CFG；数据源来自 MON/PROT/DCH。
 *
 * 支撑需求：REQ-FUNC-040/042、REQ-IF-001/002/003。
 */

#ifndef APP_COMM_H
#define APP_COMM_H

#include "app.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化：注册 RX 中断回调并使能 CAN RX FIFO0 中断。 */
void AppComm_Init(void);

/**
 * @brief CAN RX 中断回调（ISR 上下文）：仅把帧搬入缓存并置“收到命令”标志，
 *        不做任何业务处理或 CAN 发送。签名匹配 Bsp_Can_RxCallback_T。
 */
void AppComm_OnRx(const Bsp_Can_Frame_T *frame);

/** @brief 主循环每圈调用：若有待处理帧则解析、分派、组应答并 TX。 */
void AppComm_Dispatch(void);

/** @brief 上电完成通告帧（A5 5A 5A FF FF FF FF FF），无条件发送一次。 */
void AppComm_SendBootAnnounce(void);

/**
 * @brief 按读状态命令把运行快照打包为应答帧负载（§4.3.7）。
 * @param cmd  命令码（0x20~0x24）
 * @param snap 运行数据快照（仅 RAM）
 * @param resp 输出应答帧（id/dlc/data 由本函数填写）
 * @return true 已打包，false 命令不支持
 */
bool AppComm_PackSnapshot(uint8_t cmd, const App_RtData_T *snap, Bsp_Can_Frame_T *resp);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMM_H */
