/**
 * @file    canx_proto_port.c
 * @brief   canx_proto 引擎平台适配（泄放模块 GD32F303 经典 CAN 2.0B）
 *
 * 把引擎的中性帧 CanxFrame_T 与本平台 BSP 帧 Bsp_Can_Frame_T 互转，并把
 * 泄放 BSP 的「中断回调式接收」桥接成引擎期望的「recv 出队」语义：
 *   - 中断回调 CanxPort_OnRxIsr 把帧压入软件环形缓冲；
 *   - 引擎经 Canx_Port_Recv 从环形缓冲出队。
 * 引擎本体(canx_proto.c)因此完全不感知底层是中断还是轮询。
 *
 * 移植到别的平台：改本文件的帧互转与收发函数即可，canx_proto.c/.h 不动。
 *
 * @version 0.1.0
 * @date    2026-06-29
 */

#include "app.h"            /* 集中开启 BSP_USE_CAN 等宏后再引入 bsp.h（Bsp_Can_*） */
#include "canx_proto_port.h"

/* ====================================================================
 * 软件接收环形缓冲：中断入队 / recv 出队
 * ==================================================================== */
static volatile CanxFrame_T s_rxq[CANX_RXQ_DEPTH];
static volatile uint8_t     s_head;
static volatile uint8_t     s_tail;
static volatile uint32_t    s_drop;     /* 缓冲满丢帧计数 → 供 get_drop_count */

/**
 * 功能：经典 CAN BSP 帧 → 中性帧（fdf/brs/rtr 恒 0，dlc→len，拷数据）。
 * 输入参数：s —— BSP 帧；d —— 中性帧输出。
 * 输出参数：*d。返回值：无。
 */
static void classic_to_canx(const Bsp_Can_Frame_T *s, CanxFrame_T *d)
{
    uint8_t i;
    uint8_t n = (s->dlc > CANX_MAX_DATA) ? (uint8_t)CANX_MAX_DATA : s->dlc;

    d->id  = s->id;
    d->ide = s->ide;
    d->rtr = 0U;
    d->fdf = 0U;
    d->brs = 0U;
    d->len = n;
    for (i = 0U; i < n; i++) {
        d->data[i] = s->data[i];
    }
}

/**
 * 功能：中性帧 → 经典 CAN BSP 帧（len→dlc 钳 8，拷数据；fdf/brs 丢弃）。
 * 输入参数：s —— 中性帧；d —— BSP 帧输出。
 * 输出参数：*d。返回值：无。
 */
static void canx_to_classic(const CanxFrame_T *s, Bsp_Can_Frame_T *d)
{
    uint8_t i;
    uint8_t n = (s->len > 8U) ? 8U : s->len;   /* 经典 CAN 上限 8 字节 */

    d->id  = s->id;
    d->ide = s->ide;
    d->dlc = n;
    for (i = 0U; i < n; i++) {
        d->data[i] = s->data[i];
    }
}

/* ====================================================================
 * RX 中断回调（内部）：ISR 上下文仅搬运入队，签名匹配 Bsp_Can_RxCallback_T
 * ==================================================================== */
/**
 * 功能：CAN 接收中断回调——把收到的帧转中性帧压入环形缓冲（满则丢并计数）。
 * 输入参数：frame —— BSP 接收帧（只读，调用后失效）。
 * 输出参数：无（写入环形缓冲）。返回值：无。
 */
static void CanxPort_OnRxIsr(const Bsp_Can_Frame_T *frame)
{
    uint8_t next;

    if (frame == NULL) {
        return;
    }
    next = (uint8_t)((s_tail + 1U) % CANX_RXQ_DEPTH);
    if (next == s_head) {
        s_drop++;               /* 缓冲满，丢弃并计 drop（引擎据此置 RX_OVERFLOW） */
        return;
    }
    classic_to_canx(frame, (CanxFrame_T *)&s_rxq[s_tail]);
    s_tail = next;
}

/**
 * 功能：初始化接收桥接——复位环形缓冲并注册 CAN RX 中断回调。
 * 输入/输出参数：无。返回值：无。
 * 说明：须在 Bsp_Can_Init() 之后调用。
 */
void CanxPort_Init(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_drop = 0U;
    (void)Bsp_Can_EnableRxIt(CanxPort_OnRxIsr);
}

/* ====================================================================
 * BspOps 适配函数（绑定到 CanxProto_DefaultBsp）
 * ==================================================================== */
/**
 * 功能：发送一帧——中性帧 → 经典 CAN BSP 帧 → Bsp_Can_Send。
 * 输入参数：ch —— 通道（本工程恒 0，忽略）；frame —— 中性帧。
 * 输出参数：无。返回值：CANX_BUS_OK 成功，CANX_BUS_ERR 失败。
 */
static CanxBusStatus_E Canx_Port_Send(uint8_t ch, const CanxFrame_T *frame)
{
    Bsp_Can_Frame_T f;

    (void)ch;
    if (frame == NULL) {
        return CANX_BUS_ERR;
    }
    canx_to_classic(frame, &f);
    return (Bsp_Can_Send(&f) == BSP_CAN_OK) ? CANX_BUS_OK : CANX_BUS_ERR;
}

/**
 * 功能：接收一帧——从软件环形缓冲出队（空返回 EMPTY）。
 * 输入参数：ch —— 通道（忽略）；frame —— 中性帧输出。
 * 输出参数：*frame。返回值：CANX_BUS_OK 取到一帧，CANX_BUS_ERR_EMPTY 缓冲空。
 */
static CanxBusStatus_E Canx_Port_Recv(uint8_t ch, CanxFrame_T *frame)
{
    (void)ch;
    if (frame == NULL) {
        return CANX_BUS_ERR;
    }
    if (s_head == s_tail) {
        return CANX_BUS_ERR_EMPTY;
    }
    *frame = (CanxFrame_T)s_rxq[s_head];
    s_head = (uint8_t)((s_head + 1U) % CANX_RXQ_DEPTH);
    return CANX_BUS_OK;
}

/**
 * 功能：收前整理——中断已自动入队，本平台无需额外动作。
 * 输入参数：ch（忽略）。输出/返回值：无。
 */
static void Canx_Port_ServiceRx(uint8_t ch)
{
    (void)ch;   /* 中断驱动，环形缓冲已实时填充 */
}

/**
 * 功能：返回接收缓冲溢出丢帧计数（供引擎丢包检测）。
 * 输入参数：ch（忽略）。输出参数：无。返回值：累计丢帧数。
 */
static uint32_t Canx_Port_GetDrop(uint8_t ch)
{
    (void)ch;
    return s_drop;
}

/**
 * 功能：提供本平台默认 BspOps 绑定（引擎 Init 传 NULL 时用）。
 * 输入参数：无。输出参数：无。返回值：绑定好四个收发函数的 CanxBspOps_T。
 */
CanxBspOps_T CanxProto_DefaultBsp(void)
{
    CanxBspOps_T ops;
    ops.send           = Canx_Port_Send;
    ops.recv           = Canx_Port_Recv;
    ops.service_rx     = Canx_Port_ServiceRx;
    ops.get_drop_count = Canx_Port_GetDrop;
    return ops;
}

/* ====================================================================
 * 测试钩子（仅 BSP_UNIT_TEST 编译期生效，不进生产固件）
 * ==================================================================== */
#if defined(BSP_UNIT_TEST)
/**
 * 功能：复位接收环形缓冲与丢帧计数（不注册中断），供单测构造干净初态。
 */
void CanxPort_TestReset(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_drop = 0U;
}

/**
 * 功能：以原始字段构造一帧 BSP 帧并经内部 ISR 路径压入环形缓冲，
 *       替代真实 CAN 中断，供单测注入接收帧。
 * 输入参数：id/ide/dlc/data —— 帧字段（data 可空，dlc 限 8）。
 */
void CanxPort_TestInjectRx(uint32_t id, uint8_t ide, uint8_t dlc, const uint8_t *data)
{
    Bsp_Can_Frame_T f;
    uint8_t i;
    uint8_t n = (dlc > 8U) ? 8U : dlc;

    f.id  = id;
    f.ide = ide;
    f.dlc = n;
    for (i = 0U; i < 8U; i++) {
        f.data[i] = (data != NULL && i < n) ? data[i] : 0U;
    }
    CanxPort_OnRxIsr(&f);
}
#endif /* BSP_UNIT_TEST */
