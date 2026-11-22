/**
 * @file    canx_proto.h
 * @brief   通用 CAN 协议引擎（CAN2.0 / CAN-FD 共用，平台中立，主从一体）
 *
 * 由 PlanbPro 已验证的 can_proto 引擎通用化而来：把帧类型中性化、与具体 BSP
 * 解耦，做到"拷贝 canx_proto.c/.h 一行不改 + 写一个 port + 写协议回调"即可用。
 *
 *   ① 1ms 轮询状态机           —— 发→等应答→超时重发→ACK/TIMEOUT (CanxProto_PollStep)
 *   ② commit-on-ack 在途确认    —— 配置帧应答后才提交，丢帧自动重发
 *   ③ 多从站注册 + 集中派发     —— 同通道多节点，按帧内 ID 字节派发到对应引擎
 *   ④ 握手阶段(可选) + 实时阶段 —— CycleTask 骨架，协议只填回调
 *   ⑤ 超时/丢包/总线错误 → 故障位派生
 *   ⑥ 主/从角色                —— 主站主动轮询；从站被动应答(role + on_request)
 *
 * 平台无关性：本头与 canx_proto.c 不依赖任何具体 BSP；引擎只用中性帧 CanxFrame_T。
 * 底层收发由 canx_proto_port.c 提供的 CanxBspOps_T（含 CanxProto_DefaultBsp）注入。
 *
 * @version 0.1.0
 * @date    2026-06-29
 */

#ifndef CANX_PROTO_H
#define CANX_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------
 * 编译期配置（移植时可经 -D 或本处覆盖）
 * ------------------------------------------------------------------- */

/** 中性帧 data[] 字节数：经典 CAN=8；CAN-FD 平台 -DCANX_MAX_DATA=64。 */
#ifndef CANX_MAX_DATA
#define CANX_MAX_DATA               8U
#endif

/** 本平台 CAN 通道总数（注册表维度）。 */
#ifndef CANX_CH_NUM
#define CANX_CH_NUM                 1U
#endif

/** 单通道最多注册的节点(引擎)数。 */
#ifndef CANX_MAX_NODES_PER_CH
#define CANX_MAX_NODES_PER_CH       8U
#endif

/** 单次集中收帧上限，防止占用过长。 */
#ifndef CANX_SVC_RX_MAX_PER_CALL
#define CANX_SVC_RX_MAX_PER_CALL    16U
#endif

/** 编译期默认角色。不定义则主站(零回归)；从站用 CanxProto_InitSlave 覆盖。 */
#ifndef CANX_ROLE_DEFAULT
#define CANX_ROLE_DEFAULT           CANX_ROLE_MASTER
#endif

/* -------------------------------------------------------------------
 * 中性帧：平台无关，是 CAN2.0 与 CAN-FD 的字段超集
 * ------------------------------------------------------------------- */

/**
 * @brief 平台中立 CAN 帧。port 层负责与具体 BSP 帧互转。
 *   经典 CAN：fdf/brs 恒 0，len≤8；CAN-FD：len≤64，按 brs 加速。
 *   len 统一长度字段，消除 dlc(经典)/data_bytes(FD) 命名分歧。
 */
typedef struct {
    uint32_t id;                    /**< 11/29 位标识符（视 ide） */
    uint8_t  ide;                   /**< 0=标准 1=扩展 */
    uint8_t  rtr;                   /**< 经典远程帧；FD 恒 0 */
    uint8_t  fdf;                   /**< 1=CAN-FD 0=经典；经典平台恒 0 */
    uint8_t  brs;                   /**< FD 数据段加速；经典平台恒 0 */
    uint8_t  len;                   /**< 有效数据字节数（经典≤8 / FD≤64） */
    uint8_t  data[CANX_MAX_DATA];   /**< 负载 */
} CanxFrame_T;

/* -------------------------------------------------------------------
 * 状态码 / 枚举
 * ------------------------------------------------------------------- */

/** API 返回码。 */
typedef enum {
    CANX_OK            =  0,
    CANX_ERR_PARAM     = -1,
    CANX_ERR_BUS       = -3,
    CANX_ERR_FRAME     = -4,
    CANX_ERR_FULL      = -10        /* 注册表满 */
} CanxStatus_E;

/** 收发底层状态码（port 适配函数把本平台 BSP 状态翻译成这三个）。 */
typedef enum {
    CANX_BUS_OK        = 0,
    CANX_BUS_ERR_EMPTY = 1,         /* RX 无帧 */
    CANX_BUS_ERR       = 2          /* 总线/其它错误 */
} CanxBusStatus_E;

/** 通用故障位（协议可在高位自行扩展）。 */
typedef enum {
    CANX_FAULT_NONE          = 0x00000000U,
    CANX_FAULT_COMM_TIMEOUT  = 0x00000001U, /* RX 静默超时 */
    CANX_FAULT_RX_OVERFLOW   = 0x00000004U, /* BSP/缓冲丢帧 */
    CANX_FAULT_INVALID_FRAME = 0x00000008U, /* 帧解析失败 */
    CANX_FAULT_BUS_ERROR     = 0x00000010U  /* 总线错误 */
} CanxFault_E;

/** 链路/节点高层状态。 */
typedef enum {
    CANX_STATE_UNINIT    = 0,
    CANX_STATE_READY     = 1,
    CANX_STATE_HANDSHAKE = 2,
    CANX_STATE_OPERABLE  = 3,
    CANX_STATE_FAULT     = 4
} CanxState_E;

/** 轮询阶段。 */
typedef enum {
    CANX_PHASE_HANDSHAKE = 0,
    CANX_PHASE_REALTIME  = 1
} CanxPhase_E;

/** 节点角色：主站主动轮询；从站被动应答。 */
typedef enum {
    CANX_ROLE_MASTER = 0,
    CANX_ROLE_SLAVE  = 1
} CanxRole_E;

/** CanxProto_PollStep 结果。 */
typedef enum {
    CANX_ACT_NONE    = 0,
    CANX_ACT_ACK     = 1,
    CANX_ACT_TIMEOUT = 2
} CanxAct_E;

/* -------------------------------------------------------------------
 * 传输 Profile：CAN2.0 vs CAN-FD 的差异（配置化）
 * ------------------------------------------------------------------- */

/** 收帧派发模式：决定 ServiceChannel 如何把帧路由到引擎实例。 */
typedef enum {
    CANX_DISPATCH_DATA_BYTE = 0,  /* 按帧内 data[rx_id_offset] 字节匹配 node_id（多从站，数据字节寻址） */
    CANX_DISPATCH_CAN_ID    = 1,  /* 按 frame.id 匹配 node_id（多从站，CAN-ID 寻址） */
    CANX_DISPATCH_SINGLE    = 2   /* 单节点：本通道所有帧都派给唯一引擎，过滤交 decode_rx */
} CanxDispatch_E;

typedef struct {
    uint8_t frame_bytes;  /**< 发送帧数据字节数(经典≤8；FD 8/12/.../64) */
    uint8_t fdf;          /**< 1=CAN-FD，0=经典 CAN 2.0 */
    uint8_t brs;          /**< 数据段加速(仅 FD)，0/1 */
    uint8_t tx_ide;       /**< 发送默认帧类型：0=标准 1=扩展 */
    uint8_t rx_id_offset; /**< 帧内"节点ID"字节偏移（DATA_BYTE 模式用） */
    uint8_t dispatch;     /**< CanxDispatch_E：收帧派发模式（默认 DATA_BYTE） */
} CanxXferProfile_T;

/** 收发底层绑定（函数指针）。由 CanxProto_DefaultBsp()(port.c) 提供默认实现。 */
typedef struct {
    CanxBusStatus_E (*send)(uint8_t ch, const CanxFrame_T *frame);
    CanxBusStatus_E (*recv)(uint8_t ch, CanxFrame_T *frame);
    void            (*service_rx)(uint8_t ch);
    uint32_t        (*get_drop_count)(uint8_t ch);
} CanxBspOps_T;

/* -------------------------------------------------------------------
 * 引擎句柄 + 协议回调
 * ------------------------------------------------------------------- */

struct CanxEngine_S;
typedef struct CanxEngine_S CanxEngine_T;

/** 协议回调表（每种协议实现一份；可作 const 静态表）。 */
typedef struct {
    /** 解析一帧（派发命中本节点后调）。1=本节点有效帧；0=忽略；<0=失败。 */
    int  (*decode_rx)(CanxEngine_T *engine, const CanxFrame_T *frame, uint32_t now_ms);
    /** 握手阶段发一帧(心跳/探测)。has_handshake=0 时可 NULL。 */
    void (*handshake_send)(CanxEngine_T *engine, uint32_t now_ms);
    /** 实时阶段推进一步：协议自管子命令 + commit-on-ack，内部用 CanxProto_PollStep。 */
    void (*realtime_step)(CanxEngine_T *engine, uint32_t now_ms, uint8_t rx_got);
    /** 可选：刚进 OPERABLE 时回调。可 NULL。 */
    void (*on_operable)(CanxEngine_T *engine, uint32_t now_ms);
    /** 从站(被动)应答：decode_rx 返回 >0 后调。>0 已答；0 不答；<0 失败。主站不用。 */
    int  (*on_request)(CanxEngine_T *engine, const CanxFrame_T *req, uint32_t now_ms);
    uint8_t has_handshake;     /**< 1=先走握手；0=Init 后直进 REALTIME */
    uint8_t handshake_need_ok; /**< 握手连续应答多少次算成功 */
} CanxOps_T;

struct CanxEngine_S {
    /* ---- 静态配置 ---- */
    CanxXferProfile_T xfer;
    CanxBspOps_T      bsp;
    const CanxOps_T  *ops;
    void             *proto_ctx;
    uint8_t           channel;
    uint16_t          node_id;
    uint16_t          peer_id;
    CanxRole_E        role;
    uint32_t          comm_lost_ms;
    uint32_t          ready_wait_ms;

    /* ---- 运行状态 ---- */
    CanxState_E       state;
    CanxPhase_E       phase;
    uint32_t          fault_bits;
    uint32_t          last_rx_ms;
    uint32_t          last_bsp_drop;
    uint32_t          ready_wait_start_ms;

    /* ---- poll 状态机 ---- */
    uint8_t           tx_enable;
    uint8_t           rx_enable;
    uint8_t           retry_cnt;
    uint8_t           retry_max;
    uint8_t           ok_cnt;
    uint32_t          interval_ms;
    uint32_t          next_tx_ms;
    uint8_t           fb_fresh;

    /* ---- commit-on-ack 在途 token ---- */
    uint8_t           inflight_field;
    int32_t           inflight_value;   /* 中性化：整型即可承载阈值类配置 */

    /* ---- 统计 ---- */
    uint32_t          tx_count;
    uint32_t          rx_count;
    uint32_t          timeout_count;
    uint32_t          drop_count;
};

/** 子命令发送函数原型。 */
typedef void (*CanxSendFn_T)(CanxEngine_T *engine, uint32_t now_ms);

/* -------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------- */

/** 默认 BSP 绑定（在 canx_proto_port.c 实现，绑定本平台收发函数）。 */
CanxBspOps_T CanxProto_DefaultBsp(void);

CanxXferProfile_T CanxProto_ProfileCanFd16(void);
CanxXferProfile_T CanxProto_ProfileClassic8(void);

CanxStatus_E CanxProto_Init(CanxEngine_T            *engine,
                            uint8_t                  channel,
                            uint16_t                 node_id,
                            uint16_t                 peer_id,
                            const CanxXferProfile_T *xfer,
                            const CanxBspOps_T      *bsp,
                            const CanxOps_T         *ops,
                            void                    *proto_ctx);

/** 以「从站(被动)」角色初始化：role=SLAVE、上电即 OPERABLE、tx/rx_enable=1。 */
CanxStatus_E CanxProto_InitSlave(CanxEngine_T            *engine,
                                 uint8_t                  channel,
                                 uint16_t                 node_id,
                                 uint16_t                 peer_id,
                                 const CanxXferProfile_T *xfer,
                                 const CanxBspOps_T      *bsp,
                                 const CanxOps_T         *ops,
                                 void                    *proto_ctx);

CanxStatus_E CanxProto_Register(CanxEngine_T *engine);
void         CanxProto_ResetRegistry(void);

void         CanxProto_ServiceChannel(uint8_t channel, uint32_t now_ms);
void         CanxProto_Tick(CanxEngine_T *engine, uint32_t now_ms);
CanxStatus_E CanxProto_CycleTask(CanxEngine_T *engine, uint32_t now_ms);

CanxAct_E    CanxProto_PollStep(CanxEngine_T *engine, CanxSendFn_T send_fn,
                                uint8_t rx_got, uint32_t now_ms);

void         CanxProto_MarkInflight(CanxEngine_T *engine, uint8_t field, int32_t value);
void         CanxProto_ClearInflight(CanxEngine_T *engine);

void         CanxProto_InitTxFrame(const CanxEngine_T *engine, CanxFrame_T *frame, uint32_t id);

void         CanxProto_RaiseFault(CanxEngine_T *engine, uint32_t mask);
void         CanxProto_ClearFault(CanxEngine_T *engine, uint32_t mask);

#ifdef __cplusplus
}
#endif

#endif /* CANX_PROTO_H */
