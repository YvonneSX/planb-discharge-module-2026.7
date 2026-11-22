/**
 * @file    canx_proto.c
 * @brief   通用 CAN 协议引擎实现（平台中立；逻辑承自已验证的 can_proto v1.1.0）
 *
 * 只含与协议无关的调度内核：轮询状态机、commit-on-ack、多从站注册/派发、
 * 握手/实时骨架、故障派生、主/从角色调度。不依赖任何具体 BSP——只用中性帧
 * CanxFrame_T，底层收发经 CanxBspOps_T 注入（默认实现见 canx_proto_port.c）。
 *
 * @version 0.1.0
 * @date    2026-06-29
 */

#include <string.h>

#include "canx_proto.h"

/* -------------------------------------------------------------------
 * Profile 预设
 * ------------------------------------------------------------------- */

CanxXferProfile_T CanxProto_ProfileCanFd16(void)
{
    CanxXferProfile_T profile;
    profile.frame_bytes  = 16U;
    profile.fdf          = 1U;
    profile.brs          = 0U;
    profile.tx_ide       = 1U;
    profile.rx_id_offset = 15U;
    profile.dispatch     = CANX_DISPATCH_DATA_BYTE;
    return profile;
}

CanxXferProfile_T CanxProto_ProfileClassic8(void)
{
    CanxXferProfile_T profile;
    profile.frame_bytes  = 8U;
    profile.fdf          = 0U;
    profile.brs          = 0U;
    profile.tx_ide       = 0U;
    profile.rx_id_offset = 0U;
    profile.dispatch     = CANX_DISPATCH_DATA_BYTE; /* 经典 CAN 默认数据字节寻址；CAN-ID 寻址改 CANX_DISPATCH_CAN_ID */
    return profile;
}

/* -------------------------------------------------------------------
 * 通道级注册表 + 集中派发
 * ------------------------------------------------------------------- */

static CanxEngine_T *s_reg[CANX_CH_NUM][CANX_MAX_NODES_PER_CH];
static uint32_t      s_svc_last_ms[CANX_CH_NUM];
static uint8_t       s_svc_seen[CANX_CH_NUM];

void CanxProto_ResetRegistry(void)
{
    (void)memset(s_reg, 0, sizeof(s_reg));
    (void)memset(s_svc_last_ms, 0, sizeof(s_svc_last_ms));
    (void)memset(s_svc_seen, 0, sizeof(s_svc_seen));
}

CanxStatus_E CanxProto_Register(CanxEngine_T *engine)
{
    uint8_t ch, i, empty = 0xFFU;

    if (engine == NULL) {
        return CANX_ERR_PARAM;
    }
    if ((uint32_t)engine->channel >= (uint32_t)CANX_CH_NUM) {
        return CANX_ERR_PARAM;
    }
    ch = engine->channel;
    for (i = 0U; i < CANX_MAX_NODES_PER_CH; i++) {
        if (s_reg[ch][i] == engine) {
            return CANX_OK;                          /* 已按指针注册 */
        }
        if ((s_reg[ch][i] != NULL) && (s_reg[ch][i]->node_id == engine->node_id)) {
            s_reg[ch][i] = engine;                   /* 同 node_id 重登记:替换 */
            return CANX_OK;
        }
        if ((s_reg[ch][i] == NULL) && (empty == 0xFFU)) {
            empty = i;
        }
    }
    if (empty == 0xFFU) {
        return CANX_ERR_FULL;
    }
    s_reg[ch][empty] = engine;
    return CANX_OK;
}

static CanxEngine_T *CanxProto_FindByNodeId(uint8_t channel, uint16_t node_id)
{
    uint8_t i;

    for (i = 0U; i < CANX_MAX_NODES_PER_CH; i++) {
        CanxEngine_T *engine = s_reg[channel][i];
        if ((engine != NULL) && (engine->node_id == node_id)) {
            return engine;
        }
    }
    return NULL;
}

/* -------------------------------------------------------------------
 * 内部辅助
 * ------------------------------------------------------------------- */

static void CanxProto_RefreshState(CanxEngine_T *engine)
{
    if ((engine->state == CANX_STATE_UNINIT) ||
        (engine->state == CANX_STATE_READY)  ||
        (engine->state == CANX_STATE_HANDSHAKE)) {
        return;   /* 通信前/握手中不由故障派生改写 */
    }
    if (engine->fault_bits != 0U) {
        engine->state = CANX_STATE_FAULT;
        return;
    }
    if (engine->state == CANX_STATE_FAULT) {
        engine->state = CANX_STATE_OPERABLE;   /* 故障清除→恢复 */
    }
}

/** 派发命中后处理一帧：调协议 decode_rx，按返回值刷新通用状态。 */
static void CanxProto_HandleFrame(CanxEngine_T *engine, const CanxFrame_T *frame, uint32_t now_ms)
{
    int rc;

    if ((engine->ops == NULL) || (engine->ops->decode_rx == NULL)) {
        return;
    }
    rc = engine->ops->decode_rx(engine, frame, now_ms);
    if (rc > 0) {
        engine->fb_fresh    = 1U;
        engine->last_rx_ms  = now_ms;
        engine->rx_count++;
        engine->fault_bits &= ~(uint32_t)CANX_FAULT_COMM_TIMEOUT; /* 自愈 */

        /* 从站(被动)：命中本节点的请求 → 就地构建并发送应答(无 poll、无重发)。
         * 主站模式不进入本分支(role!=SLAVE)。tx_enable=0 可临时静音应答。 */
        if ((engine->role == CANX_ROLE_SLAVE) &&
            (engine->ops->on_request != NULL) &&
            (engine->tx_enable != 0U)) {
            int ans = engine->ops->on_request(engine, frame, now_ms);
            if (ans > 0) {
                engine->tx_count++;
            } else if (ans < 0) {
                engine->fault_bits |= (uint32_t)CANX_FAULT_INVALID_FRAME;
            } else {
                /* ans==0：只读不回，不计错误 */
            }
        }
    } else if (rc < 0) {
        engine->fault_bits |= (uint32_t)CANX_FAULT_INVALID_FRAME;
    } else {
        /* rc==0：可忽略帧(非本节点)，不计错误 */
    }
}

/* -------------------------------------------------------------------
 * 公共 API：初始化
 * ------------------------------------------------------------------- */

CanxStatus_E CanxProto_Init(CanxEngine_T            *engine,
                            uint8_t                  channel,
                            uint16_t                 node_id,
                            uint16_t                 peer_id,
                            const CanxXferProfile_T *xfer,
                            const CanxBspOps_T      *bsp,
                            const CanxOps_T         *ops,
                            void                    *proto_ctx)
{
    if ((engine == NULL) || (xfer == NULL) || (ops == NULL)) {
        return CANX_ERR_PARAM;
    }
    if ((uint32_t)channel >= (uint32_t)CANX_CH_NUM) {
        return CANX_ERR_PARAM;
    }

    (void)memset(engine, 0, sizeof(*engine));
    engine->xfer      = *xfer;
    engine->bsp       = (bsp != NULL) ? *bsp : CanxProto_DefaultBsp();
    engine->ops       = ops;
    engine->proto_ctx = proto_ctx;
    engine->channel   = channel;
    engine->node_id   = node_id;
    engine->peer_id   = peer_id;
    engine->role      = CANX_ROLE_DEFAULT;   /* 默认主站(零回归)；从站走 InitSlave 覆盖 */

    engine->comm_lost_ms  = 1000U;
    engine->ready_wait_ms = 0U;
    engine->retry_max     = 3U;
    engine->interval_ms   = 10U;

    engine->state     = CANX_STATE_READY;
    engine->phase     = CANX_PHASE_HANDSHAKE;
    engine->tx_enable = 1U;
    engine->rx_enable = 0U;

    return CanxProto_Register(engine);
}

CanxStatus_E CanxProto_InitSlave(CanxEngine_T            *engine,
                                 uint8_t                  channel,
                                 uint16_t                 node_id,
                                 uint16_t                 peer_id,
                                 const CanxXferProfile_T *xfer,
                                 const CanxBspOps_T      *bsp,
                                 const CanxOps_T         *ops,
                                 void                    *proto_ctx)
{
    CanxStatus_E rc;

    rc = CanxProto_Init(engine, channel, node_id, peer_id, xfer, bsp, ops, proto_ctx);
    if (rc != CANX_OK) {
        return rc;
    }

    /* 从站(被动)初态：上电即可应答，无主动握手/轮询 */
    engine->role       = CANX_ROLE_SLAVE;
    engine->state      = CANX_STATE_OPERABLE;
    engine->phase      = CANX_PHASE_REALTIME;
    engine->tx_enable  = 1U;   /* 1=允许应答；置 0 可临时静音 */
    engine->rx_enable  = 1U;
    engine->last_rx_ms = 0U;   /* 由首帧刷新；OPERABLE 下 comm_lost_ms 监测主站静默 */

    return CANX_OK;
}

/* -------------------------------------------------------------------
 * 收帧：通道级集中派发
 * ------------------------------------------------------------------- */

void CanxProto_ServiceChannel(uint8_t channel, uint32_t now_ms)
{
    CanxFrame_T     frame;
    CanxBusStatus_E bus_rc;
    uint8_t         count = 0U;
    uint8_t         i;
    CanxEngine_T   *first = NULL;
    uint8_t         id_off;

    if ((uint32_t)channel >= (uint32_t)CANX_CH_NUM) {
        return;
    }

    /* 同一毫秒内本通道已排空过则跳过(幂等)：多节点共用通道只消费一次队列 */
    if ((s_svc_seen[channel] != 0U) && (s_svc_last_ms[channel] == now_ms)) {
        return;
    }
    s_svc_seen[channel]    = 1U;
    s_svc_last_ms[channel] = now_ms;

    for (i = 0U; i < CANX_MAX_NODES_PER_CH; i++) {
        if (s_reg[channel][i] != NULL) { first = s_reg[channel][i]; break; }
    }
    if ((first == NULL) || (first->bsp.recv == NULL)) {
        return;   /* 本通道无注册节点 */
    }
    id_off = first->xfer.rx_id_offset;

    if (first->bsp.service_rx != NULL) {
        first->bsp.service_rx(channel);
    }
    while (count < CANX_SVC_RX_MAX_PER_CALL) {
        bus_rc = first->bsp.recv(channel, &frame);
        if (bus_rc == CANX_BUS_ERR_EMPTY) {
            break;
        }
        count++;
        if (bus_rc != CANX_BUS_OK) {
            uint8_t k;
            for (k = 0U; k < CANX_MAX_NODES_PER_CH; k++) {
                if (s_reg[channel][k] != NULL) {
                    s_reg[channel][k]->fault_bits |= (uint32_t)CANX_FAULT_BUS_ERROR;
                }
            }
            break;
        }
        /* 按 Profile 的派发模式路由到引擎实例 */
        {
            CanxEngine_T *engine = NULL;
            switch (first->xfer.dispatch) {
            case CANX_DISPATCH_CAN_ID:
                engine = CanxProto_FindByNodeId(channel, (uint16_t)frame.id);
                break;
            case CANX_DISPATCH_SINGLE:
                engine = first;   /* 单节点：全派给首个引擎，过滤交 decode_rx */
                break;
            case CANX_DISPATCH_DATA_BYTE:
            default:
                if (id_off < CANX_MAX_DATA) {
                    engine = CanxProto_FindByNodeId(channel, (uint16_t)frame.data[id_off]);
                }
                break;
            }
            if (engine != NULL) {
                CanxProto_HandleFrame(engine, &frame, now_ms);
            }
        }
    }
}

/* -------------------------------------------------------------------
 * 周期维护 + 主循环调度
 * ------------------------------------------------------------------- */

void CanxProto_Tick(CanxEngine_T *engine, uint32_t now_ms)
{
    uint32_t cur_drop;

    if (engine == NULL) {
        return;
    }
    if (engine->state == CANX_STATE_UNINIT) {
        return;
    }

    /* 1. 收帧：通道集中排空 + 按 ID 派发(幂等) */
    CanxProto_ServiceChannel(engine->channel, now_ms);

    /* 1.5 BSP 队列丢包检测 */
    if (engine->bsp.get_drop_count != NULL) {
        cur_drop = engine->bsp.get_drop_count(engine->channel);
        if (cur_drop != engine->last_bsp_drop) {
            engine->fault_bits |= (uint32_t)CANX_FAULT_RX_OVERFLOW;
            engine->last_bsp_drop = cur_drop;
        }
    }

    /* 2. 通信丢失检测(仅 OPERABLE) */
    if (engine->state == CANX_STATE_OPERABLE) {
        /* 从站上电即 OPERABLE 且未收首帧(last_rx_ms==0)时不计超时，
         * 避免上电瞬间误判主站掉线；收到首帧后正常监测主站静默。 */
        if ((engine->role == CANX_ROLE_SLAVE) && (engine->last_rx_ms == 0U)) {
            /* 等待首帧，跳过超时判定 */
        } else if ((now_ms - engine->last_rx_ms) > engine->comm_lost_ms) {
            if ((engine->fault_bits & (uint32_t)CANX_FAULT_COMM_TIMEOUT) == 0U) {
                engine->timeout_count++;
            }
            engine->fault_bits |= (uint32_t)CANX_FAULT_COMM_TIMEOUT;
        }
    }

    /* 3. 刷新 state */
    CanxProto_RefreshState(engine);
}

CanxStatus_E CanxProto_CycleTask(CanxEngine_T *engine, uint32_t now_ms)
{
    uint8_t rx_got;

    if (engine == NULL) {
        return CANX_ERR_PARAM;
    }

    /* 从站(被动)路径：只做收帧+派发+应答(HandleFrame 内就地)+超时检测。 */
    if (engine->role == CANX_ROLE_SLAVE) {
        CanxProto_Tick(engine, now_ms);
        return CANX_OK;
    }

    /* ① 先读 */
    CanxProto_Tick(engine, now_ms);

    /* READY：可选上电等待窗口，到点或收到帧后启动通信 */
    if (engine->state == CANX_STATE_READY) {
        uint8_t start = 1U;
        if (engine->ready_wait_ms != 0U) {
            if (engine->ready_wait_start_ms == 0U) {
                engine->ready_wait_start_ms = now_ms;
            }
            {
                uint32_t elapsed = now_ms - engine->ready_wait_start_ms;
                uint8_t rx_seen = (engine->last_rx_ms >= engine->ready_wait_start_ms) ? 1U : 0U;
                start = ((rx_seen != 0U) || (elapsed >= engine->ready_wait_ms)) ? 1U : 0U;
            }
        }
        if (start == 0U) {
            return CANX_OK;
        }
        if (engine->ops->has_handshake != 0U) {
            engine->state = CANX_STATE_HANDSHAKE;
            engine->phase = CANX_PHASE_HANDSHAKE;
        } else {
            engine->state = CANX_STATE_OPERABLE;
            engine->phase = CANX_PHASE_REALTIME;
            if (engine->ops->on_operable != NULL) {
                engine->ops->on_operable(engine, now_ms);
            }
        }
        engine->ok_cnt    = 0U;
        engine->tx_enable = 1U;
        engine->rx_enable = 0U;
    }

    if (engine->state == CANX_STATE_FAULT) {
        return CANX_OK;   /* 故障保持，等外部/收帧自愈 */
    }
    if ((engine->state != CANX_STATE_HANDSHAKE) &&
        (engine->state != CANX_STATE_OPERABLE)) {
        return CANX_OK;
    }

    switch (engine->phase) {
    case CANX_PHASE_HANDSHAKE: {
        uint32_t elapsed;

        rx_got = (engine->fb_fresh != 0U) ? 1U : 0U;
        engine->fb_fresh = 0U;

        if (engine->tx_enable != 0U) {
            if (engine->ops->handshake_send != NULL) {
                engine->ops->handshake_send(engine, now_ms);
            }
            engine->next_tx_ms = now_ms;
            engine->tx_enable  = 0U;
            engine->rx_enable  = 1U;
            break;
        }
        if (engine->rx_enable == 0U) {
            break;
        }
        elapsed = now_ms - engine->next_tx_ms;
        if (elapsed < engine->interval_ms) {
            if (rx_got != 0U) {
                engine->tx_enable  = 1U;
                engine->rx_enable  = 0U;
                engine->next_tx_ms = now_ms;
                engine->last_rx_ms = now_ms;
                engine->ok_cnt++;
                engine->retry_cnt = 0U;
                if (engine->ok_cnt >= engine->ops->handshake_need_ok) {
                    engine->state = CANX_STATE_OPERABLE;
                    engine->phase = CANX_PHASE_REALTIME;
                    engine->ok_cnt = 0U;
                    if (engine->ops->on_operable != NULL) {
                        engine->ops->on_operable(engine, now_ms);
                    }
                }
            }
            break;
        }
        /* 超时：重发心跳，永不 FAULT */
        engine->tx_enable  = 1U;
        engine->rx_enable  = 0U;
        engine->next_tx_ms = now_ms;
        engine->ok_cnt     = 0U;
        engine->retry_cnt++;
        break;
    }

    case CANX_PHASE_REALTIME:
        rx_got = (engine->fb_fresh != 0U) ? 1U : 0U;
        engine->fb_fresh = 0U;
        if (engine->ops->realtime_step != NULL) {
            engine->ops->realtime_step(engine, now_ms, rx_got);
        }
        break;

    default:
        break;
    }

    return CANX_OK;
}

CanxAct_E CanxProto_PollStep(CanxEngine_T *engine, CanxSendFn_T send_fn,
                             uint8_t rx_got, uint32_t now_ms)
{
    uint32_t elapsed;

    if (engine == NULL) {
        return CANX_ACT_NONE;
    }

    /* -- 发送阶段 -- */
    if (engine->tx_enable != 0U) {
        if (send_fn != NULL) {
            send_fn(engine, now_ms);
        }
        engine->next_tx_ms = now_ms;
        engine->tx_enable  = 0U;
        engine->rx_enable  = 1U;
        return CANX_ACT_NONE;
    }

    /* -- 接收等待阶段 -- */
    if (engine->rx_enable == 0U) {
        return CANX_ACT_NONE;
    }

    elapsed = now_ms - engine->next_tx_ms;

    if (elapsed < engine->interval_ms) {
        if (rx_got != 0U) {
            engine->tx_enable  = 1U;
            engine->rx_enable  = 0U;
            engine->retry_cnt  = 0U;
            engine->last_rx_ms = now_ms;
            return CANX_ACT_ACK;
        }
        return CANX_ACT_NONE;
    }

    /* 超时：重发 */
    engine->tx_enable  = 1U;
    engine->rx_enable  = 0U;
    engine->next_tx_ms = now_ms;

    if (engine->retry_cnt < engine->retry_max) {
        engine->retry_cnt++;
        return CANX_ACT_NONE;
    }
    return CANX_ACT_TIMEOUT;
}

/* -------------------------------------------------------------------
 * 工具：在途确认 / 建帧 / 故障
 * ------------------------------------------------------------------- */

void CanxProto_MarkInflight(CanxEngine_T *engine, uint8_t field, int32_t value)
{
    if (engine == NULL) { return; }
    engine->inflight_field = field;
    engine->inflight_value = value;
}

void CanxProto_ClearInflight(CanxEngine_T *engine)
{
    if (engine == NULL) { return; }
    engine->inflight_field = 0U;
}

void CanxProto_InitTxFrame(const CanxEngine_T *engine, CanxFrame_T *frame, uint32_t id)
{
    if ((engine == NULL) || (frame == NULL)) { return; }
    (void)memset(frame, 0, sizeof(*frame));
    frame->id  = id;
    frame->ide = engine->xfer.tx_ide;
    frame->rtr = 0U;
    frame->fdf = engine->xfer.fdf;
    frame->brs = engine->xfer.brs;
    frame->len = engine->xfer.frame_bytes;
}

void CanxProto_RaiseFault(CanxEngine_T *engine, uint32_t mask)
{
    if (engine == NULL) { return; }
    engine->fault_bits |= mask;
    CanxProto_RefreshState(engine);
}

void CanxProto_ClearFault(CanxEngine_T *engine, uint32_t mask)
{
    if (engine == NULL) { return; }
    engine->fault_bits &= ~mask;
    CanxProto_RefreshState(engine);
}
