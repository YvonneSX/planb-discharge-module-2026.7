/**
 * @file    test_canx_proto.c
 * @brief   canx_proto 引擎单元测试（模块1：引擎本体，mock 驱动）
 *
 * 全部用例靠 mock BspOps + mock Ops 回调驱动，不依赖真实 CAN 硬件。
 * 沿用工程测试框架（Test_Stats_T + TEST_ASSERT*），由 test_runner 汇总。
 */

#include <string.h>

#include "test_canx_proto.h"
#include "canx_proto.h"

/* ================================================================
 * mock BSP：用全局环形队列模拟收发 + 丢包 + 总线错误
 * ================================================================ */
#define MOCK_RXQ_DEPTH   8U

static struct {
    CanxFrame_T rxq[MOCK_RXQ_DEPTH];
    uint8_t     head, tail, cnt;
    uint8_t     force_bus_err;
    uint32_t    drop_count;
    uint32_t    send_count;
    uint32_t    svc_count;
    CanxFrame_T last_tx;
} g_bus;

static void mock_reset(void)
{
    (void)memset(&g_bus, 0, sizeof(g_bus));
}

/* 推一帧入 mock RX 队列：id_byte 放 data[id_off]，can_id 放 frame.id，code 放 data[1] */
static void mock_push(uint32_t can_id, uint8_t id_off_val, uint8_t id_off, uint8_t code)
{
    CanxFrame_T *f;
    if (g_bus.cnt >= MOCK_RXQ_DEPTH) { return; }
    f = &g_bus.rxq[g_bus.tail];
    (void)memset(f, 0, sizeof(*f));
    f->id  = can_id;
    f->ide = 0U;
    f->len = 8U;
    f->data[id_off] = id_off_val;
    f->data[1]      = code;
    g_bus.tail = (uint8_t)((g_bus.tail + 1U) % MOCK_RXQ_DEPTH);
    g_bus.cnt++;
}

static CanxBusStatus_E mock_send(uint8_t ch, const CanxFrame_T *f)
{
    (void)ch;
    if (f != NULL) { g_bus.last_tx = *f; }
    g_bus.send_count++;
    return CANX_BUS_OK;
}

static CanxBusStatus_E mock_recv(uint8_t ch, CanxFrame_T *f)
{
    (void)ch;
    if (g_bus.force_bus_err != 0U) {
        g_bus.force_bus_err = 0U;
        return CANX_BUS_ERR;
    }
    if (g_bus.cnt == 0U) {
        return CANX_BUS_ERR_EMPTY;
    }
    if (f != NULL) { *f = g_bus.rxq[g_bus.head]; }
    g_bus.head = (uint8_t)((g_bus.head + 1U) % MOCK_RXQ_DEPTH);
    g_bus.cnt--;
    return CANX_BUS_OK;
}

static void     mock_service_rx(uint8_t ch) { (void)ch; g_bus.svc_count++; }
static uint32_t mock_get_drop(uint8_t ch)   { (void)ch; return g_bus.drop_count; }

static CanxBspOps_T mock_bsp(void)
{
    CanxBspOps_T ops;
    ops.send           = mock_send;
    ops.recv           = mock_recv;
    ops.service_rx     = mock_service_rx;
    ops.get_drop_count = mock_get_drop;
    return ops;
}

/* ================================================================
 * mock 协议回调（主站）
 * ================================================================ */
static struct {
    uint32_t decode_cnt, hs_send_cnt, rt_step_cnt, on_operable_cnt;
} g_ops;

static void ops_reset(void) { (void)memset(&g_ops, 0, sizeof(g_ops)); }

static int ops_decode_rx(CanxEngine_T *e, const CanxFrame_T *f, uint32_t now)
{
    (void)e; (void)now;
    g_ops.decode_cnt++;
    if (f->data[1] == 0xFFU) { return -1; }   /* 坏帧 */
    if (f->data[1] == 0U)    { return 0; }     /* 忽略 */
    return 1;
}
static void ops_hs_send(CanxEngine_T *e, uint32_t now)
{
    CanxFrame_T fr;
    (void)now;
    g_ops.hs_send_cnt++;
    CanxProto_InitTxFrame(e, &fr, 0x200U);
    if (e->bsp.send != NULL) { (void)e->bsp.send(e->channel, &fr); }
}
static void ops_rt_step(CanxEngine_T *e, uint32_t now, uint8_t rx_got)
{ (void)e; (void)now; (void)rx_got; g_ops.rt_step_cnt++; }
static void ops_on_operable(CanxEngine_T *e, uint32_t now)
{ (void)e; (void)now; g_ops.on_operable_cnt++; }

static const CanxOps_T OPS_HANDSHAKE = {
    .decode_rx = ops_decode_rx, .handshake_send = ops_hs_send,
    .realtime_step = ops_rt_step, .on_operable = ops_on_operable,
    .on_request = NULL, .has_handshake = 1U, .handshake_need_ok = 2U,
};
static const CanxOps_T OPS_NOHANDSHAKE = {
    .decode_rx = ops_decode_rx, .handshake_send = NULL,
    .realtime_step = ops_rt_step, .on_operable = ops_on_operable,
    .on_request = NULL, .has_handshake = 0U, .handshake_need_ok = 0U,
};

static uint32_t g_pollsend_cnt;
static void pollstep_send(CanxEngine_T *e, uint32_t now) { (void)e; (void)now; g_pollsend_cnt++; }

/* DATA_BYTE 派发用 Profile：rx_id_offset=0，节点 ID 在 data[0] */
static CanxXferProfile_T prof_databyte(void)
{
    CanxXferProfile_T p = CanxProto_ProfileClassic8();
    p.dispatch = CANX_DISPATCH_DATA_BYTE;
    p.rx_id_offset = 0U;
    return p;
}

/* ================================================================
 * 主站用例
 * ================================================================ */

static void t_profiles(Test_Stats_T *_stats)
{
    CanxXferProfile_T fd = CanxProto_ProfileCanFd16();
    CanxXferProfile_T cl = CanxProto_ProfileClassic8();
    TEST_CASE("Profile: FD16 / Classic8 默认字段");
    TEST_ASSERT(fd.frame_bytes == 16U && fd.fdf == 1U && fd.tx_ide == 1U);
    TEST_ASSERT(cl.frame_bytes == 8U && cl.fdf == 0U && cl.tx_ide == 0U);
    TEST_ASSERT_EQ_INT(cl.dispatch, CANX_DISPATCH_DATA_BYTE);
}

static void t_init(Test_Stats_T *_stats)
{
    static CanxEngine_T e;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T b = mock_bsp();
    int ctx = 0;

    CanxProto_ResetRegistry();
    TEST_CASE("Init: 主站初态与参数");
    TEST_ASSERT_EQ_INT(CanxProto_Init(&e, 0U, 0x17U, 0x64U, &p, &b, &OPS_NOHANDSHAKE, &ctx), CANX_OK);
    TEST_ASSERT_EQ_INT(e.state, CANX_STATE_READY);
    TEST_ASSERT_EQ_INT(e.role, CANX_ROLE_MASTER);
    TEST_ASSERT(e.tx_enable == 1U && e.rx_enable == 0U);
    TEST_ASSERT(e.retry_max == 3U && e.interval_ms == 10U && e.comm_lost_ms == 1000U);
    TEST_ASSERT(e.proto_ctx == &ctx && e.node_id == 0x17U && e.peer_id == 0x64U);
    TEST_ASSERT_EQ_INT(CanxProto_Init(NULL, 0U, 1U, 1U, &p, &b, &OPS_NOHANDSHAKE, NULL), CANX_ERR_PARAM);
    TEST_ASSERT_EQ_INT(CanxProto_Init(&e, CANX_CH_NUM, 1U, 1U, &p, &b, &OPS_NOHANDSHAKE, NULL), CANX_ERR_PARAM);
}

static void t_register_full(Test_Stats_T *_stats)
{
    static CanxEngine_T pool[CANX_MAX_NODES_PER_CH];
    static CanxEngine_T replace_eng, overflow_eng;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T b = mock_bsp();
    uint8_t i;
    int all_ok = 1;

    CanxProto_ResetRegistry();
    TEST_CASE("Register: 满表/重登记替换/溢出");
    for (i = 0U; i < CANX_MAX_NODES_PER_CH; i++) {
        if (CanxProto_Init(&pool[i], 0U, (uint16_t)(0x20U + i), 0x64U,
                           &p, &b, &OPS_NOHANDSHAKE, &pool[i]) != CANX_OK) { all_ok = 0; }
    }
    TEST_ASSERT(all_ok != 0);
    replace_eng.channel = 0U; replace_eng.node_id = 0x20U;
    TEST_ASSERT_EQ_INT(CanxProto_Register(&replace_eng), CANX_OK);   /* 同 ID 替换 */
    overflow_eng.channel = 0U; overflow_eng.node_id = 0xFEU;
    TEST_ASSERT_EQ_INT(CanxProto_Register(&overflow_eng), CANX_ERR_FULL);
}

static void t_inittxframe(Test_Stats_T *_stats)
{
    static CanxEngine_T e;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T b = mock_bsp();
    CanxFrame_T f;

    CanxProto_ResetRegistry();
    (void)CanxProto_Init(&e, 0U, 0x11U, 0x64U, &p, &b, &OPS_NOHANDSHAKE, &e);
    TEST_CASE("InitTxFrame: id/ide/len 取自 profile，负载清零");
    (void)memset(&f, 0xCC, sizeof(f));
    CanxProto_InitTxFrame(&e, &f, 0x321U);
    TEST_ASSERT_EQ_INT(f.id, 0x321U);
    TEST_ASSERT(f.ide == p.tx_ide && f.fdf == p.fdf && f.brs == p.brs);
    TEST_ASSERT_EQ_INT(f.len, p.frame_bytes);
    TEST_ASSERT(f.data[0] == 0U && f.data[7] == 0U);
}

static void t_inflight_fault(Test_Stats_T *_stats)
{
    static CanxEngine_T e;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T b = mock_bsp();

    CanxProto_ResetRegistry();
    (void)CanxProto_Init(&e, 0U, 0x12U, 0x64U, &p, &b, &OPS_NOHANDSHAKE, &e);
    TEST_CASE("Inflight: Mark/Clear");
    CanxProto_MarkInflight(&e, 7U, 1234);
    TEST_ASSERT(e.inflight_field == 7U && e.inflight_value == 1234);
    CanxProto_ClearInflight(&e);
    TEST_ASSERT_EQ_INT(e.inflight_field, 0U);

    TEST_CASE("Fault: Raise/Clear 与状态派生");
    e.state = CANX_STATE_OPERABLE; e.fault_bits = 0U;
    CanxProto_RaiseFault(&e, (uint32_t)CANX_FAULT_BUS_ERROR);
    TEST_ASSERT_EQ_INT(e.state, CANX_STATE_FAULT);
    CanxProto_ClearFault(&e, (uint32_t)CANX_FAULT_BUS_ERROR);
    TEST_ASSERT(e.fault_bits == 0U && e.state == CANX_STATE_OPERABLE);
}

static void t_pollstep(Test_Stats_T *_stats)
{
    static CanxEngine_T e;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T b = mock_bsp();
    CanxAct_E act;

    CanxProto_ResetRegistry();
    (void)CanxProto_Init(&e, 0U, 0x13U, 0x64U, &p, &b, &OPS_NOHANDSHAKE, &e);
    g_pollsend_cnt = 0U;
    e.tx_enable = 1U; e.rx_enable = 0U; e.retry_cnt = 0U; e.retry_max = 2U; e.interval_ms = 10U;

    TEST_CASE("PollStep: 发→等→ACK / 超时重发→TIMEOUT");
    act = CanxProto_PollStep(&e, pollstep_send, 0U, 10U);
    TEST_ASSERT(act == CANX_ACT_NONE && g_pollsend_cnt == 1U && e.rx_enable == 1U);
    act = CanxProto_PollStep(&e, pollstep_send, 1U, 12U);
    TEST_ASSERT(act == CANX_ACT_ACK && e.retry_cnt == 0U && e.tx_enable == 1U);

    (void)CanxProto_PollStep(&e, pollstep_send, 0U, 20U);
    act = CanxProto_PollStep(&e, pollstep_send, 0U, 35U);
    TEST_ASSERT(act == CANX_ACT_NONE && e.retry_cnt == 1U);
    (void)CanxProto_PollStep(&e, pollstep_send, 0U, 36U);
    act = CanxProto_PollStep(&e, pollstep_send, 0U, 50U);
    TEST_ASSERT(act == CANX_ACT_NONE && e.retry_cnt == 2U);
    (void)CanxProto_PollStep(&e, pollstep_send, 0U, 51U);
    act = CanxProto_PollStep(&e, pollstep_send, 0U, 65U);
    TEST_ASSERT_EQ_INT(act, CANX_ACT_TIMEOUT);
}

static void t_servicechannel(Test_Stats_T *_stats)
{
    static CanxEngine_T a, b_eng;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T bops = mock_bsp();

    CanxProto_ResetRegistry();
    (void)CanxProto_Init(&a,     0U, 0x11U, 0x64U, &p, &bops, &OPS_NOHANDSHAKE, &a);
    (void)CanxProto_Init(&b_eng, 0U, 0x12U, 0x64U, &p, &bops, &OPS_NOHANDSHAKE, &b_eng);
    ops_reset(); mock_reset();

    TEST_CASE("ServiceChannel: DATA_BYTE 派发/幂等/未知丢弃/坏帧/总线错误");
    mock_push(0x100U, 0x11U, 0U, 1U);            /* 发给 a */
    CanxProto_ServiceChannel(0U, 200U);
    TEST_ASSERT(a.rx_count == 1U && a.fb_fresh == 1U && b_eng.rx_count == 0U);
    TEST_ASSERT_EQ_INT(g_bus.svc_count, 1U);

    mock_push(0x100U, 0x11U, 0U, 1U);
    CanxProto_ServiceChannel(0U, 200U);          /* 同毫秒幂等 */
    TEST_ASSERT_EQ_INT(a.rx_count, 1U);
    CanxProto_ServiceChannel(0U, 201U);          /* 下一毫秒排空 */
    TEST_ASSERT_EQ_INT(a.rx_count, 2U);

    mock_push(0x100U, 0x7EU, 0U, 1U);            /* 未知 ID */
    CanxProto_ServiceChannel(0U, 202U);
    TEST_ASSERT_EQ_INT(a.rx_count, 2U);

    mock_push(0x100U, 0x12U, 0U, 0xFFU);         /* 坏帧 → b 置 INVALID_FRAME */
    CanxProto_ServiceChannel(0U, 203U);
    TEST_ASSERT((b_eng.fault_bits & (uint32_t)CANX_FAULT_INVALID_FRAME) != 0U);

    a.fault_bits = 0U; b_eng.fault_bits = 0U;
    g_bus.force_bus_err = 1U;
    CanxProto_ServiceChannel(0U, 204U);
    TEST_ASSERT((a.fault_bits & (uint32_t)CANX_FAULT_BUS_ERROR) != 0U &&
                (b_eng.fault_bits & (uint32_t)CANX_FAULT_BUS_ERROR) != 0U);
}

static void t_tick(Test_Stats_T *_stats)
{
    static CanxEngine_T e;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T b = mock_bsp();

    CanxProto_ResetRegistry();
    (void)CanxProto_Init(&e, 0U, 0x13U, 0x64U, &p, &b, &OPS_NOHANDSHAKE, &e);
    mock_reset();

    TEST_CASE("Tick: 丢包→RX_OVERFLOW / 超时→COMM_TIMEOUT / 收帧自愈");
    e.state = CANX_STATE_OPERABLE; e.last_rx_ms = 400U; e.last_bsp_drop = 0U;
    g_bus.drop_count = 5U;
    CanxProto_Tick(&e, 401U);
    TEST_ASSERT((e.fault_bits & (uint32_t)CANX_FAULT_RX_OVERFLOW) != 0U && e.last_bsp_drop == 5U);

    e.fault_bits = 0U; e.state = CANX_STATE_OPERABLE; e.last_rx_ms = 400U; e.timeout_count = 0U;
    CanxProto_Tick(&e, 400U + e.comm_lost_ms + 1U);
    TEST_ASSERT((e.fault_bits & (uint32_t)CANX_FAULT_COMM_TIMEOUT) != 0U && e.timeout_count == 1U);
    TEST_ASSERT_EQ_INT(e.state, CANX_STATE_FAULT);

    mock_push(0x100U, 0x13U, 0U, 1U);
    CanxProto_Tick(&e, 400U + e.comm_lost_ms + 5U);
    TEST_ASSERT((e.fault_bits & (uint32_t)CANX_FAULT_COMM_TIMEOUT) == 0U && e.state == CANX_STATE_OPERABLE);
}

static void t_cycletask_master(Test_Stats_T *_stats)
{
    static CanxEngine_T eh, en;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T b = mock_bsp();
    uint32_t t = 1000U;

    /* 握手路径 */
    CanxProto_ResetRegistry(); ops_reset(); mock_reset();
    (void)CanxProto_Init(&eh, 0U, 0x14U, 0x64U, &p, &b, &OPS_HANDSHAKE, &eh);
    TEST_CASE("CycleTask 握手: READY→HANDSHAKE→OPERABLE");
    (void)CanxProto_CycleTask(&eh, t++);
    TEST_ASSERT(eh.state == CANX_STATE_HANDSHAKE && g_ops.hs_send_cnt == 1U);
    mock_push(0x100U, 0x14U, 0U, 1U);
    (void)CanxProto_CycleTask(&eh, t++);
    TEST_ASSERT(eh.ok_cnt == 1U && eh.state == CANX_STATE_HANDSHAKE);
    (void)CanxProto_CycleTask(&eh, t++);                 /* 握手#2 发 */
    mock_push(0x100U, 0x14U, 0U, 1U);
    (void)CanxProto_CycleTask(&eh, t++);
    TEST_ASSERT(eh.state == CANX_STATE_OPERABLE && g_ops.on_operable_cnt == 1U);

    /* 无握手路径 */
    CanxProto_ResetRegistry(); ops_reset(); mock_reset();
    (void)CanxProto_Init(&en, 0U, 0x15U, 0x64U, &p, &b, &OPS_NOHANDSHAKE, &en);
    TEST_CASE("CycleTask 无握手: 首拍直达 OPERABLE");
    (void)CanxProto_CycleTask(&en, 2000U);
    TEST_ASSERT(en.state == CANX_STATE_OPERABLE && g_ops.on_operable_cnt == 1U);
    (void)CanxProto_CycleTask(&en, 2001U);
    TEST_ASSERT(g_ops.rt_step_cnt >= 1U);
}

/* ================================================================
 * mock 协议回调（从站）+ 从站用例
 * ================================================================ */
#define SL_FC_READ   0x01U
#define SL_FC_WVOLT  0x02U
#define SL_FC_WEN    0x03U

static struct {
    uint16_t set_voltage_mv;
    uint8_t  enable;
    uint16_t fb_voltage_mv;
    uint8_t  fb_status;
    uint32_t on_request_cnt;
} g_slave;

static void slave_reset(void) { (void)memset(&g_slave, 0, sizeof(g_slave)); }

/* 从站请求帧：node_id 在 data[0]（rx_id_offset=0），功能码在 data[1] */
static void mock_push_slave_req(uint8_t node_id, uint8_t fc, uint8_t d2, uint8_t d3, uint8_t d4)
{
    CanxFrame_T *f;
    if (g_bus.cnt >= MOCK_RXQ_DEPTH) { return; }
    f = &g_bus.rxq[g_bus.tail];
    (void)memset(f, 0, sizeof(*f));
    f->id = 0x300U + node_id; f->ide = 0U; f->len = 8U;
    f->data[0] = node_id; f->data[1] = fc; f->data[2] = d2; f->data[3] = d3; f->data[4] = d4;
    g_bus.tail = (uint8_t)((g_bus.tail + 1U) % MOCK_RXQ_DEPTH);
    g_bus.cnt++;
}

static int slave_decode_rx(CanxEngine_T *e, const CanxFrame_T *f, uint32_t now)
{
    (void)now;
    if (f->data[e->xfer.rx_id_offset] != (uint8_t)e->node_id) { return 0; }
    switch (f->data[1]) {
    case SL_FC_READ:  break;
    case SL_FC_WVOLT: g_slave.set_voltage_mv = (uint16_t)((uint16_t)f->data[2] | ((uint16_t)f->data[3] << 8)); break;
    case SL_FC_WEN:   g_slave.enable = f->data[4]; break;
    default: return -1;
    }
    return 1;
}
static int slave_on_request(CanxEngine_T *e, const CanxFrame_T *req, uint32_t now)
{
    CanxFrame_T ans;
    (void)now;
    g_slave.on_request_cnt++;
    CanxProto_InitTxFrame(e, &ans, e->node_id);
    ans.data[e->xfer.rx_id_offset] = (uint8_t)e->node_id;
    switch (req->data[1]) {
    case SL_FC_READ:
        ans.data[1] = (uint8_t)(SL_FC_READ | 0x80U);
        ans.data[2] = (uint8_t)(g_slave.fb_voltage_mv & 0xFFU);
        ans.data[3] = (uint8_t)(g_slave.fb_voltage_mv >> 8);
        ans.data[4] = g_slave.fb_status;
        break;
    case SL_FC_WVOLT: ans.data[1] = (uint8_t)(SL_FC_WVOLT | 0x80U); break;
    case SL_FC_WEN:   ans.data[1] = (uint8_t)(SL_FC_WEN | 0x80U); break;
    default: return 0;
    }
    if (e->bsp.send(e->channel, &ans) != CANX_BUS_OK) { return -1; }
    return 1;
}
static const CanxOps_T OPS_SLAVE = {
    .decode_rx = slave_decode_rx, .handshake_send = NULL, .realtime_step = NULL,
    .on_operable = NULL, .on_request = slave_on_request, .has_handshake = 0U, .handshake_need_ok = 0U,
};

static void t_slave(Test_Stats_T *_stats)
{
    static CanxEngine_T e;
    CanxXferProfile_T p = prof_databyte();
    CanxBspOps_T b = mock_bsp();

    CanxProto_ResetRegistry(); mock_reset(); slave_reset();
    TEST_CASE("InitSlave: 从站初态");
    TEST_ASSERT_EQ_INT(CanxProto_InitSlave(&e, 0U, 0x31U, 0U, &p, &b, &OPS_SLAVE, &e), CANX_OK);
    TEST_ASSERT(e.role == CANX_ROLE_SLAVE && e.state == CANX_STATE_OPERABLE);
    TEST_ASSERT(e.phase == CANX_PHASE_REALTIME && e.tx_enable == 1U && e.rx_enable == 1U);
    g_slave.fb_voltage_mv = 0x1234U; g_slave.fb_status = 0x5AU;

    TEST_CASE("从站: 纯被动(无请求不发)");
    (void)CanxProto_CycleTask(&e, 5000U);
    (void)CanxProto_CycleTask(&e, 5001U);
    TEST_ASSERT_EQ_INT(g_bus.send_count, 0U);

    TEST_CASE("从站: 读请求→应答负载正确");
    mock_push_slave_req(0x31U, SL_FC_READ, 0, 0, 0);
    (void)CanxProto_CycleTask(&e, 5002U);
    TEST_ASSERT(g_slave.on_request_cnt == 1U && g_bus.send_count == 1U);
    TEST_ASSERT(g_bus.last_tx.data[1] == (uint8_t)(SL_FC_READ | 0x80U) &&
                g_bus.last_tx.data[2] == 0x34U && g_bus.last_tx.data[3] == 0x12U &&
                g_bus.last_tx.data[4] == 0x5AU);
    TEST_ASSERT_EQ_INT(e.tx_count, 1U);

    TEST_CASE("从站: 写电压落库 + ACK");
    mock_push_slave_req(0x31U, SL_FC_WVOLT, 0xCDU, 0xABU, 0);
    (void)CanxProto_CycleTask(&e, 5003U);
    TEST_ASSERT_EQ_INT(g_slave.set_voltage_mv, 0xABCDU);
    TEST_ASSERT(g_bus.last_tx.data[1] == (uint8_t)(SL_FC_WVOLT | 0x80U));

    TEST_CASE("从站: 非本节点请求被丢弃");
    {
        uint32_t ps = g_bus.send_count, pr = g_slave.on_request_cnt;
        mock_push_slave_req(0x7FU, SL_FC_READ, 0, 0, 0);
        (void)CanxProto_CycleTask(&e, 5004U);
        TEST_ASSERT(g_bus.send_count == ps && g_slave.on_request_cnt == pr);
    }

    TEST_CASE("从站: 应答静音(tx_enable=0 落库不回)");
    e.tx_enable = 0U;
    {
        uint32_t ps = g_bus.send_count;
        mock_push_slave_req(0x31U, SL_FC_WVOLT, 0x10U, 0x00U, 0);
        (void)CanxProto_CycleTask(&e, 5005U);
        TEST_ASSERT(g_slave.set_voltage_mv == 0x0010U && g_bus.send_count == ps);
    }
    e.tx_enable = 1U;

    TEST_CASE("从站: 主站静默→COMM_TIMEOUT");
    e.fault_bits = 0U; e.timeout_count = 0U;
    (void)CanxProto_CycleTask(&e, 5005U + e.comm_lost_ms + 1U);
    TEST_ASSERT((e.fault_bits & (uint32_t)CANX_FAULT_COMM_TIMEOUT) != 0U && e.timeout_count == 1U);
}

static void t_dispatch_modes(Test_Stats_T *_stats)
{
    static CanxEngine_T e_id, e_single;
    CanxXferProfile_T p_id = CanxProto_ProfileClassic8();
    CanxXferProfile_T p_sg = CanxProto_ProfileClassic8();
    CanxBspOps_T b = mock_bsp();

    /* CAN_ID 派发：node_id 对 frame.id */
    p_id.dispatch = CANX_DISPATCH_CAN_ID;
    CanxProto_ResetRegistry(); mock_reset(); slave_reset();
    (void)CanxProto_InitSlave(&e_id, 0U, 0x0AU, 0U, &p_id, &b, &OPS_SLAVE, &e_id);
    /* 让 decode 命中：rx_id_offset=0，slave_decode 比对 data[0]==node_id；故 data[0] 置 0x0A */
    TEST_CASE("Dispatch CAN_ID: frame.id 匹配 node_id 才派发");
    mock_push_slave_req(0x0AU, SL_FC_READ, 0, 0, 0);   /* frame.id=0x300+0x0A，不等 node_id 0x0A */
    g_bus.rxq[0].id = 0x0AU;                            /* 改成 frame.id==node_id */
    (void)CanxProto_CycleTask(&e_id, 7000U);
    TEST_ASSERT_EQ_INT(g_slave.on_request_cnt, 1U);

    /* SINGLE 派发：本通道帧全进唯一引擎 */
    p_sg.dispatch = CANX_DISPATCH_SINGLE;
    CanxProto_ResetRegistry(); mock_reset(); slave_reset();
    (void)CanxProto_InitSlave(&e_single, 0U, 0x0BU, 0U, &p_sg, &b, &OPS_SLAVE, &e_single);
    TEST_CASE("Dispatch SINGLE: 任意 ID 帧都派给唯一引擎");
    mock_push_slave_req(0x0BU, SL_FC_READ, 0, 0, 0);   /* data[0]=0x0B 命中 decode */
    g_bus.rxq[0].id = 0x999U;                          /* 任意 frame.id */
    (void)CanxProto_CycleTask(&e_single, 7100U);
    TEST_ASSERT_EQ_INT(g_slave.on_request_cnt, 1U);
}

/* ================================================================
 * 入口
 * ================================================================ */
void Test_CanxProto_RunAll(Test_Stats_T *_stats)
{
    TEST_LOG("==== canx_proto 引擎单元测试 ====");
    t_profiles(_stats);
    t_init(_stats);
    t_register_full(_stats);
    t_inittxframe(_stats);
    t_inflight_fault(_stats);
    t_pollstep(_stats);
    t_servicechannel(_stats);
    t_tick(_stats);
    t_cycletask_master(_stats);
    t_slave(_stats);
    t_dispatch_modes(_stats);
    CanxProto_ResetRegistry();   /* 收尾清注册表，不影响后续模块 */
}


