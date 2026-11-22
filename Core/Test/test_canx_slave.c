/**
 * @file    test_canx_slave.c
 * @brief   泄放从站接入层 app_can_slave 单元测试（模块3）
 *
 * 复用 module-1 风格 mock bsp（捕获 TX），以 SINGLE 派发 + 真实从站 Ops
 * (AppCanSlave_TestGetOps) 经引擎驱动，注入主站请求帧验证：
 *   过滤 / 读状态应答 / 控制命令业务联动。
 * 注：on_request 经 mock bsp.send 捕获应答，不触真实 CAN 发送。
 */

#include <string.h>

#include "test_canx_slave.h"
#include "canx_proto.h"
#include "app_can_slave.h"
#include "app_protect.h"
#include "app_discharge.h"

/* 集中开启 BSP 宏以取 Bsp_Config_* 默认 ID */
#include "app.h"

/* ---- mock bsp：捕获 TX，提供受控 RX 队列 ---- */
#define SLVQ_DEPTH 8U
static struct {
    CanxFrame_T rxq[SLVQ_DEPTH];
    uint8_t     head, tail, cnt;
    uint32_t    send_count;
    CanxFrame_T last_tx;
} g_b;

static void b_reset(void) { (void)memset(&g_b, 0, sizeof(g_b)); }

static void b_push(uint32_t id, uint8_t cmd, uint8_t d1, uint8_t d2)
{
    CanxFrame_T *f;
    if (g_b.cnt >= SLVQ_DEPTH) { return; }
    f = &g_b.rxq[g_b.tail];
    (void)memset(f, 0, sizeof(*f));
    f->id = id; f->ide = 0U; f->len = 8U;
    f->data[0] = cmd; f->data[1] = d1; f->data[2] = d2;
    g_b.tail = (uint8_t)((g_b.tail + 1U) % SLVQ_DEPTH);
    g_b.cnt++;
}

static CanxBusStatus_E b_send(uint8_t ch, const CanxFrame_T *f)
{ (void)ch; if (f != NULL) { g_b.last_tx = *f; } g_b.send_count++; return CANX_BUS_OK; }
static CanxBusStatus_E b_recv(uint8_t ch, CanxFrame_T *f)
{
    (void)ch;
    if (g_b.cnt == 0U) { return CANX_BUS_ERR_EMPTY; }
    if (f != NULL) { *f = g_b.rxq[g_b.head]; }
    g_b.head = (uint8_t)((g_b.head + 1U) % SLVQ_DEPTH);
    g_b.cnt--;
    return CANX_BUS_OK;
}
static void     b_svc(uint8_t ch)  { (void)ch; }
static uint32_t b_drop(uint8_t ch) { (void)ch; return 0U; }
static CanxBspOps_T b_bsp(void)
{ CanxBspOps_T o; o.send=b_send; o.recv=b_recv; o.service_rx=b_svc; o.get_drop_count=b_drop; return o; }

void Test_CanxSlave_RunAll(Test_Stats_T *_stats)
{
    static CanxEngine_T e;
    CanxXferProfile_T p = CanxProto_ProfileClassic8();
    CanxBspOps_T b = b_bsp();
    const CanxOps_T *ops = AppCanSlave_TestGetOps();
    uint16_t self_id   = Bsp_Config_GetSelfId();
    uint16_t master_id = Bsp_Config_GetMasterId();

    TEST_LOG("==== app_can_slave 从站回调单元测试 ====");

    /* 业务模块初始化（控制命令联动需要） */
    AppProt_Init();
    AppDch_Init();

    p.dispatch = CANX_DISPATCH_SINGLE;   /* 与 AppCanSlave_Init 一致 */
    CanxProto_ResetRegistry();
    b_reset();
    (void)CanxProto_InitSlave(&e, 0U, self_id, 0U, &p, &b, ops, &e);

    /* 1. 非绑定主站请求 → 不应答 */
    TEST_CASE("slave: 非绑定主站 ID 丢弃不应答");
    b_push(0x222U /*非主站*/, 0x20U, 0, 0);
    (void)CanxProto_CycleTask(&e, 1000U);
    TEST_ASSERT_EQ_INT(g_b.send_count, 0U);

    /* 2. 绑定主站读状态 0x20（电压电流）→ 应答回显命令码 + DLC */
    TEST_CASE("slave: 0x20 读状态应答");
    b_reset();
    b_push(master_id, 0x20U, 0, 0);
    (void)CanxProto_CycleTask(&e, 1001U);
    TEST_ASSERT_EQ_INT(g_b.send_count, 1U);
    TEST_ASSERT_EQ_INT(g_b.last_tx.data[0], 0x20U);   /* 回显命令码 */
    TEST_ASSERT_EQ_INT(g_b.last_tx.len, 5U);          /* V(2)+I(2)+cmd(1) */
    TEST_ASSERT_EQ_INT(g_b.last_tx.id, self_id);      /* 源为本机 ID */

    /* 3. 读设备信息 0x24 → DLC=7 */
    TEST_CASE("slave: 0x24 设备信息应答 DLC=7");
    b_reset();
    b_push(master_id, 0x24U, 0, 0);
    (void)CanxProto_CycleTask(&e, 1002U);
    TEST_ASSERT_EQ_INT(g_b.last_tx.data[0], 0x24U);
    TEST_ASSERT_EQ_INT(g_b.last_tx.len, 7U);

    /* 4. 控制 0x31 清故障 → 经 PROT，应答回显 + fault_bits(2) */
    TEST_CASE("slave: 0x31 清故障联动 PROT");
    b_reset();
    b_push(master_id, 0x31U, 0, 0);
    (void)CanxProto_CycleTask(&e, 1003U);
    TEST_ASSERT_EQ_INT(g_b.last_tx.data[0], 0x31U);
    TEST_ASSERT_EQ_INT(g_b.last_tx.len, 3U);

    /* 5. 控制 0x30 泄放允许/禁止 → 经 DCH，验证 IsAllowed 联动 */
    TEST_CASE("slave: 0x30 允许门联动 DCH");
    b_reset();
    b_push(master_id, 0x30U, 0U, 0);   /* data[1]=0 禁止 */
    (void)CanxProto_CycleTask(&e, 1004U);
    TEST_ASSERT_EQ_INT(g_b.last_tx.data[0], 0x30U);
    TEST_ASSERT(AppDch_IsAllowed() == false);
    b_reset();
    b_push(master_id, 0x30U, 1U, 0);   /* data[1]=1 允许 */
    (void)CanxProto_CycleTask(&e, 1005U);
    TEST_ASSERT(AppDch_IsAllowed() == true);

    /* 6. 未知命令 → 回 UNKNOWN(0xEE) */
    TEST_CASE("slave: 未知命令回 0xEE");
    b_reset();
    b_push(master_id, 0x7AU, 0, 0);
    (void)CanxProto_CycleTask(&e, 1006U);
    TEST_ASSERT_EQ_INT(g_b.last_tx.data[1], 0xEEU);

    CanxProto_ResetRegistry();
}
