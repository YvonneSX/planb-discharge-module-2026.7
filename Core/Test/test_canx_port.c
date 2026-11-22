/**
 * @file    test_canx_port.c
 * @brief   canx_proto port 适配层单元测试（模块2，纯软件，无 CAN 硬件依赖）
 *
 * 用 CanxPort_TestInjectRx 注入帧（替代真实 CAN 中断），经 DefaultBsp().recv
 * 出队，验证环形缓冲顺序/空/溢出与帧互转。
 */

#include <string.h>

#include "test_canx_port.h"
#include "canx_proto.h"
#include "canx_proto_port.h"

void Test_CanxPort_RunAll(Test_Stats_T *_stats)
{
    CanxBspOps_T   bsp = CanxProto_DefaultBsp();
    CanxFrame_T    f;
    uint8_t        d[8];
    uint8_t        i;
    CanxBusStatus_E rc;

    TEST_LOG("==== canx_proto port 适配层单元测试 ====");

    /* 1. 空队列 → EMPTY */
    TEST_CASE("port: 空缓冲 recv 返回 EMPTY");
    CanxPort_TestReset();
    TEST_ASSERT_EQ_INT(bsp.recv(0U, &f), CANX_BUS_ERR_EMPTY);

    /* 2. 注入一帧 → 出队 + 帧互转正确（dlc→len，data 拷贝，fdf/brs/rtr=0） */
    TEST_CASE("port: 注入→出队，BSP帧→中性帧互转");
    CanxPort_TestReset();
    for (i = 0U; i < 8U; i++) { d[i] = (uint8_t)(0x10U + i); }
    CanxPort_TestInjectRx(0x123U, 0U, 5U, d);   /* dlc=5 */
    rc = bsp.recv(0U, &f);
    TEST_ASSERT_EQ_INT(rc, CANX_BUS_OK);
    TEST_ASSERT_EQ_INT(f.id, 0x123U);
    TEST_ASSERT_EQ_INT(f.ide, 0U);
    TEST_ASSERT_EQ_INT(f.len, 5U);
    TEST_ASSERT(f.fdf == 0U && f.brs == 0U && f.rtr == 0U);
    TEST_ASSERT(f.data[0] == 0x10U && f.data[4] == 0x14U);
    TEST_ASSERT_EQ_INT(bsp.recv(0U, &f), CANX_BUS_ERR_EMPTY);   /* 取空 */

    /* 3. FIFO 顺序：连续注入 3 帧按序出队 */
    TEST_CASE("port: 多帧 FIFO 顺序");
    CanxPort_TestReset();
    d[0] = 0xA1U; CanxPort_TestInjectRx(0x01U, 0U, 1U, d);
    d[0] = 0xA2U; CanxPort_TestInjectRx(0x02U, 0U, 1U, d);
    d[0] = 0xA3U; CanxPort_TestInjectRx(0x03U, 0U, 1U, d);
    (void)bsp.recv(0U, &f); TEST_ASSERT(f.id == 0x01U && f.data[0] == 0xA1U);
    (void)bsp.recv(0U, &f); TEST_ASSERT(f.id == 0x02U && f.data[0] == 0xA2U);
    (void)bsp.recv(0U, &f); TEST_ASSERT(f.id == 0x03U && f.data[0] == 0xA3U);
    TEST_ASSERT_EQ_INT(bsp.recv(0U, &f), CANX_BUS_ERR_EMPTY);

    /* 4. 溢出：注入超过 (CANX_RXQ_DEPTH-1) 帧 → drop 计数增长 */
    TEST_CASE("port: 缓冲溢出计 drop");
    CanxPort_TestReset();
    TEST_ASSERT_EQ_INT(bsp.get_drop_count(0U), 0U);
    /* 环形缓冲可用容量 = DEPTH-1；注入 DEPTH+2 帧，应丢 3 帧 */
    for (i = 0U; i < (uint8_t)(CANX_RXQ_DEPTH + 2U); i++) {
        d[0] = i;
        CanxPort_TestInjectRx(0x10U + i, 0U, 1U, d);
    }
    TEST_ASSERT_EQ_INT(bsp.get_drop_count(0U), 3U);

    /* 5. dlc 超 8 钳制（防御）：注入 dlc=8（上限），len=8 */
    TEST_CASE("port: dlc 上限 8");
    CanxPort_TestReset();
    CanxPort_TestInjectRx(0x55U, 0U, 8U, d);
    (void)bsp.recv(0U, &f);
    TEST_ASSERT_EQ_INT(f.len, 8U);

    /* 6. service_rx 不崩（中断驱动平台为空实现） */
    TEST_CASE("port: service_rx 空实现可调用");
    bsp.service_rx(0U);
    TEST_ASSERT(1);
}
