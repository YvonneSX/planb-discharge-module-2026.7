/**
 * @file    test_bsp_fram.c
 * @brief   FRAM铁电存储单元测试 实现
 *
 * 覆盖接口: Init / IsInitialized / Read / Write
 *
 * 真机说明: MB85RS64 为真实在板器件, 可直接写读回比对验证。
 *   FRAM 无擦除、无写延迟, 写后立即可读。测试用独立地址区避免影响黑匣子数据。
 *   测试完成后将测试区写回 0xFF (FRAM 出厂态), 减少对后续使用的影响。
 *
 * @version 1.1.0
 * @date    2026-06-13
 */

#include "test_bsp_fram.h"

#define BSP_USE_FRAM
#include "bsp.h"

/* 测试用地址 (选靠近末尾的区域, 避开黑匣子主数据区) */
#define TEST_FRAM_ADDR      0x1F00U
#define TEST_FRAM_LEN       8U

void Test_BspFram_RunAll(Test_Stats_T *_stats)
{
    uint8_t wbuf[TEST_FRAM_LEN] = {0x12U, 0x34U, 0x56U, 0x78U, 0x9AU, 0xBCU, 0xDEU, 0xF0U};
    uint8_t rbuf[TEST_FRAM_LEN] = {0U};
    uint8_t clr[TEST_FRAM_LEN];
    uint8_t one = 0xA5U, back = 0U;
    uint16_t i;

    /* ---- 初始化 ---- */
    TEST_CASE("FRAM: 初始化");
    TEST_ASSERT_EQ(Bsp_Fram_Init(), BSP_FRAM_OK);
    TEST_ASSERT(Bsp_Fram_IsInitialized() != 0);

    TEST_CASE("FRAM: 重复初始化幂等");
    TEST_ASSERT_EQ(Bsp_Fram_Init(), BSP_FRAM_OK);

    /* ---- 参数校验 ---- */
    TEST_CASE("FRAM: 参数校验 (NULL / len=0)");
    TEST_ASSERT_EQ(Bsp_Fram_Read(0U, NULL, 4U), BSP_FRAM_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Fram_Read(0U, rbuf, 0U), BSP_FRAM_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Fram_Write(0U, NULL, 4U), BSP_FRAM_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Fram_Write(0U, wbuf, 0U), BSP_FRAM_ERR_PARAM);

    /* ---- 越界校验 (addr+len > 8192) ---- */
    TEST_CASE("FRAM: 越界校验");
    TEST_ASSERT_EQ(Bsp_Fram_Write(BSP_FRAM_SIZE_BYTES, wbuf, 4U), BSP_FRAM_ERR_RANGE);
    TEST_ASSERT_EQ(Bsp_Fram_Read(BSP_FRAM_SIZE_BYTES - 2U, rbuf, 4U), BSP_FRAM_ERR_RANGE);
    /* 边界恰好不越界: addr+len == SIZE 应允许 */
    TEST_ASSERT_EQ(Bsp_Fram_Read(BSP_FRAM_SIZE_BYTES - TEST_FRAM_LEN, rbuf, TEST_FRAM_LEN), BSP_FRAM_OK);

    /* ---- 真机写读回比对 (多字节) ---- */
    TEST_CASE("FRAM: 多字节写后读回比对");
    TEST_ASSERT_EQ(Bsp_Fram_Write(TEST_FRAM_ADDR, wbuf, TEST_FRAM_LEN), BSP_FRAM_OK);
    for (i = 0U; i < TEST_FRAM_LEN; i++) { rbuf[i] = 0U; }
    TEST_ASSERT_EQ(Bsp_Fram_Read(TEST_FRAM_ADDR, rbuf, TEST_FRAM_LEN), BSP_FRAM_OK);
    TEST_LOG_VAL("FRAM@0x%04X 写[%02X %02X %02X %02X..] 读[%02X %02X %02X %02X..]",
                 TEST_FRAM_ADDR, wbuf[0], wbuf[1], wbuf[2], wbuf[3],
                 rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
    TEST_ASSERT_MEM_EQ(rbuf, wbuf, TEST_FRAM_LEN);

    /* ---- 单字节写读 ---- */
    TEST_CASE("FRAM: 单字节写后读回");
    TEST_ASSERT_EQ(Bsp_Fram_Write(TEST_FRAM_ADDR, &one, 1U), BSP_FRAM_OK);
    TEST_ASSERT_EQ(Bsp_Fram_Read(TEST_FRAM_ADDR, &back, 1U), BSP_FRAM_OK);
    TEST_LOG_VAL("FRAM 单字节: 写0x%02X 读0x%02X", one, back);
    TEST_ASSERT_EQ_INT(back, 0xA5);

    /* ---- 清理: 测试区写回 0xFF ---- */
    TEST_CASE("FRAM: 测试区清理写回 0xFF");
    for (i = 0U; i < TEST_FRAM_LEN; i++) { clr[i] = 0xFFU; }
    TEST_ASSERT_EQ(Bsp_Fram_Write(TEST_FRAM_ADDR, clr, TEST_FRAM_LEN), BSP_FRAM_OK);
}
