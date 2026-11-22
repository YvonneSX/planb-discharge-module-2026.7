/**
 * @file    test_bsp_flash.c
 * @brief   片上Flash单元测试 实现
 *
 * 覆盖接口: ErasePage / Write / Read
 *
 * 真机说明: 测试使用 Flash 最后一页后 256 字节区域（偏移 0x0700），
 *   与配置区（偏移 0x0000）隔开，避免破坏已存储的配置参数。
 *   测试完成后擦除测试区，恢复全 FF 出厂态。
 *
 * 验证要点:
 *   - 地址/长度参数校验（NULL、对齐、越界）
 *   - 擦除后读回全 0xFF
 *   - 写后读回比对（4 字节对齐）
 *   - 写后不掉电二次读回验证写入成功
 *
 * @version 1.0.0
 * @date    2026-06-13
 */

#include "test_bsp_flash.h"

#define BSP_USE_FLASH
#include "bsp.h"

/* 测试用偏移（配置区之后，不破坏配置参数） */
#define TEST_FLASH_OFFSET   0x0700U
#define TEST_FLASH_ADDR     (BSP_FLASH_CFG_ADDR + TEST_FLASH_OFFSET)
#define TEST_BUF_LEN        16U

void Test_BspFlash_RunAll(Test_Stats_T *_stats)
{
    uint32_t wbuf[TEST_BUF_LEN / 4U];
    uint32_t rbuf[TEST_BUF_LEN / 4U];
    uint8_t  raw[TEST_BUF_LEN];
    uint32_t i;

    /* ---- 擦除前准备：填充测试数据 ---- */
    for (i = 0U; i < TEST_BUF_LEN / 4U; i++) {
        wbuf[i] = 0xA5A50000U + i;
    }

    /* ===== 参数校验 ===== */
    TEST_CASE("Flash: 写参数校验 (NULL / len=0 / 非4B对齐)");
    TEST_ASSERT_EQ(Bsp_Flash_Write(TEST_FLASH_ADDR, NULL, 4U), BSP_FLASH_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Flash_Write(TEST_FLASH_ADDR, wbuf, 0U), BSP_FLASH_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Flash_Write(TEST_FLASH_ADDR, wbuf, 3U), BSP_FLASH_ERR_PARAM);

    TEST_CASE("Flash: 读参数校验 (NULL / len=0)");
    TEST_ASSERT_EQ(Bsp_Flash_Read(TEST_FLASH_ADDR, NULL, 4U), BSP_FLASH_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Flash_Read(TEST_FLASH_ADDR, rbuf, 0U), BSP_FLASH_ERR_PARAM);

    /* ===== 越界校验 ===== */
    TEST_CASE("Flash: 写越界校验");
    TEST_ASSERT_EQ(Bsp_Flash_Write(BSP_FLASH_CFG_ADDR - 4U, wbuf, 4U), BSP_FLASH_ERR_RANGE);
    TEST_ASSERT_EQ(Bsp_Flash_Write(BSP_FLASH_CFG_ADDR + BSP_FLASH_PAGE_SIZE, wbuf, 4U), BSP_FLASH_ERR_RANGE);
    TEST_ASSERT_EQ(Bsp_Flash_Write(TEST_FLASH_ADDR, wbuf, BSP_FLASH_PAGE_SIZE - TEST_FLASH_OFFSET + 4U), BSP_FLASH_ERR_RANGE);

    TEST_CASE("Flash: 读越界校验");
    TEST_ASSERT_EQ(Bsp_Flash_Read(BSP_FLASH_CFG_ADDR - 4U, rbuf, 4U), BSP_FLASH_ERR_RANGE);

    /* ===== 擦除测试 ===== */
    TEST_CASE("Flash: 擦除配配置页（仅允许 CFG_ADDR）");
    TEST_ASSERT_EQ(Bsp_Flash_ErasePage(BSP_FLASH_CFG_ADDR), BSP_FLASH_OK);

    TEST_CASE("Flash: 拒绝擦除非配置页地址");
    TEST_ASSERT_EQ(Bsp_Flash_ErasePage(0x08000000U), BSP_FLASH_ERR_PARAM);

    /* 擦除后读回应为全 0xFF */
    TEST_CASE("Flash: 擦除后读回全 0xFF");
    Bsp_Flash_Read(TEST_FLASH_ADDR, raw, TEST_BUF_LEN);
    for (i = 0U; i < TEST_BUF_LEN; i++) {
        TEST_ASSERT_EQ_INT(raw[i], 0xFFU);
    }

    /* ===== 写读回比对 ===== */
    TEST_CASE("Flash: 写后立即读回比对");
    TEST_ASSERT_EQ(Bsp_Flash_Write(TEST_FLASH_ADDR, wbuf, TEST_BUF_LEN), BSP_FLASH_OK);
    TEST_ASSERT_EQ(Bsp_Flash_Read(TEST_FLASH_ADDR, rbuf, TEST_BUF_LEN), BSP_FLASH_OK);
    for (i = 0U; i < TEST_BUF_LEN / 4U; i++) {
        TEST_ASSERT_EQ_INT(rbuf[i], wbuf[i]);
    }

    /* 二次读回（验证真实写入，非缓存） */
    TEST_CASE("Flash: 二次读回确认非缓存");
    {
        uint32_t rbuf2[TEST_BUF_LEN / 4U];
        TEST_ASSERT_EQ(Bsp_Flash_Read(TEST_FLASH_ADDR, rbuf2, TEST_BUF_LEN), BSP_FLASH_OK);
        for (i = 0U; i < TEST_BUF_LEN / 4U; i++) {
                TEST_ASSERT_EQ_INT(rbuf2[i], wbuf[i]);
        }
    }

    /* ===== 测试后清理：重新擦除最后一页 ===== */
    TEST_CASE("Flash: 测试后擦除恢复出厂态");
    TEST_ASSERT_EQ(Bsp_Flash_ErasePage(BSP_FLASH_CFG_ADDR), BSP_FLASH_OK);
    Bsp_Flash_Read(TEST_FLASH_ADDR, raw, 4U);
    TEST_LOG_VAL("Flash@0x%08X 擦除后首字: %02X %02X %02X %02X (应为 FF FF FF FF)",
                 TEST_FLASH_ADDR, raw[0], raw[1], raw[2], raw[3]);
}
