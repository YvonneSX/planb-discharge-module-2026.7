/**
 * @file    test_bsp_spi.c
 * @brief   SPI 总线单元测试 实现
 *
 * 覆盖接口: Init / IsInitialized / TransferByte / TransferBuffer
 *
 * 真机说明: SPI1 上挂有 FRAM(MB85RS64), 不便单独短接 MOSI-MISO 做回环。
 *   故通过「读 FRAM 状态寄存器(RDSR=0x05)」间接验证 SPI 收发链路:
 *   FRAM 上电默认状态寄存器值是确定的 (非 0xFF 浮空、非 0x00),
 *   能稳定读到合理值即证明 TransferByte 收发正常。
 *   注: 本测试需 FRAM 的 CS 由调用方控制, 这里直接操作 FRAM CS(PB12)。
 *
 * @version 1.1.0
 * @date    2026-06-13
 */

#include "test_bsp_spi.h"

#define BSP_USE_SPI
#define BSP_USE_FRAM        /* 借用 FRAM 的 CS 引脚宏 (PB12) 做收发链路验证 */
#include "bsp.h"

#define TEST_FRAM_CMD_RDSR   0x05U   /* FRAM 读状态寄存器命令 */

void Test_BspSpi_RunAll(Test_Stats_T *_stats)
{
    uint8_t rx[2] = {0U, 0U};
    uint8_t tx[2] = {TEST_FRAM_CMD_RDSR, 0xFFU};
    uint8_t sr1, sr2;

    /* ---- 初始化 ---- */
    TEST_CASE("SPI: 初始化");
    TEST_ASSERT_EQ(Bsp_Spi_Init(), BSP_SPI_OK);
    TEST_ASSERT(Bsp_Spi_IsInitialized() != 0);

    TEST_CASE("SPI: 重复初始化幂等");
    TEST_ASSERT_EQ(Bsp_Spi_Init(), BSP_SPI_OK);

    /* ---- 块收发参数校验 ---- */
    TEST_CASE("SPI: 块收发参数校验 (len=0)");
    TEST_ASSERT_EQ(Bsp_Spi_TransferBuffer(NULL, rx, 0U), BSP_SPI_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Spi_TransferBuffer(tx, rx, 0U), BSP_SPI_ERR_PARAM);

    /* ---- 块收发正常返回 ---- */
    TEST_CASE("SPI: 块收发正常返回 OK");
    TEST_ASSERT_EQ(Bsp_Spi_TransferBuffer(tx, rx, 2U), BSP_SPI_OK);
    /* tx=NULL 时发 dummy, 也应 OK */
    TEST_ASSERT_EQ(Bsp_Spi_TransferBuffer(NULL, rx, 2U), BSP_SPI_OK);

    /* ---- 通过 FRAM RDSR 验证收发链路 (两次读应一致, 证明链路稳定) ---- */
    TEST_CASE("SPI: 经 FRAM RDSR 验证收发链路稳定");
    /* CS 拉低 (FRAM CS = PB12) */
    gpio_bit_reset(BSP_FRAM_PORT, BSP_FRAM_CS_PIN);
    (void)Bsp_Spi_TransferByte(TEST_FRAM_CMD_RDSR);
    sr1 = Bsp_Spi_TransferByte(0xFFU);
    gpio_bit_set(BSP_FRAM_PORT, BSP_FRAM_CS_PIN);

    gpio_bit_reset(BSP_FRAM_PORT, BSP_FRAM_CS_PIN);
    (void)Bsp_Spi_TransferByte(TEST_FRAM_CMD_RDSR);
    sr2 = Bsp_Spi_TransferByte(0xFFU);
    gpio_bit_set(BSP_FRAM_PORT, BSP_FRAM_CS_PIN);

    /* 两次读状态寄存器应一致 (FRAM 状态稳定); 不应是浮空 0xFF */
    TEST_LOG_VAL("FRAM RDSR: 1st=0x%02X 2nd=0x%02X (应一致且!=0xFF)", sr1, sr2);
    TEST_ASSERT_EQ_INT(sr1, sr2);
    TEST_ASSERT(sr1 != 0xFFU);   /* MISO 浮空才会恒 0xFF, 说明从机无响应 */
}
