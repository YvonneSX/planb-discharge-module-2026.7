/**
 * @file    bsp_fram.c
 * @brief   FRAM 铁电存储 BSP 驱动 (GD32F303CCT6)
 *
 * PB10 — MCU_FRAM_WP   (写保护)
 * PB11 — MCU_FRAM_HOLD (保持)
 * PB12 — MCU_FRAM_CS   (片选, 本驱动管理)
 * SCK/MISO/MOSI (PB13/14/15) 由通用 SPI 总线驱动 bsp_spi.c 管理。
 *
 * MB85RS64PNF-G 64Kbit(8KB) SPI FRAM 黑匣子存储, 16bit 地址, SPI Mode0。
 * 分层: 本文件为器件层, 通过调用 Bsp_Spi_* 完成总线收发 (依赖 bsp_spi.c)。
 *
 * 对应需求: REQ-FUNC-050/051 (黑匣子非易失记录)
 *
 * @version 1.0.0
 * @date    2026-06-12
 */

#define BSP_USE_FRAM
#define BSP_USE_SPI         /* 依赖通用 SPI 总线驱动 */
#include "bsp.h"

/* MB85RS64 操作码 (依据 FRAM 数据手册) */
#define BSP_FRAM_CMD_WREN   0x06U    /* 写使能 */
#define BSP_FRAM_CMD_WRDI   0x04U    /* 写禁止 */
#define BSP_FRAM_CMD_RDSR   0x05U    /* 读状态寄存器 */
#define BSP_FRAM_CMD_WRSR   0x01U    /* 写状态寄存器 */
#define BSP_FRAM_CMD_READ   0x03U    /* 读存储 */
#define BSP_FRAM_CMD_WRITE  0x02U    /* 写存储 */

/* ---------------------------------------------------------------- */
static volatile uint8_t s_fram_initialized = 0U;

/* ---------------------------------------------------------------- */
/**
 * 功能：配置 FRAM 的 CS/WP/HOLD 控制脚为推挽输出并置默认电平（SCK/MISO/MOSI 由 bsp_spi 管理）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Fram_GpioConfig(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    /* PB12 CS / PB10 WP / PB11 HOLD = 推挽输出 */
    gpio_init(BSP_FRAM_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,
              BSP_FRAM_CS_PIN | BSP_FRAM_WP_PIN | BSP_FRAM_HOLD_PIN);
    /* 默认: CS=高(未选中), HOLD=高(不暂停), WP=高(不写保护引脚, 写保护由软件/状态寄存器控制) */
    gpio_bit_set(BSP_FRAM_PORT, BSP_FRAM_CS_PIN);
    gpio_bit_set(BSP_FRAM_PORT, BSP_FRAM_HOLD_PIN);
    gpio_bit_set(BSP_FRAM_PORT, BSP_FRAM_WP_PIN);
}

/**
 * 功能：拉低片选选中 FRAM（CS 低有效）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Fram_CsLow(void)
{
    gpio_bit_reset(BSP_FRAM_PORT, BSP_FRAM_CS_PIN);
}
/**
 * 功能：拉高片选取消选中 FRAM。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Fram_CsHigh(void)
{
    gpio_bit_set(BSP_FRAM_PORT, BSP_FRAM_CS_PIN);
}

/* ---------------------------------------------------------------- */
/**
 * 功能：初始化 FRAM 器件层（控制脚 GPIO + 复用 SPI 总线，片选默认不选中）。幂等。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Fram_Status_T，BSP_FRAM_OK 表示成功。
 */
Bsp_Fram_Status_T Bsp_Fram_Init(void)
{
    if (s_fram_initialized) {
        return BSP_FRAM_OK;
    }
    Bsp_Fram_GpioConfig();
    (void)Bsp_Spi_Init();       /* 复用通用 SPI 总线 (内部幂等, 不重复初始化) */
    Bsp_Fram_CsHigh();          /* 默认不选中 */
    s_fram_initialized = 1U;
    return BSP_FRAM_OK;
}

/**
 * 功能：查询 FRAM 是否已初始化。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，非 0=已初始化，0=未初始化。
 */
int Bsp_Fram_IsInitialized(void)
{
    return (int)s_fram_initialized;
}

/**
 * 功能：从 FRAM 指定地址读取一段数据（READ 命令 + 16bit 地址）。
 * 输入参数：addr —— 起始地址；len —— 读取字节数。
 * 输出参数：buf —— 接收缓冲区，写入读出的数据。
 * 返回值：Bsp_Fram_Status_T，OK 成功，ERR_PARAM 参数非法，ERR_RANGE 越界。
 */
Bsp_Fram_Status_T Bsp_Fram_Read(uint16_t addr, void *buf, uint16_t len)
{
    uint8_t *p = (uint8_t *)buf;
    uint16_t i;

    if (buf == NULL || len == 0U) {
        return BSP_FRAM_ERR_PARAM;
    }
    if ((uint32_t)addr + len > BSP_FRAM_SIZE_BYTES) {
        return BSP_FRAM_ERR_RANGE;
    }
    Bsp_Fram_CsLow();
    (void)Bsp_Spi_TransferByte(BSP_FRAM_CMD_READ);
    if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    (void)Bsp_Spi_TransferByte((uint8_t)(addr >> 8));    /* 16bit 地址, 高字节先 */
    if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    (void)Bsp_Spi_TransferByte((uint8_t)(addr & 0xFFU));
    if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    for (i = 0U; i < len; i++) {
        p[i] = Bsp_Spi_TransferByte(0xFFU);             /* 读: 发 dummy 收数据 */
        if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    }
    Bsp_Fram_CsHigh();
    return BSP_FRAM_OK;
}

/**
 * 功能：向 FRAM 指定地址写入一段数据（先 WREN，再 WRITE + 16bit 地址 + 数据，无需忙等）。
 * 输入参数：addr —— 起始地址；buf —— 源数据指针；len —— 写入字节数。
 * 输出参数：无
 * 返回值：Bsp_Fram_Status_T，OK 成功，ERR_PARAM 参数非法，ERR_RANGE 越界。
 */
Bsp_Fram_Status_T Bsp_Fram_Write(uint16_t addr, const void *buf, uint16_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    uint16_t i;

    if (buf == NULL || len == 0U) {
        return BSP_FRAM_ERR_PARAM;
    }
    if ((uint32_t)addr + len > BSP_FRAM_SIZE_BYTES) {
        return BSP_FRAM_ERR_RANGE;
    }
    /* 1. 置写使能锁 (WREN): 独立一次 CS 脉冲 */
    Bsp_Fram_CsLow();
    (void)Bsp_Spi_TransferByte(BSP_FRAM_CMD_WREN);
    if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    Bsp_Fram_CsHigh();

    /* 2. 写入 (WRITE + 16bit 地址 + 数据); FRAM 无需忙等待 */
    Bsp_Fram_CsLow();
    (void)Bsp_Spi_TransferByte(BSP_FRAM_CMD_WRITE);
    if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    (void)Bsp_Spi_TransferByte((uint8_t)(addr >> 8));
    if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    (void)Bsp_Spi_TransferByte((uint8_t)(addr & 0xFFU));
    if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    for (i = 0U; i < len; i++) {
        (void)Bsp_Spi_TransferByte(p[i]);
        if (Bsp_Spi_XferFailed()) { Bsp_Fram_CsHigh(); return BSP_FRAM_ERR_TIMEOUT; }
    }
    Bsp_Fram_CsHigh();
    return BSP_FRAM_OK;
}
