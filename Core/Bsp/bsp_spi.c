/**
 * @file    bsp_spi.c
 * @brief   SPI1 通用总线 BSP 驱动 (GD32F303CCT6)
 *
 * PB13 — SPI1_SCK  (时钟)
 * PB14 — SPI1_MISO (主入从出)
 * PB15 — SPI1_MOSI (主出从入)
 * 主模式, SPI Mode0 (CPOL=0,CPHA=0)。
 *
 * 通用总线层, 不含任何器件语义; 片选(CS)由各器件驱动自行管理。
 * 当前挂载: FRAM(MB85RS64)。后续挂其他 SPI 器件可直接复用本驱动。
 * 参考: 官方 demo 12_SPI_SPI_Flash 的总线层 (实例由 SPI0 改 SPI1)。
 *
 * 对应需求: REQ-FUNC-050/051 (黑匣子存储的底层总线)
 *
 * @version 1.0.0
 * @date    2026-06-12
 */

#define BSP_USE_SPI
#include "bsp.h"

/* ---------------------------------------------------------------- */
static volatile uint8_t s_spi_initialized = 0U;
static volatile uint8_t s_spi_xfer_fail   = 0U;   /* 最近一次 TransferByte 是否因超时失败 */

#ifndef BSP_SPI_XFER_TIMEOUT
#define BSP_SPI_XFER_TIMEOUT    100000U          /* 单字节收发忙等上限（防 FRAM/SPI 异常死锁） */
#endif

/* ---------------------------------------------------------------- */
/**
 * 功能：配置 SPI1 相关 GPIO（SCK/MOSI 复用推挽，MISO 浮空输入）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Spi_GpioConfig(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI1);

    /* PB13 SCK / PB15 MOSI = 复用推挽; PB14 MISO = 浮空输入 */
    gpio_init(BSP_SPI_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              BSP_SPI_SCK_PIN | BSP_SPI_MOSI_PIN);
    gpio_init(BSP_SPI_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,
              BSP_SPI_MISO_PIN);
}

/**
 * 功能：配置并使能 SPI1 主模式参数（全双工、Mode0、8bit、MSB、分频）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Spi_PeriphConfig(void)
{
    spi_parameter_struct spi_init_struct;

    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;  /* Mode0 */
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = BSP_SPI_PRESCALER;       /* APB1/8 ≤20MHz */
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(BSP_SPI_INSTANCE, &spi_init_struct);

    spi_enable(BSP_SPI_INSTANCE);
}

/* ---------------------------------------------------------------- */
/**
 * 功能：初始化 SPI1 通用总线（GPIO + 主模式参数）。幂等。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Spi_Status_T，BSP_SPI_OK 表示成功。
 */
Bsp_Spi_Status_T Bsp_Spi_Init(void)
{
    if (s_spi_initialized) {
        return BSP_SPI_OK;
    }
    Bsp_Spi_GpioConfig();
    Bsp_Spi_PeriphConfig();
    s_spi_initialized = 1U;
    return BSP_SPI_OK;
}

/**
 * 功能：查询 SPI1 总线是否已初始化。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，非 0=已初始化，0=未初始化。
 */
int Bsp_Spi_IsInitialized(void)
{
    return (int)s_spi_initialized;
}

/**
 * 功能：全双工收发一个字节（阻塞等待发送缓冲空与接收非空）。
 * 输入参数：tx —— 待发送字节。
 * 输出参数：无
 * 返回值：uint8_t，同时收到的字节。
 */
uint8_t Bsp_Spi_TransferByte(uint8_t tx)
{
    uint32_t timeout;

    /* 全双工: 等发送缓冲空→发→等接收非空→读 */
    timeout = BSP_SPI_XFER_TIMEOUT;
    while (RESET == spi_i2s_flag_get(BSP_SPI_INSTANCE, SPI_FLAG_TBE)) {
        if (--timeout == 0U) { s_spi_xfer_fail = 1U; return 0xFFU; }
    }
    spi_i2s_data_transmit(BSP_SPI_INSTANCE, tx);
    timeout = BSP_SPI_XFER_TIMEOUT;
    while (RESET == spi_i2s_flag_get(BSP_SPI_INSTANCE, SPI_FLAG_RBNE)) {
        if (--timeout == 0U) { s_spi_xfer_fail = 1U; return 0xFFU; }
    }
    s_spi_xfer_fail = 0U;
    return (uint8_t)spi_i2s_data_receive(BSP_SPI_INSTANCE);
}

/**
 * 功能：查询最近一次 Bsp_Spi_TransferByte 是否因超时失败。
 * 返回值：int，非 0=失败，0=成功。
 */
int Bsp_Spi_XferFailed(void)
{
    return (int)s_spi_xfer_fail;
}

/**
 * 功能：全双工收发一段缓冲（只读时发 dummy 0xFF，只写时忽略接收）。
 * 输入参数：tx —— 发送缓冲指针（可空）；len —— 收发字节数。
 * 输出参数：rx —— 接收缓冲指针（可空，写入收到的数据）。
 * 返回值：Bsp_Spi_Status_T，BSP_SPI_OK 成功，BSP_SPI_ERR_PARAM 表示 len 为 0。
 */
Bsp_Spi_Status_T Bsp_Spi_TransferBuffer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint16_t i;
    if (len == 0U) {
        return BSP_SPI_ERR_PARAM;
    }
    for (i = 0U; i < len; i++) {
        uint8_t out = (tx != NULL) ? tx[i] : 0xFFU;   /* 只读时发 dummy 0xFF */
        uint8_t in  = Bsp_Spi_TransferByte(out);
        if (Bsp_Spi_XferFailed()) {
            return BSP_SPI_ERR_TIMEOUT;
        }
        if (rx != NULL) {
            rx[i] = in;
        }
    }
    return BSP_SPI_OK;
}
