/**
 * @file    bsp_uart_log.c
 * @brief   调试串口 BSP 驱动 (GD32F303CCT6)
 *
 * USART0 Remap → PB6(TX) / PB7(RX), 115200 8N1, 轮询发送。
 * 基于 PlanbPro GD32H75E bsp_uart_log.c 适配 GD32F30x API。
 *
 * @version 1.0.0
 * @date    2026-06-11
 */

#define BSP_USE_UART_LOG
#include "bsp.h"

/* ----------------------------------------------------------------
 * Module state
 * ---------------------------------------------------------------- */

static volatile uint8_t s_uart_initialized = 0U;

/* 波特率宏字符串化 */
#define BSP_UART_LOG_XSTR(x)    #x
#define BSP_UART_LOG_STR(x)     BSP_UART_LOG_XSTR(x)

/* ----------------------------------------------------------------
 * 内部函数
 * ---------------------------------------------------------------- */

/**
 * 功能：配置 USART0 TX/RX GPIO 为复用推挽并启用 Remap（PB6/PB7）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_UartLog_GpioConfig(void)
{
    /* 使能 GPIOB 和 AF 时钟 */
    rcu_periph_clock_enable(BSP_UART_LOG_RCU_TX);
    rcu_periph_clock_enable(RCU_AF);

    /* USART0 Remap: 从 PA9/PA10 映射到 PB6/PB7 */
    gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);

    /* TX: PB6, 复用推挽输出, 50MHz */
    gpio_init(BSP_UART_LOG_TX_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              BSP_UART_LOG_TX_PIN);

    /* RX: PB7, 浮空输入 */
    gpio_init(BSP_UART_LOG_RX_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,
              BSP_UART_LOG_RX_PIN);
}

/**
 * 功能：配置 USART0 波特率/数据位/停止位/校验并使能收发。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_UartLog_PeriphConfig(void)
{
    rcu_periph_clock_enable(BSP_UART_LOG_RCU_UART);

    usart_deinit(BSP_UART_LOG_INSTANCE);

    usart_baudrate_set(BSP_UART_LOG_INSTANCE, BSP_UART_LOG_BAUD);
    usart_word_length_set(BSP_UART_LOG_INSTANCE, USART_WL_8BIT);
    usart_stop_bit_set(BSP_UART_LOG_INSTANCE, USART_STB_1BIT);
    usart_parity_config(BSP_UART_LOG_INSTANCE, USART_PM_NONE);

    usart_hardware_flow_rts_config(BSP_UART_LOG_INSTANCE, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(BSP_UART_LOG_INSTANCE, USART_CTS_DISABLE);

    usart_receive_config(BSP_UART_LOG_INSTANCE, USART_RECEIVE_ENABLE);
    usart_transmit_config(BSP_UART_LOG_INSTANCE, USART_TRANSMIT_ENABLE);
    usart_enable(BSP_UART_LOG_INSTANCE);
}

/**
 * 功能：轮询发送单字节，阻塞等待 TBE 与 TC 标志（带超时）。
 * 输入参数：ch —— 待发送字节。
 * 输出参数：无
 * 返回值：int，BSP_UART_LOG_OK 成功，BSP_UART_LOG_ERR_TIMEOUT 超时。
 */
static int Bsp_UartLog_TxByte(uint8_t ch)
{
    uint32_t timeout;

    timeout = BSP_UART_LOG_TX_TIMEOUT;
    while (RESET == usart_flag_get(BSP_UART_LOG_INSTANCE, USART_FLAG_TBE)) {
        if (--timeout == 0U) {
            return BSP_UART_LOG_ERR_TIMEOUT;
        }
    }
    usart_data_transmit(BSP_UART_LOG_INSTANCE, (uint32_t)ch);

    timeout = BSP_UART_LOG_TX_TIMEOUT;
    while (RESET == usart_flag_get(BSP_UART_LOG_INSTANCE, USART_FLAG_TC)) {
        if (--timeout == 0U) {
            return BSP_UART_LOG_ERR_TIMEOUT;
        }
    }
    return BSP_UART_LOG_OK;
}

/**
 * 功能：绕过初始化门控，按显式长度发送原始数据（内部 banner 用）。
 * 输入参数：buf —— 数据指针；len —— 字节数。
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_UartLog_SendRaw(const void *buf, uint32_t len)
{
    if (buf == NULL || len == 0U) {
        return;
    }

    const uint8_t *p = (const uint8_t *)buf;
    uint32_t i;

    for (i = 0U; i < len; i++) {
        if (Bsp_UartLog_TxByte(p[i]) != BSP_UART_LOG_OK) {
            break;
        }
    }
}

/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

/**
 * 功能：初始化调试串口（GPIO + USART0 参数）并打印启动 banner。幂等。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_UartLog_Status_T，BSP_UART_LOG_OK 表示成功。
 */
Bsp_UartLog_Status_T Bsp_UartLog_Init(void)
{
    if (s_uart_initialized) {
        return BSP_UART_LOG_OK;
    }

    Bsp_UartLog_GpioConfig();
    Bsp_UartLog_PeriphConfig();

    static const char k_banner_crlf[]   = "\r\n";
    static const char k_banner_init[]   = "[BSP_UART_LOG] init OK @ "
                                          BSP_UART_LOG_STR(BSP_UART_LOG_BAUD)
                                          " 8N1 (PB6/PB7)\r\n";

    Bsp_UartLog_SendRaw(k_banner_crlf, sizeof(k_banner_crlf) - 1U);
    Bsp_UartLog_SendRaw(k_banner_init, sizeof(k_banner_init) - 1U);

    s_uart_initialized = 1U;
    return BSP_UART_LOG_OK;
}

/**
 * 功能：反初始化调试串口（关闭并复位 USART0）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_UartLog_Deinit(void)
{
    if (!s_uart_initialized) {
        return;
    }
    usart_disable(BSP_UART_LOG_INSTANCE);
    usart_deinit(BSP_UART_LOG_INSTANCE);
    s_uart_initialized = 0U;
}

/**
 * 功能：查询调试串口是否已初始化。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，非 0=已初始化，0=未初始化。
 */
int Bsp_UartLog_IsInitialized(void)
{
    return (int)s_uart_initialized;
}

/**
 * 功能：轮询发送一段数据（未初始化/参数非法则不发送）。
 * 输入参数：buf —— 数据指针；len —— 字节数。
 * 输出参数：无
 * 返回值：int，实际发送的字节数。
 */
int Bsp_UartLog_Write(const void *buf, uint32_t len)
{
    if (!s_uart_initialized || buf == NULL || len == 0U) {
        return 0;
    }

    const uint8_t *p = (const uint8_t *)buf;
    uint32_t i;
    for (i = 0U; i < len; i++) {
        if (Bsp_UartLog_TxByte(p[i]) != BSP_UART_LOG_OK) {
            break;
        }
    }
    return (int)i;
}

/**
 * 功能：发送单个字符。
 * 输入参数：ch —— 待发送字节。
 * 输出参数：无
 * 返回值：int，BSP_UART_LOG_OK 成功，BSP_UART_LOG_ERR_INIT 未初始化，超时返回错误码。
 */
int Bsp_UartLog_Putc(uint8_t ch)
{
    if (!s_uart_initialized) {
        return BSP_UART_LOG_ERR_INIT;
    }
    return Bsp_UartLog_TxByte(ch);
}

/**
 * 功能：非阻塞读取一个接收字节（无数据则立即返回）。
 * 输入参数：out —— 接收字节的指针。
 * 输出参数：*out —— 写入读到的字节。
 * 返回值：int，1=读到一字节，0=当前无数据，负值=未初始化/参数错。
 */
int Bsp_UartLog_ReadByte(uint8_t *out)
{
    if (!s_uart_initialized || out == NULL) {
        return BSP_UART_LOG_ERR_PARAM;
    }
    if (RESET == usart_flag_get(BSP_UART_LOG_INSTANCE, USART_FLAG_RBNE)) {
        return 0;
    }
    *out = (uint8_t)(usart_data_receive(BSP_UART_LOG_INSTANCE) & 0xFFU);
    return 1;
}
