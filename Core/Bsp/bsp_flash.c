/**
 * @file    bsp_flash.c
 * @brief   片上 Flash BSP 驱动 (GD32F303CCT6, HD 系列 2KB/页)
 *
 * 用于配置参数持久化: 使用 Flash 最后一页 (0x0803F800, 2KB) 存储。
 * Flash 须先擦除(全1)后才能写; 写以 32bit 字为单位 (fmc_word_program)。
 * 本模块为通用 Flash 原语层 (擦/写/读), 不含配置语义 (配置见 bsp_config)。
 *
 * 对应需求: REQ-NF-003 (CAN 波特率等参数可配置切换) 的持久化底层,
 *           REQ-CON-003 (软件资源占用须在 Flash/RAM 范围内) 约束下使用最后一页
 *
 * @version 1.0.0
 * @date    2026-06-13
 */

#define BSP_USE_FLASH
#include "bsp.h"

/* ---------------------------------------------------------------- */
/**
 * 功能：校验访问地址合法性（4 字节对齐且落在配置页范围内，防误操作代码区）。
 * 输入参数：addr —— 起始地址；len —— 访问长度（字节）。
 * 输出参数：无
 * 返回值：int，1=合法，0=非法。
 */
static int Bsp_Flash_AddrValid(uint32_t addr, uint32_t len)
{
    if ((addr & 0x3U) != 0U) {            /* 4 字节对齐 */
        return 0;
    }
    if (addr < BSP_FLASH_CFG_ADDR) {
        return 0;
    }
    if ((uint32_t)(addr + len) > (BSP_FLASH_CFG_ADDR + BSP_FLASH_PAGE_SIZE)) {
        return 0;                          /* 越出最后一页 */
    }
    return 1;
}

/* ---------------------------------------------------------------- */
/**
 * 功能：擦除配置页（仅允许擦配置页地址，防误擦代码区）。
 * 输入参数：page_addr —— 页起始地址（须等于配置页地址）。
 * 输出参数：无
 * 返回值：Bsp_Flash_Status_T，OK 成功，ERR_PARAM 地址非配置页，ERR_ERASE 擦除失败。
 */
Bsp_Flash_Status_T Bsp_Flash_ErasePage(uint32_t page_addr)
{
    fmc_state_enum st;

    if (page_addr != BSP_FLASH_CFG_ADDR) {
        return BSP_FLASH_ERR_PARAM;        /* 仅允许擦配置页, 防误擦代码区 */
    }
    fmc_unlock();
    st = fmc_page_erase(page_addr);
    fmc_lock();
    return (st == FMC_READY) ? BSP_FLASH_OK : BSP_FLASH_ERR_ERASE;
}

/**
 * 功能：以 32bit 字为单位写入 Flash（小端组字；须先擦除，长度须 4 字节整数倍）。
 * 输入参数：addr —— 起始地址（4 字节对齐）；buf —— 源数据指针；len —— 字节数（4 的倍数）。
 * 输出参数：无
 * 返回值：Bsp_Flash_Status_T，OK 成功，ERR_PARAM 参数非法，ERR_RANGE 越界，ERR_WRITE 写失败。
 */
Bsp_Flash_Status_T Bsp_Flash_Write(uint32_t addr, const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    uint32_t i;
    fmc_state_enum st;

    if (buf == NULL || len == 0U || (len & 0x3U) != 0U) {
        return BSP_FLASH_ERR_PARAM;        /* 长度须 4 字节整数倍 */
    }
    if (!Bsp_Flash_AddrValid(addr, len)) {
        return BSP_FLASH_ERR_RANGE;
    }

    fmc_unlock();
    for (i = 0U; i < len; i += 4U) {
        /* 小端组字 */
        uint32_t word = (uint32_t)p[i]
                      | ((uint32_t)p[i + 1U] << 8)
                      | ((uint32_t)p[i + 2U] << 16)
                      | ((uint32_t)p[i + 3U] << 24);
        st = fmc_word_program(addr + i, word);
        if (st != FMC_READY) {
            fmc_lock();
            return BSP_FLASH_ERR_WRITE;
        }
    }
    fmc_lock();
    return BSP_FLASH_OK;
}

/**
 * 功能：从 Flash 读取一段数据（Flash 已映射到地址空间，直接按字节读）。
 * 输入参数：addr —— 起始地址；len —— 读取字节数。
 * 输出参数：buf —— 接收缓冲区，写入读出的数据。
 * 返回值：Bsp_Flash_Status_T，OK 成功，ERR_PARAM 参数非法，ERR_RANGE 越界。
 */
Bsp_Flash_Status_T Bsp_Flash_Read(uint32_t addr, void *buf, uint32_t len)
{
    uint8_t *p = (uint8_t *)buf;
    uint32_t i;

    if (buf == NULL || len == 0U) {
        return BSP_FLASH_ERR_PARAM;
    }
    if (!Bsp_Flash_AddrValid(addr, (len + 3U) & ~0x3U)) {
        return BSP_FLASH_ERR_RANGE;
    }
    /* Flash 已映射到地址空间, 直接读 */
    for (i = 0U; i < len; i++) {
        p[i] = *(volatile const uint8_t *)(addr + i);
    }
    return BSP_FLASH_OK;
}
