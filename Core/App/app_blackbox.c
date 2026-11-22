/**
 * @file    app_blackbox.c
 * @brief   HLD-MOD-BBX 黑匣子模块实现（接口骨架；FRAM 段布局后续落地）
 *
 * 本版仅落地接口契约与 pwr_seq/异常掉电判定骨架，便于上层编排与联调。
 * 实际段布局（统计/事件环/趋势环、magic+CRC+ping-pong）按
 * 《泄放模块FRAM存储分配设计》后续逐段实现，标注 TODO。
 */

#include "app_blackbox.h"
#include "uart_log.h"

#define BBX_TAG "BBX"

static bool     s_inited = false;
static uint16_t s_pwr_seq = 0U;

/**
 * 功能：初始化黑匣子模块（校验 FRAM 就绪、递增上电序号 pwr_seq、置初始化标志）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：App_Bbx_Status_T，APP_BBX_OK 成功，APP_BBX_ERR_FRAM 表示 FRAM 未就绪。
 */
App_Bbx_Status_T AppBbx_Init(void)
{
    if (!Bsp_Fram_IsInitialized()) {
        UART_LOGE(BBX_TAG, "fram not ready");
        return APP_BBX_ERR_FRAM;
    }
    /* TODO: 读复位原因(evt=7)、判 session_active 异常掉电、递增 pwr_seq、
     *       校验各段 magic/CRC（损坏重建）。当前仅置初始化标志。 */
    s_pwr_seq++;
    s_inited = true;
    UART_LOGI(BBX_TAG, "init done (skeleton), pwr_seq=%u", (unsigned)s_pwr_seq);
    return APP_BBX_OK;
}

/**
 * 功能：执行掉电前的干净收尾（清会话活动标志、刷写统计段）。当前为骨架。
 * 输入参数：无
 * 输出参数：无
 * 返回值：App_Bbx_Status_T，操作状态（未初始化返回 ERR_PARAM；骨架返回 NOT_IMPL）。
 */
App_Bbx_Status_T AppBbx_ShutdownClean(void)
{
    if (!s_inited) { return APP_BBX_ERR_PARAM; }
    /* TODO: 清 session_active 标志 + 刷写统计段。 */
    return APP_BBX_NOT_IMPL;
}

/**
 * 功能：周期记录运行快照到趋势环（带限速）。当前为骨架。
 * 输入参数：snap —— 运行量测快照指针。
 * 输出参数：无
 * 返回值：App_Bbx_Status_T，操作状态（参数非法返回 ERR_PARAM；骨架返回 NOT_IMPL）。
 */
App_Bbx_Status_T AppBbx_LogPeriodic(const App_RtData_T *snap)
{
    if (!s_inited)      { return APP_BBX_ERR_PARAM; }
    if (snap == NULL)   { return APP_BBX_ERR_PARAM; }
    /* TODO: 写趋势环（限速）。 */
    return APP_BBX_NOT_IMPL;
}

/**
 * 功能：记录一条事件到事件环并打趋势锚点。当前为骨架。
 * 输入参数：evt —— 事件类型；snap —— 事件发生时的运行快照指针。
 * 输出参数：无
 * 返回值：App_Bbx_Status_T，操作状态（未初始化返回 ERR_PARAM；骨架返回 NOT_IMPL）。
 */
App_Bbx_Status_T AppBbx_LogEvent(App_EvtType_T evt, const App_RtData_T *snap)
{
    if (!s_inited) { return APP_BBX_ERR_PARAM; }
    (void)snap;
    /* TODO: 写事件环 + 打趋势锚点（N+K）。 */
    UART_LOGI(BBX_TAG, "log event=%u (skeleton)", (unsigned)evt);
    return APP_BBX_NOT_IMPL;
}

/**
 * 功能：记录一条配置变更审计（evt=8，含来源与前后摘要）。当前为骨架。
 * 输入参数：src —— 配置变更来源。
 * 输出参数：无
 * 返回值：App_Bbx_Status_T，操作状态（未初始化返回 ERR_PARAM；骨架返回 NOT_IMPL）。
 */
App_Bbx_Status_T AppBbx_LogConfigChange(App_CfgSource_T src)
{
    if (!s_inited) { return APP_BBX_ERR_PARAM; }
    (void)src;
    /* TODO: 写 evt=8 审计记录（变更前后摘要 + 来源）。 */
    return APP_BBX_NOT_IMPL;
}

/**
 * 功能：读取统计/极值段。当前为骨架。
 * 输入参数：buf —— 接收缓冲区指针；len —— 缓冲区长度。
 * 输出参数：*buf —— 写入统计数据。
 * 返回值：App_Bbx_Status_T，操作状态（参数非法返回 ERR_PARAM；骨架返回 NOT_IMPL）。
 */
App_Bbx_Status_T AppBbx_ReadStats(void *buf, uint16_t len)
{
    if (buf == NULL || len == 0U) { return APP_BBX_ERR_PARAM; }
    /* TODO: 读统计/极值段。 */
    return APP_BBX_NOT_IMPL;
}

/**
 * 功能：按索引读取一条事件记录。当前为骨架。
 * 输入参数：index —— 事件索引；buf —— 接收缓冲区指针；len —— 缓冲区长度。
 * 输出参数：*buf —— 写入事件记录。
 * 返回值：App_Bbx_Status_T，操作状态（参数非法返回 ERR_PARAM；骨架返回 NOT_IMPL）。
 */
App_Bbx_Status_T AppBbx_ReadEvent(uint16_t index, void *buf, uint16_t len)
{
    if (buf == NULL || len == 0U) { return APP_BBX_ERR_PARAM; }
    (void)index;
    return APP_BBX_NOT_IMPL;
}

/**
 * 功能：按页读取趋势环数据。当前为骨架。
 * 输入参数：page —— 趋势页号；buf —— 接收缓冲区指针；len —— 缓冲区长度。
 * 输出参数：*buf —— 写入趋势页数据。
 * 返回值：App_Bbx_Status_T，操作状态（参数非法返回 ERR_PARAM；骨架返回 NOT_IMPL）。
 */
App_Bbx_Status_T AppBbx_ReadTrendPage(uint16_t page, void *buf, uint16_t len)
{
    if (buf == NULL || len == 0U) { return APP_BBX_ERR_PARAM; }
    (void)page;
    return APP_BBX_NOT_IMPL;
}

/**
 * 功能：清空黑匣子各段并重建 magic。当前为骨架。
 * 输入参数：无
 * 输出参数：无
 * 返回值：App_Bbx_Status_T，操作状态（未初始化返回 ERR_PARAM；骨架返回 NOT_IMPL）。
 */
App_Bbx_Status_T AppBbx_ClearAll(void)
{
    if (!s_inited) { return APP_BBX_ERR_PARAM; }
    /* TODO: 清各段并重建 magic。 */
    return APP_BBX_NOT_IMPL;
}
