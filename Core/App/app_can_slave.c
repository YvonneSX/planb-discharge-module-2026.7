/**
 * @file    app_can_slave.c
 * @brief   泄放 CAN 从站接入层实现（基于 canx_proto 通用引擎）
 *
 * 见 app_can_slave.h。本模块用 canx_proto 从站模式承载泄放 CAN 协议：
 *   decode_rx 过滤主站 ID 并对写类命令落库；on_request 按命令码组应答。
 * 命令语义与《泄放模块概要设计》§4.3 一致（0x01~0x33、0xA5）。
 */

#include "app_can_slave.h"
#include "canx_proto_port.h" /* CanxPort_Init */
#include "app_comm.h"        /* 复用 AppComm_PackSnapshot 组读状态应答 */
#include "app_monitor.h"
#include "app_protect.h"
#include "app_discharge.h"
#include "app_blackbox.h"
#include "uart_log.h"

#define SLV_TAG "CANSLV"

/* 应答状态码（与 app_comm 一致，§4.3.1） */
#define SLV_ST_OK           0x00U
#define SLV_ST_PARAM_ERR    0x01U
#define SLV_ST_NOT_ALLOWED  0x02U
#define SLV_ST_STORE_FAIL   0x03U
#define SLV_ST_UNKNOWN      0xEEU

/* 泄放从站本通道单节点：node_id 取自身 CAN ID，dispatch=SINGLE，主站过滤在 decode_rx */
static CanxEngine_T s_eng;
static bool         s_inited = false;

/* —— 小工具：中性帧负载小端读写 —— */
static void slv_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

/**
 * 功能：合成完整运行快照（MON 量测 + DCH 会话 + PROT 状态/故障），供读状态应答。
 * 输入参数：rt —— 接收快照的结构体指针。
 * 输出参数：*rt。返回值：无。
 */
static void slv_build_rtdata(App_RtData_T *rt)
{
    AppMon_GetSnapshot(rt);
    rt->duty_pct     = AppDch_GetDuty();
    rt->dch_on       = AppDch_IsOn() ? 1U : 0U;
    rt->dch_total_ms = AppDch_GetTotalMs();
    rt->dch_state    = AppDch_GetLevel();   /* 泄放级别 L0/L1/L2 */
    rt->fault_bits   = AppProt_GetFaults();
    rt->ov2_count    = AppProt_GetOv2Count();
    rt->otp_count    = AppProt_GetOtpCount();
}

/**
 * 功能：从站请求帧解析回调——按配置的主站 ID 过滤请求帧。
 * 输入参数：e —— 引擎；f —— 请求帧（中性帧）；now_ms —— 时间戳。
 * 输出参数：无。
 * 返回值：1=命中本节点（引擎随后调 on_request 应答）；0=非绑定主站/忽略；<0=解析失败。
 * 实现原理：dispatch=SINGLE 时本通道帧全进本回调，此处做软件过滤：
 *   仅受理标准帧且发送方为当前绑定主站的请求。
 */
static int slv_decode_rx(CanxEngine_T *e, const CanxFrame_T *f, uint32_t now_ms)
{
    (void)e; (void)now_ms;

    if (f->ide != 0U || f->len < 1U) {
        return 0;   /* 非标准帧或空帧：忽略 */
    }
    bool from_master = (f->id == (uint32_t)Bsp_Config_GetMasterId());
    if (!from_master) {
        return 0;   /* 非绑定主站：静默丢弃，不应答 */
    }
    return 1;       /* 命中：交 on_request 处理与应答（写类落库也在 on_request 内做，保持单点） */
}

/* —— 应答封装：构造应答帧基础（源为自身 ID，回显命令码） —— */
static void slv_init_resp(CanxFrame_T *resp, uint8_t cmd)
{
    CanxProto_InitTxFrame(&s_eng, resp, (uint32_t)Bsp_Config_GetSelfId());
    resp->len     = 1U;
    resp->data[0] = cmd;
}

/* —— 回一条状态应答（data[1]=状态码） —— */
static int slv_reply_status(CanxEngine_T *e, uint8_t cmd, uint8_t status)
{
    CanxFrame_T resp;
    slv_init_resp(&resp, cmd);
    resp.data[1] = status;
    resp.len = 2U;
    return (e->bsp.send(e->channel, &resp) == CANX_BUS_OK) ? 1 : -1;
}

/* —— 读状态 0x20~0x24：复用 AppComm_PackSnapshot（基于 Bsp_Can_Frame_T），再转中性帧发出 —— */
static int slv_handle_read_status(CanxEngine_T *e, uint8_t cmd)
{
    App_RtData_T    rt;
    Bsp_Can_Frame_T bsp_resp;
    CanxFrame_T     resp;
    uint8_t         i;

    slv_build_rtdata(&rt);
    if (!AppComm_PackSnapshot(cmd, &rt, &bsp_resp)) {
        return slv_reply_status(e, cmd, SLV_ST_UNKNOWN);
    }
    /* Bsp_Can_Frame_T → 中性帧 */
    slv_init_resp(&resp, cmd);
    resp.len = bsp_resp.dlc;
    for (i = 0U; i < bsp_resp.dlc && i < CANX_MAX_DATA; i++) {
        resp.data[i] = bsp_resp.data[i];
    }
    return (e->bsp.send(e->channel, &resp) == CANX_BUS_OK) ? 1 : -1;
}

/* —— 控制命令 0x10/0x30/0x31/0x32/0x33/0x34 —— */
static int slv_handle_control(CanxEngine_T *e, const CanxFrame_T *f)
{
    const uint8_t cmd = f->data[0];

    switch (cmd) {
    case 0x10U: {   /* 强制全开（受冷却闭锁约束） */
        bool en = (f->len >= 2U) ? (f->data[1] != 0U) : true;
        return slv_reply_status(e, cmd, AppDch_ForceFull(en) ? SLV_ST_OK : SLV_ST_NOT_ALLOWED);
    }
    case 0x30U: {   /* 泄放允许/禁止 */
        bool allow = (f->len >= 2U) ? (f->data[1] != 0U) : true;
        AppDch_SetAllowed(allow);
        return slv_reply_status(e, cmd, SLV_ST_OK);
    }
    case 0x31U: {   /* 清除故障锁存，回 fault_bits */
        uint16_t remain = AppProt_ClearFaults();
        CanxFrame_T resp;
        slv_init_resp(&resp, cmd);
        slv_put_u16(&resp.data[1], remain);
        resp.len = 3U;
        return (e->bsp.send(e->channel, &resp) == CANX_BUS_OK) ? 1 : -1;
    }
    case 0x32U:     /* 保存配置到工作区 */
        return slv_reply_status(e, cmd,
            (Bsp_Config_Save() == BSP_CFG_OK) ? SLV_ST_OK : SLV_ST_STORE_FAIL);
    case 0x33U:     /* 恢复原厂 */
        if (Bsp_Config_RestoreFromFactory() == BSP_CFG_OK) {
            (void)AppBbx_LogConfigChange(APP_CFG_SRC_RESTORE);
            return slv_reply_status(e, cmd, SLV_ST_OK);
        }
        return slv_reply_status(e, cmd, SLV_ST_STORE_FAIL);
    case 0x34U:     /* 软复位：先应答，再收尾，最后复位 */
        (void)slv_reply_status(e, cmd, SLV_ST_OK);
        (void)AppBbx_ShutdownClean();
        NVIC_SystemReset();
        return 1;   /* 不会执行到此 */
    default:
        return 0;   /* 不属本组 */
    }
}

/**
 * 功能：从站被动应答回调——按命令码分派读状态/控制命令并组应答帧发送。
 * 输入参数：e —— 引擎；req —— 请求帧；now_ms —— 时间戳。
 * 输出参数：无。
 * 返回值：>0 已应答；0 不应答；<0 应答失败（引擎置 INVALID_FRAME）。
 * 实现原理：读状态 0x20~0x24 复用 AppComm_PackSnapshot；控制 0x10/0x3x 调 PROT/DCH/Config；
 *   读写配置 0x01~0x07/0x11~0x17 当前回 UNKNOWN（待后续接 Bsp_Config_Get/Set，与 app_comm 对齐）。
 */
static int slv_on_request(CanxEngine_T *e, const CanxFrame_T *req, uint32_t now_ms)
{
    const uint8_t cmd = req->data[0];
    (void)now_ms;

    if (cmd >= 0x20U && cmd <= 0x24U) {
        return slv_handle_read_status(e, cmd);
    }
    if (cmd == 0x10U || (cmd >= 0x30U && cmd <= 0x34U)) {
        return slv_handle_control(e, req);
    }
    /* 尚未实现的命令统一返回 UNKNOWN。 */
    return slv_reply_status(e, cmd, SLV_ST_UNKNOWN);
}

/* 从站 Ops 表：只填 decode_rx + on_request。 */
static const CanxOps_T SLAVE_OPS = {
    .decode_rx         = slv_decode_rx,
    .handshake_send    = NULL,
    .realtime_step     = NULL,
    .on_operable       = NULL,
    .on_request        = slv_on_request,
    .has_handshake     = 0U,
    .handshake_need_ok = 0U,
};

/**
 * 功能：初始化泄放 CAN 从站——接收桥接 + canx_proto 从站引擎（单节点 CAN-ID 寻址）。
 * 输入/输出参数：无。返回值：无。
 */
void AppCanSlave_Init(void)
{
    CanxXferProfile_T p = CanxProto_ProfileClassic8();
    p.dispatch = CANX_DISPATCH_SINGLE;   /* 单节点：本通道帧全入 decode_rx，主站过滤在内 */

    CanxPort_Init();                     /* 复位环形缓冲 + 注册 RX 中断回调（须 Bsp_Can_Init 后） */
    (void)CanxProto_InitSlave(&s_eng, 0U /*ch*/, Bsp_Config_GetSelfId(), 0U /*peer*/,
                              &p, NULL /*默认BSP*/, &SLAVE_OPS, NULL);
    s_eng.comm_lost_ms = 1000U;          /* 主站静默 >1s 判掉线 */
    s_inited = true;
    UART_LOGI(SLV_TAG, "init done (canx_proto slave)");
}

/**
 * 功能：主循环每圈驱动从站引擎（收帧/派发/被动应答/超时检测）。
 * 输入参数：now_ms —— 当前毫秒时间戳。输出/返回值：无。
 */
void AppCanSlave_Task(uint32_t now_ms)
{
    if (!s_inited) { return; }
    (void)CanxProto_CycleTask(&s_eng, now_ms);
}

/**
 * 功能：发送上电完成通告帧（0xA5 0x5A 0x5A 0xFF×5），无条件一次。
 * 输入/输出参数：无。返回值：无。
 */
void AppCanSlave_SendBootAnnounce(void)
{
    CanxFrame_T f;
    CanxProto_InitTxFrame(&s_eng, &f, (uint32_t)Bsp_Config_GetSelfId());
    f.len = 8U;
    f.data[0] = 0xA5U; f.data[1] = 0x5AU; f.data[2] = 0x5AU; f.data[3] = 0xFFU;
    f.data[4] = 0xFFU; f.data[5] = 0xFFU; f.data[6] = 0xFFU; f.data[7] = 0xFFU;
    (void)s_eng.bsp.send(s_eng.channel, &f);
}

/* ====================================================================
 * 测试钩子（仅 BSP_UNIT_TEST）：暴露从站 Ops，供单测以 mock bsp 驱动真实回调
 * ==================================================================== */
#if defined(BSP_UNIT_TEST)
/**
 * 功能：返回从站 Ops 表指针，供单测用 mock bsp + 本 Ops 验证 decode_rx/on_request。
 * 返回值：&SLAVE_OPS。
 */
const CanxOps_T *AppCanSlave_TestGetOps(void)
{
    return &SLAVE_OPS;
}
#endif
