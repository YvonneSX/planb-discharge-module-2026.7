/**
 * @file    app_comm.c
 * @brief   HLD-MOD-COMM 通信模块实现（事件驱动从站；首版骨架）
 *
 * 已落地：RX 中断入缓存 + 主循环分派框架、上电通告帧、读状态命令
 * (0x20~0x24) 从 RAM 打包、控制命令处理和配置读写。
 * 待细化（TODO）：应答状态码和接收环形缓冲。
 */

#include "app_comm.h"
#include "app_monitor.h"
#include "app_protect.h"
#include "app_discharge.h"
#include "app_blackbox.h"
#include "uart_log.h"

#define COMM_TAG "COMM"

/* 应答状态码（§4.3.1） */
#define COMM_ST_OK          0x00U
#define COMM_ST_PARAM_ERR   0x01U
#define COMM_ST_NOT_ALLOWED 0x02U
#define COMM_ST_STORE_FAIL  0x03U
#define COMM_ST_UNKNOWN     0xEEU

/* 单帧接收缓存 + 忙标志（ISR 写、主循环读）。 */
static volatile Bsp_Can_Frame_T s_rx_frame;
static volatile bool            s_rx_pending = false;
static bool                     s_inited = false;

/**
 * 功能：以小端序将 16 位无符号数写入缓冲区。
 * 输入参数：p —— 目标缓冲区指针（≥2 字节）；v —— 待写入的 16 位值。
 * 输出参数：p[0..1] —— 小端写入结果。
 * 返回值：无
 */
static void comm_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

/**
 * 功能：以小端序从缓冲区读取 16 位无符号数。
 * 输入参数：p —— 源缓冲区指针（≥2 字节）。
 * 输出参数：无
 * 返回值：uint16_t，解析出的 16 位值。
 */
static uint16_t comm_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * 功能：合成完整运行快照（MON 量测 + PROT 状态/故障 + DCH 占空比/会话）。
 * 输入参数：rt —— 接收快照的结构体指针。
 * 输出参数：*rt —— 写入合成后的运行数据。
 * 返回值：无
 */
static void comm_build_rtdata(App_RtData_T *rt)
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
 * 功能：经 BSP CAN 发送一帧（忽略发送返回值）。
 * 输入参数：f —— 待发送的 CAN 帧指针。
 * 输出参数：无
 * 返回值：无
 */
static void comm_send(const Bsp_Can_Frame_T *f)
{
    (void)Bsp_Can_Send(f);
}

/**
 * 功能：构造应答帧基础（源为本机 self id、data[0] 回显命令码、清零数据区）。
 * 输入参数：resp —— 应答帧指针；cmd —— 回显的命令码。
 * 输出参数：*resp —— 写入初始化后的应答帧。
 * 返回值：无
 */
static void comm_init_resp(Bsp_Can_Frame_T *resp, uint8_t cmd)
{
    resp->id  = Bsp_Config_GetSelfId();
    resp->ide = 0U;
    resp->dlc = 1U;
    for (uint32_t i = 0U; i < 8U; i++) { resp->data[i] = 0U; }
    resp->data[0] = cmd;
}

/**
 * 功能：回送一条状态应答帧（data[0]=命令码、data[1]=状态码）。
 * 输入参数：cmd —— 命令码；status —— 状态码（COMM_ST_*）。
 * 输出参数：无
 * 返回值：无
 */
static void comm_reply_status(uint8_t cmd, uint8_t status)
{
    Bsp_Can_Frame_T resp;
    comm_init_resp(&resp, cmd);
    resp.data[1] = status;
    resp.dlc = 2U;
    comm_send(&resp);
}

/**
 * 功能：初始化通信模块，注册 CAN 接收中断回调并置就绪。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppComm_Init(void)
{
    s_rx_pending = false;
    (void)Bsp_Can_EnableRxIt(AppComm_OnRx);
    s_inited = true;
    UART_LOGI(COMM_TAG, "init done");
}

/**
 * 功能：CAN 接收中断回调，将收到的帧存入单帧缓存并置待处理标志（ISR 上下文，极短）。
 * 输入参数：frame —— 收到的 CAN 帧指针。
 * 输出参数：无（写入模块接收缓存）
 * 返回值：无
 */
void AppComm_OnRx(const Bsp_Can_Frame_T *frame)
{
    if (frame == NULL) { return; }
    if (s_rx_pending)  { return; }   /* 上一帧未处理完，丢弃（保持 ISR 极短） */
    s_rx_frame   = *frame;
    s_rx_pending = true;
}

/**
 * 功能：按读状态命令（0x20~0x24）将运行快照/设备信息打包为应答帧。
 * 输入参数：cmd —— 读状态命令码；snap —— 运行快照指针。
 * 输出参数：*resp —— 写入打包好的应答帧。
 * 返回值：bool，true=已打包，false=参数非法或命令不支持。
 */
bool AppComm_PackSnapshot(uint8_t cmd, const App_RtData_T *snap, Bsp_Can_Frame_T *resp)
{
    if (snap == NULL || resp == NULL) { return false; }
    comm_init_resp(resp, cmd);

    switch (cmd) {
    case 0x20U:  /* V(2)+I(2) */
        comm_put_u16(&resp->data[1], snap->vbus_0v1);
        comm_put_u16(&resp->data[3], (uint16_t)snap->ibus_0a01);
        resp->dlc = 5U;
        return true;
    case 0x21U:  /* P_inst/avg/peak */
        comm_put_u16(&resp->data[1], snap->p_inst_w);
        comm_put_u16(&resp->data[3], snap->p_avg_w);
        comm_put_u16(&resp->data[5], snap->p_peak_w);
        resp->dlc = 7U;
        return true;
    case 0x22U:  /* T(2)+duty(1)+state(1)+dch_on(1) */
        comm_put_u16(&resp->data[1], (uint16_t)snap->temp_0c1);
        resp->data[3] = snap->duty_pct;
        resp->data[4] = snap->dch_state;
        resp->data[5] = snap->dch_on;
        resp->dlc = 6U;
        return true;
    case 0x23U:  /* fault_bits(2)+ov2_count(2)+otp_count(2)，数据源严格为 RAM，不读 FRAM */
        comm_put_u16(&resp->data[1], snap->fault_bits);
        comm_put_u16(&resp->data[3], snap->ov2_count);
        comm_put_u16(&resp->data[5], snap->otp_count);
        resp->dlc = 7U;
        return true;
    case 0x24U:  /* 设备信息：来自代码段宏，不读 FRAM */
        resp->data[1] = (uint8_t)BSP_APP_FW_MAJOR;
        resp->data[2] = (uint8_t)BSP_APP_FW_MINOR;
        resp->data[3] = (uint8_t)BSP_APP_FW_PATCH;
        resp->data[4] = (uint8_t)BSP_APP_HW_VER;
        comm_put_u16(&resp->data[5], (uint16_t)BSP_APP_DEV_TYPE);
        resp->dlc = 7U;
        return true;
    default:
        return false;
    }
}

/**
 * 功能：保存工作配置到 FRAM 并写一条配置变更审计（evt=8）。
 * 输入参数：src —— 配置变更来源。
 * 输出参数：无
 * 返回值：bool，true=保存成功，false=保存失败。
 */
static bool comm_persist(App_CfgSource_T src)
{
    bool ok = (Bsp_Config_Save() == BSP_CFG_OK);
    if (ok) { (void)AppBbx_LogConfigChange(src); }
    return ok;
}

/**
 * 功能：处理读配置命令（0x01~0x07：波特率/CAN ID/过压/保护/报警阈值/PWM 参数），打包并发送应答。
 * 输入参数：cmd —— 读配置命令码。
 * 输出参数：无（直接发送应答帧）
 * 返回值：bool，true=已处理并应答，false=命令不属于本组。
 */
static bool comm_handle_read_cfg(uint8_t cmd)
{
    Bsp_Can_Frame_T resp;
    comm_init_resp(&resp, cmd);

    switch (cmd) {
    case 0x01U:  /* 读波特率档位 */
        resp.data[1] = (uint8_t)Bsp_Config_GetBaud();
        resp.dlc = 2U;
        break;
    case 0x02U:  /* 读自身 CAN ID */
        comm_put_u16(&resp.data[1], Bsp_Config_GetSelfId());
        resp.dlc = 3U;
        break;
    case 0x03U:  /* 读主站 CAN ID */
        comm_put_u16(&resp.data[1], Bsp_Config_GetMasterId());
        resp.dlc = 3U;
        break;
    case 0x04U:  /* 读过压阈值：启动/停止（0.1V，持久化于 FRAM） */
        comm_put_u16(&resp.data[1], Bsp_Config_GetOvpOn());
        comm_put_u16(&resp.data[3], Bsp_Config_GetOvpOff());
        resp.dlc = 5U;
        break;
    case 0x05U:  /* 读保护阈值：过温(0.1℃)/过流(0.01A)/欠压(0.1V=泄放停止 ovp_off) */
        comm_put_u16(&resp.data[1], Bsp_Config_GetOtpOff());
        comm_put_u16(&resp.data[3], Bsp_Config_GetOvc());
        comm_put_u16(&resp.data[5], Bsp_Config_GetOvpOff());
        resp.dlc = 7U;
        break;
    case 0x07U:  /* 读报警阈值：一级/二级动作值（0.1V），恢复值按固定迟滞派生 */
        comm_put_u16(&resp.data[1], Bsp_Config_GetOvpL1());
        comm_put_u16(&resp.data[3], Bsp_Config_GetOvpL2());
        resp.dlc = 5U;
        break;
    case 0x06U:  /* 读 PWM 参数：占空比上限/启动占空比/频率(Hz/10)/斜率 */
        resp.data[1] = Bsp_Config_GetPwmDutyMax();
        resp.data[2] = Bsp_Config_GetPwmDutyStart();
        comm_put_u16(&resp.data[3], (uint16_t)(Bsp_Config_GetPwmFreq() / 10U));
        comm_put_u16(&resp.data[5], (uint16_t)Bsp_Config_GetPwmDutyStep());
        resp.dlc = 7U;
        break;
    default:
        return false;
    }
    comm_send(&resp);
    return true;
}

/**
 * 功能：写 CAN ID（自身/主站），compare-before-write 后保存，复位后生效，并回送应答（data[2]=1 已写/0 未变）。
 * 输入参数：cmd —— 命令码；new_id —— 新 CAN ID；is_self —— true=自身 ID，false=主站 ID。
 * 输出参数：无（直接发送应答帧）
 * 返回值：无
 */
static void comm_write_id(uint8_t cmd, uint16_t new_id, bool is_self)
{
    Bsp_Can_Frame_T resp;
    comm_init_resp(&resp, cmd);
    resp.dlc = 3U;

    if (new_id > 0x7FFU) {        /* 标准帧 11bit 越界 */
        resp.data[1] = COMM_ST_PARAM_ERR;
        resp.data[2] = 0U;
        comm_send(&resp);
        return;
    }
    uint16_t cur = is_self ? Bsp_Config_GetSelfId() : Bsp_Config_GetMasterId();
    if (new_id == cur) {          /* 未变，不写 FRAM */
        resp.data[1] = COMM_ST_OK;
        resp.data[2] = 0U;
        comm_send(&resp);
        return;
    }
    Bsp_Cfg_Status_T st = is_self ? Bsp_Config_SetSelfId(new_id)
                                  : Bsp_Config_SetMasterId(new_id);
    if (st != BSP_CFG_OK || !comm_persist(APP_CFG_SRC_MASTER)) {
        resp.data[1] = COMM_ST_STORE_FAIL;
        resp.data[2] = 0U;
    } else {
        resp.data[1] = COMM_ST_OK;
        resp.data[2] = 1U;        /* 已写 FRAM，复位后生效 */
    }
    comm_send(&resp);
}

/**
 * 功能：处理写配置命令（0x11~0x17：CAN ID/波特率/过压/保护/报警阈值/PWM 参数），校验后写入并持久化、回送应答。
 * 输入参数：f —— 命令帧指针（data[0]=命令码，含参数与 DLC）。
 * 输出参数：无（直接发送应答帧）
 * 返回值：bool，true=已处理并应答，false=命令不属于本组。
 */
static bool comm_handle_write_cfg(const Bsp_Can_Frame_T *f)
{
    const uint8_t cmd = f->data[0];

    switch (cmd) {
    case 0x11U:  /* 设自身 ID */
        if (f->dlc < 3U) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        comm_write_id(cmd, comm_get_u16(&f->data[1]), true);
        return true;
    case 0x12U:  /* 设主站 ID */
        if (f->dlc < 3U) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        comm_write_id(cmd, comm_get_u16(&f->data[1]), false);
        return true;
    case 0x13U: {  /* 设波特率档位 */
        if (f->dlc < 2U) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        uint8_t idx = f->data[1];
        Bsp_Can_Frame_T resp;
        comm_init_resp(&resp, cmd);
        resp.dlc = 3U;
        if (idx > (uint8_t)BSP_CFG_BAUD_1M) {
            resp.data[1] = COMM_ST_PARAM_ERR; resp.data[2] = 0U;
        } else if ((uint8_t)Bsp_Config_GetBaud() == idx) {
            resp.data[1] = COMM_ST_OK;        resp.data[2] = 0U;
        } else if (Bsp_Config_SetBaud((Bsp_Cfg_Baud_T)idx) == BSP_CFG_OK
                   && comm_persist(APP_CFG_SRC_MASTER)) {
            resp.data[1] = COMM_ST_OK;        resp.data[2] = 1U;
        } else {
            resp.data[1] = COMM_ST_STORE_FAIL; resp.data[2] = 0U;
        }
        comm_send(&resp);
        return true;
    }
    case 0x14U: {  /* 设过压阈值：启动(0.1V)/停止(0.1V)。需 on>off，按安全顺序写避免瞬时违反迟滞 */
        if (f->dlc < 5U) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        uint16_t on  = comm_get_u16(&f->data[1]);
        uint16_t off = comm_get_u16(&f->data[3]);
        if (off >= on) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        bool ok;
        if (on > Bsp_Config_GetOvpOff()) {   /* 先抬高 on 不破坏 on>off 不变式 */
            ok = (Bsp_Config_SetOvpOn(on) == BSP_CFG_OK) && (Bsp_Config_SetOvpOff(off) == BSP_CFG_OK);
        } else {
            ok = (Bsp_Config_SetOvpOff(off) == BSP_CFG_OK) && (Bsp_Config_SetOvpOn(on) == BSP_CFG_OK);
        }
        if (!ok) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        bool saved = comm_persist(APP_CFG_SRC_MASTER);
        /* 同步 ADC 看门狗窗口（按当前泄放态选待机/泄放窗口，阈值随配置即时生效，不误开下限监测） */
        AppDch_RefreshWdgWindow();
        comm_reply_status(cmd, saved ? COMM_ST_OK : COMM_ST_STORE_FAIL);
        return true;
    }
    case 0x15U: {  /* 设保护阈值：过温(0.1℃)/过流(0.01A)/欠压(0.1V=ovp_off) */
        if (f->dlc < 7U) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        uint16_t otp = comm_get_u16(&f->data[1]);
        uint16_t ovc = comm_get_u16(&f->data[3]);
        uint16_t uv  = comm_get_u16(&f->data[5]);   /* 欠压复用泄放停止电压 ovp_off */
        bool ok = (Bsp_Config_SetOtpOff(otp) == BSP_CFG_OK)
               && (Bsp_Config_SetOvc(ovc)    == BSP_CFG_OK)
               && (Bsp_Config_SetOvpOff(uv)  == BSP_CFG_OK);
        if (!ok) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        bool saved15 = comm_persist(APP_CFG_SRC_MASTER);
        /* 欠压复用 ovp_off，改动后按当前泄放态重设看门狗窗口（泄放态下限阈值即时生效） */
        AppDch_RefreshWdgWindow();
        comm_reply_status(cmd, saved15 ? COMM_ST_OK : COMM_ST_STORE_FAIL);
        return true;
    }
    case 0x17U: {  /* 设报警阈值：一级/二级动作值（0.1V），恢复值按固定迟滞派生 */
        if (f->dlc < 5U) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        uint16_t l1 = comm_get_u16(&f->data[1]);
        uint16_t l2 = comm_get_u16(&f->data[3]);
        bool ok = (Bsp_Config_SetOvpL1(l1) == BSP_CFG_OK)
               && (Bsp_Config_SetOvpL2(l2) == BSP_CFG_OK);
        if (!ok) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        comm_reply_status(cmd, comm_persist(APP_CFG_SRC_MASTER) ? COMM_ST_OK : COMM_ST_STORE_FAIL);
        return true;
    }
    case 0x16U: {  /* 设 PWM 参数：上限/启动/频率(Hz·10)/斜率，越界由 BSP Set* 钳制并报错 */
        if (f->dlc < 7U) { comm_reply_status(cmd, COMM_ST_PARAM_ERR); return true; }
        uint8_t  duty_max   = f->data[1];
        uint8_t  duty_start = f->data[2];
        uint16_t freq_hz    = (uint16_t)(comm_get_u16(&f->data[3]) * 10U);
        uint8_t  step       = (uint8_t)comm_get_u16(&f->data[5]);
        bool ok = (Bsp_Config_SetPwmDutyMax(duty_max)     == BSP_CFG_OK)
               && (Bsp_Config_SetPwmDutyStart(duty_start) == BSP_CFG_OK)
               && (Bsp_Config_SetPwmFreq(freq_hz)         == BSP_CFG_OK)
               && (Bsp_Config_SetPwmDutyStep(step)        == BSP_CFG_OK);
        if (!ok) {
            comm_reply_status(cmd, COMM_ST_PARAM_ERR);
        } else {
            comm_reply_status(cmd, comm_persist(APP_CFG_SRC_MASTER) ? COMM_ST_OK : COMM_ST_STORE_FAIL);
        }
        return true;
    }
    default:
        return false;
    }
}

/**
 * 功能：处理控制命令（0x10 强制全开、0x30 泄放允许、0x31 清故障、0x32 保存、0x33 恢复原厂、0x34 软复位），回送应答。
 * 输入参数：f —— 命令帧指针（data[0]=命令码，含参数与 DLC）。
 * 输出参数：无（直接发送应答帧）
 * 返回值：bool，true=已处理并应答，false=命令不属于本组。
 */
static bool comm_handle_control(const Bsp_Can_Frame_T *f)
{
    const uint8_t cmd = f->data[0];

    switch (cmd) {
    case 0x10U: {  /* 强制全开（受冷却闭锁约束）：0 接受 / 2 拒绝 */
        bool en = (f->dlc >= 2U) ? (f->data[1] != 0U) : true;
        comm_reply_status(cmd, AppDch_ForceFull(en) ? COMM_ST_OK : COMM_ST_NOT_ALLOWED);
        return true;
    }
    case 0x30U: {  /* 泄放允许/禁止 */
        bool allow = (f->dlc >= 2U) ? (f->data[1] != 0U) : true;
        AppDch_SetAllowed(allow);
        comm_reply_status(cmd, COMM_ST_OK);
        return true;
    }
    case 0x31U: {  /* 清除故障锁存，回 fault_bits */
        uint16_t remain = AppProt_ClearFaults();
        Bsp_Can_Frame_T resp;
        comm_init_resp(&resp, cmd);
        comm_put_u16(&resp.data[1], remain);
        resp.dlc = 3U;
        comm_send(&resp);
        return true;
    }
    case 0x32U:  /* 保存配置到工作区 */
        comm_reply_status(cmd, comm_persist(APP_CFG_SRC_MASTER) ? COMM_ST_OK : COMM_ST_STORE_FAIL);
        return true;
    case 0x33U:  /* 恢复原厂：代码段默认 → FRAM 工作区（CAN ID/波特率须复位生效） */
        if (Bsp_Config_RestoreFromFactory() == BSP_CFG_OK) {
            (void)AppBbx_LogConfigChange(APP_CFG_SRC_RESTORE);
            comm_reply_status(cmd, COMM_ST_OK);
        } else {
            comm_reply_status(cmd, COMM_ST_STORE_FAIL);
        }
        return true;
    case 0x34U:  /* 软复位：先应答，再清理黑匣子会话，最后复位 */
        comm_reply_status(cmd, COMM_ST_OK);
        (void)AppBbx_ShutdownClean();
        NVIC_SystemReset();
        return true;   /* 不会执行到此 */
    default:
        return false;
    }
}

/**
 * 功能：主循环命令分派——取出已收帧，过滤非绑定主站请求，并按命令类别处理和应答。须每控制圈调用。
 * 输入参数：无（从模块接收缓存读取）
 * 输出参数：无（按需发送应答帧）
 * 返回值：无
 */
void AppComm_Dispatch(void)
{
    if (!s_inited || !s_rx_pending) { return; }

    Bsp_Can_Frame_T frame = *(const Bsp_Can_Frame_T *)&s_rx_frame;
    s_rx_pending = false;   /* 尽早释放缓存接纳下一帧 */

    if (frame.ide != 0U || frame.dlc < 1U) { return; }

    const uint8_t cmd = frame.data[0];
    const bool from_master = (frame.id == Bsp_Config_GetMasterId());
    if (!from_master) { return; }   /* 软件过滤：非绑定主站不应答 */

    /* —— 读状态（客户，仅 RAM 数据源，禁止读黑匣子 FRAM）—— */
    if (cmd >= 0x20U && cmd <= 0x24U) {
        App_RtData_T rt;
        Bsp_Can_Frame_T resp;
        comm_build_rtdata(&rt);
        if (AppComm_PackSnapshot(cmd, &rt, &resp)) { comm_send(&resp); }
        else { comm_reply_status(cmd, COMM_ST_UNKNOWN); }
        return;
    }

    /* —— 读/写配置（客户，仅工作配置区）—— */
    if (cmd >= 0x01U && cmd <= 0x07U) {
        if (!comm_handle_read_cfg(cmd)) { comm_reply_status(cmd, COMM_ST_UNKNOWN); }
        return;
    }
    if (cmd >= 0x11U && cmd <= 0x17U) {
        if (!comm_handle_write_cfg(&frame)) { comm_reply_status(cmd, COMM_ST_UNKNOWN); }
        return;
    }

    /* —— 控制（客户）—— */
    if (cmd == 0x10U || (cmd >= 0x30U && cmd <= 0x34U)) {
        if (!comm_handle_control(&frame)) { comm_reply_status(cmd, COMM_ST_UNKNOWN); }
        return;
    }

    comm_reply_status(cmd, COMM_ST_UNKNOWN);
}

/**
 * 功能：发送上电通告帧（固定头 0xA5 0x5A，源为本机 self id）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void AppComm_SendBootAnnounce(void)
{
    Bsp_Can_Frame_T f;
    f.id  = Bsp_Config_GetSelfId();
    f.ide = 0U;
    f.dlc = 8U;
    f.data[0] = 0xA5U; f.data[1] = 0x5AU; f.data[2] = 0x5AU; f.data[3] = 0xFFU;
    f.data[4] = 0xFFU; f.data[5] = 0xFFU; f.data[6] = 0xFFU; f.data[7] = 0xFFU;
    comm_send(&f);
}
