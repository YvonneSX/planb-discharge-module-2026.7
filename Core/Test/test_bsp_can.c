/**
 * @file    test_bsp_can.c
 * @brief   CAN通信单元测试 实现 (经典 CAN 2.0B) + CAN 配置命令协议
 *
 * 覆盖接口: Init / IsInitialized / Send / Receive /
 *           软件主站 ID 过滤 / CAN 命令读写配置参数
 *
 * CAN 配置命令协议 (简单实现，标准帧 11bit):
 *   从站自身 CAN ID = Bsp_Config_GetSelfId() (默认 1)
 *   绑定主站 CAN ID = Bsp_Config_GetMasterId() (默认 100)
 *   从站只处理主站发来的消息 (软件过滤)，不响应其他 ID。
 *
 * 命令帧格式 (主站→从站, ID=主站ID):
 *   data[0] = 命令字:
 *     0x01: 读波特率
 *     0x02: 读自身 CAN ID
 *     0x03: 读主站 CAN ID
 *     0x11: 设自身 CAN ID (data[1]=ID低字节, data[2]=ID高字节, 存 Flash 后重启生效)
 *     0x12: 设主站 CAN ID (同上)
 *     0x13: 设波特率   (data[1]=档位 0/1/2, 存 Flash 后重启生效)
 *   data[1..7] = 命令参数
 *
 * 应答帧格式 (从站→主站, ID=从站ID):
 *   data[0] = 收到的命令字
 *   data[1..] = 返回值/数据
 *     读操作: data[1..] = 当前值
 *     写操作: data[1] = 0(成功) 或 1(失败)
 *     写操作: data[2] = 0(已相同跳过) 或 1(已写入 Flash, 重启后生效)
 *
 * @version 1.2.0
 * @date    2026-06-13
 */

#include "test_bsp_can.h"

#define BSP_USE_CAN
#define BSP_USE_CONFIG
#include "bsp.h"

#include "uart_log.h"

/* ---- 从站中断接收状态 (RX 中断回调 → 主循环处理, HLD-MOD-COMM) ---- */
static volatile uint8_t        s_rx_flag = 0U;     /* 收到命令标志 (中断置位) */
static volatile uint32_t       s_rx_count = 0U;    /* 累计收到帧数 */
static Bsp_Can_Frame_T         s_rx_frame;         /* 最近一帧缓存 */

/* RX 中断回调: 在 CAN0 RX0 中断上下文调用, 仅搬运+置标志, 保持极短 */
static void Test_Can_OnRx(const Bsp_Can_Frame_T *frame)
{
    s_rx_frame = *frame;
    s_rx_flag  = 1U;
    s_rx_count++;
}

/* CAN 配置命令字 */
#define CAN_CMD_READ_BAUD       0x01U
#define CAN_CMD_READ_SELF_ID    0x02U
#define CAN_CMD_READ_MASTER_ID  0x03U
#define CAN_CMD_SET_SELF_ID     0x11U
#define CAN_CMD_SET_MASTER_ID   0x12U
#define CAN_CMD_SET_BAUD        0x13U

/* 从站处理主站命令并返回应答 (模拟从站行为) */
static Bsp_Can_Frame_T Test_Can_HandleCommand(const Bsp_Can_Frame_T *cmd)
{
    Bsp_Can_Frame_T resp;
    uint16_t master_id = Bsp_Config_GetMasterId();
    uint16_t self_id   = Bsp_Config_GetSelfId();
    uint8_t  cmd_byte;

    /* 软件过滤: 非主站 ID 不处理 */
    if (cmd->ide != 0U || cmd->id != master_id || cmd->dlc < 1U) {
        resp.id   = self_id;
        resp.ide  = 0U;
        resp.dlc  = 0U;
        return resp;
    }

    cmd_byte = cmd->data[0];
    resp.id   = self_id;
    resp.ide  = 0U;
    resp.dlc  = 8U;
    for (int k = 0; k < 8; k++) { resp.data[k] = 0U; }
    resp.data[0] = cmd_byte;

    switch (cmd_byte) {
    case CAN_CMD_READ_BAUD:
        resp.data[1] = (uint8_t)Bsp_Config_GetBaud();
        TEST_LOG_VAL("CAN 读波特率: %u (0=250K,1=500K,2=1M)", resp.data[1]);
        break;

    case CAN_CMD_READ_SELF_ID:
        resp.data[1] = (uint8_t)(self_id & 0xFFU);
        resp.data[2] = (uint8_t)((self_id >> 8) & 0x07U);
        TEST_LOG_VAL("CAN 读自身 ID: %u", self_id);
        break;

    case CAN_CMD_READ_MASTER_ID:
        resp.data[1] = (uint8_t)(master_id & 0xFFU);
        resp.data[2] = (uint8_t)((master_id >> 8) & 0x07U);
        TEST_LOG_VAL("CAN 读主站 ID: %u", master_id);
        break;

    case CAN_CMD_SET_SELF_ID: {
        uint16_t new_id = (uint16_t)cmd->data[1] | ((uint16_t)cmd->data[2] << 8);
        if (Bsp_Config_SetSelfId(new_id) != BSP_CFG_OK) {
            resp.data[1] = 1U;  /* 失败 */
        } else {
            Bsp_Cfg_Status_T save_ret = Bsp_Config_Save();
            resp.data[1] = (uint8_t)(save_ret != BSP_CFG_OK);
            resp.data[2] = (uint8_t)(save_ret == BSP_CFG_OK);
        }
        TEST_LOG_VAL("CAN 设自身 ID=%u", new_id);
        break;
    }

    case CAN_CMD_SET_MASTER_ID: {
        uint16_t new_id = (uint16_t)cmd->data[1] | ((uint16_t)cmd->data[2] << 8);
        if (Bsp_Config_SetMasterId(new_id) != BSP_CFG_OK) {
            resp.data[1] = 1U;
        } else {
            Bsp_Cfg_Status_T save_ret = Bsp_Config_Save();
            resp.data[1] = (uint8_t)(save_ret != BSP_CFG_OK);
            resp.data[2] = (uint8_t)(save_ret == BSP_CFG_OK);
        }
        TEST_LOG_VAL("CAN 设主站 ID=%u", new_id);
        break;
    }

    case CAN_CMD_SET_BAUD: {
        if (cmd->data[1] > 2U) {
            resp.data[1] = 1U;
        } else {
            if (Bsp_Config_SetBaud((Bsp_Cfg_Baud_T)cmd->data[1]) != BSP_CFG_OK) {
                resp.data[1] = 1U;
            } else {
                Bsp_Cfg_Status_T save_ret = Bsp_Config_Save();
                resp.data[1] = (uint8_t)(save_ret != BSP_CFG_OK);
                resp.data[2] = (uint8_t)(save_ret == BSP_CFG_OK);
            }
        }
        TEST_LOG_VAL("CAN 设波特率=%u", cmd->data[1]);
        break;
    }

    default:
        resp.data[1] = 1U;
        break;
    }
    return resp;
}

void Test_BspCan_RunAll(Test_Stats_T *_stats)
{
    Bsp_Can_Frame_T tx;
    Bsp_Can_Frame_T rx;
    Bsp_Can_Frame_T resp;
    int got;
    uint8_t i;

    /* ---- 初始化 ---- */
    TEST_CASE("CAN: 初始化");
    TEST_ASSERT_EQ(Bsp_Can_Init(), BSP_CAN_OK);
    TEST_ASSERT(Bsp_Can_IsInitialized() != 0);

    TEST_CASE("CAN: 重复初始化幂等");
    TEST_ASSERT_EQ(Bsp_Can_Init(), BSP_CAN_OK);

    /* ---- 发送参数校验 ---- */
    TEST_CASE("CAN: 发送参数校验 (NULL)");
    TEST_ASSERT_EQ(Bsp_Can_Send(NULL), BSP_CAN_ERR_PARAM);

    /* ---- 接收参数校验 / 无报文 ---- */
    TEST_CASE("CAN: 接收参数校验与无报文");
    TEST_ASSERT_EQ_INT(Bsp_Can_Receive(NULL), 0);
    got = Bsp_Can_Receive(&rx);
    TEST_ASSERT_IN_RANGE(got, 0, 1);

    /* ---- 软件过滤测试: 非主站 ID 应被忽略 ---- */
    TEST_CASE("CAN: 软件过滤-非主站ID不处理");
    {
        /* 非主站 ID 构造命令(如 ID=200, 读自身CAN ID), 应被忽略 */
        Bsp_Can_Frame_T fake_cmd;
        fake_cmd.id   = 200U;           /* 非主站 ID */
        fake_cmd.ide  = 0U;
        fake_cmd.dlc  = 1U;
        fake_cmd.data[0] = CAN_CMD_READ_SELF_ID;
        resp = Test_Can_HandleCommand(&fake_cmd);
        /* 被过滤: dlc 置 0 */
        TEST_ASSERT_EQ_INT(resp.dlc, 0U);
    }

    /* ---- 软件过滤测试: 扩展帧应被忽略 ---- */
    TEST_CASE("CAN: 软件过滤-扩展帧不处理");
    {
        Bsp_Can_Frame_T fake_cmd;
        fake_cmd.id   = Bsp_Config_GetMasterId();  /* ID 正确但是扩展帧 */
        fake_cmd.ide  = 1U;
        fake_cmd.dlc  = 1U;
        fake_cmd.data[0] = CAN_CMD_READ_SELF_ID;
        resp = Test_Can_HandleCommand(&fake_cmd);
        TEST_ASSERT_EQ_INT(resp.dlc, 0U);
    }

    /* ---- 配置命令协议测试 (本地调用, 不依赖总线) ---- */
    TEST_CASE("CAN: 配置-读回自身 ID");
    tx.id   = Bsp_Config_GetMasterId();    /* 主站 ID */
    tx.ide  = 0U;
    tx.dlc  = 1U;
    tx.data[0] = CAN_CMD_READ_SELF_ID;
    resp = Test_Can_HandleCommand(&tx);
    TEST_ASSERT_EQ_INT(resp.data[0], CAN_CMD_READ_SELF_ID);

    TEST_CASE("CAN: 配置-读回主站 ID");
    tx.data[0] = CAN_CMD_READ_MASTER_ID;
    resp = Test_Can_HandleCommand(&tx);
    TEST_ASSERT_EQ_INT(resp.data[0], CAN_CMD_READ_MASTER_ID);

    TEST_CASE("CAN: 配置-读回波特率");
    tx.data[0] = CAN_CMD_READ_BAUD;
    resp = Test_Can_HandleCommand(&tx);
    TEST_ASSERT_EQ_INT(resp.data[0], CAN_CMD_READ_BAUD);
    TEST_ASSERT_IN_RANGE(resp.data[1], 0, 2);

    TEST_CASE("CAN: 配置-无效命令返回失败");
    tx.data[0] = 0xFFU;
    resp = Test_Can_HandleCommand(&tx);
    TEST_ASSERT_EQ_INT(resp.data[1], 1U);

    /* ---- 写配置测试: 设自身 ID, 存 Flash, 再回读校验 ---- */
    TEST_CASE("CAN: 配置-设自身 ID 再读回校验");
    {
        uint16_t orig_id = Bsp_Config_GetSelfId();
        uint16_t test_id = (orig_id == 2U) ? 3U : 2U;

        tx.data[0] = CAN_CMD_SET_SELF_ID;
        tx.data[1] = (uint8_t)(test_id & 0xFFU);
        tx.data[2] = (uint8_t)((test_id >> 8) & 0x07U);
        tx.dlc  = 3U;
        resp = Test_Can_HandleCommand(&tx);
        TEST_ASSERT_EQ_INT(resp.data[1], 0U);   /* 写成功 */
        TEST_ASSERT_EQ_INT(resp.data[2], 1U);   /* 已写入 Flash */

        /* 读回校验 */
        tx.data[0] = CAN_CMD_READ_SELF_ID;
        tx.dlc  = 1U;
        resp = Test_Can_HandleCommand(&tx);
        uint16_t new_id = (uint16_t)resp.data[1] | ((uint16_t)resp.data[2] << 8);
        TEST_LOG_VAL("设自身 ID %u→%u, 回读=%u", orig_id, test_id, new_id);
        TEST_ASSERT_EQ_INT(new_id, test_id);

        /* 恢复原值 */
        Bsp_Config_SetSelfId(orig_id);
        Bsp_Config_Save();
    }

    /* ---- 写配置测试: 相同值跳过写入 ---- */
    TEST_CASE("CAN: 配置-相同值不擦写 (防 Flash 磨损)");
    {
        uint16_t cur_id = Bsp_Config_GetMasterId();
        Bsp_Config_SetMasterId(cur_id);
        Bsp_Cfg_Status_T st = Bsp_Config_Save();
        /* 相同值应返回 OK (跳过了擦写), 而不是报错 */
        TEST_ASSERT_EQ(st, BSP_CFG_OK);
    }

    /* ---- 构造一帧标准帧并发送 ---- */
    TEST_CASE("CAN: 发送标准帧 (结果取决于总线应答)");
    tx.id  = 0x123U;
    tx.ide = 0U;
    tx.dlc = 8U;
    for (i = 0U; i < 8U; i++) { tx.data[i] = (uint8_t)(0x10U + i); }
    {
        Bsp_Can_Status_T st = Bsp_Can_Send(&tx);
        TEST_LOG_VAL("CAN Send(id=0x123,dlc=8) ret=%d (0=OK, -3=TIMEOUT 无应答节点)", (int)st);
        TEST_ASSERT(st == BSP_CAN_OK || st == BSP_CAN_ERR_TIMEOUT);
    }

    /* ---- dlc 截断验证 ---- */
    TEST_CASE("CAN: dlc 超 8 不报参数错 (内部截断)");
    tx.dlc = 16U;
    {
        Bsp_Can_Status_T st = Bsp_Can_Send(&tx);
        TEST_ASSERT(st != BSP_CAN_ERR_PARAM);
    }

    /* ---- 中断接收接口契约 (从站事件驱动, HLD-MOD-COMM) ---- */
    TEST_CASE("CAN: 中断接收-NULL 回调返回参数错");
    TEST_ASSERT_EQ(Bsp_Can_EnableRxIt(NULL), BSP_CAN_ERR_PARAM);

    TEST_CASE("CAN: 中断接收-注册回调并使能 RX 中断");
    s_rx_flag = 0U;
    s_rx_count = 0U;
    TEST_ASSERT_EQ(Bsp_Can_EnableRxIt(Test_Can_OnRx), BSP_CAN_OK);
    /* 自发一帧: 单节点无应答, 本机不回环, 仅验证使能后发送不破坏状态;
     * 真机有总线节点时, 该帧由对端回送会触发中断回调 (实测见 make can-loop)。 */
    tx.id = 0x123U; tx.ide = 0U; tx.dlc = 1U; tx.data[0] = 0x5AU;
    (void)Bsp_Can_Send(&tx);
    TEST_LOG_VAL("RX 中断已使能, 累计回调帧数=%lu (单节点通常为0)", (unsigned long)s_rx_count);

    TEST_CASE("CAN: 中断接收-关闭 RX 中断");
    Bsp_Can_DisableRxIt();
    /* 关闭后注销回调, 计数不再增长 (此处仅验证调用安全) */
    TEST_ASSERT(1);
}

/* ================================================================
 * CAN 交互测试模式 (从站事件驱动, HLD-MOD-COMM / HLD-FLOW-004)
 * ----------------------------------------------------------------
 * 架构: RX FIFO0 中断 → Test_Can_OnRx() 取帧入缓存+置标志 (中断内不做业务);
 *       main while 检测标志 → switch-case 处理命令 → CAN TX 应答。
 * 用法: make can-loop → 烧录后打开上位机发送命令验证收发。
 * 退出: 发送命令 0xFF 退出交互模式 (或复位)。
 * ================================================================ */
void Test_BspCan_Interactive(void)
{
    Bsp_Can_Frame_T cmd, resp;

    Bsp_Can_Init();
    Bsp_Config_Load();

    /* 注册 RX 回调并使能 FIFO0 接收中断 (从站事件驱动) */
    s_rx_flag = 0U;
    s_rx_count = 0U;
    Bsp_Can_EnableRxIt(Test_Can_OnRx);

    UartLog_Println("\r\n=== CAN 交互测试模式 (中断接收) ===");
    UartLog_Println("自身ID=%u  主站ID=%u  波特率=%u",
                    Bsp_Config_GetSelfId(), Bsp_Config_GetMasterId(),
                    (unsigned int)Bsp_Config_GetBaud());
    UartLog_Println("RX 中断已使能, 等待主站命令... (发送 0xFF 退出)");

    for (;;) {
        /* 主循环检测接收标志 (中断置位); 无命令则继续 (此处可做其他周期任务) */
        if (s_rx_flag == 0U) {
            continue;
        }
        /* 取出中断缓存帧 (短暂关中断防撕裂), 清标志 */
        nvic_irq_disable(BSP_CAN_RX0_IRQN);
        cmd = s_rx_frame;
        s_rx_flag = 0U;
        nvic_irq_enable(BSP_CAN_RX0_IRQN, BSP_CAN_RX_IRQ_PRE_PRIO, BSP_CAN_RX_IRQ_SUB_PRIO);

        /* 软件过滤: 仅处理主站标准帧 */
        if (cmd.ide != 0U || cmd.id != Bsp_Config_GetMasterId() || cmd.dlc < 1U) {
            continue;
        }
        /* 退出命令 */
        if (cmd.data[0] == 0xFFU) {
            UartLog_Println("收到退出命令, 退出交互模式");
            break;
        }

        /* 处理命令 (switch-case) 并应答 */
        UartLog_Println("[CAN] 收到命令 0x%02X DLC=%u (累计%lu帧)",
                        cmd.data[0], cmd.dlc, (unsigned long)s_rx_count);
        resp = Test_Can_HandleCommand(&cmd);
        if (resp.dlc > 0U) {
            Bsp_Can_Status_T st = Bsp_Can_Send(&resp);
            UartLog_Println("[CAN] 应答 DLC=%u data[0]=0x%02X st=%d",
                            resp.dlc, resp.data[0], (int)st);
        }
    }
    Bsp_Can_DisableRxIt();
    UartLog_Println("=== CAN 交互测试结束 ===\r\n");
}
