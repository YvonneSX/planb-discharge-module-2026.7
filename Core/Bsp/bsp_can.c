/**
 * @file    bsp_can.c
 * @brief   CAN 通信 BSP 驱动（GD32F303CCT6，经典 CAN 2.0B）
 *
 * PA11 — CAN0_RX
 * PA12 — CAN0_TX（默认引脚，无需 remap）
 * 收发器 TCAN3414（物理层）；MCU 片内 CAN0 为经典 CAN 2.0B。
 *
 * 注：GD32F303 片内 CAN 控制器仅支持经典 CAN 2.0B（≤8 字节/帧），不支持 CAN-FD；
 *     TCAN3414 虽为 CAN-FD 收发器，仅是物理层，无法弥补 MCU 控制器能力缺失。
 * 对应需求：REQ-FUNC-040/042，REQ-IF-001/002，REQ-NF-003（波特率可配置切换）
 * 参考：官方 demo 17_CAN_Network（引脚 PD0/1→PA11/12，波特率按 100MHz 重算）
 *
 * 波特率：位时序固定 SJW=1 + BS1=7 + BS2=2 = 10TQ/bit；prescaler 取自设备配置
 *   （Bsp_Config 持久化的波特率档位，缺省 500kbps@APB1=50MHz，prescaler=10）。
 *
 * @version 1.1.0
 * @date    2026-06-13
 */

#define BSP_USE_CAN
#define BSP_USE_CONFIG
#include "bsp.h"

/* ------------------------------------------------------------------ */
static volatile uint8_t s_can_initialized = 0U;
static Bsp_Can_RxCallback_T s_rx_callback = NULL;   /* RX 中断回调（从站事件驱动） */

/**
 * 功能：配置 CAN GPIO（PA12 TX 复用推挽、PA11 RX 上拉输入，默认引脚无需 remap）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Can_GpioConfig(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_AF);

    /* PA12 = CAN0_TX，复用推挽；PA11 = CAN0_RX，上拉输入 */
    gpio_init(BSP_CAN_TX_PORT, GPIO_MODE_AF_PP,  GPIO_OSPEED_50MHZ, BSP_CAN_TX_PIN);
    gpio_init(BSP_CAN_RX_PORT, GPIO_MODE_IPU,    GPIO_OSPEED_50MHZ, BSP_CAN_RX_PIN);
}

/**
 * 功能：配置经典 CAN 2.0B 参数（位时序 10TQ、按配置档位取 prescaler）与全通过滤器路由到 FIFO0。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
static void Bsp_Can_ConfigClassic(void)
{
    can_parameter_struct        can_parameter;
    can_filter_parameter_struct can_filter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    can_deinit(BSP_CAN_INSTANCE);

    can_parameter.time_triggered        = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up          = DISABLE;
    can_parameter.auto_retrans          = ENABLE;
    can_parameter.rec_fifo_overwrite    = DISABLE;
    can_parameter.trans_fifo_order      = DISABLE;
    can_parameter.working_mode          = CAN_NORMAL_MODE;
    /* 位时序：SJW=1 + BS1=7 + BS2=2 = 10 TQ/bit
     * APB1=50MHz；波特率 = APB1 / (prescaler × 10TQ)
     * prescaler 由设备配置档位决定：250k→20 / 500k→10 / 1M→5（缺省 500k）。
     * 配置损坏/未写入时 Bsp_Config 自动回落默认档位，保证 CAN 始终可用。 */
    can_parameter.resync_jump_width     = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1        = CAN_BT_BS1_7TQ;
    can_parameter.time_segment_2        = CAN_BT_BS2_2TQ;
    can_parameter.prescaler             = Bsp_Config_BaudToPrescaler(Bsp_Config_GetBaud());
    can_init(BSP_CAN_INSTANCE, &can_parameter);

    /* 过滤器：全通（接收所有 ID），路由到 FIFO0 */
    can_filter.filter_number      = 0U;
    can_filter.filter_mode        = CAN_FILTERMODE_MASK;
    can_filter.filter_bits        = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high   = 0x0000;
    can_filter.filter_list_low    = 0x0000;
    can_filter.filter_mask_high   = 0x0000;
    can_filter.filter_mask_low    = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable      = ENABLE;
    can_filter_init(&can_filter);
}

/* ------------------------------------------------------------------ */
/**
 * 功能：初始化 CAN0（GPIO + 经典 CAN 参数，波特率取自设备配置）。幂等。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Can_Status_T，BSP_CAN_OK 表示成功。
 */
Bsp_Can_Status_T Bsp_Can_Init(void)
{
    if (s_can_initialized) {
        return BSP_CAN_OK;
    }
    Bsp_Can_GpioConfig();
    Bsp_Can_ConfigClassic();
    s_can_initialized = 1U;
    return BSP_CAN_OK;
}

/**
 * 功能：查询 CAN0 是否已初始化。
 * 输入参数：无
 * 输出参数：无
 * 返回值：int，非 0=已初始化，0=未初始化。
 */
int Bsp_Can_IsInitialized(void)
{
    return (int)s_can_initialized;
}

/**
 * 功能：发送一帧 CAN 报文（标准/扩展帧，DLC 上限 8 字节），阻塞等待发送完成或超时。
 * 输入参数：frame —— 待发送帧指针。
 * 输出参数：无
 * 返回值：Bsp_Can_Status_T，OK 成功，ERR_PARAM 空指针，ERR_TIMEOUT 发送超时。
 */
Bsp_Can_Status_T Bsp_Can_Send(const Bsp_Can_Frame_T *frame)
{
    can_trasnmit_message_struct tx;
    uint8_t mailbox;
    uint32_t timeout = 0xFFFFU;
    uint8_t i;
    uint8_t dlc;

    if (frame == NULL) {
        return BSP_CAN_ERR_PARAM;
    }
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx);

    if (frame->ide) {
        tx.tx_ff   = CAN_FF_EXTENDED;
        tx.tx_efid = frame->id;
        tx.tx_sfid = 0U;
    } else {
        tx.tx_ff   = CAN_FF_STANDARD;
        tx.tx_sfid = frame->id;
        tx.tx_efid = 0U;
    }
    tx.tx_ft  = CAN_FT_DATA;
    dlc = (frame->dlc > 8U) ? 8U : frame->dlc;   /* 经典 CAN 上限 8 字节 */
    tx.tx_dlen = dlc;
    for (i = 0U; i < dlc; i++) {
        tx.tx_data[i] = frame->data[i];
    }

    mailbox = can_message_transmit(BSP_CAN_INSTANCE, &tx);
    while ((CAN_TRANSMIT_OK != can_transmit_states(BSP_CAN_INSTANCE, mailbox)) && (0U != timeout)) {
        timeout--;
    }
    return (timeout == 0U) ? BSP_CAN_ERR_TIMEOUT : BSP_CAN_OK;
}

/**
 * 功能：轮询方式从 FIFO0 接收一帧（无报文则立即返回）。
 * 输入参数：无
 * 输出参数：frame —— 接收帧指针，写入收到的报文。
 * 返回值：int，1=收到一帧，0=无报文或空指针。
 */
int Bsp_Can_Receive(Bsp_Can_Frame_T *frame)
{
    can_receive_message_struct rx;
    uint8_t i;

    if (frame == NULL) {
        return 0;
    }
    /* 查询 FIFO0 是否有报文 */
    if (can_receive_message_length_get(BSP_CAN_INSTANCE, CAN_FIFO0) == 0U) {
        return 0;
    }
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx);
    can_message_receive(BSP_CAN_INSTANCE, CAN_FIFO0, &rx);

    frame->ide = (rx.rx_ff == CAN_FF_EXTENDED) ? 1U : 0U;
    frame->id  = frame->ide ? rx.rx_efid : rx.rx_sfid;
    frame->dlc = rx.rx_dlen;
    for (i = 0U; i < rx.rx_dlen && i < sizeof(frame->data); i++) {
        frame->data[i] = rx.rx_data[i];
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/**
 * 功能：使能 CAN RX FIFO0 非空中断并注册接收回调（从站事件驱动）。
 * 输入参数：cb —— 接收回调函数指针。
 * 输出参数：无
 * 返回值：Bsp_Can_Status_T，OK 成功，ERR_PARAM 回调为空，ERR_INIT 未初始化。
 */
Bsp_Can_Status_T Bsp_Can_EnableRxIt(Bsp_Can_RxCallback_T cb)
{
    if (cb == NULL) {
        return BSP_CAN_ERR_PARAM;
    }
    if (!s_can_initialized) {
        return BSP_CAN_ERR_INIT;
    }
    s_rx_callback = cb;
    nvic_irq_enable(BSP_CAN_RX0_IRQN, BSP_CAN_RX_IRQ_PRE_PRIO, BSP_CAN_RX_IRQ_SUB_PRIO);
    can_interrupt_enable(BSP_CAN_INSTANCE, CAN_INTEN_RFNEIE0);
    return BSP_CAN_OK;
}

/**
 * 功能：禁用 CAN RX FIFO0 中断并清除接收回调。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Can_DisableRxIt(void)
{
    can_interrupt_disable(BSP_CAN_INSTANCE, CAN_INTEN_RFNEIE0);
    nvic_irq_disable(BSP_CAN_RX0_IRQN);
    s_rx_callback = NULL;
}

/**
 * 功能：RX FIFO0 中断处理体（由中断向量调用），排空 FIFO 并逐帧回调（无回调则丢弃防中断风暴）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：无
 */
void Bsp_Can_Rx0IrqHandler(void)
{
    can_receive_message_struct rx;
    Bsp_Can_Frame_T frame;
    uint8_t i;

    /* 循环排空 FIFO0：单次中断可能积累多帧 */
    while (can_receive_message_length_get(BSP_CAN_INSTANCE, CAN_FIFO0) != 0U) {
        can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx);
        can_message_receive(BSP_CAN_INSTANCE, CAN_FIFO0, &rx);   /* 读取即释放该帧 */

        if (s_rx_callback != NULL) {
            frame.ide = (rx.rx_ff == CAN_FF_EXTENDED) ? 1U : 0U;
            frame.id  = frame.ide ? rx.rx_efid : rx.rx_sfid;
            frame.dlc = rx.rx_dlen;
            for (i = 0U; i < rx.rx_dlen && i < sizeof(frame.data); i++) {
                frame.data[i] = rx.rx_data[i];
            }
            s_rx_callback(&frame);
        }
    }
}
