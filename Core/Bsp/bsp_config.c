/**
 * @file    bsp_config.c
 * @brief   设备配置 BSP 驱动 (GD32F303CCT6) — FRAM 存储 CAN 参数
 *
 * 管理从站 CAN ID、绑定主站 CAN ID、CAN 波特率档位。
 * 存储于 FRAM 末尾 16 字节 (BSP_CFG_FRAM_ADDR)，带 magic + CRC16 校验，
 * 损坏/为 0/非法时自动回落默认值。
 * 通过 CAN 总线下发的 ID/波特率修改，只改 FRAM 中的配置值，不直接动 CAN 寄存器；
 * 模块重启上电后由 Bsp_Can_Init() 从 FRAM 取最新值初始化波特率。
 *
 * 规则:
 *   - 上电加载: 从 FRAM 读取。magic/CRC 整体损坏 → 全部回落默认宏值；
 *     magic/CRC 通过但某字段越界 → 该字段单独回落其默认宏值(逐字段防御)。
 *   - 修改逻辑: 先把 FRAM 中存储值与当前配置值读出比对，若不一致才写 FRAM，
 *     若完全一致则跳过写 (FRAM 无擦除/无磨损顾虑，比对仅为省去无谓总线写)。
 *   - 上电后: Bsp_Config_Load() 读出到 RAM；Get/Set 操作 RAM 副本；
 *     Save() 方写入 FRAM。上层据 Get* 取值传入各 BSP 初始化(如 PWM 频率)。
 *
 * FRAM 存储布局 (34 字节, 位于 BSP_CFG_FRAM_ADDR = 0x1F80 工作配置区):
 *   偏移 0x00: magic         (4B, 0x47444347 = "GDCG"，v3 含可标定阈值)
 *   偏移 0x04: self_id        (2B)
 *   偏移 0x06: master_id      (2B)
 *   偏移 0x08: baud_idx       (1B)
 *   偏移 0x09: pwm_duty_max    (1B)  占空比上限 % (5~90, 硬顶 90)
 *   偏移 0x0A: pwm_duty_step   (1B)  斜坡步进 %
 *   偏移 0x0B: pwm_duty_start  (1B)  默认启动占空比 % (4~duty_max)
 *   偏移 0x0C: pwm_freq_hz     (2B)  PWM 频率 Hz
 *   偏移 0x0E: ovp_on_0v1      (2B)  泄放启动 0.1V
 *   偏移 0x10: ovp_off_0v1     (2B)  泄放停止/欠压 0.1V
 *   偏移 0x12: ovp_l1_0v1      (2B)  一级动作 0.1V
 *   偏移 0x14: ovp_l1_off_0v1  (2B)  一级恢复 0.1V
 *   偏移 0x16: ovp_l2_0v1      (2B)  二级动作 0.1V
 *   偏移 0x18: ovp_l2_off_0v1  (2B)  二级恢复 0.1V
 *   偏移 0x1A: otp_off_0c1     (2B)  过温关断 0.1℃
 *   偏移 0x1C: otp_recover_0c1 (2B)  过温恢复 0.1℃
 *   偏移 0x1E: ovc_0a01        (2B)  过流 0.01A
 *   偏移 0x20: crc16          (2B)  CRC-16/XMODEM 算 0x04..0x1F 共 28 字节
 *
 * 注: 片上 Flash 方案 (bsp_flash) 暂时停用，配置改存 FRAM；如需切回，
 *     恢复 BSP_USE_FLASH 并把 Bsp_Fram_* 换回 Bsp_Flash_* 即可。
 *
 * 出厂/默认备份: 代码段 const g_bsp_cfg_factory_backup；恢复出厂从代码段复写 FRAM 工作区（见 FRAM 文档 §3.0）。
 * 对应需求: REQ-CON-001 (配置持久化), REQ-FUNC-040/042 (CAN 参数可配置),
 *           REQ-NF-001 (关键保护/PWM 参数可配置标定), REQ-FUNC-013/015 (PWM 占空比/频率)
 *
 * @version 2.2.0
 * @date    2026-06-16
 */

#define BSP_USE_CONFIG
#define BSP_USE_FRAM
#include "bsp.h"

/* 存储 magic：v3 "GDCG"（含可标定保护阈值；旧 v2 "GDCF" 自动判失效→回落默认） */
#define BSP_CFG_MAGIC               0x47444347U

/* 配置存储结构体 (packed, 34 bytes) */
typedef struct __attribute__((packed)) {
    uint32_t magic;         /* 魔数校验 */
    uint16_t self_id;       /* 从站自身 CAN ID */
    uint16_t master_id;     /* 绑定主站 CAN ID */
    uint8_t  baud_idx;      /* 波特率档位 0/1/2 */
    uint8_t  pwm_duty_max;   /* PWM 占空比上限 % (5~90) */
    uint8_t  pwm_duty_step;  /* PWM 斜坡步进 % */
    uint8_t  pwm_duty_start; /* PWM 默认启动占空比 % (4~duty_max) */
    uint16_t pwm_freq_hz;    /* PWM 频率 Hz */
    uint16_t ovp_on_0v1;     /* 泄放启动 0.1V */
    uint16_t ovp_off_0v1;    /* 泄放停止/欠压 0.1V */
    uint16_t ovp_l1_0v1;     /* 一级动作 0.1V */
    uint16_t ovp_l1_off_0v1; /* 一级恢复 0.1V */
    uint16_t ovp_l2_0v1;     /* 二级动作 0.1V */
    uint16_t ovp_l2_off_0v1; /* 二级恢复 0.1V */
    uint16_t otp_off_0c1;    /* 过温关断 0.1℃ */
    uint16_t otp_recover_0c1;/* 过温恢复 0.1℃ */
    uint16_t ovc_0a01;       /* 过流 0.01A */
    uint16_t crc;           /* CRC-16/XMODEM (算 self_id..ovc_0a01 共 28 字节) */
} ConfigBlob_T;

/* 阈值字段默认值集合（代码段备份与 RAM 初值共用，避免重复书写） */
#define BSP_CFG_THRESH_DEFAULTS                              \
    .ovp_on_0v1     = BSP_CFG_DEFAULT_OVP_ON_0V1,            \
    .ovp_off_0v1    = BSP_CFG_DEFAULT_OVP_OFF_0V1,           \
    .ovp_l1_0v1     = BSP_CFG_DEFAULT_OVP_L1_0V1,            \
    .ovp_l1_off_0v1 = BSP_CFG_DEFAULT_OVP_L1_OFF_0V1,        \
    .ovp_l2_0v1     = BSP_CFG_DEFAULT_OVP_L2_0V1,            \
    .ovp_l2_off_0v1 = BSP_CFG_DEFAULT_OVP_L2_OFF_0V1,        \
    .otp_off_0c1    = BSP_CFG_DEFAULT_OTP_OFF_0C1,           \
    .otp_recover_0c1= BSP_CFG_DEFAULT_OTP_RECOVER_0C1,       \
    .ovc_0a01       = BSP_CFG_DEFAULT_OVC_0A01

/* 代码段出厂默认备份（恢复出厂/Load 回落唯一来源） */
const Bsp_Cfg_FactoryBackup_T g_bsp_cfg_factory_backup = {
    .self_id        = BSP_CFG_DEFAULT_SELF_ID,
    .master_id      = BSP_CFG_DEFAULT_MASTER_ID,
    .baud_idx       = BSP_CFG_DEFAULT_BAUD,
    .pwm_duty_max   = BSP_CFG_DEFAULT_DUTY_MAX,
    .pwm_duty_step  = BSP_CFG_DEFAULT_DUTY_STEP,
    .pwm_duty_start = BSP_CFG_DEFAULT_DUTY_START,
    .pwm_freq_hz    = BSP_CFG_DEFAULT_PWM_FREQ_HZ,
    BSP_CFG_THRESH_DEFAULTS,
};

/* CRC 计算的 payload 长度 (self_id..ovc_0a01) */
#define BSP_CFG_CRC_PAYLOAD_LEN     28U

/* RAM 配置副本 (加载后用此) */
static ConfigBlob_T s_ram = {
    .magic         = BSP_CFG_MAGIC,
    .self_id       = BSP_CFG_DEFAULT_SELF_ID,
    .master_id     = BSP_CFG_DEFAULT_MASTER_ID,
    .baud_idx      = BSP_CFG_DEFAULT_BAUD,
    .pwm_duty_max   = BSP_CFG_DEFAULT_DUTY_MAX,
    .pwm_duty_step  = BSP_CFG_DEFAULT_DUTY_STEP,
    .pwm_duty_start = BSP_CFG_DEFAULT_DUTY_START,
    .pwm_freq_hz    = BSP_CFG_DEFAULT_PWM_FREQ_HZ,
    BSP_CFG_THRESH_DEFAULTS,
    .crc           = 0U,
};
static uint8_t s_loaded = 0U;

/**
 * 功能：计算 CRC-16/XMODEM 校验（poly 0x1021，init 0x0000）。
 * 输入参数：data —— 数据指针；len —— 字节数。
 * 输出参数：无
 * 返回值：uint16_t，CRC-16 校验值。
 */
static uint16_t Bsp_Config_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0U;
    uint16_t i, j;

    for (i = 0U; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (j = 0U; j < 8U; j++) {
            if (crc & 0x8000U) {
                crc = (crc << 1) ^ 0x1021U;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * 功能：计算配置块的 CRC（覆盖 self_id..ovc_0a01 共 28 字节 payload）。
 * 输入参数：cfg —— 配置块指针。
 * 输出参数：无
 * 返回值：uint16_t，配置 payload 的 CRC-16 值。
 */
static uint16_t Bsp_Config_CalcCrc(const ConfigBlob_T *cfg)
{
    return Bsp_Config_Crc16((const uint8_t *)&cfg->self_id, BSP_CFG_CRC_PAYLOAD_LEN);
}

/**
 * 功能：判定电压阈值是否在合法量程内（0.1V 单位）。
 * 输入参数：v0v1 —— 电压阈值（0.1V/LSB）。
 * 输出参数：无
 * 返回值：int，1=在范围内，0=越界。
 */
static int Bsp_Config_VoltInRange(uint16_t v0v1)
{
    return (v0v1 >= BSP_CFG_VOLT_MIN_0V1 && v0v1 <= BSP_CFG_VOLT_MAX_0V1) ? 1 : 0;
}

/**
 * 功能：逐字段合法性归一，任一字段越界或迟滞关系颠倒则单独回落其默认宏值（不影响其他字段）。
 * 输入参数：无（操作 RAM 配置副本）
 * 输出参数：无（就地修正 s_ram 各字段）
 * 返回值：无
 */
static void Bsp_Config_Normalize(void)
{
    if (s_ram.self_id == 0U || s_ram.self_id > 0x7FFU) {
        s_ram.self_id = BSP_CFG_DEFAULT_SELF_ID;
    }
    if (s_ram.master_id == 0U || s_ram.master_id > 0x7FFU) {
        s_ram.master_id = BSP_CFG_DEFAULT_MASTER_ID;
    }
    if (s_ram.baud_idx > (uint8_t)BSP_CFG_BAUD_1M) {
        s_ram.baud_idx = BSP_CFG_DEFAULT_BAUD;
    }
    if (s_ram.pwm_freq_hz < BSP_CFG_PWM_FREQ_MIN_HZ ||
        s_ram.pwm_freq_hz > BSP_CFG_PWM_FREQ_MAX_HZ) {
        s_ram.pwm_freq_hz = BSP_CFG_DEFAULT_PWM_FREQ_HZ;
    }
    if (s_ram.pwm_duty_max < BSP_CFG_DUTY_MIN || s_ram.pwm_duty_max > BSP_CFG_DUTY_MAX_CEIL) {
        s_ram.pwm_duty_max = BSP_CFG_DEFAULT_DUTY_MAX;
    }
    if (s_ram.pwm_duty_step == 0U || s_ram.pwm_duty_step > BSP_CFG_DUTY_STEP_MAX) {
        s_ram.pwm_duty_step = BSP_CFG_DEFAULT_DUTY_STEP;
    }
    /* 启动占空比须在 [DUTY_MIN, duty_max] 内；越界回落默认（默认若超上限则取上限） */
    if (s_ram.pwm_duty_start < BSP_CFG_DUTY_MIN || s_ram.pwm_duty_start > s_ram.pwm_duty_max) {
        s_ram.pwm_duty_start = (BSP_CFG_DEFAULT_DUTY_START <= s_ram.pwm_duty_max)
                                   ? (uint8_t)BSP_CFG_DEFAULT_DUTY_START
                                   : s_ram.pwm_duty_max;
    }

    /* —— 可标定阈值逐字段归一（越界或迟滞关系颠倒则回落默认）—— */
    if (!Bsp_Config_VoltInRange(s_ram.ovp_on_0v1))  { s_ram.ovp_on_0v1  = BSP_CFG_DEFAULT_OVP_ON_0V1; }
    if (!Bsp_Config_VoltInRange(s_ram.ovp_off_0v1) || s_ram.ovp_off_0v1 >= s_ram.ovp_on_0v1) {
        s_ram.ovp_off_0v1 = BSP_CFG_DEFAULT_OVP_OFF_0V1;
    }
    if (!Bsp_Config_VoltInRange(s_ram.ovp_l1_0v1))  { s_ram.ovp_l1_0v1  = BSP_CFG_DEFAULT_OVP_L1_0V1; }
    if (!Bsp_Config_VoltInRange(s_ram.ovp_l1_off_0v1) || s_ram.ovp_l1_off_0v1 >= s_ram.ovp_l1_0v1) {
        s_ram.ovp_l1_off_0v1 = (s_ram.ovp_l1_0v1 > BSP_CFG_ALARM_HYST_0V1)
                                   ? (uint16_t)(s_ram.ovp_l1_0v1 - BSP_CFG_ALARM_HYST_0V1)
                                   : BSP_CFG_DEFAULT_OVP_L1_OFF_0V1;
    }
    if (!Bsp_Config_VoltInRange(s_ram.ovp_l2_0v1))  { s_ram.ovp_l2_0v1  = BSP_CFG_DEFAULT_OVP_L2_0V1; }
    if (!Bsp_Config_VoltInRange(s_ram.ovp_l2_off_0v1) || s_ram.ovp_l2_off_0v1 >= s_ram.ovp_l2_0v1) {
        s_ram.ovp_l2_off_0v1 = (s_ram.ovp_l2_0v1 > BSP_CFG_ALARM_HYST_0V1)
                                   ? (uint16_t)(s_ram.ovp_l2_0v1 - BSP_CFG_ALARM_HYST_0V1)
                                   : BSP_CFG_DEFAULT_OVP_L2_OFF_0V1;
    }
    if (s_ram.otp_off_0c1 < BSP_CFG_TEMP_MIN_0C1 || s_ram.otp_off_0c1 > BSP_CFG_TEMP_MAX_0C1) {
        s_ram.otp_off_0c1 = BSP_CFG_DEFAULT_OTP_OFF_0C1;
    }
    if (s_ram.otp_recover_0c1 < BSP_CFG_TEMP_MIN_0C1 || s_ram.otp_recover_0c1 >= s_ram.otp_off_0c1) {
        s_ram.otp_recover_0c1 = BSP_CFG_DEFAULT_OTP_RECOVER_0C1;
    }
    if (s_ram.ovc_0a01 < BSP_CFG_CURR_MIN_0A01 || s_ram.ovc_0a01 > BSP_CFG_CURR_MAX_0A01) {
        s_ram.ovc_0a01 = BSP_CFG_DEFAULT_OVC_0A01;
    }
}

/* ----------------------------------------------------------------
 * 公共接口
 * ---------------------------------------------------------------- */

/**
 * 功能：从 FRAM 加载配置到 RAM（magic/CRC/逐字段校验，任一不符则回落默认）。重复调用幂等。
 * 输入参数：无
 * 输出参数：无（结果载入 RAM 配置副本）
 * 返回值：Bsp_Cfg_Status_T，BSP_CFG_OK 校验通过，BSP_CFG_ERR_CRC 校验失败已回落默认。
 */
Bsp_Cfg_Status_T Bsp_Config_Load(void)
{
    ConfigBlob_T fram_cfg;
    uint16_t calc_crc;

    if (s_loaded) {
        return BSP_CFG_OK;
    }

    (void)Bsp_Fram_Init();      /* 幂等，确保 FRAM/SPI 总线就绪 */

    /* 从 FRAM 读出 */
    if (Bsp_Fram_Read(BSP_CFG_FRAM_ADDR, &fram_cfg, sizeof(fram_cfg)) != BSP_FRAM_OK) {
        goto use_default;
    }

    /* magic 校验（未写入过的 FRAM 为全 0/0xFF，magic 不匹配→整体默认） */
    if (fram_cfg.magic != BSP_CFG_MAGIC) {
        goto use_default;
    }

    /* CRC 校验（整体完整性；不通过→整体默认，内容不可信） */
    calc_crc = Bsp_Config_CalcCrc(&fram_cfg);
    if (fram_cfg.crc != calc_crc) {
        goto use_default;
    }

    /* magic+CRC 通过, 载入 RAM; 再逐字段归一(越界字段单独回落默认) */
    s_ram = fram_cfg;
    Bsp_Config_Normalize();
    s_loaded = 1U;
    return BSP_CFG_OK;

use_default:
    (void)Bsp_Config_RestoreFromFactory();
    s_loaded = 1U;
    return BSP_CFG_ERR_CRC;
}

/**
 * 功能：保存当前 RAM 配置到 FRAM（先算 CRC，再 compare-before-write，一致则跳过写）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，BSP_CFG_OK 成功（含跳过写），BSP_CFG_ERR_SAVE 写失败。
 */
Bsp_Cfg_Status_T Bsp_Config_Save(void)
{
    ConfigBlob_T stored;
    uint16_t calc_crc;

    (void)Bsp_Fram_Init();      /* 幂等，确保 FRAM/SPI 总线就绪 */

    /* 计算 CRC 并填入 s_ram */
    s_ram.magic = BSP_CFG_MAGIC;
    calc_crc = Bsp_Config_CalcCrc(&s_ram);
    s_ram.crc = calc_crc;

    /* 读出 FRAM 当前存储值，与配置值逐字节比对，若完全一致则跳过写 */
    if (Bsp_Fram_Read(BSP_CFG_FRAM_ADDR, &stored, sizeof(stored)) == BSP_FRAM_OK) {
        int same = 1;
        const uint8_t *pa = (const uint8_t *)&s_ram;
        const uint8_t *pb = (const uint8_t *)&stored;
        uint32_t k;
        for (k = 0U; k < sizeof(stored); k++) {
            if (pa[k] != pb[k]) { same = 0; break; }
        }
        if (same) {
            return BSP_CFG_OK;  /* 配置值与存储值一致, 跳过 */
        }
    }

    /* 不一致: 直接写入 FRAM（无需擦除） */
    if (Bsp_Fram_Write(BSP_CFG_FRAM_ADDR, &s_ram, sizeof(s_ram)) != BSP_FRAM_OK) {
        return BSP_CFG_ERR_SAVE;
    }
    return BSP_CFG_OK;
}

/**
 * 功能：将 RAM 配置副本恢复为代码段出厂默认值（不写 FRAM），并重算 CRC。
 * 输入参数：无
 * 输出参数：无（覆盖 RAM 配置副本）
 * 返回值：无
 */
void Bsp_Config_SetDefault(void)
{
    s_ram.magic          = BSP_CFG_MAGIC;
    s_ram.self_id        = g_bsp_cfg_factory_backup.self_id;
    s_ram.master_id      = g_bsp_cfg_factory_backup.master_id;
    s_ram.baud_idx       = g_bsp_cfg_factory_backup.baud_idx;
    s_ram.pwm_duty_max   = g_bsp_cfg_factory_backup.pwm_duty_max;
    s_ram.pwm_duty_step  = g_bsp_cfg_factory_backup.pwm_duty_step;
    s_ram.pwm_duty_start = g_bsp_cfg_factory_backup.pwm_duty_start;
    s_ram.pwm_freq_hz    = g_bsp_cfg_factory_backup.pwm_freq_hz;
    s_ram.ovp_on_0v1     = g_bsp_cfg_factory_backup.ovp_on_0v1;
    s_ram.ovp_off_0v1    = g_bsp_cfg_factory_backup.ovp_off_0v1;
    s_ram.ovp_l1_0v1     = g_bsp_cfg_factory_backup.ovp_l1_0v1;
    s_ram.ovp_l1_off_0v1 = g_bsp_cfg_factory_backup.ovp_l1_off_0v1;
    s_ram.ovp_l2_0v1     = g_bsp_cfg_factory_backup.ovp_l2_0v1;
    s_ram.ovp_l2_off_0v1 = g_bsp_cfg_factory_backup.ovp_l2_off_0v1;
    s_ram.otp_off_0c1    = g_bsp_cfg_factory_backup.otp_off_0c1;
    s_ram.otp_recover_0c1= g_bsp_cfg_factory_backup.otp_recover_0c1;
    s_ram.ovc_0a01       = g_bsp_cfg_factory_backup.ovc_0a01;
    s_ram.crc            = Bsp_Config_CalcCrc(&s_ram);
}

/**
 * 功能：恢复出厂——以代码段默认覆盖 RAM 并写入 FRAM 工作区。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，保存结果（OK/ERR_SAVE）。
 */
Bsp_Cfg_Status_T Bsp_Config_RestoreFromFactory(void)
{
    Bsp_Config_SetDefault();
    return Bsp_Config_Save();
}

/**
 * 功能：读取从站自身 CAN ID（未加载则先加载）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，自身 CAN ID。
 */
uint16_t Bsp_Config_GetSelfId(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.self_id;
}

/**
 * 功能：设置从站自身 CAN ID（仅改 RAM，需 Save 落 FRAM，复位后生效）。
 * 输入参数：id —— 新 CAN ID（1~0x7FF）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界（0 或 >0x7FF）。
 */
Bsp_Cfg_Status_T Bsp_Config_SetSelfId(uint16_t id)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (id > 0x7FFU || id == 0U) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.self_id = id;
    return BSP_CFG_OK;
}

/**
 * 功能：读取绑定主站 CAN ID（未加载则先加载）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，主站 CAN ID。
 */
uint16_t Bsp_Config_GetMasterId(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.master_id;
}

/**
 * 功能：设置绑定主站 CAN ID（仅改 RAM，需 Save 落 FRAM）。
 * 输入参数：id —— 新主站 CAN ID（1~0x7FF）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界。
 */
Bsp_Cfg_Status_T Bsp_Config_SetMasterId(uint16_t id)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (id > 0x7FFU || id == 0U) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.master_id = id;
    return BSP_CFG_OK;
}

/**
 * 功能：读取 CAN 波特率档位（未加载则先加载）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：Bsp_Cfg_Baud_T，波特率档位。
 */
Bsp_Cfg_Baud_T Bsp_Config_GetBaud(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return (Bsp_Cfg_Baud_T)s_ram.baud_idx;
}

/**
 * 功能：设置 CAN 波特率档位（仅改 RAM，需 Save 落 FRAM，复位后生效）。
 * 输入参数：baud —— 波特率档位（≤BSP_CFG_BAUD_1M）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 档位非法。
 */
Bsp_Cfg_Status_T Bsp_Config_SetBaud(Bsp_Cfg_Baud_T baud)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (baud > BSP_CFG_BAUD_1M) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.baud_idx = (uint8_t)baud;
    return BSP_CFG_OK;
}

/**
 * 功能：将波特率档位映射为 CAN 预分频值（位时序固定 10TQ）。
 * 输入参数：baud —— 波特率档位。
 * 输出参数：无
 * 返回值：uint16_t，预分频值（250k→20/500k→10/1M→5，越界回落 500k）。
 */
uint16_t Bsp_Config_BaudToPrescaler(Bsp_Cfg_Baud_T baud)
{
    switch (baud) {
    case BSP_CFG_BAUD_250K: return 20U;
    case BSP_CFG_BAUD_500K: return 10U;
    case BSP_CFG_BAUD_1M:   return 5U;
    default:                return 10U;  /* 默认 500K */
    }
}

/* ---------------- PWM 泄放参数 ---------------- */

/**
 * 功能：读取 PWM 频率（未加载则先加载）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，PWM 频率（Hz）。
 */
uint16_t Bsp_Config_GetPwmFreq(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.pwm_freq_hz;
}

/**
 * 功能：设置 PWM 频率（仅改 RAM，需 Save 落 FRAM）。
 * 输入参数：hz —— PWM 频率（Hz）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界。
 */
Bsp_Cfg_Status_T Bsp_Config_SetPwmFreq(uint16_t hz)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (hz < BSP_CFG_PWM_FREQ_MIN_HZ || hz > BSP_CFG_PWM_FREQ_MAX_HZ) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.pwm_freq_hz = hz;
    return BSP_CFG_OK;
}

/**
 * 功能：读取 PWM 占空比上限（未加载则先加载）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint8_t，占空比上限（%）。
 */
uint8_t Bsp_Config_GetPwmDutyMax(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.pwm_duty_max;
}

/**
 * 功能：设置 PWM 占空比上限（范围 [DUTY_MIN, 安全硬顶]，下调后若启动值超限则钳到新上限）。
 * 输入参数：pct —— 占空比上限（%）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界。
 */
Bsp_Cfg_Status_T Bsp_Config_SetPwmDutyMax(uint8_t pct)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    /* 占空比上限须在 [DUTY_MIN(5), 安全硬顶(90)] 内 */
    if (pct < BSP_CFG_DUTY_MIN || pct > BSP_CFG_DUTY_MAX_CEIL) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.pwm_duty_max = pct;
    /* 上限下调后若启动值超出，钳到新上限 */
    if (s_ram.pwm_duty_start > pct) {
        s_ram.pwm_duty_start = pct;
    }
    return BSP_CFG_OK;
}

/**
 * 功能：读取 PWM 启动占空比（未加载则先加载）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint8_t，启动占空比（%）。
 */
uint8_t Bsp_Config_GetPwmDutyStart(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.pwm_duty_start;
}

/**
 * 功能：设置 PWM 启动占空比（范围 [DUTY_MIN, 当前 duty_max]）。
 * 输入参数：pct —— 启动占空比（%）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界。
 */
Bsp_Cfg_Status_T Bsp_Config_SetPwmDutyStart(uint8_t pct)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    /* 启动占空比须在 [DUTY_MIN(4), 当前 duty_max] 内 */
    if (pct < BSP_CFG_DUTY_MIN || pct > s_ram.pwm_duty_max) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.pwm_duty_start = pct;
    return BSP_CFG_OK;
}

/**
 * 功能：读取 PWM 斜坡步进（未加载则先加载）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint8_t，斜坡步进（%/步）。
 */
uint8_t Bsp_Config_GetPwmDutyStep(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.pwm_duty_step;
}

/**
 * 功能：设置 PWM 斜坡步进（范围 (0, DUTY_STEP_MAX]）。
 * 输入参数：pct —— 斜坡步进（%/步）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界。
 */
Bsp_Cfg_Status_T Bsp_Config_SetPwmDutyStep(uint8_t pct)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (pct == 0U || pct > BSP_CFG_DUTY_STEP_MAX) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.pwm_duty_step = pct;
    return BSP_CFG_OK;
}

/* ---------------- 可标定保护阈值 ---------------- */

/**
 * 功能：读取过压启动阈值（未加载则先加载）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，过压启动阈值（0.1V/LSB）。
 */
uint16_t Bsp_Config_GetOvpOn(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.ovp_on_0v1;
}

/**
 * 功能：设置过压启动阈值（量程内且须高于停止值，维持迟滞 on > off）。
 * 输入参数：v0v1 —— 过压启动阈值（0.1V/LSB）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界或破坏迟滞。
 */
Bsp_Cfg_Status_T Bsp_Config_SetOvpOn(uint16_t v0v1)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    /* 须在量程内且高于停止值（迟滞：on > off） */
    if (!Bsp_Config_VoltInRange(v0v1) || v0v1 <= s_ram.ovp_off_0v1) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.ovp_on_0v1 = v0v1;
    return BSP_CFG_OK;
}

/**
 * 功能：读取过压停止阈值（亦复用为欠压/泄放停止电压）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，过压停止阈值（0.1V/LSB）。
 */
uint16_t Bsp_Config_GetOvpOff(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.ovp_off_0v1;
}

/**
 * 功能：设置过压停止阈值（量程内且须低于启动值，维持迟滞 off < on）。
 * 输入参数：v0v1 —— 过压停止阈值（0.1V/LSB）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界或破坏迟滞。
 */
Bsp_Cfg_Status_T Bsp_Config_SetOvpOff(uint16_t v0v1)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (!Bsp_Config_VoltInRange(v0v1) || v0v1 >= s_ram.ovp_on_0v1) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.ovp_off_0v1 = v0v1;
    return BSP_CFG_OK;
}

/**
 * 功能：读取一级过压报警动作阈值。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，一级报警动作阈值（0.1V/LSB）。
 */
uint16_t Bsp_Config_GetOvpL1(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.ovp_l1_0v1;
}

/**
 * 功能：读取一级过压报警恢复阈值（按固定迟滞派生）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，一级报警恢复阈值（0.1V/LSB）。
 */
uint16_t Bsp_Config_GetOvpL1Off(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.ovp_l1_off_0v1;
}

/**
 * 功能：设置一级过压报警动作阈值，并按固定迟滞派生恢复阈值。
 * 输入参数：v0v1 —— 一级报警动作阈值（0.1V/LSB）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界或低于迟滞量。
 */
Bsp_Cfg_Status_T Bsp_Config_SetOvpL1(uint16_t v0v1)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (!Bsp_Config_VoltInRange(v0v1) || v0v1 <= BSP_CFG_ALARM_HYST_0V1) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.ovp_l1_0v1     = v0v1;
    s_ram.ovp_l1_off_0v1 = (uint16_t)(v0v1 - BSP_CFG_ALARM_HYST_0V1);  /* 恢复值按固定迟滞派生 */
    return BSP_CFG_OK;
}

/**
 * 功能：读取二级过压报警动作阈值。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，二级报警动作阈值（0.1V/LSB）。
 */
uint16_t Bsp_Config_GetOvpL2(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.ovp_l2_0v1;
}

/**
 * 功能：读取二级过压报警恢复阈值（按固定迟滞派生）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，二级报警恢复阈值（0.1V/LSB）。
 */
uint16_t Bsp_Config_GetOvpL2Off(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.ovp_l2_off_0v1;
}

/**
 * 功能：设置二级过压报警动作阈值，并按固定迟滞派生恢复阈值。
 * 输入参数：v0v1 —— 二级报警动作阈值（0.1V/LSB）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界或低于迟滞量。
 */
Bsp_Cfg_Status_T Bsp_Config_SetOvpL2(uint16_t v0v1)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (!Bsp_Config_VoltInRange(v0v1) || v0v1 <= BSP_CFG_ALARM_HYST_0V1) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.ovp_l2_0v1     = v0v1;
    s_ram.ovp_l2_off_0v1 = (uint16_t)(v0v1 - BSP_CFG_ALARM_HYST_0V1);
    return BSP_CFG_OK;
}

/**
 * 功能：读取过温关断阈值。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，过温关断阈值（0.1℃/LSB）。
 */
uint16_t Bsp_Config_GetOtpOff(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.otp_off_0c1;
}

/**
 * 功能：设置过温关断阈值（量程内且须高于恢复阈值）。
 * 输入参数：t0c1 —— 过温关断阈值（0.1℃/LSB）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界或低于恢复阈值。
 */
Bsp_Cfg_Status_T Bsp_Config_SetOtpOff(uint16_t t0c1)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (t0c1 < BSP_CFG_TEMP_MIN_0C1 || t0c1 > BSP_CFG_TEMP_MAX_0C1 || t0c1 <= s_ram.otp_recover_0c1) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.otp_off_0c1 = t0c1;
    return BSP_CFG_OK;
}

/**
 * 功能：读取过温恢复阈值（冷却解除判据）。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，过温恢复阈值（0.1℃/LSB）。
 */
uint16_t Bsp_Config_GetOtpRecover(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.otp_recover_0c1;
}

/**
 * 功能：读取过流阈值。
 * 输入参数：无
 * 输出参数：无
 * 返回值：uint16_t，过流阈值（0.01A/LSB）。
 */
uint16_t Bsp_Config_GetOvc(void)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    return s_ram.ovc_0a01;
}

/**
 * 功能：设置过流阈值（量程内）。
 * 输入参数：i0a01 —— 过流阈值（0.01A/LSB）。
 * 输出参数：无
 * 返回值：Bsp_Cfg_Status_T，OK 成功，ERR_PARAM 越界。
 */
Bsp_Cfg_Status_T Bsp_Config_SetOvc(uint16_t i0a01)
{
    if (!s_loaded) { (void)Bsp_Config_Load(); }
    if (i0a01 < BSP_CFG_CURR_MIN_0A01 || i0a01 > BSP_CFG_CURR_MAX_0A01) {
        return BSP_CFG_ERR_PARAM;
    }
    s_ram.ovc_0a01 = i0a01;
    return BSP_CFG_OK;
}
