/**
 * @file    test_bsp_config.c
 * @brief   设备配置（CAN 参数持久化）单元测试 实现
 *
 * 覆盖接口: SetDefault / Get/Set SelfId / Get/Set MasterId / Get/Set Baud /
 *           Get/Set PwmFreq / Get/Set PwmDutyMax / Get/Set PwmDutyStart / Get/Set PwmDutyStep /
 *           BaudToPrescaler / Save（持久化到 FRAM BSP_CFG_FRAM_ADDR）。
 *
 * 真机说明:
 *   - 本测试会改写 FRAM 配置区（末尾 16 字节 BSP_CFG_FRAM_ADDR），
 *     与黑匣子主数据区及 FRAM 自测区（0x1F00）互不重叠。
 *   - 测试结束会调用 SetDefault + Save，把出厂默认值写回 FRAM，
 *     避免遗留测试用的临时 CAN ID/波特率。
 *
 * 验证要点:
 *   - SetSelfId/SetMasterId 拒绝 0 与 >0x7FF（标准帧 11bit 越界）
 *   - SetBaud 拒绝非法档位（>1M）
 *   - BaudToPrescaler 档位→分频映射正确（250k→20 / 500k→10 / 1M→5）
 *   - SetDefault 后 Get* 返回默认宏值
 *   - RAM Set/Get 往返一致
 *   - Save 后直接读 FRAM 原始字节，校验 magic / 字段 / CRC 持久化正确
 *   - 同值再次 Save 走跳过分支（配置值与存储值一致则跳过写），仍返回 OK
 *
 * @version 2.0.0
 * @date    2026-06-13
 */

#include "test_bsp_config.h"

#define BSP_USE_CONFIG
#define BSP_USE_FRAM
#include "bsp.h"

/* 与 bsp_config.c 内部一致的存储布局常量（用于读回校验, 34 字节 v3，含可标定阈值） */
#define TEST_CFG_MAGIC          0x47444347U     /* "GDCG" v3 */
#define TEST_CFG_BLOB_LEN       34U             /* magic(4)+CAN/PWM(10)+阈值(18)+crc(2) */
#define TEST_CFG_CRC_PAYLOAD    28U             /* CRC 覆盖 self_id..ovc_0a01 (偏移 4~31) */
#define TEST_CFG_OFF_MAGIC      0U
#define TEST_CFG_OFF_SELF       4U
#define TEST_CFG_OFF_MASTER     6U
#define TEST_CFG_OFF_BAUD       8U
#define TEST_CFG_OFF_DUTYMAX    9U
#define TEST_CFG_OFF_DUTYSTEP   10U
#define TEST_CFG_OFF_DUTYSTART  11U
#define TEST_CFG_OFF_FREQ       12U
#define TEST_CFG_OFF_OVP_ON     14U
#define TEST_CFG_OFF_OVP_OFF    16U
#define TEST_CFG_OFF_OVC        30U
#define TEST_CFG_OFF_CRC        32U

/* 与 bsp_config.c 内部一致的 CRC-16/XMODEM（poly 0x1021, init 0x0000），
 * 用于在测试侧独立复算并比对 Flash 中存储的 CRC，验证持久化完整性 */
static uint16_t Test_Cfg_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0U;
    uint16_t i, j;

    for (i = 0U; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (j = 0U; j < 8U; j++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t Test_Cfg_Rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t Test_Cfg_Rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void Test_BspConfig_RunAll(Test_Stats_T *_stats)
{
    uint8_t blob[TEST_CFG_BLOB_LEN];

    /* 确保 FRAM/SPI 总线就绪（Save/Load 内部亦会幂等初始化） */
    TEST_CASE("Config: FRAM 总线初始化");
    TEST_ASSERT_EQ(Bsp_Fram_Init(), BSP_FRAM_OK);

    /* ===== 参数校验：自身 CAN ID ===== */
    TEST_CASE("Config: SetSelfId 参数校验 (0 / >0x7FF 拒绝)");
    TEST_ASSERT_EQ(Bsp_Config_SetSelfId(0U), BSP_CFG_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Config_SetSelfId(0x800U), BSP_CFG_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Config_SetSelfId(0x123U), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetSelfId(), 0x123U);

    /* ===== 参数校验：主站 CAN ID ===== */
    TEST_CASE("Config: SetMasterId 参数校验 (0 / >0x7FF 拒绝)");
    TEST_ASSERT_EQ(Bsp_Config_SetMasterId(0U), BSP_CFG_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Config_SetMasterId(0x800U), BSP_CFG_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Config_SetMasterId(0x222U), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetMasterId(), 0x222U);

    /* ===== 参数校验：波特率档位 ===== */
    TEST_CASE("Config: SetBaud 参数校验 (非法档位拒绝)");
    TEST_ASSERT_EQ(Bsp_Config_SetBaud((Bsp_Cfg_Baud_T)3U), BSP_CFG_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Config_SetBaud(BSP_CFG_BAUD_1M), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetBaud(), BSP_CFG_BAUD_1M);

    /* ===== 波特率档位 → 分频映射 ===== */
    TEST_CASE("Config: BaudToPrescaler 映射 (250k=20 / 500k=10 / 1M=5)");
    TEST_ASSERT_EQ_INT(Bsp_Config_BaudToPrescaler(BSP_CFG_BAUD_250K), 20U);
    TEST_ASSERT_EQ_INT(Bsp_Config_BaudToPrescaler(BSP_CFG_BAUD_500K), 10U);
    TEST_ASSERT_EQ_INT(Bsp_Config_BaudToPrescaler(BSP_CFG_BAUD_1M),   5U);
    TEST_ASSERT_EQ_INT(Bsp_Config_BaudToPrescaler((Bsp_Cfg_Baud_T)9U), 10U); /* 越界回落 500k */

    /* ===== 参数校验：PWM 频率 ===== */
    TEST_CASE("Config: SetPwmFreq 参数校验 (越界拒绝)");
    TEST_ASSERT_EQ(Bsp_Config_SetPwmFreq((uint16_t)(BSP_CFG_PWM_FREQ_MIN_HZ - 1U)), BSP_CFG_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Config_SetPwmFreq(60000U), BSP_CFG_ERR_PARAM);   /* >50kHz */
    TEST_ASSERT_EQ(Bsp_Config_SetPwmFreq(20000U), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetPwmFreq(), 20000U);

    /* ===== 参数校验：PWM 占空比上限 / 启动值 / 步进 ===== */
    TEST_CASE("Config: SetPwmDutyMax 参数校验 (<5 / >90 拒绝, 硬顶 90%)");
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyMax(3U), BSP_CFG_ERR_PARAM);    /* 低于下限 5% */
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyMax(91U), BSP_CFG_ERR_PARAM);   /* 超 90% 安全硬顶 */
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyMax(8U), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetPwmDutyMax(), 8U);

    TEST_CASE("Config: SetPwmDutyStart 参数校验 (<4 / >duty_max 拒绝)");
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyStart(3U), BSP_CFG_ERR_PARAM);  /* 低于下限 4% */
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyStart(9U), BSP_CFG_ERR_PARAM);  /* 超当前 duty_max=8 */
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyStart(4U), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetPwmDutyStart(), 4U);

    TEST_CASE("Config: SetPwmDutyStep 参数校验 (0 / 越界拒绝)");
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyStep(0U), BSP_CFG_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyStep((uint8_t)(BSP_CFG_DUTY_STEP_MAX + 1U)), BSP_CFG_ERR_PARAM);
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyStep(2U), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetPwmDutyStep(), 2U);

    /* ===== 参数校验：可标定保护阈值 ===== */
    TEST_CASE("Config: SetOvpOn/Off 迟滞与范围校验 (on>off, 量程内)");
    TEST_ASSERT_EQ(Bsp_Config_SetOvpOff(540U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetOvpOn(540U), BSP_CFG_ERR_PARAM);   /* on<=off 拒绝 */
    TEST_ASSERT_EQ(Bsp_Config_SetOvpOn(9999U), BSP_CFG_ERR_PARAM);  /* 超量程 */
    TEST_ASSERT_EQ(Bsp_Config_SetOvpOn(565U), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOvpOn(), 565U);

    TEST_CASE("Config: SetOvpL1/L2 写动作值, 恢复值按迟滞派生");
    TEST_ASSERT_EQ(Bsp_Config_SetOvpL1(565U), BSP_CFG_OK);   /* 一级报警 56.5V */
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOvpL1(), 565U);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOvpL1Off(), (uint16_t)(565U - BSP_CFG_ALARM_HYST_0V1));
    TEST_ASSERT_EQ(Bsp_Config_SetOvpL2(580U), BSP_CFG_OK);   /* 二级报警 58.0V（无 60V） */
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOvpL2Off(), (uint16_t)(580U - BSP_CFG_ALARM_HYST_0V1));

    TEST_CASE("Config: SetOtpOff/SetOvc 范围校验");
    TEST_ASSERT_EQ(Bsp_Config_SetOtpOff(50U), BSP_CFG_ERR_PARAM);   /* 低于温度下限 */
    TEST_ASSERT_EQ(Bsp_Config_SetOtpOff(1000U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetOvc(50U), BSP_CFG_ERR_PARAM);      /* 低于电流下限 1A */
    TEST_ASSERT_EQ(Bsp_Config_SetOvc(1200U), BSP_CFG_OK);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOvc(), 1200U);

    /* ===== 恢复默认值 ===== */
    TEST_CASE("Config: SetDefault 后 Get* 返回默认宏值");
    Bsp_Config_SetDefault();
    TEST_ASSERT_EQ_INT(Bsp_Config_GetSelfId(),   BSP_CFG_DEFAULT_SELF_ID);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetMasterId(), BSP_CFG_DEFAULT_MASTER_ID);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetBaud(),     BSP_CFG_DEFAULT_BAUD);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetPwmFreq(),      BSP_CFG_DEFAULT_PWM_FREQ_HZ);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetPwmDutyMax(),   BSP_CFG_DEFAULT_DUTY_MAX);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetPwmDutyStart(), BSP_CFG_DEFAULT_DUTY_START);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetPwmDutyStep(),  BSP_CFG_DEFAULT_DUTY_STEP);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOvpOn(),    BSP_CFG_DEFAULT_OVP_ON_0V1);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOvpOff(),   BSP_CFG_DEFAULT_OVP_OFF_0V1);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOtpOff(),   BSP_CFG_DEFAULT_OTP_OFF_0C1);
    TEST_ASSERT_EQ_INT(Bsp_Config_GetOvc(),      BSP_CFG_DEFAULT_OVC_0A01);

    /* ===== Save 持久化 → 读回 Flash 原始字节校验 ===== */
    TEST_CASE("Config: 设定已知值并 Save 到 Flash");
    TEST_ASSERT_EQ(Bsp_Config_SetSelfId(0x1AAU),   BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetMasterId(0x2BBU), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetBaud(BSP_CFG_BAUD_250K), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetPwmFreq(18000U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyMax(9U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyStart(6U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetPwmDutyStep(3U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetOvpOff(530U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetOvpOn(555U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_SetOvc(1100U), BSP_CFG_OK);
    TEST_ASSERT_EQ(Bsp_Config_Save(), BSP_CFG_OK);

    TEST_CASE("Config: 读回 FRAM 校验 magic/字段/CRC 持久化正确");
    TEST_ASSERT_EQ(Bsp_Fram_Read(BSP_CFG_FRAM_ADDR, blob, TEST_CFG_BLOB_LEN), BSP_FRAM_OK);
    TEST_ASSERT_EQ_INT(Test_Cfg_Rd32(&blob[TEST_CFG_OFF_MAGIC]), TEST_CFG_MAGIC);
    TEST_ASSERT_EQ_INT(Test_Cfg_Rd16(&blob[TEST_CFG_OFF_SELF]),   0x1AAU);
    TEST_ASSERT_EQ_INT(Test_Cfg_Rd16(&blob[TEST_CFG_OFF_MASTER]), 0x2BBU);
    TEST_ASSERT_EQ_INT(blob[TEST_CFG_OFF_BAUD], (uint8_t)BSP_CFG_BAUD_250K);
    TEST_ASSERT_EQ_INT(blob[TEST_CFG_OFF_DUTYMAX],   9U);
    TEST_ASSERT_EQ_INT(blob[TEST_CFG_OFF_DUTYSTEP],  3U);
    TEST_ASSERT_EQ_INT(blob[TEST_CFG_OFF_DUTYSTART], 6U);
    TEST_ASSERT_EQ_INT(Test_Cfg_Rd16(&blob[TEST_CFG_OFF_FREQ]), 18000U);
    TEST_ASSERT_EQ_INT(Test_Cfg_Rd16(&blob[TEST_CFG_OFF_OVP_ON]),  555U);
    TEST_ASSERT_EQ_INT(Test_Cfg_Rd16(&blob[TEST_CFG_OFF_OVP_OFF]), 530U);
    TEST_ASSERT_EQ_INT(Test_Cfg_Rd16(&blob[TEST_CFG_OFF_OVC]),     1100U);
    /* CRC 覆盖 self_id..pwm_freq_hz 共 10 字节（偏移 4~13） */
    TEST_ASSERT_EQ_INT(Test_Cfg_Rd16(&blob[TEST_CFG_OFF_CRC]),
                       Test_Cfg_Crc16(&blob[TEST_CFG_OFF_SELF], TEST_CFG_CRC_PAYLOAD));
    TEST_LOG_VAL("Config 持久化: self=0x%03X master=0x%03X baud=%u freq=%u crc=0x%04X",
                 Test_Cfg_Rd16(&blob[TEST_CFG_OFF_SELF]),
                 Test_Cfg_Rd16(&blob[TEST_CFG_OFF_MASTER]),
                 blob[TEST_CFG_OFF_BAUD],
                 Test_Cfg_Rd16(&blob[TEST_CFG_OFF_FREQ]),
                 Test_Cfg_Rd16(&blob[TEST_CFG_OFF_CRC]));

    /* ===== 同值再次 Save → 走跳过分支（防磨损） ===== */
    TEST_CASE("Config: 同值再次 Save 仍返回 OK (跳过写, 防磨损)");
    TEST_ASSERT_EQ(Bsp_Config_Save(), BSP_CFG_OK);

    /* ===== 清理：写回出厂默认，避免遗留测试值 ===== */
    TEST_CASE("Config: 测试后恢复默认配置并持久化");
    Bsp_Config_SetDefault();
    TEST_ASSERT_EQ(Bsp_Config_Save(), BSP_CFG_OK);
}
