/**
 * @file    test_bsp_can.h
 * @brief   CAN通信单元测试 头文件
 * @version 1.0.0
 * @date    2026-06-12
 */

#ifndef TEST_BSP_CAN_H
#define TEST_BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "test_framework.h"

void Test_BspCan_RunAll(Test_Stats_T *stats);

/** CAN 交互测试: 循环监听主站命令并应答, 配合上位机实时验证收发 */
void Test_BspCan_Interactive(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_BSP_CAN_H */
