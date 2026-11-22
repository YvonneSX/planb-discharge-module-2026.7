/*!
    \file    gd32f30x_it.c
    \brief   interrupt service routines

    \version 2025-08-20, V3.0.2, demo for GD32F30x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x_it.h"
#include "systick.h"
#include "app_features.h"   /* APP_CAN_ENABLE：CAN 应用+中断总开关 */

/* CAN RX FIFO0 中断处理由 BSP 提供（bsp_can.c, BSP_USE_CAN 时编入）。
 * 此处仅做向量分发，避免在 it.c 引入完整 bsp.h。仅 APP_CAN_ENABLE 时启用。 */
#if APP_CAN_ENABLE
extern void Bsp_Can_Rx0IrqHandler(void);
#endif

/* ADC0 模拟看门狗（过压/欠压快速响应）中断处理由 BSP 提供（bsp_analog.c）。
 * ADC0_1 向量在 startup 为 weak，此处覆盖为向量分发。 */
extern void Bsp_Analog_WatchdogIrq(void);

/* TIMER1 步进节拍（分级泄放 100µs 步进/二级判定）中断，由 BSP 提供（bsp_timer.c）。 */
extern void Bsp_StepTimer_Irq(void);

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

/*!
    \brief      this function handles CAN0 RX0 exception (RX FIFO0 非空)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBD_LP_CAN0_RX0_IRQHandler(void)
{
#if APP_CAN_ENABLE
    Bsp_Can_Rx0IrqHandler();
#endif
}

/*!
    \brief      this function handles ADC0/ADC1 exception (模拟看门狗 WDE: 过压/欠压快速响应)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC0_1_IRQHandler(void)
{
    Bsp_Analog_WatchdogIrq();
}

/*!
    \brief      this function handles TIMER1 exception (步进节拍: 分级泄放 100us 步进/二级判定)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER1_IRQHandler(void)
{
    Bsp_StepTimer_Irq();
}
