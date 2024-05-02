/*
 ******************************************************************************
 * @file      stm32f446xx_hal_rcc.h
 * @author    Rahul Singh
 * @brief     Header file for STM32F446xx RCC driver
 *
 ******************************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2024 Rahul Singh
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************
*/

#ifndef _STM32F446XX_HAL_RCC_H_
#define _STM32F446XX_HAL_RCC_H_

#include "stm32f446xx.h"

/* ******************* Macro Definitions ********************* */

/* System Clock Source */
#define RCC_SCLK_SRC_HSI_OSC           (0x00U)
#define RCC_SCLK_SRC_HSE_OSC           (0x01U)
#define RCC_SCLK_SRC_PLL_P             (0x02U)
#define RCC_SCLK_SRC_PLL_R             (0x03U)

/* AHB/APB1/APB2 minimum prescalar register value */
#define RCC_AHB_MIN_PRSCLR_REG_VAL     (0x8U)
#define RCC_APB1_MIN_PRSCLR_REG_VAL    (0x4U)
#define RCC_APB2_MIN_PRSCLR_REG_VAL    (0x4U)

/* RCC CFGR register bit positions */
#define BITP_RCC_CFGR_SW               (0U)
#define BITP_RCC_CFGR_SWS              (2U)
#define BITP_RCC_CFGR_HPRE             (4U)
#define BITP_RCC_CFGR_PRE1             (10U)
#define BITP_RCC_CFGR_PRE2             (13U)
#define BITP_RCC_CFGR_RTCPRE           (16U)
#define BITP_RCC_CFGR_MCO1             (21U)
#define BITP_RCC_CFGR_MCO1PRE          (24U)
#define BITP_RCC_CFGR_MCO2PRE          (27U)
#define BITP_RCC_CFGR_MCO2             (30U)

/* RCC CFGR register bit mask */
#define BITM_RCC_CFGR_SW               (0x3 << BITP_RCC_CFGR_SW)
#define BITM_RCC_CFGR_SWS              (0x3 << BITP_RCC_CFGR_SWS)
#define BITM_RCC_CFGR_HPRE             (0xF << BITP_RCC_CFGR_SW)
#define BITM_RCC_CFGR_PRE1             (0x7 << BITP_RCC_CFGR_SW)
#define BITM_RCC_CFGR_PRE2             (0x7 << BITP_RCC_CFGR_SW)
#define BITM_RCC_CFGR_RTCPRE           (0x1F << BITP_RCC_CFGR_SW)
#define BITM_RCC_CFGR_MCO1             (0x3 << BITP_RCC_CFGR_SW)
#define BITM_RCC_CFGR_MCO1PRE          (0x7 << BITP_RCC_CFGR_SW)
#define BITM_RCC_CFGR_MCO2PRE          (0x7 << BITP_RCC_CFGR_SW)
#define BITM_RCC_CFGR_MCO2             (0x3 << BITP_RCC_CFGR_SW)

/* ***************** Function Declarations ******************* */

/*
 * @brief        Get APB1 peripheral clock frequency
 *
 * @return       APB1 peripheral clock frequency in Hz
 */
uint32_t RCC_getPCLK1freq();

/*
 * @brief        Get APB2 peripheral clock frequency
 *
 * @return       APB2 peripheral clock frequency in Hz
 */
uint32_t RCC_getPCLK2freq();

#endif
