/*
 ******************************************************************************
 * @file      stm32f446xx_hal_rcc.c
 * @author    Rahul Singh
 * @brief     Source file for STM32F446xx RCC driver
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

#include <stdbool.h>
#include "stm32f446xx_hal_rcc.h"


const uint16_t AHB_prescalar[] = {2, 4, 8, 16, 64, 128, 256, 512};
const uint8_t APB1_prescalar[] = {2, 4, 8, 16};
const uint8_t APB2_prescalar[] = {2, 4, 8, 16};

/*
 * @brief        Get APB1 peripheral clock frequency
 *
 * @return       APB1 peripheral clock frequency in Hz
 */
uint32_t RCC_getPCLK1freq() {
	uint32_t pclk1_freq = 0;

	/* determine the clock source */
	uint32_t reg_val = RCC->CFGR;

	uint8_t sys_clk_sws = (reg_val & BITM_RCC_CFGR_SWS) >> BITP_RCC_CFGR_SWS;
	if(sys_clk_sws == RCC_SCLK_SRC_HSI_OSC) {
		pclk1_freq = HSI_OSC_FREQ;
	} else if (sys_clk_sws == RCC_SCLK_SRC_HSE_OSC) {
		pclk1_freq = HSE_OSC_FREQ;
	} else {
		/* TODO: add support for PLL */
		pclk1_freq = HSI_OSC_FREQ;
	}

	/* get the AHB prescalar value */
	uint8_t temp = (reg_val & BITM_RCC_CFGR_HPRE) >> BITP_RCC_CFGR_HPRE;
	uint8_t ahb_prescalar = (temp < RCC_AHB_MIN_PRSCLR_REG_VAL) ?
				1 : AHB_prescalar[temp - RCC_AHB_MIN_PRSCLR_REG_VAL];

	/* get the APB1 prescalar value */
	temp = (reg_val & BITM_RCC_CFGR_PRE1) >> BITP_RCC_CFGR_PRE1;
	uint8_t apb1_prescalar = (temp < RCC_APB1_MIN_PRSCLR_REG_VAL) ?
				1 : APB1_prescalar[temp - RCC_APB1_MIN_PRSCLR_REG_VAL];

	/* obtain the APB1 clock frequency after
	 * dividing AHB and APB1 prescalar value */
	pclk1_freq /= ahb_prescalar;
	pclk1_freq /= apb1_prescalar;

	return pclk1_freq;
}

/*
 * @brief        Get APB2 peripheral clock frequency
 *
 * @return       APB2 peripheral clock frequency in Hz
 */
uint32_t RCC_getPCLK2freq() {
	uint32_t pclk2_freq = 0;

	/* determine the clock source */
	uint32_t reg_val = RCC->CFGR;

	uint8_t sys_clk_sws = (reg_val & BITM_RCC_CFGR_SWS) >> BITP_RCC_CFGR_SWS;
	if(sys_clk_sws == RCC_SCLK_SRC_HSI_OSC) {
		pclk2_freq = HSI_OSC_FREQ;
	} else if (sys_clk_sws == RCC_SCLK_SRC_HSE_OSC) {
		pclk2_freq = HSE_OSC_FREQ;
	} else {
		/* TODO: add support for PLL */
		pclk2_freq = HSI_OSC_FREQ;
	}

	/* get the AHB prescalar value */
	uint8_t temp = (reg_val & BITM_RCC_CFGR_HPRE) >> BITP_RCC_CFGR_HPRE;
	uint8_t ahb_prescalar = (temp < RCC_AHB_MIN_PRSCLR_REG_VAL) ?
			1 : AHB_prescalar[temp - RCC_AHB_MIN_PRSCLR_REG_VAL];

	/* get the APB2 prescalar value */
	temp = (reg_val & BITM_RCC_CFGR_PRE2) >> BITP_RCC_CFGR_PRE2;
	uint8_t apb2_prescalar = (temp < RCC_APB2_MIN_PRSCLR_REG_VAL) ?
			1 : APB2_prescalar[temp - RCC_APB2_MIN_PRSCLR_REG_VAL];

	/* obtain the APB2 clock frequency after
	 * dividing AHB and APB1 prescalar value */
	pclk2_freq /= ahb_prescalar;
	pclk2_freq /= apb2_prescalar;

	return pclk2_freq;
}
