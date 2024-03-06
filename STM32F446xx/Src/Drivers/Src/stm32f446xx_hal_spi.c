/*
 ******************************************************************************
 * @file      stm32f446xx_hal_spi.c
 * @author    Rahul Singh
 * @brief     Source file for STM32F446xx SPI driver
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
#include "stm32f446xx_hal_spi.h"

/*
 * @brief        Enable or Disable SPI peripheral clock
 *
 * @param[in]    pSPIx : SPI peripheral base address
 * @param[in]    en    : 0 -> disable, !0-> enable
 *
 * @return       None
 */
static void HAL_SPI_pclk_ctrl(SPI_reg_t *pSPIx, bool en)
{
	NULL_PTR_CHK(pSPIx);

	/* check whether to enable or disable the SPI peripheral clock */
	if(en) {
		/* enable the specified SPI peripheral clock */
		if(pSPIx == SPI1) { __SPI1_CLK_EN(); }
		else if(pSPIx == SPI2) { __SPI2_CLK_EN(); }
		else if(pSPIx == SPI3) { __SPI3_CLK_EN(); }
		else if(pSPIx == SPI4) { __SPI4_CLK_EN(); }
	} else {
		/* disable the specified SPI peripheral clock */
		if(pSPIx == SPI1) { __SPI1_CLK_DIS(); }
		else if(pSPIx == SPI2) { __SPI2_CLK_DIS(); }
		else if(pSPIx == SPI3) { __SPI3_CLK_DIS(); }
		else if(pSPIx == SPI4) { __SPI4_CLK_DIS(); }
	}
}

/*
 * @brief        Get SPI peripheral flag status
 *
 * @param[in]    pSPIx : SPI peripheral base address
 * @param[in]    flag  : Bit mask of flag in SPI status register
 *
 * @return       flag status : 0(FLAG_NOT_SET) or 1(FLAG_SET)
 */
uint8_t HAL_SPI_flag_status(SPI_reg_t *pSPIx, uint8_t flag) {
	if(pSPIx->SR & flag) {
		return FLAG_SET;
	}
	return FLAG_NOT_SET;
}

/*
 * @brief        Enable or Disable SPI peripheral
 *
 * @param[in]    pSPIx : SPI peripheral base address
 * @param[in]    en    : 0 -> disable, !0-> enable
 *
 * @return       None
 */
void HAL_SPI_ctrl(SPI_reg_t *pSPIx, uint8_t en) {
	if(en) {
		pSPIx->CR1 |= (0x1 << BITP_SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(0x1 << BITP_SPI_CR1_SPE);
	}
}

/*
 * @brief        Initialize/Setup/Configure SPI peripheral
 *
 * @param[in]    pSPIhandle : Pointer to SPI handle object
 *
 * @return       None
 */
void HAL_SPI_init(SPI_handle_t *pSPIhandle)
{
	NULL_PTR_CHK(pSPIhandle);

	uint32_t reg_val = 0;

	/* enable the SPI peripheral clock */
	HAL_SPI_pclk_ctrl(pSPIhandle->pSPIx, true);

	/* set device mode */
	reg_val |= pSPIhandle->cfg.dev_mode << BITP_SPI_CR1_MSTR;

	/* set sclk rate */
	reg_val |= pSPIhandle->cfg.speed << BITP_SPI_CR1_BR;

	/* set sclk polarity */
	reg_val |= pSPIhandle->cfg.cpol << BITP_SPI_CR1_CPOL;

	/* set sclk phase */
	reg_val |= pSPIhandle->cfg.cpha << BITP_SPI_CR1_CPHA;

	/* set data frame format */
	reg_val |= pSPIhandle->cfg.dff << BITP_SPI_CR1_DFF;

	/* set communication mode */
	if(pSPIhandle->cfg.comm_mode == SPI_COMM_MODE_HALF_DUPLEX) {
		/* set BIDIMODE bit to enable 1-line bidirectional mode */
		reg_val |= (0x1 << BITP_SPI_CR1_BIDIMODE);
	} else if(pSPIhandle->cfg.comm_mode == SPI_COMM_MODE_SIMPLEX_RX) {
		/* clear BIDIMODE bit to enable 2-line unidirectional mode */
		reg_val &= ~(0x1 << BITP_SPI_CR1_BIDIMODE);
		/* set RXONLY bit for Rx only mode */
		reg_val |= (0x1 << BITP_SPI_CR1_RXONLY);
	} else { /* when pSPIhandle->cfg.comm_mode == SPI_COMM_MODE_FULL_DUPLEX  */
		/* clear BIDIMODE bit to enable 2-line unidirectional mode */
		reg_val &= ~(0x1 << BITP_SPI_CR1_BIDIMODE);
		/* clear RXONLY bit for full duplex mode */
		reg_val &= ~(0x1 << BITP_SPI_CR1_RXONLY);
	}

	/* set slave management type */
	reg_val |= pSPIhandle->cfg.ssm << BITP_SPI_CR1_SSM;

	/* configure slave management */
	if(pSPIhandle->cfg.dev_mode == SPI_MASTER_MODE){
		if(pSPIhandle->cfg.ssm == SPI_SW_SLAVE_MGMT) {
			/* set the SSI bit to avoid MODF mode fault */
			reg_val |= (0x1 << BITP_SPI_CR1_SSI); //NSS pin high
		} else {  //SPI_HW_SLAVE_MGMT
			/* set SSOE bit i.e. output enable to configure NSS pin
		   * as slave select output controlled by the hardware */
			pSPIhandle->pSPIx->CR2 |= (0x1 << BITP_SPI_CR2_SSOE);
		}
	} else{
		if(pSPIhandle->cfg.ssm == SPI_SW_SLAVE_MGMT) {
			/*  do nothing as by default SSI=0 i.e. NSS pin low */
			//reg_val &= ~(0x1 << BITP_SPI_CR1_SSI);
		} else {  //SPI_HW_SLAVE_MGMT
			/* clear SSOE bit i.e. output disable to configure NSS pin
			 * as slave select input controlled by the hardware */
			pSPIhandle->pSPIx->CR2 &= ~(0x1 << BITP_SPI_CR2_SSOE);
		}
	}

	/* enable the SPI peripheral */
	//reg_val |= 0x1 << BITP_SPI_CR1_SPE; //done through a separate function

	/* set the SPI ctrl register with the
	 * value as per specified configuration */
	pSPIhandle->pSPIx->CR1 = reg_val;
}

/*
 * @brief        De-initialize SPI peripheral
 *
 * @param[in]    pSPIx : SPI peripheral base address
 *
 * @return       None
 */
void HAL_SPI_deinit(SPI_reg_t *pSPIx)
{
	NULL_PTR_CHK(pSPIx);

	/* reset the specified SPI peripheral clock */
	if(pSPIx == SPI1) { __SPI1_RST(); }
	else if(pSPIx == SPI2) { __SPI2_RST(); }
	else if(pSPIx == SPI3) { __SPI3_RST(); }
	else if(pSPIx == SPI4) { __SPI4_RST(); }

	/* disable the SPI peripheral clock */
	HAL_SPI_pclk_ctrl(pSPIx, false);
}

/*
 * @brief        Read data over SPI peripheral
 *
 * @param[in]    pSPIx     : SPI peripheral base address
 * @param[in]    pRxBuffer : Pointer to the buffer storing received data
 * @param[in]    len       : Size of data to be received (in bytes)
 *
 * @return       None
 */
void HAL_SPI_read_data(SPI_reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	NULL_PTR_CHK(pSPIx);
	NULL_PTR_CHK(pRxBuffer);

	if(len == 0) { return; }

	/* check the data frame format */
	uint8_t dff = (pSPIx->CR1 >> BITP_SPI_CR1_DFF) & 0x1;

	/* iterate over len bytes */
	while(len > 0) {
		/* wait till HW Rx buffer is empty */
		while(HAL_SPI_flag_status(pSPIx, SPI_SR_RXNE_FLAG) == FLAG_NOT_SET);

		if(dff == SPI_DATA_FRAME_8BIT) {
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		} else {
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len--; len--;
			pRxBuffer++; pRxBuffer++; //equivalent to (uint16_t*)pRxBuffer++;
		}
	}
}

/*
 * @brief        Send data over SPI peripheral
 *
 * @param[in]    pSPIx     : SPI peripheral base address
 * @param[in]    pTxBuffer : Pointer to the buffer sending data to be sent
 * @param[in]    len       : Size of data to be sent (in bytes)
 *
 * @return       None
 */
void HAL_SPI_send_data(SPI_reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	NULL_PTR_CHK(pSPIx);
	NULL_PTR_CHK(pTxBuffer);

	if(len == 0) { return; }

	/* check the data frame format */
	uint8_t dff = (pSPIx->CR1 >> BITP_SPI_CR1_DFF) & 0x1;

	/* iterate over len bytes */
	while(len > 0) {
		/* wait till HW Tx buffer is not empty */
		while(HAL_SPI_flag_status(pSPIx, SPI_SR_TXE_FLAG) == FLAG_NOT_SET);

		if(dff == SPI_DATA_FRAME_8BIT) {
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		} else {
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--; len--;
			pTxBuffer++; pTxBuffer++;
		}
	}
}
