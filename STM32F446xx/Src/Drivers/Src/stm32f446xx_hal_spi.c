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

/* SPI interrupt events handler declaration */
static void _HAL_SPI_rxne_evt_handler(SPI_handle_t *pSPIhandle);
static void _HAL_SPI_txe_evt_handler(SPI_handle_t *pSPIhandle);
static void _HAL_SPI_err_evt_handler(SPI_handle_t *pSPIhandle);
static void _HAL_SPI_ovr_evt_handler(SPI_handle_t *pSPIhandle);

/*
 * @brief        Enable or Disable SPI peripheral clock
 *
 * @param[in]    pSPIx : SPI peripheral base address
 * @param[in]    en    : 0 -> disable, !0-> enable
 *
 * @return       None
 */
static void _HAL_SPI_pclk_ctrl(SPI_reg_t *pSPIx, bool en)
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
	_HAL_SPI_pclk_ctrl(pSPIhandle->pSPIx, true);

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
	_HAL_SPI_pclk_ctrl(pSPIx, false);
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

	/* if there is nothing to be received then return */
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

	/* if there is nothing to be sent then return */
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

/*
 * @brief        Read data over SPI peripheral (non-blocking mode)
 *
 * @param[in]    pSPIhandle: Pointer to SPI handle object
 * @param[in]    pRxBuffer : Pointer to the buffer storing received data
 * @param[in]    len       : Size of data to be received (in bytes)
 *
 * @return       SPI peripheral Rx state when this function is called
 *
 */
HAL_SPI_state_t HAL_SPI_read_data_IT(SPI_handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t len)
{
	if(pSPIhandle == NULL) { return 0xFF; }
	if(pRxBuffer == NULL) { return 0xFF; }

	/* if there is nothing to be received then return */
	if(len == 0) { return 0xFF; }

	uint8_t rxState = pSPIhandle->rxState;

	if(rxState == HAL_SPI_READY)
	{
		/* store address of the buffer where
		 * the received data needs to be stored
		 * and the size of data */
		pSPIhandle->pRxBuff = pRxBuffer;
		pSPIhandle->rxLen = len;

		/* set the SPI RX busy status */
		pSPIhandle->rxState = HAL_SPI_BUSY_IN_RX;

		/* enable RXNE interrupt to processor when data is received */
		pSPIhandle->pSPIx->CR2 |= (0x1 << BITP_SPI_CR2_RXNEIE);
	}

	return rxState;
}

/*
 * @brief        Send data over SPI peripheral (non-blocking mode)
 *
 * @param[in]    pSPIhandle: Pointer to SPI handle object
 * @param[in]    pTxBuffer : Pointer to the buffer sending data to be sent
 * @param[in]    len       : Size of data to be sent (in bytes)
 *
 * @return       SPI peripheral Tx state when this function is called
 *
 */
HAL_SPI_state_t HAL_SPI_send_data_IT(SPI_handle_t *pSPIhandle, uint8_t *pTxBuffer, uint32_t len)
{
	if(pSPIhandle == NULL) { return 0xFF; }
	if(pTxBuffer == NULL) { return 0xFF; }

	/* if there is nothing to be sent then return */
	if(len == 0) { return 0xFF; }

	uint8_t txState = pSPIhandle->txState;

	if(txState == HAL_SPI_READY)
	{
		/* store address of the buffer where
		 * the data to be sent is stored
		 * and the size of data */
		pSPIhandle->pTxBuff = pTxBuffer;
		pSPIhandle->txLen = len;

		/* set the SPI TX busy status */
		pSPIhandle->txState = HAL_SPI_BUSY_IN_TX;

		/* enable TXE interrupt to processor when SPI TX buffer is empty */
		pSPIhandle->pSPIx->CR2 |= (0x1 << BITP_SPI_CR2_TXEIE);
	}

	return txState;
}

/*
 * @brief        Configure the IRQ for a SPI peripheral
 *
 * @param[in]    IRQ_num  : IRQ number for a specific SPI peripheral
 * @param[in]    IRQ_prio : Priority level for the specified IRQ (0 to 16)
 * @param[in]    en       : 1 -> enable the IRQ, 0-> Disable the IRQ
 *
 * @return       None
 *
 */
void HAL_SPI_IRQ_config(uint32_t IRQ_num, uint32_t IRQ_prio, uint8_t en)
{
	/* set the priority level for the IRQn in the NVIC */
	__NVIC_SET_PRIORITY(IRQ_num, IRQ_prio);

	/* enable or disable the IRQn in the NVIC */
	if(en) {
		__NVIC_ENABLE_IRQ(IRQ_num);
	} else {
		__NVIC_DISABLE_IRQ(IRQ_num);
	}
}

/*
 * @brief        SPI IRQ handler
 *
 * @param[in]    pSPIhandle: Pointer to SPI handle object
 *
 * @return       None
 *
 */
void HAL_SPI_IRQ_handler(SPI_handle_t *pSPIhandle)
{
	NULL_PTR_CHK(pSPIhandle);

	uint8_t status = pSPIhandle->pSPIx->SR;
	uint8_t cr2_val = pSPIhandle->pSPIx->CR2;

	/* check which SPI event generated the interrupt
	 * and call the respective handler to service it */

	/* checking if Rx buffer not empty
	 * event generated the interrupt */
	if((cr2_val & (0x1 << BITP_SPI_CR2_RXNEIE))
		&& (status & (0x1 << BITP_SPI_SR_RXNE)))
	{
		/* call the Rx buffer not empty event handler */
		_HAL_SPI_rxne_evt_handler(pSPIhandle);
	}
	/* checking if Tx buffer empty event
	* generated the interrupt */
	if((cr2_val & (0x1 << BITP_SPI_CR2_TXEIE))
			&& (status & (0x1 << BITP_SPI_SR_TXE)))
	{
		/* call the Tx buffer empty event handler */
		_HAL_SPI_txe_evt_handler(pSPIhandle);
	}
	/* checking if any SPI error event
	 * generated the interrupt */
	if(cr2_val & (0x1 << BITP_SPI_CR2_ERRIE))
	{
		/* if overrun error */
		if(status & (0x1 << BITP_SPI_SR_OVR)) {
			 /* call the overrun event handler */
			_HAL_SPI_ovr_evt_handler(pSPIhandle);
		} else { // CRC error, frame error, etc...
			/* call the error event handler */
			_HAL_SPI_err_evt_handler(pSPIhandle);
		}
	}
}

/*
 * @brief        Callback function for the application
 *               to handle different possible SPI event
 *
 * @param[in]    pSPIhandle : Pointer to SPI handle object
 * @param[in]    evt        : SPI event that called this function
 *
 * @return       None
 *
 */
__attribute((weak)) void HAL_SPI_app_evt_callback(
		SPI_handle_t *pSPIhandle, HAL_SPI_events_t evt)
{
    /* weak implementation - doing nothing */
}

/*
 * @brief        SPI RXNE interrupt event handler
 *               to receive a data frame
 *
 * @param[in]    pSPIhandle: Pointer to SPI handle object
 *
 * @return       None
 *
 */
static void _HAL_SPI_rxne_evt_handler(SPI_handle_t *pSPIhandle) {
	NULL_PTR_CHK(pSPIhandle);

	if(pSPIhandle->rxLen == 0) { return; }

	/* check the data frame format */
	uint8_t dff = (pSPIhandle->pSPIx->CR1 >> BITP_SPI_CR1_DFF) & 0x1;

	/* receive a data frame */
	if(dff == SPI_DATA_FRAME_8BIT) {
		*pSPIhandle->pRxBuff = pSPIhandle->pSPIx->DR;
		pSPIhandle->rxLen--;
		pSPIhandle->pRxBuff++;
	} else {
		*((uint16_t*)pSPIhandle->pRxBuff) = pSPIhandle->pSPIx->DR;
		pSPIhandle->rxLen--; pSPIhandle->rxLen--;
		(uint16_t*)pSPIhandle->pRxBuff++;
	}

	/* check if all the bytes have been received */
	if(pSPIhandle->rxLen == 0) {
		/* reset the SPI S/W Rx buffer ptr and state */
		pSPIhandle->pRxBuff = NULL;
		pSPIhandle->rxState  = HAL_SPI_READY;
		/* disable RXNE interrupt as data rx is completed */
		pSPIhandle->pSPIx->CR2 &= ~(0x1 << BITP_SPI_CR2_RXNEIE);
		/* notify the application that SPI Rx is completed */
		HAL_SPI_app_evt_callback(pSPIhandle, HAL_SPI_RX_DONE);
	}
}

/*
 * @brief        SPI TXE interrupt event handler
 *               to send a data frame
 *
 * @param[in]    pSPIhandle: Pointer to SPI handle object
 *
 * @return       None
 *
 */
static void _HAL_SPI_txe_evt_handler(SPI_handle_t *pSPIhandle) {
	NULL_PTR_CHK(pSPIhandle);

	if(pSPIhandle->txLen == 0) { return; }

	/* check the data frame format */
	uint8_t dff = (pSPIhandle->pSPIx->CR1 >> BITP_SPI_CR1_DFF) & 0x1;

	/* send a data frame */
	if(dff == SPI_DATA_FRAME_8BIT) {
		pSPIhandle->pSPIx->DR = *pSPIhandle->pTxBuff;
		pSPIhandle->txLen--;
		pSPIhandle->pTxBuff++;
	} else {
		pSPIhandle->pSPIx->DR = *((uint16_t*)pSPIhandle->pTxBuff);
		pSPIhandle->txLen--; pSPIhandle->txLen--;
		pSPIhandle->pTxBuff++; pSPIhandle->pTxBuff++;
	}

	/* check if all the bytes have been sent */
	if(pSPIhandle->txLen == 0) {
		/* reset the SPI S/W Tx buffer ptr and state */
		pSPIhandle->pTxBuff = NULL;
		pSPIhandle->txState  = HAL_SPI_READY;
		/* disable TXE interrupt as data tx is completed */
		pSPIhandle->pSPIx->CR2 &= ~(0x1 << BITP_SPI_CR2_TXEIE);
		/* notify the application that SPI Tx is completed */
		HAL_SPI_app_evt_callback(pSPIhandle, HAL_SPI_TX_DONE);
	}
}

/*
 * @brief        SPI error interrupt event handler
 *               to report the error
 *
 * @param[in]    pSPIhandle: Pointer to SPI handle object
 *
 * @return       None
 *
 */
static void _HAL_SPI_err_evt_handler(SPI_handle_t *pSPIhandle) {
	NULL_PTR_CHK(pSPIhandle);

	/* notify the application that SPI peripheral
	 * generated an interrupt reporting some error */
	HAL_SPI_app_evt_callback(pSPIhandle, HAL_SPI_ERR_REPORTED);
}

/*
 * @brief        SPI overrun error interrupt event handler
 *               to report the error
 *
 * @param[in]    pSPIhandle: Pointer to SPI handle object
 *
 * @return       None
 *
 */
static void _HAL_SPI_ovr_evt_handler(SPI_handle_t *pSPIhandle) {
	NULL_PTR_CHK(pSPIhandle);

	uint8_t temp = 0;
	/* check if application is not busy in Tx,
	 * then only read the data and status register
	 * to clear the overrun bit */
	if(pSPIhandle->txState == HAL_SPI_READY) {
		temp = pSPIhandle->pSPIx->DR;
		temp = pSPIhandle->pSPIx->SR;
	}

	/* to remove gcc compiler warnings */
	(void)temp;

	/* notify the application that SPI peripheral
	 * generated an interrupt reporting some error */
	HAL_SPI_app_evt_callback(pSPIhandle, HAL_SPI_OVR_ERR_REPORTED);
}
