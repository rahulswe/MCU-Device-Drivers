/*
 ******************************************************************************
 * @file      stm32f446xx_hal_usart.c
 * @author    Rahul Singh
 * @brief     Source file for STM32F446xx USART driver
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
#include "stm32f446xx_hal_usart.h"
#include "stm32f446xx_hal_rcc.h"

/* USART interrupt events handler declaration */
static void _HAL_USART_rxne_evt_handler(USART_handle_t *pUSARThandle);
static void _HAL_USART_txe_evt_handler(USART_handle_t *pUSARThandle);
static void _HAL_USART_err_evt_handler(USART_handle_t *pUSARThandle);
static void _HAL_USART_ovr_evt_handler(USART_handle_t *pUSARThandle);
static void _HAL_USART_tc_evt_handler(USART_handle_t *pUSARThandle);

/*
 * @brief        Enable or Disable USART peripheral clock
 *
 * @param[in]    pUSARTx : USART peripheral base address
 * @param[in]    en      : 0 -> disable, !0-> enable
 *
 * @return       None
 */
static void _HAL_USART_pclk_ctrl(USART_reg_t *pUSARTx, bool en)
{
	NULL_PTR_CHK(pUSARTx);

	/* check whether to enable or disable the USART peripheral clock */
	if(en) {
		/* enable the specified USART peripheral clock */
		if(pUSARTx == USART1) { __USART1_CLK_EN(); }
		else if(pUSARTx == USART2) { __USART2_CLK_EN(); }
		else if(pUSARTx == USART3) { __USART3_CLK_EN(); }
		else if(pUSARTx == UART4) { __UART4_CLK_EN(); }
		else if(pUSARTx == UART5) { __UART5_CLK_EN(); }
		else if(pUSARTx == USART6) { __USART6_CLK_EN(); }
	} else {
		/* disable the specified USART peripheral clock */
		if(pUSARTx == USART1) { __USART1_CLK_DIS(); }
		else if(pUSARTx == USART2) { __USART2_CLK_DIS(); }
		else if(pUSARTx == USART3) { __USART3_CLK_DIS(); }
		else if(pUSARTx == UART4) { __UART4_CLK_DIS(); }
		else if(pUSARTx == UART5) { __UART5_CLK_DIS(); }
		else if(pUSARTx == USART6) { __USART6_CLK_DIS(); }
	}
}

/*
 * @brief        Set USART baud rate
 *
 * @param[in]    pUSARTx   : USART peripheral base address
 * @param[in]    baud_rate : baud rate value in bits per second
 *
 * @return       None
 */
static void __HAL_USART_set_baud_rate(USART_reg_t *pUSARTx, uint32_t baud_rate)
{
	uint32_t fpclk = 0;
	/*
	 * USART1 & USART6 are hanging on APB2 Bus
	 * and rest all of the USART peripherals
	 * are on APB1 Bus
	 */
	if((pUSARTx == USART1) || (pUSARTx == USART6)) {
		fpclk = RCC_getPCLK2freq(); //get APB2 peripheral clock frequency
	} else {
		fpclk = RCC_getPCLK1freq(); //get APB1 peripheral clock frequency
	}

	/* get the oversampling configuration */
	uint32_t over8 = (pUSARTx->CR1 >> BITP_USART_CR1_OVER8) & 0x1;
	float usart_div = (float)fpclk/(8*(2 - over8)*baud_rate);

	/* estimate the mantissa value */
	uint32_t mantissa = (uint32_t)usart_div & 0xFFF;

	/* estimate the fraction value */
	uint32_t fraction = (uint32_t)(usart_div*100) - (mantissa*100);
	fraction = ((fraction*8*(2-over8)) + 50)/100 & 0xF; //rounding off to nearest value
	if(over8) {
		fraction &= 0x7;//clear the 4th bit if over8 = 1
	}

	/* configure the BRR register with mantissa and
	 * fraction value to get the desired baud rate */
	pUSARTx->BRR = (mantissa << BITP_USART_BRR_MANTISSA) | fraction;
}

/*
 * @brief        Get USART peripheral flag status
 *
 * @param[in]    pUSARTx : USART peripheral base address
 * @param[in]    flag    : Bit mask of flag in USART status register
 *
 * @return       flag status : 0(FLAG_NOT_SET) or 1(FLAG_SET)
 */
uint8_t HAL_USART_flag_status(USART_reg_t *pUSARTx, uint8_t flag)
{
	if(pUSARTx->SR & flag) {
		return FLAG_SET;
	}
	return FLAG_NOT_SET;
}

/*
 * @brief        Enable or Disable USART peripheral
 *
 * @param[in]    pUSARTx : USART peripheral base address
 * @param[in]    en      : 0 -> disable, !0-> enable
 *
 * @return       None
 */
void HAL_USART_ctrl(USART_reg_t *pUSARTx, uint8_t en)
{
	if(en) {
		pUSARTx->CR1 |= (0x1 << BITP_USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(0x1 << BITP_USART_CR1_UE);
	}
}

/*
 * @brief        Initialize/Setup/Configure USART peripheral
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
void HAL_USART_init(USART_handle_t *pUSARThandle)
{
	NULL_PTR_CHK(pUSARThandle);

	/* read the default CR1 register value */
	uint32_t reg_val = pUSARThandle->pUSARTx->CR1;

	/* enable the USART peripheral clock */
	_HAL_USART_pclk_ctrl(pUSARThandle->pUSARTx, true);

	/* set communication mode */
	if(pUSARThandle->cfg.mode == USART_TX_MODE) {
		reg_val |= 0x1 << BITP_USART_CR1_TE;
	} else if(pUSARThandle->cfg.mode == USART_RX_MODE) {
		reg_val |= 0x1 << BITP_USART_CR1_RE;
	} else { // USART_TXRX_MODE
		reg_val |= 0x1 << BITP_USART_CR1_TE;
		reg_val |= 0x1 << BITP_USART_CR1_RE;
	}

	/* set word length */
	reg_val |= pUSARThandle->cfg.word_len << BITP_USART_CR1_M;

	/* configure parity control  */
	if(pUSARThandle->cfg.parity_ctrl != USART_PARITY_DISABLE) {
		/* enable parity control */
		reg_val |= 0x1 << BITP_USART_CR1_PCE;
		/* select even or odd parity */
		reg_val |= pUSARThandle->cfg.parity_ctrl << BITP_USART_CR1_PS;
	}

	/*
	 * keeping default value for over8 bit
	 * i.e 0 (oversampling by 16) therefore,
	 * commented below line
	 */
	// reg_val |= 0x1 << BITP_USART_CR1_OVER8;

	/* set the USART CR1 register with the
	* value as per specified configuration */
	pUSARThandle->pUSARTx->CR1 = reg_val;

	/* read the default CR2 register value */
	reg_val = pUSARThandle->pUSARTx->CR2;
	/* set stop bit length */
	reg_val |= pUSARThandle->cfg.stop_bit_len << BITP_USART_CR2_STOP;

	/* set the USART CR2 register with the
	* value as per specified configuration */
	pUSARThandle->pUSARTx->CR2 = reg_val;

	/* configure h/w flow control  */
	if(pUSARThandle->cfg.hw_flow_ctrl != USART_HW_FLOW_CTRL_DISABLE) {
		/* read the default CR3 register value */
		reg_val = pUSARThandle->pUSARTx->CR3;
		if(pUSARThandle->cfg.hw_flow_ctrl == USART_HW_FLOW_CTRL_EN_RTS) {
			reg_val |= 0x1 << BITP_USART_CR3_RTSE;
		} else if(pUSARThandle->cfg.hw_flow_ctrl == USART_HW_FLOW_CTRL_EN_CTS) {
			reg_val |= 0x1 << BITP_USART_CR3_CTSE;
		} else { //USART_HW_FLOW_CTRL_EN_RTS_CTS
			reg_val |= 0x1 << BITP_USART_CR3_RTSE;
			reg_val |= 0x1 << BITP_USART_CR3_CTSE;
		}
		/* set the USART CR3 register with the
		* value as per specified configuration */
		pUSARThandle->pUSARTx->CR3 = reg_val;
	}

	/* set baud rate */
	__HAL_USART_set_baud_rate(pUSARThandle->pUSARTx, pUSARThandle->cfg.baud_rate);

	/* enable the USART peripheral */
	//done through a separate function
}

/*
 * @brief        De-initialize USART peripheral
 *
 * @param[in]    pUSARTx : USART peripheral base address
 *
 * @return       None
 *
 */
void HAL_USART_deinit(USART_reg_t *pUSARTx)
{
	NULL_PTR_CHK(pUSARTx);

	/* reset the specified SPI peripheral clock */
	if(pUSARTx == USART1) { __USART1_RST(); }
	else if(pUSARTx == USART2) { __USART2_RST(); }
	else if(pUSARTx == USART3) { __USART3_RST(); }
	else if(pUSARTx == UART4)  { __UART4_RST(); }
	else if(pUSARTx == UART4)  { __UART5_RST(); }
	else if(pUSARTx == USART6) { __USART6_RST(); }

	/* disable the USART peripheral clock */
	_HAL_USART_pclk_ctrl(pUSARTx, false);
}

/*
 * @brief        Read data over USART peripheral (blocking mode)
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 * @param[in]    pRxBuffer : Pointer to the buffer storing received data
 * @param[in]    len       : Size of data to be received (in bytes)
 *
 * @return       None
 *
 */
void HAL_USART_read_data(USART_handle_t *pUSARThandle, uint8_t *pRxBuffer, uint32_t len)
{
	/* if len = 0, return */
	if(len == 0) {
		return;
	}

	for(uint32_t idx = 0; idx < len; idx++) {
		/* wait till RXNE is not set */
		while(!HAL_USART_flag_status(pUSARThandle->pUSARTx, USART_SR_RXNE_FLAG));

		/* if data frame word length == 9 bits */
		if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_M)) {
			/* if parity control enable, last bit is parity bit in the frame */
			if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_PCE)) {
				/* first 8 bits are data bits and last bit (msb) is parity bit */
				*pRxBuffer = pUSARThandle->pUSARTx->DR & 0xFF;
				pRxBuffer++;
			} else {/* if parity control disabled, no parity bit in the frame */
				/* all of the 9 bits are data bits */
				*((uint16_t *)pRxBuffer) = pUSARThandle->pUSARTx->DR & 0x01FF;
				//((uint16_t *)pRxBuffer)++; //increment pRxBuffer by 2 bytes
				pRxBuffer++;
				pRxBuffer++;
			}
		} else {/* if data frame word length == 8 bits */
			/* if parity control enable, last bit is parity bit in the frame */
			if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_PCE)) {
				/* first 7 bits are data bits and last bit (msb) is parity bit */
				*pRxBuffer = pUSARThandle->pUSARTx->DR & 0x7F;
				pRxBuffer++;
			} else {/* if parity control disabled, no parity bit in the frame */
				/* all of the 8 bits are data bits */
				*pRxBuffer = pUSARThandle->pUSARTx->DR & 0xFF;
				pRxBuffer++;
			}
		}
	}
}

/*
 * @brief        Send data over USART peripheral (blocking mode)
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 * @param[in]    pTxBuffer : Pointer to the buffer sending data to be sent
 * @param[in]    len       : Size of data to be sent (in bytes)
 *
 * @return       None
 *
 */
void HAL_USART_send_data(USART_handle_t *pUSARThandle, uint8_t *pTxBuffer, uint32_t len)
{
	/* if len = 0, return */
	if(len == 0) {
		return;
	}

	for(uint32_t idx = 0; idx < len; idx++) {
		/* wait till TXE is not set */
		while(!HAL_USART_flag_status(pUSARThandle->pUSARTx, USART_SR_TXE_FLAG));

		/* if data frame word length == 9 bits */
		if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_M)) {
			/* if parity control enable, last bit is parity bit in the frame */
			if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_PCE)) {
				/* first 8 bits are data bits and last bit (msb) is parity bit */
				pUSARThandle->pUSARTx->DR = *pTxBuffer;
				pTxBuffer++;
			} else {/* if parity control disabled, no parity bit in the frame */
				/* all of the 9 bits are data bits */
				pUSARThandle->pUSARTx->DR = *((uint16_t *)pTxBuffer) & 0x1FF;
				pTxBuffer++;
				pTxBuffer++;
			}
		} else { /* if data frame word length == 8 bits */
#if 0
			/* if parity control enable, last bit is parity bit in the frame */
			if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_PCE)) {
				/* first 7 bits are data bits and last bit (msb) is parity bit */
				pUSARThandle->pUSARTx->DR = *pTxBuffer & 0x7F;
				pTxBuffer++;
			} else {/* if parity control disabled, no parity bit in the frame */
				/* all of the 8 bits are data bits */
				pUSARThandle->pUSARTx->DR = *pTxBuffer;
				pTxBuffer++;
			}
#else
			/* if parity control enabled, msb will be
			 * updated with parity bit by the HW */
			pUSARThandle->pUSARTx->DR = *pTxBuffer;
			pTxBuffer++;
#endif
		}
	}
	/* wait till data transmission is not completed */
	while(!HAL_USART_flag_status(pUSARThandle->pUSARTx, USART_SR_TC_FLAG));
}

/*
 * @brief        Read data over USART peripheral (non-blocking mode)
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 * @param[in]    pRxBuffer    : Pointer to the buffer storing received data
 * @param[in]    len          : Size of data to be received (in bytes)
 *
 * @return       USART peripheral Rx state when this function is called
 *
 */
HAL_USART_state_t HAL_USART_read_data_IT(USART_handle_t *pUSARThandle,
		uint8_t *pRxBuffer, uint32_t len)
{
	if(pUSARThandle == NULL) { return 0xFF; }
	if(pRxBuffer == NULL) { return 0xFF; }

	/* if there is nothing to be received then return */
	if(len == 0) {
		return 0xFF;
	}

	uint8_t rx_state = pUSARThandle->rxState;

	if(rx_state == HAL_USART_READY)
	{
		/* store address of the buffer where
		* the data to be received is stored
		* and the size of data */
		pUSARThandle->pRxBuff = pRxBuffer;
		pUSARThandle->rxLen = len;

		/* set the USART Rx busy status */
		pUSARThandle->rxState = HAL_USART_BUSY_IN_RX;

		/* enable RXNE interrupt */
		pUSARThandle->pUSARTx->CR1 |= 0x1 << BITP_USART_CR1_RXNEIE;
	}

	return rx_state;
}

/*
 * @brief        Send data over USART peripheral (non-blocking mode)
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 * @param[in]    pTxBuffer    : Pointer to the buffer sending data to be sent
 * @param[in]    len          : Size of data to be sent (in bytes)
 *
 * @return       USART peripheral Tx state when this function is called
 *
 */
HAL_USART_state_t HAL_USART_send_data_IT(USART_handle_t *pUSARThandle, uint8_t *pTxBuffer, uint32_t len)
{
	if(pUSARThandle == NULL) { return 0xFF; }
	if(pTxBuffer == NULL) { return 0xFF; }

	/* if there is nothing to be sent then return */
	if(len == 0) {
		return 0xFF;
	}

	uint8_t tx_state = pUSARThandle->txState;

	if(tx_state == HAL_USART_READY)
	{
		/* store address of the buffer where
		* the data to be sent is stored
		* and the size of data */
		pUSARThandle->pTxBuff = pTxBuffer;
		pUSARThandle->txLen = len;

		/* set the USART Tx busy status */
		pUSARThandle->txState = HAL_USART_BUSY_IN_TX;

		/* enable TXE interrupt */
		pUSARThandle->pUSARTx->CR1 |= 0x1 << BITP_USART_CR1_TXEIE;

		/* enable TC interrupt */
		pUSARThandle->pUSARTx->CR1 |= 0x1 << BITP_USART_CR1_TCIE;
	}

	return tx_state;
}

/*
 * @brief        Configure the IRQ for a USART peripheral
 *
 * @param[in]    IRQ_num  : IRQ number for a specific USART peripheral
 * @param[in]    IRQ_prio : Priority level for the specified IRQ (0 to 16)
 * @param[in]    en       : 1 -> enable the IRQ, 0-> Disable the IRQ
 *
 * @return       None
 *
 */
void HAL_USART_IRQ_config(uint32_t IRQ_num, uint32_t IRQ_prio, uint8_t en)
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
 * @brief        USART IRQ handler
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
void HAL_USART_IRQ_handler(USART_handle_t *pUSARThandle)
{
	NULL_PTR_CHK(pUSARThandle);

	uint8_t status = pUSARThandle->pUSARTx->SR;
	uint8_t cr1_val = pUSARThandle->pUSARTx->CR1;
	uint8_t cr3_val = pUSARThandle->pUSARTx->CR3;

	/* check which USART event generated the interrupt
	 * and call the respective handler to service it */

	/* checking if Rx buffer not empty
	 * event generated the interrupt */
	if((cr1_val & (0x1 << BITP_USART_CR1_RXNEIE))
		&& (status & USART_SR_RXNE_FLAG))
	{
		/* call the Rx buffer not empty event handler */
		_HAL_USART_rxne_evt_handler(pUSARThandle);
	}
	/* checking if Tx buffer empty event
	* generated the interrupt */
	if((cr1_val & (0x1 << BITP_USART_CR1_TXEIE))
		&& (status & USART_SR_TXE_FLAG))
	{
		/* call the Tx buffer empty event handler */
		_HAL_USART_txe_evt_handler(pUSARThandle);
	}
	/* checking if transmission completed (TC)
	 * event generated the interrupt */
	if((cr1_val & (0x1 << BITP_USART_CR1_TCIE))
		&& (status & USART_SR_TC_FLAG))
	{
		/* call the transmission completed event handler */
		_HAL_USART_tc_evt_handler(pUSARThandle);
	}

	/* checking if parity error event
	* generated the interrupt */
	if((cr1_val & (0x1 << BITP_USART_CR1_PEIE))
			&& (status & USART_SR_PE_FLAG))
	{
		/* call the error event handler */
		_HAL_USART_err_evt_handler(pUSARThandle);
	}

	/* checking if any USART error event
	 * generated the interrupt */
	if(cr3_val & (0x1 << BITP_USART_CR3_EIE))
	{
		/* if overrun error */
		if(status & USART_SR_ORE_FLAG) {
			 /* call the overrun error event handler */
			_HAL_USART_ovr_evt_handler(pUSARThandle);
		} else { // Noise flag detected or frame error
			/* call the error event handler */
			_HAL_USART_err_evt_handler(pUSARThandle);
		}
	}
}

/*
 * @brief        Callback function for the application
 *               to handle different possible USART event
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 * @param[in]    evt          : USART event that called this function
 *
 * @return       None
 *
 */
__attribute((weak)) void HAL_USART_app_evt_callback(
		USART_handle_t *pUSARThandle, HAL_USART_events_t evt)
{
    /* weak implementation - doing nothing */
}

/*
 * @brief        USART RXNE interrupt event handler
 *               to receive a data frame
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
static void _HAL_USART_rxne_evt_handler(USART_handle_t *pUSARThandle) {
	NULL_PTR_CHK(pUSARThandle);

	if(pUSARThandle->rxLen == 0) { return; }

	/* receive a data frame */
	/* if data frame word length == 9 bits */
	if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_M)) {
		/* if parity control enable, last bit is parity bit in the frame */
		if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_PCE)) {
			/* first 8 bits are data bits and last bit (msb) is parity bit */
			*pUSARThandle->pRxBuff = pUSARThandle->pUSARTx->DR & 0xFF;
			pUSARThandle->pRxBuff++;
			/* decrement num received bytes by 1 */
			pUSARThandle->rxLen--;
		} else {/* if parity control disabled, no parity bit in the frame */
			/* all of the 9 bits are data bits */
			*((uint16_t *)pUSARThandle->pRxBuff) = pUSARThandle->pUSARTx->DR & 0x01FF;
			pUSARThandle->pRxBuff++;
			pUSARThandle->pRxBuff++;
			/* decrement num received bytes by 2 */
			pUSARThandle->rxLen -= 2;
		}
	} else {/* if data frame word length == 8 bits */
		/* if parity control enable, last bit is parity bit in the frame */
		if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_PCE)) {
			/* first 7 bits are data bits and last bit (msb) is parity bit */
			*pUSARThandle->pRxBuff = pUSARThandle->pUSARTx->DR & 0x7F;
			pUSARThandle->pRxBuff++;
		} else {/* if parity control disabled, no parity bit in the frame */
			/* all of the 8 bits are data bits */
			*pUSARThandle->pRxBuff = pUSARThandle->pUSARTx->DR & 0xFF;
			pUSARThandle->pRxBuff++;
		}
		/* decrement num received bytes by 1 */
		pUSARThandle->rxLen--;
	}

	/* check if all the bytes have been received */
	if(pUSARThandle->rxLen == 0) {
		/* reset the USART S/W Rx buffer ptr and state */
		pUSARThandle->pRxBuff = NULL;
		pUSARThandle->rxState  = HAL_USART_READY;
		/* disable RXNE interrupt as data rx is completed */
		pUSARThandle->pUSARTx->CR1 &= ~(0x1 << BITP_USART_CR1_RXNEIE);
		/* notify the application that USART Rx is completed */
		HAL_USART_app_evt_callback(pUSARThandle, HAL_USART_RX_DONE);
	}
}

/*
 * @brief        USART TXE interrupt event handler
 *               to send a data frame
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
static void _HAL_USART_txe_evt_handler(USART_handle_t *pUSARThandle) {
	NULL_PTR_CHK(pUSARThandle);

	if(pUSARThandle->txLen == 0) { return; }

	/* if data frame word length == 9 bits */
	if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_M)) {
		/* if parity control enable, last bit is parity bit in the frame */
		if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_PCE)) {
			/* first 8 bits are data bits and last bit (msb) is parity bit */
			pUSARThandle->pUSARTx->DR = *pUSARThandle->pTxBuff;
			pUSARThandle->pTxBuff++;
		} else {/* if parity control disabled, no parity bit in the frame */
			/* all of the 9 bits are data bits */
			pUSARThandle->pUSARTx->DR = *((uint16_t *)pUSARThandle->pTxBuff) & 0x1FF;
			pUSARThandle->pTxBuff++;
			pUSARThandle->pTxBuff++;
		}
	} else { /* if data frame word length == 8 bits */
#if 0
		/* if parity control enable, last bit is parity bit in the frame */
		if(pUSARThandle->pUSARTx->CR1 & (0x1 << BITP_USART_CR1_PCE)) {
			/* first 7 bits are data bits and last bit (msb) is parity bit */
			pUSARThandle->pUSARTx->DR = *pUSARThandle->pTxBuff & 0x7F;
			pUSARThandle->pTxBuff++;
		} else {/* if parity control disabled, no parity bit in the frame */
			/* all of the 8 bits are data bits */
			pUSARThandle->pUSARTx->DR = *pUSARThandle->pTxBuff;
			pUSARThandle->pTxBuff++;
		}
#else
		/* if parity control enable, msb will be
		 * updated with parity bit by the HW */
		pUSARThandle->pUSARTx->DR = *pUSARThandle->pTxBuff;
		pUSARThandle->pTxBuff++;
#endif
	}

	/* decrement num transmitted bytes by 1 */
	pUSARThandle->txLen--;

	/* check if all the bytes have been sent */
	if(pUSARThandle->txLen == 0) {
		/* disable TXE interrupt as required data has been sent */
		pUSARThandle->pUSARTx->CR1 &= ~(0x1 << BITP_USART_CR1_TXEIE);
		/* application will be notified when TC flag is set and
		 * interrupt triggered indicating data tx is completed */
	}
}

/*
 * @brief        USART transmission completed (TC) interrupt event
 *               handler to send a data frame
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
static void _HAL_USART_tc_evt_handler(USART_handle_t *pUSARThandle) {
	NULL_PTR_CHK(pUSARThandle);

	/* TC flag is sent for each frame transmitted successfully */

	/* check if all the bytes have been sent */
	if(pUSARThandle->txLen == 0) {
		/* reset the USART S/W Tx buffer ptr and state */
		pUSARThandle->pTxBuff = NULL;
		pUSARThandle->txState  = HAL_USART_READY;
		/* disable TC interrupt as data tx is completed */
		pUSARThandle->pUSARTx->CR1 &= ~(0x1 << BITP_USART_CR1_TCIE);
		/* clear the TC flag */
		pUSARThandle->pUSARTx->SR &= ~(0x1 << BITP_USART_SR_TC);
		/* notify the application that USART Tx is completed */
		HAL_USART_app_evt_callback(pUSARThandle, HAL_USART_TX_DONE);
	}
}

/*
 * @brief        USART error interrupt event handler
 *               to report the error
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
static void _HAL_USART_err_evt_handler(USART_handle_t *pUSARThandle) {
	NULL_PTR_CHK(pUSARThandle);

	/* notify the application that USART peripheral
	 * generated an interrupt reporting some error */
	HAL_USART_app_evt_callback(pUSARThandle, HAL_USART_ERR_REPORTED);
}

/*
 * @brief        USART overrun error interrupt event handler
 *               to report the error
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
static void _HAL_USART_ovr_evt_handler(USART_handle_t *pUSARThandle) {
	NULL_PTR_CHK(pUSARThandle);

	uint8_t temp = 0;
	/* check if application is not busy in Tx,
	 * then only read the data and status register
	 * to clear the overrun bit */
	if(pUSARThandle->txState == HAL_USART_READY) {
		temp = pUSARThandle->pUSARTx->SR;
		temp = pUSARThandle->pUSARTx->DR;
	}

	/* to remove gcc compiler warnings */
	(void)temp;

	/* notify the application that USART peripheral
	 * generated an interrupt reporting some error */
	HAL_USART_app_evt_callback(pUSARThandle, HAL_USART_OVR_ERR_REPORTED);
}
