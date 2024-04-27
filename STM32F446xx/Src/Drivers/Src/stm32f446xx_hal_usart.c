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
 * @param[in]    pUSARTx   : USART peripheral base address
 * @param[in]    pRxBuffer : Pointer to the buffer storing received data
 * @param[in]    len       : Size of data to be received (in bytes)
 *
 * @return       None
 *
 */
void HAL_USART_read_data(USART_reg_t *pUSARTx, uint8_t *pRxBuffer, uint32_t len)
{

}

/*
 * @brief        Send data over USART peripheral (blocking mode)
 *
 * @param[in]    pUSARTx   : USART peripheral base address
 * @param[in]    pTxBuffer : Pointer to the buffer sending data to be sent
 * @param[in]    len       : Size of data to be sent (in bytes)
 *
 * @return       None
 *
 */
void HAL_USART_send_data(USART_reg_t *pUSARTx, uint8_t *pTxBuffer, uint32_t len)
{

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
HAL_USART_state_t HAL_USART_read_data_IT(USART_handle_t *pUSARThandle, uint8_t *pRxBuffer, uint32_t len)
{

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
void HAL_USART_app_evt_callback(
		USART_handle_t *pUSARThandle, HAL_USART_events_t evt)
{

}
