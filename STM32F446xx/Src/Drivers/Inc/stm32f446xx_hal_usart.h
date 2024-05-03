/*
 ******************************************************************************
 * @file      stm32f446xx_hal_usart.h
 * @author    Rahul Singh
 * @brief     Header file for STM32F446xx USART driver
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

#ifndef _STM32F446XX_HAL_USART_H_
#define _STM32F446XX_HAL_USART_H_

#include "stm32f446xx.h"

/* ***************** Structure Definitions ******************* */

/* USART configuration structure */
typedef struct {
	uint8_t mode; // tx or rx or txrx
	uint8_t word_len; // word length
	uint8_t parity_ctrl; // parity control
	uint8_t stop_bit_len; // no. of stop bits
	uint8_t hw_flow_ctrl; // HW flow control
	uint32_t baud_rate; // baud rate
}USART_cfg_t;

/* USART handle structure */
typedef struct {
	USART_reg_t *pUSARTx;
	USART_cfg_t cfg;
	uint8_t *pTxBuff;
	uint8_t *pRxBuff;
	uint8_t txState;
	uint8_t rxState;
	uint32_t rxLen;
	uint32_t txLen;
}USART_handle_t;

/* ******************* Macro Definitions ********************* */

/* USART device modes */
#define USART_TX_MODE                  (0x00U)
#define USART_RX_MODE                  (0x01U)
#define USART_TXRX_MODE                (0x02U)

/* USART parity control */
#define USART_PARITY_EN_EVEN           (0x00U)
#define USART_PARITY_EN_ODD            (0x01U)
#define USART_PARITY_DISABLE           (0x02U)

/* USART hardware flow control */
#define USART_HW_FLOW_CTRL_DISABLE     (0x00U)
#define USART_HW_FLOW_CTRL_EN_RTS      (0x01U)
#define USART_HW_FLOW_CTRL_EN_CTS      (0x02U)
#define USART_HW_FLOW_CTRL_EN_RTS_CTS  (0x03U)

/* USART baud rate */
#define USART_BAUD_RATE_1200           (1200U)
#define USART_BAUD_RATE_2400           (2400U)
#define USART_BAUD_RATE_4800           (4800U)
#define USART_BAUD_RATE_9600           (9600U)
#define USART_BAUD_RATE_19200          (19200U)
#define USART_BAUD_RATE_38400          (38400U)
#define USART_BAUD_RATE_57600          (57600U)
#define USART_BAUD_RATE_115200         (115200U)
#define USART_BAUD_RATE_230400         (230400U)
#define USART_BAUD_RATE_460800         (460800U)
#define USART_BAUD_RATE_921600         (921600U)
#define USART_BAUD_RATE_2M             (2000000U)
#define USART_BAUD_RATE_3M             (3000000U)

/* USART word length */
#define USART_WORD_LEN_8BITS           (0x00U)
#define USART_WORD_LEN_9BITS           (0x01U)

/* USART stop bit length */
#define USART_STOP_BIT_LEN_1           (0x00U)
#define USART_STOP_BIT_LEN_0_5         (0x01U)
#define USART_STOP_BIT_LEN_2           (0x02U)
#define USART_STOP_BIT_LEN_1_5         (0x03U)

/* USART status register bit positions */
#define BITP_USART_SR_PE               (0U)
#define BITP_USART_SR_FE               (1U)
#define BITP_USART_SR_NF               (2U)
#define BITP_USART_SR_ORE              (3U)
#define BITP_USART_SR_IDLE             (4U)
#define BITP_USART_SR_RXNE             (5U)
#define BITP_USART_SR_TC               (6U)
#define BITP_USART_SR_TXE              (7U)
#define BITP_USART_SR_LBD              (8U)
#define BITP_USART_SR_CTS              (9U)

/* USART status register flags bit mask */
#define USART_SR_PE_FLAG               (0x1 << BITP_USART_SR_PE)
#define USART_SR_FE_FLAG               (0x1 << BITP_USART_SR_FE)
#define USART_SR_NF_FLAG               (0x1 << BITP_USART_SR_NF)
#define USART_SR_ORE_FLAG              (0x1 << BITP_USART_SR_ORE)
#define USART_SR_IDLE_FLAG             (0x1 << BITP_USART_SR_IDLE)
#define USART_SR_RXNE_FLAG             (0x1 << BITP_USART_SR_RXNE)
#define USART_SR_TC_FLAG               (0x1 << BITP_USART_SR_TC)
#define USART_SR_TXE_FLAG              (0x1 << BITP_USART_SR_TXE)
#define USART_SR_LBD_FLAG              (0x1 << BITP_USART_SR_LBD)
#define USART_SR_CTS_FLAG              (0x1 << BITP_USART_SR_CTS)

/* USART CR1 register bit positions */
#define BITP_USART_CR1_SBK             (0U)
#define BITP_USART_CR1_RWU             (1U)
#define BITP_USART_CR1_RE              (2U)
#define BITP_USART_CR1_TE              (3U)
#define BITP_USART_CR1_IDLEIE          (4U)
#define BITP_USART_CR1_RXNEIE          (5U)
#define BITP_USART_CR1_TCIE            (6U)
#define BITP_USART_CR1_TXEIE           (7U)
#define BITP_USART_CR1_PEIE            (8U)
#define BITP_USART_CR1_PS              (9U)
#define BITP_USART_CR1_PCE             (10U)
#define BITP_USART_CR1_WAKE            (11U)
#define BITP_USART_CR1_M               (12U)
#define BITP_USART_CR1_UE              (13U)
#define BITP_USART_CR1_OVER8           (15U)

/* USART CR2 register bit positions */
#define BITP_USART_CR2_ADD             (0U)
#define BITP_USART_CR2_LBDL            (5U)
#define BITP_USART_CR2_LBDIE           (6U)
#define BITP_USART_CR2_LBCL            (8U)
#define BITP_USART_CR2_CPHA            (9U)
#define BITP_USART_CR2_CPOL            (10U)
#define BITP_USART_CR2_CLKEN           (11U)
#define BITP_USART_CR2_STOP            (12U)
#define BITP_USART_CR2_LINEN           (14U)

/* USART CR3 register bit positions */
#define BITP_USART_CR3_EIE             (0U)
#define BITP_USART_CR3_IREN            (1U)
#define BITP_USART_CR3_IRLP            (2U)
#define BITP_USART_CR3_HDSEL           (3U)
#define BITP_USART_CR3_NACK            (4U)
#define BITP_USART_CR3_SCEN            (5U)
#define BITP_USART_CR3_DMAR            (6U)
#define BITP_USART_CR3_DMAT            (7U)
#define BITP_USART_CR3_RTSE            (8U)
#define BITP_USART_CR3_CTSE            (9U)
#define BITP_USART_CR3_CTSIE           (10U)
#define BITP_USART_CR3_ONEBIT          (11U)

/* USART BRR register bit positions */
#define BITP_USART_BRR_FRACTION        (0U)
#define BITP_USART_BRR_MANTISSA        (4U)

/* ******************* Enum Definitions ********************* */

/* USART peripheral state */
typedef enum {
	HAL_USART_READY = 0,
	HAL_USART_BUSY_IN_TX,
	HAL_USART_BUSY_IN_RX,
	HAL_USART_NUM_STATES
} HAL_USART_state_t;

/* USART events */
typedef enum {
	HAL_USART_TX_DONE = 0,
	HAL_USART_RX_DONE,
	HAL_USART_OVR_ERR_REPORTED,
	HAL_USART_ERR_REPORTED,
	HAL_USART_NUM_EVENTS
} HAL_USART_events_t;

/* ***************** Function Declarations ******************* */

/*
 * @brief        Get USART peripheral flag status
 *
 * @param[in]    pUSARTx : USART peripheral base address
 * @param[in]    flag    : Bit mask of flag in USART status register
 *
 * @return       flag status : 0(FLAG_NOT_SET) or 1(FLAG_SET)
 */
uint8_t HAL_USART_flag_status(USART_reg_t *pUSARTx, uint8_t flag);

/*
 * @brief        Enable or Disable USART peripheral
 *
 * @param[in]    pUSARTx : USART peripheral base address
 * @param[in]    en      : 0 -> disable, !0-> enable
 *
 * @return       None
 */
void HAL_USART_ctrl(USART_reg_t *pUSARTx, uint8_t en);

/*
 * @brief        Initialize/Setup/Configure USART peripheral
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
void HAL_USART_init(USART_handle_t *pUSARThandle);

/*
 * @brief        De-initialize USART peripheral
 *
 * @param[in]    pUSARTx : USART peripheral base address
 *
 * @return       None
 *
 */
void HAL_USART_deinit(USART_reg_t *pUSARTx);

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
void HAL_USART_read_data(USART_handle_t *pUSARThandle, uint8_t *pRxBuffer, uint32_t len);

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
void HAL_USART_send_data(USART_handle_t *pUSARThandle, uint8_t *pTxBuffer, uint32_t len);

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
HAL_USART_state_t HAL_USART_read_data_IT(USART_handle_t *pUSARThandle, uint8_t *pRxBuffer, uint32_t len);

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
HAL_USART_state_t HAL_USART_send_data_IT(USART_handle_t *pUSARThandle, uint8_t *pTxBuffer, uint32_t len);

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
void HAL_USART_IRQ_config(uint32_t IRQ_num, uint32_t IRQ_prio, uint8_t en);

/*
 * @brief        USART IRQ handler
 *
 * @param[in]    pUSARThandle : Pointer to USART handle object
 *
 * @return       None
 *
 */
void HAL_USART_IRQ_handler(USART_handle_t *pUSARThandle);

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
		USART_handle_t *pUSARThandle, HAL_USART_events_t evt);

#endif
