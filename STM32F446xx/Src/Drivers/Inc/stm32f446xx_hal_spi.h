/*
 ******************************************************************************
 * @file      stm32f446xx_hal_spi.h
 * @author    Rahul Singh
 * @brief     Header file for STM32F446xx SPI driver
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

#ifndef _STM32F446XX_HAL_SPI_H_
#define _STM32F446XX_HAL_SPI_H_

#include "stm32f446xx.h"

/* ***************** Structure Definitions ******************* */

/* SPI configuration structure */
typedef struct {
	uint8_t dev_mode; // device mode
	uint8_t comm_mode; // communication mode
	uint8_t speed; // baud rate
	uint8_t cpol; // sclk polarity
	uint8_t cpha; // sclk phase
	uint8_t dff; // data frame format
	uint8_t ssm; // sw slave management ctrl
}SPI_cfg_t;

/* SPI handle structure */
typedef struct {
	SPI_reg_t *pSPIx;
	SPI_cfg_t cfg;
	uint8_t *pTxBuff;
	uint32_t txLen;
	uint8_t txState;
	uint8_t *pRxBuff;
	uint32_t rxLen;
	uint8_t rxState;
}SPI_handle_t;

/* ******************* Macro Definitions ********************* */

/* SPI device modes */
#define SPI_SLAVE_MODE                  (0x00U)
#define SPI_MASTER_MODE                 (0x01U)

/* SPI clock polarity */
#define SPI_CPOL_LOW                    (0x00U)
#define SPI_CPOL_HIGH                   (0x01U)

/* SPI clock phase */
#define SPI_CPHA_FIRST_CLK_EDGE         (0x00U)
#define SPI_CPHA_SECOND_CLK_EDGE        (0x01U)

/* SPI slave management types */
#define SPI_HW_SLAVE_MGMT               (0x00U)
#define SPI_SW_SLAVE_MGMT               (0x01U)

/* SPI data frame formats */
#define SPI_DATA_FRAME_8BIT             (0x00U)
#define SPI_DATA_FRAME_16BIT            (0x01U)

/* SPI communication modes */
#define SPI_COMM_MODE_FULL_DUPLEX       (0x00U)
#define SPI_COMM_MODE_HALF_DUPLEX       (0x01U)
#define SPI_COMM_MODE_SIMPLEX_RX        (0x02U)

/* SPI SCLK speed */
#define SPI_SCLK_SPEED_PCLK_DIV2        (0x00U)
#define SPI_SCLK_SPEED_PCLK_DIV4        (0x01U)
#define SPI_SCLK_SPEED_PCLK_DIV8        (0x02U)
#define SPI_SCLK_SPEED_PCLK_DIV16       (0x03U)
#define SPI_SCLK_SPEED_PCLK_DIV32       (0x04U)
#define SPI_SCLK_SPEED_PCLK_DIV64       (0x05U)
#define SPI_SCLK_SPEED_PCLK_DIV128      (0x06U)
#define SPI_SCLK_SPEED_PCLK_DIV256      (0x07U)

/* SPI status register flags bit mask */
#define SPI_SR_RXNE_FLAG                (0x1 << 0x00U)
#define SPI_SR_TXE_FLAG                 (0x1 << 0x01U)
#define SPI_SR_CHSIDE_FLAG              (0x1 << 0x02U)
#define SPI_SR_UDR_FLAG                 (0x1 << 0x03U)
#define SPI_SR_CRCERR_FLAG              (0x1 << 0x04U)
#define SPI_SR_MODF_FLAG                (0x1 << 0x05U)
#define SPI_SR_OVR_FLAG                 (0x1 << 0x06U)
#define SPI_SR_BSY_FLAG                 (0x1 << 0x07U)
#define SPI_SR_FRE_FLAG                 (0x1 << 0x08U)

/* bit positions in SPI_CR1 register */
#define BITP_SPI_CR1_CPHA               (0U)
#define BITP_SPI_CR1_CPOL               (1U)
#define BITP_SPI_CR1_MSTR               (2U)
#define BITP_SPI_CR1_BR                 (3U)
#define BITP_SPI_CR1_SPE                (6U)
#define BITP_SPI_CR1_LSBFIRST           (7U)
#define BITP_SPI_CR1_SSI                (8U)
#define BITP_SPI_CR1_SSM                (9U)
#define BITP_SPI_CR1_RXONLY             (10U)
#define BITP_SPI_CR1_DFF                (11U)
#define BITP_SPI_CR1_CRCNEXT            (12U)
#define BITP_SPI_CR1_CRCEN              (13U)
#define BITP_SPI_CR1_BIDIOE             (14U)
#define BITP_SPI_CR1_BIDIMODE           (15U)

/* bit positions in SPI_CR2 register */
#define BITP_SPI_CR2_RXDMAEN            (0U)
#define BITP_SPI_CR2_TXDMAEN            (1U)
#define BITP_SPI_CR2_SSOE               (2U)
#define BITP_SPI_CR2_FRF                (4U)
#define BITP_SPI_CR2_ERRIE              (5U)
#define BITP_SPI_CR2_RXNEIE             (6U)
#define BITP_SPI_CR2_TXEIE              (7U)

/* bit positions in SPI_SR register */
#define BITP_SPI_SR_RXNE                (0U)
#define BITP_SPI_SR_TXE                 (1U)
#define BITP_SPI_SR_CHSIDE              (2U)
#define BITP_SPI_SR_UDR                 (3U)
#define BITP_SPI_SR_CRCERR              (4U)
#define BITP_SPI_SR_MODF                (5U)
#define BITP_SPI_SR_OVR                 (6U)
#define BITP_SPI_SR_BSY                 (7U)
#define BITP_SPI_SR_FRE                 (8U)

/* ******************* Enum Definitions ********************* */

/* SPI peripheral state */
typedef enum {
	HAL_SPI_READY = 0,
	HAL_SPI_BUSY_IN_TX,
	HAL_SPI_BUSY_IN_RX,
	HAL_SPI_NUM_STATES
} HAL_SPI_state_t;

/* SPI events */
typedef enum {
	HAL_SPI_TX_DONE = 0,
	HAL_SPI_RX_DONE,
	HAL_SPI_ERR_REPORTED,
	HAL_SPI_NUM_EVENTS
} HAL_SPI_events_t;

/* ***************** Function Declarations ******************* */

/*
 * @brief        Get SPI peripheral flag status
 *
 * @param[in]    pSPIx : SPI peripheral base address
 * @param[in]    flag  : Bit mask of flag in SPI status register
 *
 * @return       flag status : 0(FLAG_NOT_SET) or 1(FLAG_SET)
 */
uint8_t HAL_SPI_flag_status(SPI_reg_t *pSPIx, uint8_t flag);

/*
 * @brief        Enable or Disable SPI peripheral
 *
 * @param[in]    pSPIx : SPI peripheral base address
 * @param[in]    en    : 0 -> disable, !0-> enable
 *
 * @return       None
 */
void HAL_SPI_ctrl(SPI_reg_t *pSPIx, uint8_t en);

/*
 * @brief        Initialize/Setup/Configure SPI peripheral
 *
 * @param[in]    pSPIhandle : Pointer to SPI handle object
 *
 * @return       None
 *
 */
void HAL_SPI_init(SPI_handle_t *pSPIhandle);

/*
 * @brief        De-initialize SPI peripheral
 *
 * @param[in]    pSPIx : SPI peripheral base address
 *
 * @return       None
 *
 */
void HAL_SPI_deinit(SPI_reg_t *pSPIx);

/*
 * @brief        Read data over SPI peripheral (blocking mode)
 *
 * @param[in]    pSPIx     : SPI peripheral base address
 * @param[in]    pRxBuffer : Pointer to the buffer storing received data
 * @param[in]    len       : Size of data to be received (in bytes)
 *
 * @return       None
 *
 */
void HAL_SPI_read_data(SPI_reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * @brief        Send data over SPI peripheral (blocking mode)
 *
 * @param[in]    pSPIx     : SPI peripheral base address
 * @param[in]    pTxBuffer : Pointer to the buffer sending data to be sent
 * @param[in]    len       : Size of data to be sent (in bytes)
 *
 * @return       None
 *
 */
void HAL_SPI_send_data(SPI_reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

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
HAL_SPI_state_t HAL_SPI_read_data_IT(SPI_handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t len);

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
HAL_SPI_state_t HAL_SPI_send_data_IT(SPI_handle_t *pSPIhandle, uint8_t *pTxBuffer, uint32_t len);

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
void HAL_SPI_IRQ_config(uint32_t IRQ_num, uint32_t IRQ_prio, uint8_t en);

/*
 * @brief        SPI IRQ handler
 *
 * @param[in]    pSPIhandle: Pointer to SPI handle object
 *
 * @return       None
 *
 */
void HAL_SPI_IRQ_handler(SPI_handle_t *pSPIhandle);

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
void HAL_SPI_app_evt_callback(
		SPI_handle_t *pSPIhandle, HAL_SPI_events_t evt);

#endif
