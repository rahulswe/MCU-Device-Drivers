/*
 ******************************************************************************
 * @file      stm32f446xx_hal_i2c.h
 * @author    Rahul Singh
 * @brief     Header file for STM32F446xx I2C driver
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

#ifndef _STM32F446XX_HAL_I2C_H_
#define _STM32F446XX_HAL_I2C_H_

#include "stm32f446xx.h"

/* ***************** Structure Definitions ******************* */

/* I2C configuration structure */
typedef struct {
	uint8_t dev_addr;
	uint8_t ack_ctrl;
	uint8_t fm_dutycycle;
	uint32_t speed;
}I2C_cfg_t;

/* I2C handle structure */
typedef struct {
	I2C_reg_t *pI2Cx;
	I2C_cfg_t cfg;
}I2C_handle_t;

/* ******************* Macro Definitions ********************* */

/* I2C speed */
#define I2C_SCL_SPEED_SM                (100000U)
#define I2C_SCL_SPEED_FM2K              (200000U)
#define I2C_SCL_SPEED_FM4K              (400000U)

/* I2C ACK control */
#define I2C_ACK_DISABLE                 (0U)
#define I2C_ACK_ENABLE                  (1U)

/* I2C fast mode duty cycle */
#define I2C_FM_DUTY_CYCLE_2             (0U)
#define I2C_FM_DUTY_CYCLE_16_9          (1U)

/* bit positions in I2C_CR1 register */
#define BITP_I2C_CR1_PE                 (0U)
#define BITP_I2C_CR1_SMBUS              (1U)
#define BITP_I2C_CR1_SMBTYPE            (3U)
#define BITP_I2C_CR1_ENARP              (4U)
#define BITP_I2C_CR1_ENPEC              (5U)
#define BITP_I2C_CR1_ENGC               (6U)
#define BITP_I2C_CR1_NOSTRECH           (7U)
#define BITP_I2C_CR1_START              (8U)
#define BITP_I2C_CR1_STOP               (9U)
#define BITP_I2C_CR1_ACK                (10U)
#define BITP_I2C_CR1_POS                (11U)
#define BITP_I2C_CR1_PEC                (12U)
#define BITP_I2C_CR1_ALERT              (13U)
#define BITP_I2C_CR1_SWRST              (15U)

/* bit positions in I2C_CR2 register */
#define BITP_I2C_CR2_FREQ               (0U)
#define BITP_I2C_CR2_ITERREN            (8U)
#define BITP_I2C_CR2_ITEVTEN            (9U)
#define BITP_I2C_CR2_ITBUFEN            (10U)
#define BITP_I2C_CR2_DMAEN              (11U)
#define BITP_I2C_CR2_LAST               (12U)

/* bit positions in I2C_SR register */
#define BITP_I2C_SR_SB                  (0U)
#define BITP_I2C_SR_ADDR                (1U)
#define BITP_I2C_SR_BTF                 (2U)
#define BITP_I2C_SR_ADD10               (3U)
#define BITP_I2C_SR_STOPF               (4U)
#define BITP_I2C_SR_RXNE                (6U)
#define BITP_I2C_SR_TXE                 (7U)
#define BITP_I2C_SR_RXNE                (6U)
#define BITP_I2C_SR_TXE                 (7U)
#define BITP_I2C_SR_BERR                (8U)
#define BITP_I2C_SR_ARLO                (9U)
#define BITP_I2C_SR_AF                  (10U)
#define BITP_I2C_SR_OVR                 (11U)
#define BITP_I2C_SR_PECERR              (12U)
#define BITP_I2C_SR_TIMEOUT             (14U)
#define BITP_I2C_SR_SMBALERT            (15U)

/* bit positions in I2C_CCR register */
#define BITP_I2C_CCR_CCR                (0U)
#define BITP_I2C_CCR_DUTY               (14U)
#define BITP_I2C_CCR_FS                 (15U)

/* bit positions in I2C OAR1 register */
#define BITP_I2C_OAR1_ADD0              (0U)
#define BITP_I2C_OAR1_ADD7_1            (1U)
#define BITP_I2C_OAR1_ADD9_8            (8U)
#define BITP_I2C_OAR1_ADDMODE           (15U)

/* I2C status register 1 flags bit mask */
#define I2C_SR1_SB_FLAG                (0x1 << 0x00U)
#define I2C_SR1_ADDR_FLAG              (0x1 << 0x01U)
#define I2C_SR1_BTF_FLAG               (0x1 << 0x02U)
#define I2C_SR1_ADD10_FLAG             (0x1 << 0x03U)
#define I2C_SR1_STOPF_FLAG             (0x1 << 0x04U)
#define I2C_SR1_RXNE_FLAG              (0x1 << 0x06U)
#define I2C_SR1_TXE_FLAG               (0x1 << 0x07U)
#define I2C_SR1_BERR_FLAG              (0x1 << 0x08U)
#define I2C_SR1_ARLO_FLAG              (0x1 << 0x09U)
#define I2C_SR1_AF_FLAG                (0x1 << 0x0AU)
#define I2C_SR1_OVR_FLAG               (0x1 << 0x0BU)
#define I2C_SR1_PECERR_FLAG            (0x1 << 0x0CU)
#define I2C_SR1_TIMEOUT_FLAG           (0x1 << 0x0EU)
#define I2C_SR1_SMBALERT_FLAG          (0x1 << 0x0FU)

/* ******************* Enum Definitions ********************* */

#if 0
/* SPI peripheral state */
typedef enum {
	HAL_SPI_READY = 0,
	HAL_SPI_BUSY_IN_TX,
	HAL_SPI_BUSY_IN_RX,
	HAL_SPI_NUM_STATES
} HAL_SPI_state_t;
#endif

typedef enum {
	HAL_I2C_TX_MODE = 0,
	HAL_I2C_RX_MODE,
	HAL_I2C_NUM_MODES
} HAL_I2C_txrx_mode_t;

/* I2C events */
typedef enum {
	HAL_I2C_TX_DONE = 0,
} HAL_I2C_events_t;

/* ***************** Function Declarations ******************* */

/*
 * @brief        Get I2C peripheral flag status
 *
 * @param[in]    pI2Cx : I2C peripheral base address
 * @param[in]    flag  : Bit mask of flag in I2C status register
 *
 * @return       flag status : 0(FLAG_NOT_SET) or 1(FLAG_SET)
 */
uint8_t HAL_I2C_flag_status(I2C_reg_t *pI2Cx, uint8_t flag);

/*
 * @brief        Enable or Disable SPI peripheral
 *
 * @param[in]    pI2Cx : I2C peripheral base address
 * @param[in]    en    : 0 -> disable, !0-> enable
 *
 * @return       None
 */
void HAL_I2C_ctrl(I2C_reg_t *pI2Cx, uint8_t en);

/*
 * @brief        Initialize/Setup/Configure I2C peripheral
 *
 * @param[in]    pI2Chandle : Pointer to I2C handle object
 *
 * @return       None
 *
 */
void HAL_I2C_init(I2C_handle_t *pI2Chandle);

/*
 * @brief        De-initialize I2C peripheral
 *
 * @param[in]    pI2Cx : I2C peripheral base address
 *
 * @return       None
 *
 */
void HAL_I2C_deinit(I2C_reg_t *pI2Cx);

void HAL_I2C_master_tx_data(I2C_handle_t *pI2Chandle,
		                      uint8_t slave_addr, uint8_t *pTxBuffer,
							  uint32_t len);

void HAL_I2C_master_rx_data(I2C_handle_t *pI2Chandle,
		                      uint8_t slave_addr, uint8_t *pRxBuffer,
							  uint32_t len);

void HAL_I2C_slave_rx_data(I2C_reg_t *pI2Cx,
		                      uint8_t *data);

void HAL_I2C_slave_tx_data(I2C_reg_t *pI2Cx,
		                      uint8_t data);

/*
 * @brief        Configure the IRQ for a I2C peripheral
 *
 * @param[in]    IRQ_num  : IRQ number for a specific I2C peripheral
 * @param[in]    IRQ_prio : Priority level for the specified IRQ (0 to 16)
 * @param[in]    en       : 1 -> enable the IRQ, 0-> Disable the IRQ
 *
 * @return       None
 *
 */
void HAL_I2C_IRQ_config(uint32_t IRQ_num, uint32_t IRQ_prio, uint8_t en);

/*
 * @brief        I2C IRQ handler
 *
 * @param[in]    pSPIhandle: Pointer to I2C handle object
 *
 * @return       None
 *
 */
void HAL_I2C_IRQ_handler(I2C_handle_t *pI2Chandle);

/*
 * @brief        Callback function for the application
 *               to handle different possible I2C event
 *
 * @param[in]    pI2Chandle : Pointer to I2C handle object
 * @param[in]    evt        : SPI event that called this function
 *
 * @return       None
 *
 */
void HAL_I2C_app_evt_callback(
		I2C_handle_t *pI2Chandle, HAL_I2C_events_t evt);

#endif
