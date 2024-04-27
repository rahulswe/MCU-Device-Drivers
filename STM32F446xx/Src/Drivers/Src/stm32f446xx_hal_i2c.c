/*
 ******************************************************************************
 * @file      stm32f446xx_hal_i2c.c
 * @author    Rahul Singh
 * @brief     Source file for STM32F446xx I2C driver
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
#include "stm32f446xx_hal_i2c.h"

uint16_t AHB_prescalar[] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint8_t APB1_prescalar[] = {2, 4, 8, 16};

static void _HAL_I2C_start_cond(I2C_reg_t *pI2Cx)
{
	NULL_PTR_CHK(pI2Cx);
	pI2Cx->CR1 |= 0x1 << BITP_I2C_CR1_START;
}

static void _HAL_I2C_stop_cond(I2C_reg_t *pI2Cx)
{
	NULL_PTR_CHK(pI2Cx);
	pI2Cx->CR1 |= 0x1 << BITP_I2C_CR1_STOP;
}

static void _HAL_I2C_addr_phase(I2C_reg_t *pI2Cx, uint8_t slave_addr,
		HAL_I2C_txrx_mode_t txrx_mode)
{
	NULL_PTR_CHK(pI2Cx);

	slave_addr = slave_addr << 1;

	if(txrx_mode == HAL_I2C_TX_MODE) {
		slave_addr &= ~(0x1);
	} else { //HAL_I2C_RX_MODE
		slave_addr |= 0x1;
	}

	pI2Cx->DR = slave_addr;
}

static uint32_t RCC_getPCLK1freq() {
	uint32_t pclk1_freq = 0;

	/* determine the clock source */
	uint32_t reg_val = RCC->CFGR;

	uint8_t sys_clk_sws = (reg_val >> 0x2) & 0x3;
	if(sys_clk_sws == 0x00) {
		pclk1_freq = HSI_OSC_FREQ;
	} else if (sys_clk_sws == 0x01) {
		pclk1_freq = HSE_OSC_FREQ;
	} else {
		/* TODO: add support for PLL */
		pclk1_freq = HSI_OSC_FREQ;
	}

	/* get the AHB1 prescalar value */
	uint8_t temp = (reg_val >> 0x4) & 0xF;
	uint8_t ahb_prescalar = (temp < 8) ? 1:AHB_prescalar[temp - 8];

	/* get the APB1 prescalar value */
	temp = (reg_val >> 0xA) & 0x7;
	uint8_t apb1_prescalar = (temp < 4) ? 1:APB1_prescalar[temp - 4];

	/* obtain the APB1 clock frequency after
	 * dividing AHB and APB1 prescalar value */
	pclk1_freq /= ahb_prescalar;
	pclk1_freq /= apb1_prescalar;

	return pclk1_freq;
}

/*
 * @brief        Enable or Disable I2C peripheral clock
 *
 * @param[in]    pI2Cx : I2C peripheral base address
 * @param[in]    en    : 0 -> disable, !0-> enable
 *
 * @return       None
 */
static void _HAL_I2C_pclk_ctrl(I2C_reg_t *pI2Cx, bool en)
{
	NULL_PTR_CHK(pI2Cx);

	/* check whether to enable or disable the SPI peripheral clock */
	if(en) {
		/* enable the specified I2C peripheral clock */
		if(pI2Cx == I2C1) { __I2C1_CLK_EN(); }
		else if(pI2Cx == I2C2) { __I2C2_CLK_EN(); }
		else if(pI2Cx == I2C3) { __I2C3_CLK_EN(); }
	} else {
		/* disable the specified I2C peripheral clock */
		if(pI2Cx == I2C1) { __I2C1_CLK_DIS(); }
		else if(pI2Cx == I2C2) { __I2C2_CLK_DIS(); }
		else if(pI2Cx == I2C3) { __I2C3_CLK_DIS(); }
	}
}

/*
 * @brief        Get I2C peripheral flag status
 *
 * @param[in]    pI2Cx : I2C peripheral base address
 * @param[in]    flag  : Bit mask of flag in I2C status register
 *
 * @return       flag status : 0(FLAG_NOT_SET) or 1(FLAG_SET)
 */
uint8_t HAL_I2C_flag_status(I2C_reg_t *pI2Cx, uint8_t flag)
{
	if(pI2Cx->SR1 & flag) {
		return FLAG_SET;
	}

	return FLAG_NOT_SET;
}

/*
 * @brief        Enable or Disable SPI peripheral
 *
 * @param[in]    pI2Cx : I2C peripheral base address
 * @param[in]    en    : 0 -> disable, !0-> enable
 *
 * @return       None
 */
void HAL_I2C_ctrl(I2C_reg_t *pI2Cx, uint8_t en)
{
	if(en) {
		pI2Cx->CR1 |= (0x1 << BITP_I2C_CR1_PE);
		pI2Cx->CR1 |= (0x1 << BITP_I2C_CR1_ACK);
	} else {
		pI2Cx->CR1 &= ~(0x1 << BITP_I2C_CR1_PE);
	}
}

/*
 * @brief        Initialize/Setup/Configure I2C peripheral
 *
 * @param[in]    pI2Chandle : Pointer to I2C handle object
 *
 * @return       None
 *
 */
void HAL_I2C_init(I2C_handle_t *pI2Chandle)
{
	NULL_PTR_CHK(pI2Chandle);

	/* enable the I2C peripheral clock */
	_HAL_I2C_pclk_ctrl(pI2Chandle->pI2Cx, true);

	/* configure CR1 register - ACK control */

	uint32_t reg_val = 0;//pI2Chandle->pI2Cx->CR1;
	//reg_val &= ~(0x1 << BITP_I2C_CR1_ACK);
	reg_val |= (pI2Chandle->cfg.ack_ctrl << BITP_I2C_CR1_ACK);

	pI2Chandle->pI2Cx->CR1 |= reg_val;


	/* configure OAR1 register - device address */
	pI2Chandle->pI2Cx->OAR1 |= (pI2Chandle->cfg.dev_addr << BITP_I2C_OAR1_ADD7_1);


	/* configure CR2 register - APB1 clock frequency */

	/* get the APB1 clock frequency */
	uint32_t apb1_clk_freq = RCC_getPCLK1freq();

	reg_val = (apb1_clk_freq << BITP_I2C_CR2_FREQ);
	reg_val /= 1000000U; //get the freq value in MHz

	pI2Chandle->pI2Cx->CR2 |= reg_val & 0x3F;


	/* configure CCR register */

	reg_val = 0;

	/* set I2C timings */

	uint16_t ccr = 0;

	if(pI2Chandle->cfg.speed >= I2C_SCL_SPEED_FM2K) {
		/* set fast mode */
		reg_val |= (0x1 << BITP_I2C_CCR_FS);
		/* set fast mode duty cycle */
		reg_val |= (pI2Chandle->cfg.fm_dutycycle << BITP_I2C_CCR_DUTY);
		/* set the ccr value */
		if(pI2Chandle->cfg.fm_dutycycle == I2C_FM_DUTY_CYCLE_2) {
			ccr = apb1_clk_freq/(3*pI2Chandle->cfg.speed);
		} else {
			ccr = apb1_clk_freq/(25*pI2Chandle->cfg.speed);
		}
		reg_val |= ccr & 0xFFF;
	} else {
		/* set standard mode */
		reg_val &= ~(0x1 << BITP_I2C_CCR_FS);
		/* set the ccr value */
		ccr = apb1_clk_freq/(2*pI2Chandle->cfg.speed);
		reg_val |= ccr & 0xFFF;
	}

	/* update the CCR register value */
	pI2Chandle->pI2Cx->CCR |= reg_val;

	/* set the Trise time */
	if(pI2Chandle->cfg.speed >= I2C_SCL_SPEED_FM2K) {
		reg_val = ((apb1_clk_freq * 300) / 1000000000U) + 1;
	} else {
		reg_val = ((apb1_clk_freq * 1000) / 1000000000U) + 1;
	}

	pI2Chandle->pI2Cx->TRISE = reg_val & 0x3F;
}
/*
 * @brief        De-initialize I2C peripheral
 *
 * @param[in]    pI2Cx : I2C peripheral base address
 *
 * @return       None
 *
 */
void HAL_I2C_deinit(I2C_reg_t *pI2Cx)
{
	NULL_PTR_CHK(pI2Cx);

	/* reset the specified SPI peripheral clock */
	if(pI2Cx == I2C1) { __I2C1_RST(); }
	else if(pI2Cx == I2C2) { __I2C2_RST(); }
	else if(pI2Cx == I2C3) { __I2C3_RST(); }

	/* disable the I2C peripheral clock */
	_HAL_I2C_pclk_ctrl(pI2Cx, false);
}

void HAL_I2C_master_rx_data(I2C_handle_t *pI2Chandle,
		                      uint8_t slave_addr, uint8_t *pRxBuffer,
							  uint32_t len)
{
	NULL_PTR_CHK(pI2Chandle);
	if(len == 0) { return; }

	//generate start condition
	_HAL_I2C_start_cond(pI2Chandle->pI2Cx);

	//wait till SB=1 i.e. start condition generated
	while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
			I2C_SR1_SB_FLAG) == FLAG_NOT_SET);

	//start address phase
	_HAL_I2C_addr_phase(pI2Chandle->pI2Cx, slave_addr, HAL_I2C_RX_MODE);

	//wait till ADDR=1 i.e. end of address transmission
	//i.e. address phase completed
	while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
			I2C_SR1_ADDR_FLAG) == FLAG_NOT_SET);

	if(len == 1) {
		//disable ACK
		/* configure CR1 register - ACK control */
		pI2Chandle->pI2Cx->CR1 &= ~(0x1 << BITP_I2C_CR1_ACK); //TODO: function for this

		//set STOP bit
		//generate stop condition
		_HAL_I2C_stop_cond(pI2Chandle->pI2Cx);//it can be done after clearing ADDR bit also
	}

	//clear the ADDR bit by reading SR2 register after SR1
	uint32_t temp = pI2Chandle->pI2Cx->SR2;
	(void)temp;

	//send data while len > 0
	while(len > 0) {
		while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
				I2C_SR1_RXNE_FLAG) == FLAG_NOT_SET);//once cleared this flag will be set again when
		//next byte is received into the shift register

		if(len == 2) {
			//disable ACK
			/* configure CR1 register - ACK control */
			pI2Chandle->pI2Cx->CR1 &= ~(0x1 << BITP_I2C_CR1_ACK);

			//set STOP bit
			//generate stop condition
			_HAL_I2C_stop_cond(pI2Chandle->pI2Cx);
		}

		*pRxBuffer = pI2Chandle->pI2Cx->DR;
		pRxBuffer++;
		len--;
	}

	//wait for TXE=1
	while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
			I2C_SR1_RXNE_FLAG) == FLAG_NOT_SET);

	//wait for BTF=1
	while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
			I2C_SR1_BTF_FLAG) == FLAG_NOT_SET);

	//generate stop condition
	_HAL_I2C_stop_cond(pI2Chandle->pI2Cx);

	//re-enable ACK if it was enable before calling this function
	if(pI2Chandle->cfg.ack_ctrl == I2C_ACK_ENABLE)
	{
	    //enable ACK
		/* configure CR1 register - ACK control */
		pI2Chandle->pI2Cx->CR1 |= (0x1 << BITP_I2C_CR1_ACK);
	}
}

void HAL_I2C_master_tx_data(I2C_handle_t *pI2Chandle,
		                      uint8_t slave_addr, uint8_t *pTxBuffer,
							  uint32_t len)
{
	NULL_PTR_CHK(pI2Chandle);

	//generate start condition
	_HAL_I2C_start_cond(pI2Chandle->pI2Cx);

	//wait till SB=1 i.e. start condition generated
	while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
			I2C_SR1_SB_FLAG) == FLAG_NOT_SET);

	//start address phase
	_HAL_I2C_addr_phase(pI2Chandle->pI2Cx, slave_addr, HAL_I2C_TX_MODE);

	//wait till ADDR=1 i.e. end of address transmission
	//i.e. address phase completed
	while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
			I2C_SR1_ADDR_FLAG) == FLAG_NOT_SET);

	//clear the ADDR bit by reading SR2 register after SR1
	uint32_t temp = pI2Chandle->pI2Cx->SR2;
	(void)temp;

	//send data while len > 0
	while(len > 0) {
		while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
				I2C_SR1_TXE_FLAG) == FLAG_NOT_SET);//once cleared this flag will be set again when
		//data from DR which was previously written is moved to shift register which would be once
		// prev data in shift register was transmitted and acked

		pI2Chandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//wait for TXE=1
	while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
			I2C_SR1_TXE_FLAG) == FLAG_NOT_SET);

	//wait for BTF=1
	while(HAL_I2C_flag_status(pI2Chandle->pI2Cx,
			I2C_SR1_BTF_FLAG) == FLAG_NOT_SET);

	//generate stop condition
	_HAL_I2C_stop_cond(pI2Chandle->pI2Cx);
}

void HAL_I2C_slave_rx_data(I2C_reg_t *pI2Cx,
		                      uint8_t *data)
{
	NULL_PTR_CHK(pI2Cx);
	NULL_PTR_CHK(data);

	while(HAL_I2C_flag_status(pI2Cx,
				I2C_SR1_ADDR_FLAG) == FLAG_NOT_SET);

	while(HAL_I2C_flag_status(pI2Cx,
			I2C_SR1_RXNE_FLAG) == FLAG_NOT_SET);

	*data = pI2Cx->DR;
}

void HAL_I2C_slave_tx_data(I2C_reg_t *pI2Cx,
		                      uint8_t data)
{
	NULL_PTR_CHK(pI2Cx);

	while(HAL_I2C_flag_status(pI2Cx,
			I2C_SR1_TXE_FLAG) == FLAG_NOT_SET);

	pI2Cx->DR = data;
}

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
void HAL_I2C_IRQ_config(uint32_t IRQ_num, uint32_t IRQ_prio, uint8_t en)
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
 * @brief        I2C IRQ handler
 *
 * @param[in]    pI2Chandle: Pointer to I2C handle object
 *
 * @return       None
 *
 */
void HAL_I2C_IRQ_handler(I2C_handle_t *pI2Chandle)
{
	NULL_PTR_CHK(pI2Chandle);

}

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
		I2C_handle_t *pI2Chandle, HAL_I2C_events_t evt)
{
    /* weak implementation - doing nothing */
}
