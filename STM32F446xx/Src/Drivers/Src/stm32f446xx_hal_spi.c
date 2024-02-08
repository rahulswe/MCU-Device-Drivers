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

#include <stm32f446xx_hal_spi.h>

/*
 * @brief        Enable or Disable SPI peripheral clock
 *
 * @param[in]    pSPIx : SPI peripheral base address
 * @param[in]    en    : 0 -> enable, 1 0-> disable
 *
 * @return       None
 *
 */
void HAL_SPI_pclk_ctrl(SPI_reg_t *pSPIx, uint8_t en)
{
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
 * @brief        Initialize/Setup/Configure SPI peripheral
 *
 * @param[in]    pSPIhandle : Pointer to SPI handle object
 *
 * @return       None
 *
 */
void HAL_SPI_init(SPI_handle_t *pSPIhandle)
{

}

/*
 * @brief        De-initialize SPI peripheral
 *
 * @param[in]    pSPIx : SPI peripheral base address
 *
 * @return       None
 *
 */
void HAL_SPI_deinit(SPI_reg_t *pSPIx)
{
	/* reset the specified SPI peripheral clock */
	if(pSPIx == SPI1) { __SPI1_RST(); }
	else if(pSPIx == SPI2) { __SPI2_RST(); }
	else if(pSPIx == SPI3) { __SPI3_RST(); }
	else if(pSPIx == SPI4) { __SPI4_RST(); }
}

/*
 * @brief        Read data over SPI peripheral
 *
 * @param[in]    pSPIx     : SPI peripheral base address
 * @param[in]    pRxBuffer : Pointer to the buffer storing received data
 * @param[in]    len       : Size of data received (in bytes)
 *
 * @return       None
 *
 */
void HAL_SPI_read_data(SPI_reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{

}

/*
 * @brief        Send data over SPI peripheral
 *
 * @param[in]    pSPIx     : SPI peripheral base address
 * @param[in]    pTxBuffer : Pointer to the buffer sending data to be sent
 * @param[in]    len       : Size of data to be sent (in bytes)
 *
 * @return       None
 *
 */
void HAL_SPI_send_data(SPI_reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{

}
