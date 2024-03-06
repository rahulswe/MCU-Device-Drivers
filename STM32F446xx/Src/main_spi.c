/*
 ******************************************************************************
 * @file      main_spi.c
 * @author    Rahul Singh
 * @brief     Sample application file for testing STM32F446xx SPI driver
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32f446xx_hal_gpio.h"
#include "stm32f446xx_hal_spi.h"

#define ENABLE  (1U)
#define DISABLE (0U)

/* software delay */
void sw_delay() {
	for(uint32_t i = 0; i < 300000; i++);
}

int main(void)
{
	/* configure SPI GPIOs for STM32F446RE Nucleo Board */

	/*
	 * SPI1
	 * NSS -> PA4
	 * SCLK -> PA5
	 * MISO -> PA6
	 * MOSI -> PA7
	 * */

	GPIO_handle_t SPI1_GPIO_handle = {
			.pGPIOx = GPIOA,
			//.cfg.pin_number = GPIO_PIN_5, //SCLK
			.cfg.pin_mode = GPIO_PIN_MODE_ALT, //alternate functionality mode
			.cfg.pin_otype = GPIO_PIN_OTYPE_PUSH_PULL, //push-pull
			.cfg.pin_ospeed = GPIO_PIN_OSPEED_FAST, //fast-speed
			.cfg.pupd_ctrl = GPIO_PIN_NO_PULL_UP_DOWN, //no pull-up, pull-down
			.cfg.alt_func = GPIO_PIN_ALT_FUNC_AF5
	};

	/* initialize the NSS pin for SPI1 */
	SPI1_GPIO_handle.cfg.pin_number = GPIO_PIN_4;
	HAL_GPIO_init(&SPI1_GPIO_handle);

	/* initialize the SCLK pin for SPI1 */
	SPI1_GPIO_handle.cfg.pin_number = GPIO_PIN_5;
	HAL_GPIO_init(&SPI1_GPIO_handle);

	/* initialize the MISO pin for SPI1 */
	SPI1_GPIO_handle.cfg.pin_number = GPIO_PIN_6;
	HAL_GPIO_init(&SPI1_GPIO_handle);

	/* initialize the MOSI pin for SPI1 */
	SPI1_GPIO_handle.cfg.pin_number = GPIO_PIN_7;
	HAL_GPIO_init(&SPI1_GPIO_handle);

	/* configure SPI peripheral */
	SPI_handle_t SPI1Handle = {
			.pSPIx = SPI1,
			.cfg.dev_mode = SPI_MASTER_MODE,
			.cfg.comm_mode = SPI_COMM_MODE_FULL_DUPLEX,
			.cfg.speed = SPI_SCLK_SPEED_PCLK_DIV2,
			.cfg.cpol = SPI_CPOL_LOW,
			.cfg.cpha = SPI_CPHA_FIRST_CLK_EDGE,
			.cfg.dff = SPI_DATA_FRAME_8BIT,
			.cfg.ssm = SPI_HW_SLAVE_MGMT
	};

	/* initialize SPI peripheral */
	HAL_SPI_init(&SPI1Handle);
	/* enable the SPI peripheral */
	HAL_SPI_ctrl(SPI1, ENABLE);

	uint8_t n = 5;
	while(n--) {

		/* No slave connected
		 * sending a pattern data over MOSI line
		 * n times and checking via logic analyzer
		 * */
		char data[] = "Hello SPI";

		/* wait if SPI peripheral is busy */
		while(HAL_SPI_flag_status(SPI1, SPI_SR_BSY_FLAG));
	    /* send data over MOSI line */
		HAL_SPI_send_data(SPI1, (uint8_t*)data, strlen(data));
	}

	/* wait if SPI peripheral is busy */
	while(HAL_SPI_flag_status(SPI1, SPI_SR_BSY_FLAG));
	/* disable the SPI peripheral */
	HAL_SPI_ctrl(SPI1, DISABLE);//HAL_SPI_deinit(SPI1);

    /* Loop forever */
	for(;;);
}

