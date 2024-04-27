/*
 ******************************************************************************
 * @file      main_i2c.c
 * @author    Rahul Singh
 * @brief     Sample application file for testing STM32F446xx I2C driver
 *            APIs as a slave device
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
#include <stdbool.h>
#include <string.h>

#include "stm32f446xx_hal_gpio.h"
#include "stm32f446xx_hal_i2c.h"

int main(void)
{
	/* configure GPIOs for I2C on STM32F446RE Nucleo Board */

	/*
	 * I2C1
	 * SCL -> PB8
	 * SDA -> PB9
	 * */

	GPIO_handle_t I2C1_GPIO_handle = {
			.pGPIOx = GPIOB,
			//.cfg.pin_number = GPIO_PIN_6, //SCLK
			.cfg.pin_mode = GPIO_PIN_MODE_ALT, //alternate functionality mode
			.cfg.pin_otype = GPIO_PIN_OTYPE_OPEN_DRAIN, //open drain
			.cfg.pin_ospeed = GPIO_PIN_OSPEED_FAST, //fast-speed
			.cfg.pupd_ctrl = GPIO_PIN_NO_PULL_UP_DOWN, //no pull-up, pull-down //????
			.cfg.alt_func = GPIO_PIN_ALT_FUNC_AF4
	};

	/* initialize the SCL pin for I2C1 */
	I2C1_GPIO_handle.cfg.pin_number = GPIO_PIN_8;
	HAL_GPIO_init(&I2C1_GPIO_handle);

	/* initialize the SDA pin for I2C1 */
	I2C1_GPIO_handle.cfg.pin_number = GPIO_PIN_9;
	HAL_GPIO_init(&I2C1_GPIO_handle);

	/* configure I2C peripheral */
	I2C_handle_t I2C1Handle = {
			.pI2Cx = I2C1,
			.cfg.dev_addr = 0x27,
			.cfg.speed = I2C_SCL_SPEED_SM,
			.cfg.ack_ctrl = I2C_ACK_ENABLE,
			.cfg.fm_dutycycle = I2C_FM_DUTY_CYCLE_2
	};

	/* initialize I2C peripheral */
	HAL_I2C_init(&I2C1Handle);

	while(1) {
		/* sending a pattern data over MOSI line */
		//char data[] = "Hello SPI";
		uint8_t rx_data = 0;

		/* enable the SPI peripheral */
		HAL_I2C_ctrl(I2C1, ENABLE);
#if 1
	    /* send data over SDA line */
		HAL_I2C_slave_rx_data(I2C1,&rx_data);

		printf("Received from master = %d\n", rx_data);

		/* wait if SPI peripheral is busy */
		while((I2C1->SR2 >> 0x1) & 0x1);
#endif
		/* disable the I2C peripheral */
		HAL_I2C_ctrl(I2C1, DISABLE);
	}
}
