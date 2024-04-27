/*
 ******************************************************************************
 * @file      main_i2c.c
 * @author    Rahul Singh
 * @brief     Sample application file for testing STM32F446xx I2C driver
 *            APIs as a master device
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

/* software delay */
void sw_delay() {
	for(uint32_t i = 0; i < 300000; i++);
}

/* flag to monitor button press status */
uint8_t g_button_pressed = false;

/* keeping some valid same interrupt
 * prio value for all the interrupts
 * defined in this application */
#define DEFAULT_INT_PRIO    (14U)

int main(void)
{
	/* configuration for GPIO pin connected to button on NUCLEOF446RE board */
	GPIO_handle_t GPIO_BUTTON_handle = {
			.pGPIOx = GPIOC,
			.cfg.pin_number = GPIO_PIN_13, //Button Pin
			.cfg.pin_mode = GPIO_PIN_MODE_IT_FT, //interrupt falling edge trigger mode
			.cfg.pin_otype = 0x00, //reset value
			.cfg.pin_ospeed = 0x00, //reset value
			.cfg.pupd_ctrl = GPIO_PIN_NO_PULL_UP_DOWN, //external pull-up present
			.cfg.alt_func = 0x00 //reset value
	};

	/* Initialize the GPIO pin for Button with interrupt on falling edge */
	HAL_GPIO_init(&GPIO_BUTTON_handle);
	HAL_GPIO_IRQ_config(IRQ_NO_EXTI15_10, DEFAULT_INT_PRIO, ENABLE);

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
			.cfg.dev_addr = 0xA2,
			.cfg.speed = I2C_SCL_SPEED_SM,
			.cfg.ack_ctrl = I2C_ACK_ENABLE,
			.cfg.fm_dutycycle = I2C_FM_DUTY_CYCLE_2
	};

	/* initialize I2C peripheral */
	HAL_I2C_init(&I2C1Handle);

	while(1) {
		/* wait until user button is pressed */
		while(!g_button_pressed);

		/* reset the flag for next interrupt */
		g_button_pressed = false;

		/* sending a pattern data over MOSI line */
		//char data[] = "Hello SPI";
		uint8_t data = 0x55;

		/* enable the SPI peripheral */
		HAL_I2C_ctrl(I2C1, ENABLE);
#if 1
	    /* send data over SDA line */
		HAL_I2C_master_tx_data(&I2C1Handle, 0x68, &data, sizeof(data));

		/* wait if SPI peripheral is busy */
		while((I2C1->SR2 >> 0x1) & 0x1);
#endif
		/* disable the I2C peripheral */
		HAL_I2C_ctrl(I2C1, DISABLE);
	}
}

/* EXTI IRQ Handler for the GPIO pins
 * corresponding to the Button
 * */
void EXTI15_10_IRQHandler(void)
{
	/* clear the IRQ pending bit for GPIO pin of button */
	HAL_GPIO_IRQ_handler(GPIO_PIN_13);

	/* set the button pressed flag with some
	 * delay to handle button de-bouncing */
	sw_delay();
	g_button_pressed = true;
}
