/*
 ******************************************************************************
 * @file      main_usart.c
 * @author    Rahul Singh
 * @brief     Sample application file for testing STM32F446xx USART driver APIs
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
#include "stm32f446xx_hal_usart.h"

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


	while(1) {
		/* wait until user button is pressed */
		while(!g_button_pressed);

		/* reset the flag for next interrupt */
		g_button_pressed = false;

		/* sending a pattern data over MOSI line */
		char data[] = "Hello SPI";
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
