/*
 ******************************************************************************
 * @file      main_usart.c
 * @author    Rahul Singh
 * @brief     Sample application file for testing STM32F446xx USART driver
 *            non-blocking APIs
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

/* USART2 handle object */
USART_handle_t gUSART2Handle;

/* USART transmission completed flag */
bool gTxDone = false;
bool gRxDone = false;

/* received data buffer */
uint8_t g_rx_data[10] = {0};

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

	/* configure USART2 GPIOs for STM32F446RE Nucleo Board */

	/*
		* USART2
		* Tx -> PA2
		* Rx -> PA3
		*
	*/

	GPIO_handle_t USART2_GPIO_handle = {
		.pGPIOx = GPIOA,
		//.cfg.pin_number = GPIO_PIN_5, //SCLK
		.cfg.pin_mode = GPIO_PIN_MODE_ALT, //alternate functionality mode
		.cfg.pin_otype = GPIO_PIN_OTYPE_PUSH_PULL, //push-pull
		.cfg.pin_ospeed = GPIO_PIN_OSPEED_FAST, //fast-speed
		.cfg.pupd_ctrl = GPIO_PIN_PULL_UP, //no pull-up, pull-down
		.cfg.alt_func = GPIO_PIN_ALT_FUNC_AF7
	};

	/* initialize the Tx pin for USART2 */
	USART2_GPIO_handle.cfg.pin_number = GPIO_PIN_2;
	HAL_GPIO_init(&USART2_GPIO_handle);

	/* initialize the Rx pin for USART2 */
	USART2_GPIO_handle.cfg.pin_number = GPIO_PIN_3;
	HAL_GPIO_init(&USART2_GPIO_handle);

	/* configure USART peripheral */
	USART_handle_t USART2Handle = {
		.pUSARTx = USART2,
		.cfg.mode = USART_TXRX_MODE,
		.cfg.word_len = USART_WORD_LEN_8BITS,
		.cfg.parity_ctrl = USART_PARITY_DISABLE,
		.cfg.stop_bit_len = USART_STOP_BIT_LEN_1,
		.cfg.hw_flow_ctrl = USART_HW_FLOW_CTRL_DISABLE,
		.cfg.baud_rate = USART_BAUD_RATE_115200,
	};

	memcpy(&gUSART2Handle, &USART2Handle, sizeof(USART_handle_t));

	/* initialize USART peripheral */
	HAL_USART_init(&gUSART2Handle);
	/* enable USART interrupt */
	HAL_USART_IRQ_config(IRQ_NO_USART2, DEFAULT_INT_PRIO, ENABLE);

	while(1) {
		/* wait until user button is pressed */
		while(!g_button_pressed);

		/* reset the Tx and Rx done flag */
		gTxDone = false;
		gRxDone = false;

		/* reset the flag for next interrupt */
		g_button_pressed = false;

		/* sending a pattern data over MOSI line */
		char data[] = "Hello SPI";

		/* enable the UART peripheral */
		HAL_USART_ctrl(USART2, ENABLE);

	    /* send data (non-blocking call) */
		while(HAL_USART_send_data_IT(&gUSART2Handle, (uint8_t*)data, sizeof(data)) != HAL_USART_READY);

		/* receive data (non-blocking call) */
		while(HAL_USART_read_data_IT(&gUSART2Handle, &g_rx_data[0], sizeof(g_rx_data)) != HAL_USART_READY);

		/* wait till USART Tx is completed */
		while(!gTxDone);

		/* wait till USART Rx is completed */
		while(!gRxDone);

		/* print the received data */
		for(int i = 0; i < sizeof(g_rx_data); i++) {
			printf("%d ", g_rx_data[i]);
		}
		printf("\n");

		/* disable the UART peripheral */
		HAL_USART_ctrl(USART2, DISABLE);
	}
}

void USART2_IRQHandler(void) {
	HAL_USART_IRQ_handler(&gUSART2Handle);
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

/* USART application event callback */
void HAL_USART_app_evt_callback(
		USART_handle_t *pUSARThandle, HAL_USART_events_t evt) {

	(void)pUSARThandle;

	switch(evt) {
	case HAL_USART_TX_DONE:
	{
		printf("USART Tx Done\n");
		gTxDone = true;
		break;
	}
	case HAL_USART_RX_DONE:
	{
		printf("USART Rx Done\n");
		gRxDone = true;
		break;
	}
	case HAL_USART_OVR_ERR_REPORTED:
	{
		printf("SPI Overrun Error Reported\n");
		break;
	}
	case HAL_USART_ERR_REPORTED:
	{
		printf("USART Error Reported\n");
		break;
	}
	default:
	{
		printf("USART Unknown Event Reported\n");
		break;
	}
	}
}
