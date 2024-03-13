/*
 ******************************************************************************
 * @file      main_spi_txrx_it.c
 * @author    Rahul Singh
 * @brief     Sample application file for testing STM32F446xx SPI driver
 *            interrupt based send and read API as a master device
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
#include "stm32f446xx_hal_spi.h"

/* software delay */
void sw_delay() {
	for(uint32_t i = 0; i < 300000; i++);
}

/* flag to monitor slave data ready status */
bool gb_slave_data_ready = false;

/* SPI1 handle object */
SPI_handle_t gSPI1Handle;

#define MAX_RX_BYTES  (10U)

/* global buffer to store received data */
char g_rx_buffer[MAX_RX_BYTES] = {0};

/* global variable to 1 byte received */
char g_rx_data = 0;

/* total no. of bytes received */
uint8_t g_rx_bytes_cnt = 0;

/* flag to track if data has been received over SPI */
bool gb_data_received = false;

/* keeping some valid same interrupt
 * prio value for all the interrupts
 * defined in this application */
#define DEFAULT_INT_PRIO    (14U)

int main(void)
{
	/* configuration for GPIO pin connected to slave
	 * to receive a data ready interrupt from it */
	GPIO_handle_t GPIO_handle = {
			.pGPIOx = GPIOA,
			.cfg.pin_number = GPIO_PIN_9, //pin connected to the slave
			.cfg.pin_mode = GPIO_PIN_MODE_IT_FT, //interrupt falling edge trigger mode
			.cfg.pin_otype = 0x00, //reset value
			.cfg.pin_ospeed = 0x00, //reset value
			.cfg.pupd_ctrl = GPIO_PIN_PULL_UP,
			.cfg.alt_func = 0x00 //reset value
	};

	/* Initialize the GPIO pin for slave data ready interrupt on falling edge */
	HAL_GPIO_init(&GPIO_handle);
	HAL_GPIO_IRQ_config(IRQ_NO_EXTI9_5, DEFAULT_INT_PRIO, ENABLE);

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
			.cfg.speed = SPI_SCLK_SPEED_PCLK_DIV32,
			.cfg.cpol = SPI_CPOL_LOW,
			.cfg.cpha = SPI_CPHA_FIRST_CLK_EDGE,
			.cfg.dff = SPI_DATA_FRAME_8BIT,
			.cfg.ssm = SPI_HW_SLAVE_MGMT
	};

	memcpy(&gSPI1Handle, &SPI1Handle, sizeof(SPI_handle_t));

	/* initialize SPI peripheral */
	HAL_SPI_init(&gSPI1Handle);
	/* enable SPI interrupt */
	HAL_SPI_IRQ_config(IRQ_NO_SPI1, DEFAULT_INT_PRIO, ENABLE);

	while(1) {
		/* wait until slave data is ready */
		while(!gb_slave_data_ready);

		/* reset the flag for next interrupt */
		gb_slave_data_ready = false;

		/* disable any further data ready interrupt
		 * from slave until this data is received */
		HAL_GPIO_IRQ_config(IRQ_NO_EXTI9_5, DEFAULT_INT_PRIO, DISABLE);

		/* enable the SPI peripheral */
		HAL_SPI_ctrl(SPI1, ENABLE);

		/* dummy data to be sent by master
		 * in order to receive data from slave */
		uint8_t dummy_data = 0xA4;

		/* reset the total bytes received count */
		g_rx_bytes_cnt = 0;

		while(!gb_data_received) {
			/* send data over MOSI line in non-blocking mode */
			while(HAL_SPI_send_data_IT(&gSPI1Handle, &dummy_data, sizeof(dummy_data)) != HAL_SPI_READY);
			/* receive 1 byte (1 data frame) over MOSI line in non-blocking mode */
			while(HAL_SPI_read_data_IT(&gSPI1Handle, (uint8_t *)&g_rx_data, 1) != HAL_SPI_READY);
		}

		/* reset the flag to be able to
		 * receive the same data again */
		gb_data_received = false;

		/* re-enable GPIO data ready interrupt from
		 * slave until if data is available */
		HAL_GPIO_IRQ_config(IRQ_NO_EXTI9_5, DEFAULT_INT_PRIO, ENABLE);

		/* wait if SPI peripheral is busy */
		while(HAL_SPI_flag_status(SPI1, SPI_SR_BSY_FLAG));
		/* disable the SPI peripheral */
		HAL_SPI_ctrl(SPI1, DISABLE);//HAL_SPI_deinit(SPI1);
	}
}

/* EXTI IRQ Handler for the GPIO pins
 * corresponding to the Button
 * */
void EXTI9_5_IRQHandler(void)
{
	/* clear the IRQ pending bit for GPIO pin of button */
	HAL_GPIO_IRQ_handler(GPIO_PIN_9);

	/* set the slave data ready flag with some
	 * delay to handle button de-bouncing */
	sw_delay();
	gb_slave_data_ready = true;
	printf("Received data ready interrupt from slave\n");
}

/* IRQ Handler to service the interrupts
 * generated by SPI1 peripheral
 * */
void SPI1_IRQHandler()
{
	HAL_SPI_IRQ_handler(&gSPI1Handle);
}

/* SPI application event callback */
void HAL_SPI_app_evt_callback(
		SPI_handle_t *pSPIhandle, HAL_SPI_events_t evt) {

	(void)pSPIhandle;

	switch(evt) {
	case HAL_SPI_TX_DONE:
	{
		printf("SPI Tx Done\n");
		break;
	}
	case HAL_SPI_RX_DONE:
	{
		printf("Byte received = %c\r\n", g_rx_data);
		g_rx_buffer[g_rx_bytes_cnt++] = g_rx_data;
		if((g_rx_bytes_cnt == MAX_RX_BYTES) || (g_rx_data == '\0')) {
			/* set the flag as data has been received */
			gb_data_received = true;
			printf("SPI Rx Done\n");
			printf("String received = %s\n", g_rx_buffer);
		}
		break;
	}
	case HAL_SPI_ERR_REPORTED:
	{
		printf("SPI Error Reported\n");
		break;
	}
	default:
	{
		printf("SPI Unknown Event Reported\n");
		break;
	}
	}
}