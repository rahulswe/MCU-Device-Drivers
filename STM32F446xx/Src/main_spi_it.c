/*
 ******************************************************************************
 * @file      main_spi_it.c
 * @author    Rahul Singh
 * @brief     Sample application file for testing STM32F446xx SPI driver
 *            interrupt based APIs as a master device
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

/* flag to monitor button press status */
uint8_t g_button_pressed = false;

/* SPI1 handle object */
SPI_handle_t gSPI1Handle;

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
	HAL_GPIO_IRQ_config(IRQ_NO_EXTI15_10, 14, ENABLE);

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
	HAL_SPI_IRQ_config(IRQ_NO_SPI1, 14, ENABLE);

	while(1) {
		/* wait until user button is pressed */
		while(!g_button_pressed);

		/* reset the flag for next interrupt */
		g_button_pressed = false;

		/* sending a pattern data over MOSI line */
		char data[] = "Hello SPI";

		/* enable the SPI peripheral */
		HAL_SPI_ctrl(SPI1, ENABLE);

	    /* send data over MOSI line in non-blocking mode */
		while(HAL_SPI_send_data_IT(&gSPI1Handle, (uint8_t*)data, sizeof(data)) != HAL_SPI_READY);

		/* wait if SPI peripheral is busy */
		while(HAL_SPI_flag_status(SPI1, SPI_SR_BSY_FLAG));
		/* disable the SPI peripheral */
		HAL_SPI_ctrl(SPI1, DISABLE);//HAL_SPI_deinit(SPI1);
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

/* IRQ Handler to service the interrupts
 * generated by SPI1 peripheral
 * */
void SPI1_IRQHandler()
{
	HAL_SPI_IRQ_handler(&gSPI1Handle);
}

/* SPI application event callback */
void HAL_SPI_app_evt_callback(
		SPI_handle_t *pSPIhandle, uint8_t evt) {

	(void)pSPIhandle;

	switch(evt) {
	case HAL_SPI_TX_DONE:
	{
		printf("SPI Tx Done\n");
		break;
	}
	case HAL_SPI_RX_DONE:
	{
		printf("SPI Rx Done\n");
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
