/*
 ******************************************************************************
 * @file      main_gpio.c
 * @author    Rahul Singh
 * @brief     Sample application file for testing STM32F446xx GPIO driver
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

#include <stm32f446xx_hal_gpio.h>

//#define LED_TOGGLING_ON_BUTTON_PRESS_WO_INT
#define LED_TOGGLING_ON_BUTTON_PRESS_W_INT

#define ENABLE  (1U)
#define DISABLE (0U)

#define BUTTON_PRESSED    (0U)
#define BUTTON_RELEASED   (1U)

/* software delay */
void sw_delay() {
	for(uint32_t i = 0; i < 300000; i++);
}

int main(void)
{
#if defined(LED_TOGGLING_ON_BUTTON_PRESS_WO_INT)
	/* LED toggling on button press - Polling based */

	/* configuration for GPIO pin connected to LED on NUCLEOF446RE board */
	GPIO_handle_t GPIO_LED_handle = {
			.pGPIOx = GPIOA,
			.cfg.pin_number = GPIO_PIN_5, //LED Pin
			.cfg.pin_mode = GPIO_PIN_MODE_OUT, //output mode
			.cfg.pin_otype = GPIO_PIN_OTYPE_PUSH_PULL, //push-pull
			.cfg.pin_ospeed = GPIO_PIN_OSPEED_FAST, //fast-speed
			.cfg.pupd_ctrl = GPIO_PIN_NO_PULL_UP_DOWN, //no pull-up, pull-down
			.cfg.alt_func = 0x00 // doesn't matter
	};

	/* Initialize the GPIO pin for LED */
	HAL_GPIO_init(&GPIO_LED_handle);

	/* configuration for GPIO pin connected to button on NUCLEOF446RE board */
	GPIO_handle_t GPIO_BUTTON_handle = {
			.pGPIOx = GPIOC,
			.cfg.pin_number = GPIO_PIN_13, //Button Pin
			.cfg.pin_mode = GPIO_PIN_MODE_IN, //input mode
			.cfg.pin_otype = 0x00, //reset value
			.cfg.pin_ospeed = 0x00, //reset value
			.cfg.pupd_ctrl = GPIO_PIN_NO_PULL_UP_DOWN, //external pull-up present
			.cfg.alt_func = 0x00 //reset value
	};

	/* Initialize the GPIO pin for Button */
	HAL_GPIO_init(&GPIO_BUTTON_handle);

	/* Note: In contrast to the interrupt mode, in polling mode
	 * keeping the button pressed will make the LED blink as here
	 * we are continuously checking for the pin level while in
	 * case of interrupt based, we are just checking for the falling
	 * edge trigger.
	 */
	while(1) {
		/* if button is pressed, then toggle the GPIO pin of LED */
		if(HAL_GPIO_read_pin(GPIOC, GPIO_PIN_13) == BUTTON_PRESSED) {
			/* toggle the GPIO pin of LED  with some
		    * delay to handle button de-bouncing */
			sw_delay();
			HAL_GPIO_toggle_pin(GPIOA, GPIO_PIN_5);
		}
	}
#elif defined(LED_TOGGLING_ON_BUTTON_PRESS_W_INT)
	/* LED toggling on button press - Interrupt based */

	/* configuration for GPIO pin connected to LED on NUCLEOF446RE board */
	GPIO_handle_t GPIO_LED_handle = {
			.pGPIOx = GPIOA,
			.cfg.pin_number = GPIO_PIN_5, //LED Pin
			.cfg.pin_mode = GPIO_PIN_MODE_OUT, //output mode
			.cfg.pin_otype = GPIO_PIN_OTYPE_PUSH_PULL, //push-pull
			.cfg.pin_ospeed = GPIO_PIN_OSPEED_FAST, //fast-speed
			.cfg.pupd_ctrl = GPIO_PIN_NO_PULL_UP_DOWN, //no pull-up, pull-down
			.cfg.alt_func = 0x00 //reset value
	};

	/* Initialize the GPIO pin for LED */
	HAL_GPIO_init(&GPIO_LED_handle);

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

#endif
    /* Loop forever */
	for(;;);
}

/* EXTI IRQ Handler for the GPIO pins
 * corresponding to the Button
 * */
void EXTI15_10_IRQHandler(void)
{
	/* clear the IRQ pending bit for GPIO pin of button */
	HAL_GPIO_IRQ_handler(GPIO_PIN_13);

	/* toggle the GPIO pin of LED  with some
	 * delay to handle button de-bouncing */
	sw_delay();
	HAL_GPIO_toggle_pin(GPIOA, GPIO_PIN_5);
}
