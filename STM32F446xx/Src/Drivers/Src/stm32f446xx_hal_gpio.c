/*
 ******************************************************************************
 * @file      stm32f446xx_hal_gpio.c
 * @author    Rahul Singh
 * @brief     Source file for STM32F446xx GPIO driver
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

#include <stm32f446xx_hal_gpio.h>

/*
 * @brief        Enable or Disable GPIO peripheral clock
 *
 * @param[in]    pGPIOx : GPIO port base address
 * @param[in]    en     : 0 -> enable, 1 0-> disable
 *
 * @return       None
 *
 */
void HAL_GPIO_pclk_ctrl(GPIO_reg_t *pGPIOx, uint8_t en)
{
	uint8_t port = GPIO_BASE_ADDR_TO_PORT_NUM(pGPIOx);

    /* check whether to enable or disable the GPIO peripheral clock */
	if(en) {
		/* enable the GPIO peripheral clock for the specified port */
		__GPIOx_PCLK_EN(port);
	} else {
		/* disable the GPIO peripheral clock for the specified port */
		__GPIOx_PCLK_DIS(port);
	}
}

/*
 * @brief        Initialize/Setup/Configure GPIO pin
 *
 * @param[in]    pGPIOhandle : Pointer to GPIO handle object
 *
 * @return       None
 *
 */
void HAL_GPIO_init(GPIO_handle_t *pGPIOhandle)
{
	uint32_t reg_val = 0;

	/* check the GPIO pin mode specified in the pGPIOhandle cfg structure
	 * is <= analog mode i.e. input mode or output mode or analog mode */
	if(pGPIOhandle->cfg.pin_mode <= GPIO_PIN_MODE_ANALOG) {

		/* set the GPIO pin mode as per the specified configuration */
		reg_val = ~(0x3 << (pGPIOhandle->cfg.pin_number << 1));
		pGPIOhandle->pGPIOx->MODE &= reg_val;
		reg_val = (pGPIOhandle->cfg.pin_mode << (pGPIOhandle->cfg.pin_number << 1));
		pGPIOhandle->pGPIOx->MODE |= reg_val;

		/* check if the GPIO pin mode specified is output mode */
		if(pGPIOhandle->cfg.pin_mode == GPIO_PIN_MODE_OUT)
		{
			/* set GPIO pin output type */
			reg_val = ~(0x1 << pGPIOhandle->cfg.pin_number);
			pGPIOhandle->pGPIOx->OTYPE &= reg_val;
			reg_val = (pGPIOhandle->cfg.pin_otype << pGPIOhandle->cfg.pin_number);
			pGPIOhandle->pGPIOx->OTYPE |= reg_val;

			/* set GPIO output pin speed */
			reg_val = ~(0x3 << (pGPIOhandle->cfg.pin_number << 1));
			pGPIOhandle->pGPIOx->OSPEED &= reg_val;
			reg_val = (pGPIOhandle->cfg.pin_ospeed << (pGPIOhandle->cfg.pin_number << 1));
			pGPIOhandle->pGPIOx->OSPEED |= reg_val;
		}

		/* check if the GPIO pin mode specified is alternate functionality mode */
		if(pGPIOhandle->cfg.pin_mode == GPIO_PIN_MODE_ALT)
		{
			/* set GPIO pin alternate functionality */
			uint8_t reg_idx = pGPIOhandle->cfg.pin_number / 8;
			uint8_t pin_num = pGPIOhandle->cfg.pin_number % 8;
			reg_val = ~(0xF << (pin_num << 0x2));
			pGPIOhandle->pGPIOx->AFUNC[reg_idx] &= reg_val;
			reg_val = (pGPIOhandle->cfg.alt_func << (pin_num << 0x2));
			pGPIOhandle->pGPIOx->AFUNC[reg_idx] |= reg_val;

		}
	} else {
		/* incase GPIO pin mode is interrupt mode i.e. rising trigger or
		 * falling trigger or rising and falling trigger */

		/* enable the SYSCFG peripheral clock */
		__SYSCFG_CLK_EN();

		/* enable the link between GPIO pin and EXTI line */
		uint8_t reg_idx = pGPIOhandle->cfg.pin_number / 4;
		uint8_t pos = (pGPIOhandle->cfg.pin_number % 4) << 0x2;
		uint8_t port = GPIO_BASE_ADDR_TO_PORT_NUM(pGPIOhandle->pGPIOx);
		SYSCFG->EXTICR[reg_idx] |= (port << pos);

		/* enable interrupt in the interrupt mask register */
		EXTI->IMR |= (0x1 << pGPIOhandle->cfg.pin_number);

		/* configure interrupt trigger methode as per the specified pin mode */
		if(pGPIOhandle->cfg.pin_mode == GPIO_PIN_MODE_IT_RT) {
			/* enable rising trigger */
			EXTI->RTSR |= (0x1 << pGPIOhandle->cfg.pin_number);
			/* disable falling trigger (precautionary measure) */
			EXTI->FTSR &= ~(0x1 << pGPIOhandle->cfg.pin_number);
		} else if(pGPIOhandle->cfg.pin_mode == GPIO_PIN_MODE_IT_FT) {
			/* disable rising trigger */
			EXTI->RTSR &= ~(0x1 << pGPIOhandle->cfg.pin_number);
			/* enable falling trigger (precautionary measure) */
			EXTI->FTSR |= (0x1 << pGPIOhandle->cfg.pin_number);
		} else if(pGPIOhandle->cfg.pin_mode == GPIO_PIN_MODE_IT_RT_FT) {
			/* enable rising trigger */
			EXTI->RTSR |= (0x1 << pGPIOhandle->cfg.pin_number);
			/* enable falling trigger */
			EXTI->FTSR |= (0x1 << pGPIOhandle->cfg.pin_number);
		}
	}

	/* configure GPIO internal pull-up/pull-down */
	reg_val = ~(0x3 << (pGPIOhandle->cfg.pin_number << 1));
	pGPIOhandle->pGPIOx->PUPD &= reg_val;
	reg_val = (pGPIOhandle->cfg.pupd_ctrl << (pGPIOhandle->cfg.pin_number << 1));
	pGPIOhandle->pGPIOx->PUPD |= reg_val;

}

/*
 * @brief        De-initialize GPIO port
 *
 * @param[in]    pGPIOx : GPIO port base address
 *
 * @return       None
 *
 */
void HAL_GPIO_deinit(GPIO_reg_t *pGPIOx)
{
	/* reset the GPIO port */
	uint8_t port = GPIO_BASE_ADDR_TO_PORT_NUM(pGPIOx);
	__GPIOx_RST(port);
}

/*
 * @brief        Read a GPIO pin
 *
 * @param[in]    pGPIOx : GPIO port base address
 *
 * @return       GPIO pin state -> GPIO_PIN_SET : 1, GPIO_PIN_RESET : 0
 *
 */
uint8_t HAL_GPIO_read_pin(GPIO_reg_t *pGPIOx, uint8_t pin_num)
{
	/* return the GPIO pin state */
	return ((uint8_t)((pGPIOx->IDR >> pin_num) & 0x01));
}

/*
 * @brief        Read a GPIO port
 *
 * @param[in]    pGPIOx : GPIO port base address
 *
 * @return       GPIO port value of uint16_t type
 *
 */
uint16_t HAL_GPIO_read_port(GPIO_reg_t *pGPIOx)
{
	/* return GPIO port value */
	return ((uint16_t)pGPIOx->IDR);
}

/*
 * @brief        Write a GPIO pin
 *
 * @param[in]    pGPIOx    : GPIO port base address
 * @param[in]    pin_num   : GPIO pin number
 * @param[in]    pin_state : GPIO pin state -> GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return       None
 *
 */
void HAL_GPIO_write_pin(GPIO_reg_t *pGPIOx, uint8_t pin_num, uint8_t pin_val)
{
	/* write the GPIO pin state */
	pGPIOx->ODR &= ~(0x1 << pin_num);
	pGPIOx->ODR |= ((pin_val & 0x1) << pin_num);
}

/*
 * @brief        Write a GPIO port
 *
 * @param[in]    pGPIOx  : GPIO port base address
 * @param[in]    pin_val : GPIO port value of uint16_t type
 *
 * @return       None
 *
 */
void HAL_GPIO_write_port(GPIO_reg_t *pGPIOx, uint16_t pin_val)
{
	/* set the GPIO port value */
	pGPIOx->IDR = (uint32_t)pin_val;
}

/*
 * @brief        Toggle a GPIO pin
 *
 * @param[in]    pGPIOx  : GPIO port base address
 * @param[in]    pin_num : GPIO pin number
 *
 * @return       None
 *
 */
void HAL_GPIO_toggle_pin(GPIO_reg_t *pGPIOx, uint8_t pin_num)
{
	/* toggle the GPIO pin state */
	pGPIOx->ODR ^= (0x1 << pin_num);
}

/*
 * @brief        Configure the IRQ for a GPIO pin
 *
 * @param[in]    IRQ_num  : IRQ number for a specific GPIO pin
 * @param[in]    IRQ_prio : Priority level for the specified IRQ (0 to 16)
 * @param[in]    en       : 1 -> enable the IRQ, 0-> Disable the IRQ
 *
 * @return       None
 *
 */
void HAL_GPIO_IRQ_config(uint32_t IRQ_num, uint32_t IRQ_prio, uint8_t en)
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
 * @brief        GPIO IRQ handler
 *
 * @param[in]    pin_num   : GPIO pin number
 *
 * @return       None
 *
 */
void HAL_GPIO_IRQ_handler(uint8_t pin_num)
{
	/* clear the EXTI interrupt pending bit
	corresponding to the specified GPIO pin */
	if(__EXTI_LINE_INT_STS(pin_num)) {
		__EXTI_LINE_CLR_INT(pin_num);
	}
}
