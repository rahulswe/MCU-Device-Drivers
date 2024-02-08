/*
 ******************************************************************************
 * @file      stm32f446xx_hal_spi.h
 * @author    Rahul Singh
 * @brief     Header file for STM32F446xx SPI driver
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

#ifndef _STM32F446XX_HAL_SPI_H_
#define _STM32F446XX_HAL_SPI_H_

#include "stm32f446xx.h"

/* ***************** Enum Definitions ******************* */



/* ***************** Structure Definitions ******************* */

/* SPI configuration structure */
typedef struct {
	uint8_t device_mode;
	uint8_t speed;
	uint8_t cpol; //sclk polarity
	uint8_t cpha; //sclk phase
	uint8_t dff; //data frame format
	uint8_t bus_cfg;
	uint8_t ssm; //sw slave management ctrl
}SPI_cfg_t;

/* SPI handle structure */
typedef struct {
	SPI_reg_t *pSPIx;
	SPI_cfg_t cfg;
}SPI_handle_t;


/* ******************* Macro Definitions ********************* */

/* translates GPIO port base address to GPIO port number */
#define GPIO_BASE_ADDR_TO_PORT_NUM(GPIOx) ((GPIOx == GPIOA) ? 0:\
										  (GPIOx == GPIOB) ? 1:\
										  (GPIOx == GPIOC) ? 2:\
										  (GPIOx == GPIOD) ? 3:\
										  (GPIOx == GPIOE) ? 4:\
										  (GPIOx == GPIOF) ? 5:\
										  (GPIOx == GPIOG) ? 6:\
										  (GPIOx == GPIOH) ? 7:0 \
										 )

/* GPIO pin numbers */
#define GPIO_PIN_0     (0U)
#define GPIO_PIN_1     (1U)
#define GPIO_PIN_2     (2U)
#define GPIO_PIN_3     (3U)
#define GPIO_PIN_4     (4U)
#define GPIO_PIN_5     (5U)
#define GPIO_PIN_6     (6U)
#define GPIO_PIN_7     (7U)
#define GPIO_PIN_8     (8U)
#define GPIO_PIN_9     (9U)
#define GPIO_PIN_10    (10U)
#define GPIO_PIN_11    (11U)
#define GPIO_PIN_12    (12U)
#define GPIO_PIN_13    (13U)
#define GPIO_PIN_14    (14U)
#define GPIO_PIN_15    (15U)

/* GPIO pin modes */
#define GPIO_PIN_MODE_IN            (0x00U)
#define GPIO_PIN_MODE_OUT           (0x01U)
#define GPIO_PIN_MODE_ALT           (0x02U)
#define GPIO_PIN_MODE_ANALOG        (0x03U)
#define GPIO_PIN_MODE_IT_RT         (0x04U)
#define GPIO_PIN_MODE_IT_FT         (0x05U)
#define GPIO_PIN_MODE_IT_RT_FT      (0x06U)

/* GPIO output pin type */
#define GPIO_PIN_OTYPE_PUSH_PULL      (0x00U)
#define GPIO_PIN_OTYPE_OPEN_DRAIN     (0x01U)

/* GPIO output pin speed */
#define GPIO_PIN_OSPEED_LOW           (0x00U)
#define GPIO_PIN_OSPEED_MED           (0x01U)
#define GPIO_PIN_OSPEED_FAST          (0x02U)
#define GPIO_PIN_OSPEED_HIGH          (0x03U)

/* GPIO pin internal pull-up / pull-down configuration */
#define GPIO_PIN_NO_PULL_UP_DOWN      (0x00U)
#define GPIO_PIN_PULL_UP              (0x01U)
#define GPIO_PIN_PULL_DOWN            (0x02U)

/* GPIO pin alternate functionalities */
#define GPIO_PIN_ALT_FUNC_AF0         (0x00U)
#define GPIO_PIN_ALT_FUNC_AF1         (0x01U)
#define GPIO_PIN_ALT_FUNC_AF2         (0x02U)
#define GPIO_PIN_ALT_FUNC_AF3         (0x03U)
#define GPIO_PIN_ALT_FUNC_AF4         (0x04U)
#define GPIO_PIN_ALT_FUNC_AF5         (0x05U)
#define GPIO_PIN_ALT_FUNC_AF6         (0x06U)
#define GPIO_PIN_ALT_FUNC_AF7         (0x07U)
#define GPIO_PIN_ALT_FUNC_AF8         (0x08U)
#define GPIO_PIN_ALT_FUNC_AF9         (0x09U)
#define GPIO_PIN_ALT_FUNC_AF10        (0x0AU)
#define GPIO_PIN_ALT_FUNC_AF11        (0x0BU)
#define GPIO_PIN_ALT_FUNC_AF12        (0x0CU)
#define GPIO_PIN_ALT_FUNC_AF13        (0x0DU)
#define GPIO_PIN_ALT_FUNC_AF14        (0x0EU)
#define GPIO_PIN_ALT_FUNC_AF15        (0x0FU)

/* ***************** Function Declarations ******************* */

/*
 * @brief        Enable or Disable SPI peripheral clock
 *
 * @param[in]    pSPIx : SPI peripheral base address
 * @param[in]    en     : 0 -> enable, 1 0-> disable
 *
 * @return       None
 *
 */
void HAL_SPI_pclk_ctrl(SPI_reg_t *pSPIx, uint8_t en);

/*
 * @brief        Initialize/Setup/Configure SPI peripheral
 *
 * @param[in]    pSPIhandle : Pointer to SPI handle object
 *
 * @return       None
 *
 */
void HAL_SPI_init(SPI_handle_t *pSPIhandle);

/*
 * @brief        De-initialize SPI peripheral
 *
 * @param[in]    pSPIx : SPI peripheral base address
 *
 * @return       None
 *
 */
void HAL_SPI_deinit(SPI_reg_t *SPI_reg_t);

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
void HAL_SPI_read_data(SPI_reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

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
void HAL_SPI_send_data(SPI_reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);

#endif
