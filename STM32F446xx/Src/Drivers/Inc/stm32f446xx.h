/*
 ******************************************************************************
 * @file      stm32f446xx.h
 * @author    Rahul Singh
 * @brief     This file includes STM32F446xx device specific details such as
 *            definitions for  different peripherals, memories, registers, etc.
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

#ifndef _STM32F446XX_H_
#define _STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>

/* ********* Utility Macro Definitions ********** */

/* NULL pointer check macro */
#define NULL_PTR_CHK(x) if(x == NULL) { return; }

#define ENABLE          (1U)
#define DISABLE         (0U)
#define FLAG_SET        (1U)
#define FLAG_NOT_SET    (0U)

#define NUM_BITS_IN_A_BYTE            (8U)
/* 4 most significant bits in NVIC priority register's PRI_X 8-bit field
 * are implemented to define 16 possible priority levels (0 to 15) */
#define NUM_VALID_INT_PRIO_BITS       (4U)
/* Bit position in the interrupt priority registers's
 * PRI_X 8-bit field, from where the valid bits starts */
#define VALID_INT_PRIO_BIT_START_POS  ((NUM_BITS_IN_A_BYTE) - (NUM_VALID_INT_PRIO_BITS))
/* Priority of 4 interrupts can be set per NVIC priority register */
#define NUM_INT_PRIO_PER_REG          (4U)
/* No. of interrupts which can be enabled per NVIC ISER register */
#define NUM_INT_PER_NVIC_ISER_REG     (32U)

/* ********* Cortex-M4 Processor Specific Registers ********** */

#define NVIC_ISER0             ((volatile uint32_t*)0xE000E100U)
#define NVIC_ICER0             ((volatile uint32_t*)0xE000E180U)
#define NVIC_ISPR0             ((volatile uint32_t*)0xE000E200U)
#define NVIC_ICPR0             ((volatile uint32_t*)0xE000E280U)
#define NVIC_IABR0             ((volatile uint32_t*)0xE000E300U)
#define NVIC_IPR0              ((volatile uint32_t*)0xE000E400U)

/* **************  Device Memory Base Addresses ************** */

#define FLASH_BASE_ADDR        (0x08000000U)
#define SRAM_BASE_ADDR         (0x20000000U)

/* *********** Device Peripheral's Base Addresses ************ */

/* Bus Domains Base Addresses*/
#define APB1_BASE_ADDR         (0x40000000U)
#define APB2_BASE_ADDR         (0x40010000U)

#define AHB1_BASE_ADDR         (0x40020000U)
#define AHB2_BASE_ADDR         (0x50000000U)

/* GPIO Peripheral Base Addresses */
#define GPIOA_BASE_ADDR        (AHB1_BASE_ADDR + 0x0000U)
#define GPIOB_BASE_ADDR        (AHB1_BASE_ADDR + 0x0400U)
#define GPIOC_BASE_ADDR        (AHB1_BASE_ADDR + 0x0800U)
#define GPIOD_BASE_ADDR        (AHB1_BASE_ADDR + 0x0C00U)
#define GPIOE_BASE_ADDR        (AHB1_BASE_ADDR + 0x1000U)
#define GPIOF_BASE_ADDR        (AHB1_BASE_ADDR + 0x1400U)
#define GPIOG_BASE_ADDR        (AHB1_BASE_ADDR + 0x1800U)
#define GPIOH_BASE_ADDR        (AHB1_BASE_ADDR + 0x1C00U)

/* SPI Peripheral Base Addresses */
#define SPI1_BASE_ADDR         (APB2_BASE_ADDR + 0x3000U)
#define SPI2_BASE_ADDR         (APB1_BASE_ADDR + 0x3800U)
#define SPI3_BASE_ADDR         (APB1_BASE_ADDR + 0x3C00U)
#define SPI4_BASE_ADDR         (APB2_BASE_ADDR + 0x3400U)

/* RCC Peripheral Base Address */
#define RCC_BASE_ADDR          (AHB1_BASE_ADDR + 0x3800U)

/* EXTI Peripheral Base Address */
#define EXTI_BASE_ADDR         (APB2_BASE_ADDR + 0x3C00U)
#define SYSCFG_BASE_ADDR       (APB2_BASE_ADDR + 0x3800U)

/* USART Peripheral Base Address */
#define USART1_BASE_ADDR       (APB2_BASE_ADDR + 0x1000U)
#define USART2_BASE_ADDR       (APB1_BASE_ADDR + 0x4400U)
#define USART3_BASE_ADDR       (APB1_BASE_ADDR + 0x4800U)
#define UART4_BASE_ADDR        (APB1_BASE_ADDR + 0x4C00U)
#define UART5_BASE_ADDR        (APB1_BASE_ADDR + 0x5000U)
#define USART6_BASE_ADDR       (APB2_BASE_ADDR + 0x1400U)

/* ********* Device Specific IRQ Number Definitions ********** */

#define IRQ_NO_EXTI0         (6U)
#define IRQ_NO_EXTI1         (7U)
#define IRQ_NO_EXTI2         (8U)
#define IRQ_NO_EXTI3         (9U)
#define IRQ_NO_EXTI4         (10U)
#define IRQ_NO_EXTI9_5       (23U)
#define IRQ_NO_EXTI15_10     (40U)

#define IRQ_NO_SPI1          (35U)
#define IRQ_NO_SPI2          (36U)
#define IRQ_NO_SPI3          (51U)
#define IRQ_NO_SPI4          (84U)

#define IRQ_NO_USART1          (37U)
#define IRQ_NO_USART2          (38U)
#define IRQ_NO_USART3          (39U)
#define IRQ_NO_UART4           (52U)
#define IRQ_NO_UART5           (53U)
#define IRQ_NO_USART6          (71U)

/* high speed internal oscillator frequency */
#define HSI_OSC_FREQ         (16000000U)
/* high speed external oscillator frequency */
#define HSE_OSC_FREQ         (8000000U)

/* ********** Register Set Structure Definitions ********** */

/* GPIO Register Set Structure Definition */
typedef struct {
	volatile uint32_t MODE;
	volatile uint32_t OTYPE;
	volatile uint32_t OSPEED;
	volatile uint32_t PUPD;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSR;
	volatile uint32_t LOCK;
	volatile uint32_t AFUNC[2];
} GPIO_reg_t;

/* RCC Register Set Structure Definition */
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PULL_CFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED6;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;
} RCC_reg_t;

/* EXTI Register Set Structure Definition */
typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_reg_t;

/* SYSCFG Register Set Structure Definition */
typedef struct {
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CMPCR;
	volatile uint32_t CFGR;
} SYSCFG_reg_t;

/* SPI Register Set Structure Definition */
typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_reg_t;

/* USART Register Set Structure Definition */
typedef struct {
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} USART_reg_t;

/* ************* Device Peripheral's Definitions ************* */

/* GPIO Peripherals */
#define GPIOA                  ((GPIO_reg_t*)GPIOA_BASE_ADDR)
#define GPIOB                  ((GPIO_reg_t*)GPIOB_BASE_ADDR)
#define GPIOC                  ((GPIO_reg_t*)GPIOC_BASE_ADDR)
#define GPIOD                  ((GPIO_reg_t*)GPIOD_BASE_ADDR)
#define GPIOE                  ((GPIO_reg_t*)GPIOE_BASE_ADDR)
#define GPIOF                  ((GPIO_reg_t*)GPIOF_BASE_ADDR)
#define GPIOG                  ((GPIO_reg_t*)GPIOG_BASE_ADDR)
#define GPIOH                  ((GPIO_reg_t*)GPIOH_BASE_ADDR)

/* RCC peripheral */
#define RCC                    ((RCC_reg_t*)RCC_BASE_ADDR)

/* EXTI peripheral */
#define EXTI                   ((EXTI_reg_t*)EXTI_BASE_ADDR)

/* SYSCFG peripheral */
#define SYSCFG                 ((SYSCFG_reg_t*)SYSCFG_BASE_ADDR)

/* SPI Peripherals */
#define SPI1                   ((SPI_reg_t*)SPI1_BASE_ADDR)
#define SPI2                   ((SPI_reg_t*)SPI2_BASE_ADDR)
#define SPI3                   ((SPI_reg_t*)SPI3_BASE_ADDR)
#define SPI4                   ((SPI_reg_t*)SPI4_BASE_ADDR)

/* USART Peripherals */
#define USART1                 ((USART_reg_t*)USART1_BASE_ADDR)
#define USART2                 ((USART_reg_t*)USART2_BASE_ADDR)
#define USART3                 ((USART_reg_t*)USART3_BASE_ADDR)
#define UART4                  ((USART_reg_t*)UART4_BASE_ADDR)
#define UART5                  ((USART_reg_t*)UART5_BASE_ADDR)
#define USART6                 ((USART_reg_t*)USART6_BASE_ADDR)

/* ****** Macro Definitions for Some Peripheral's Basic Functionality ****** */

/* GPIO peripheral clock enable */
#define __GPIOx_PCLK_EN(port)            (RCC->AHB1ENR |= (0x1 << port))
/* GPIO peripheral clock disable */
#define __GPIOx_PCLK_DIS(port)           (RCC->AHB1ENR &= ~(0x1 << port))
/* GPIO peripheral port register reset */
#define __GPIOx_RST(port)                { RCC->AHB1RSTR |= (0x1 << port) ; RCC->AHB1RSTR &= ~(0x1 << port); }

/* SYSCFG peripheral clock enable */
#define __SYSCFG_CLK_EN()                (RCC->APB2ENR |= (0x1 << 0xE))
/* SYSCFG peripheral clock disable */
#define __SYSCFG_CLK_DIS()               (RCC->APB2ENR &= ~(0x1 << 0xE))

/* SPI1 peripheral clock enable */
#define __SPI1_CLK_EN()                  (RCC->APB2ENR |= (0x1 << 0xC))
/* SPI1 peripheral clock disable */
#define __SPI1_CLK_DIS()                 (RCC->APB2ENR &= ~(0x1 << 0xC))
/* SPI1 peripheral register reset */
#define __SPI1_RST()                	 { RCC->APB2RSTR |= (0x1 << 0xC) ; RCC->APB2RSTR &= ~(0x1 << 0xC); }

/* SPI2 peripheral clock enable */
#define __SPI2_CLK_EN()                  (RCC->APB1ENR |= (0x1 << 0xE))
/* SPI2 peripheral clock disable */
#define __SPI2_CLK_DIS()                 (RCC->APB1ENR &= ~(0x1 << 0xE))
/* SPI2 peripheral register reset */
#define __SPI2_RST()                	 { RCC->APB1RSTR |= (0x1 << 0xE) ; RCC->APB1RSTR &= ~(0x1 << 0xE); }

/* SPI3 peripheral clock enable */
#define __SPI3_CLK_EN()                  (RCC->APB1ENR |= (0x1 << 0xF))
/* SPI3 peripheral clock disable */
#define __SPI3_CLK_DIS()                 (RCC->APB1ENR &= ~(0x1 << 0xF))
/* SPI3 peripheral register reset */
#define __SPI3_RST()                	 { RCC->APB1RSTR |= (0x1 << 0xF) ; RCC->APB1RSTR &= ~(0x1 << 0xF); }

/* SPI4 peripheral clock enable */
#define __SPI4_CLK_EN()                  (RCC->APB2ENR |= (0x1 << 0xD))
/* SPI4 peripheral clock disable */
#define __SPI4_CLK_DIS()                 (RCC->APB2ENR &= ~(0x1 << 0xD))
/* SPI4 peripheral register reset */
#define __SPI4_RST()                	 { RCC->APB2RSTR |= (0x1 << 0xD) ; RCC->APB2RSTR &= ~(0x1 << 0xD); }

/* USART1 peripheral clock enable */
#define __USART1_CLK_EN()                 (RCC->APB2ENR |= (0x1 << 0x04))
/* USART1 peripheral clock disable */
#define __USART1_CLK_DIS()                (RCC->APB2ENR &= ~(0x1 << 0x04))
/* USART1 peripheral register reset */
#define __USART1_RST()                	  { RCC->APB2RSTR |= (0x1 << 0x04) ; RCC->APB2RSTR &= ~(0x1 << 0x04); }

/* USART2 peripheral clock enable */
#define __USART2_CLK_EN()                 (RCC->APB1ENR |= (0x1 << 0x11))
/* USART2 peripheral clock disable */
#define __USART2_CLK_DIS()                (RCC->APB1ENR &= ~(0x1 << 0x11))
/* USART2 peripheral register reset */
#define __USART2_RST()                	  { RCC->APB1RSTR |= (0x1 << 0x11) ; RCC->APB1RSTR &= ~(0x1 << 0x11); }

/* USART3 peripheral clock enable */
#define __USART3_CLK_EN()                 (RCC->APB1ENR |= (0x1 << 0x12))
/* USART3 peripheral clock disable */
#define __USART3_CLK_DIS()                (RCC->APB1ENR &= ~(0x1 << 0x12))
/* USART3 peripheral register reset */
#define __USART3_RST()                	  { RCC->APB1RSTR |= (0x1 << 0x12) ; RCC->APB1RSTR &= ~(0x1 << 0x12); }

/* UART4 peripheral clock enable */
#define __UART4_CLK_EN()                  (RCC->APB1ENR |= (0x1 << 0x13))
/* UART4 peripheral clock disable */
#define __UART4_CLK_DIS()                 (RCC->APB1ENR &= ~(0x1 << 0x13))
/* UART4 peripheral register reset */
#define __UART4_RST()                	  { RCC->APB1RSTR |= (0x1 << 0x13) ; RCC->APB1RSTR &= ~(0x1 << 0x13); }

/* UART5 peripheral clock enable */
#define __UART5_CLK_EN()                  (RCC->APB1ENR |= (0x1 << 0x14))
/* UART5 peripheral clock disable */
#define __UART5_CLK_DIS()                 (RCC->APB1ENR &= ~(0x1 << 0x14))
/* UART5 peripheral register reset */
#define __UART5_RST()                	  { RCC->APB1RSTR |= (0x1 << 0x14) ; RCC->APB1RSTR &= ~(0x1 << 0x14); }

/* USART6 peripheral clock enable */
#define __USART6_CLK_EN()                 (RCC->APB2ENR |= (0x1 << 0x05))
/* USART6 peripheral clock disable */
#define __USART6_CLK_DIS()                (RCC->APB2ENR &= ~(0x1 << 0x05))
/* USART6 peripheral register reset */
#define __USART6_RST()                	  { RCC->APB2RSTR |= (0x1 << 0x05) ; RCC->APB2RSTR &= ~(0x1 << 0x05); }

/* EXTI line get interrupt status */
#define __EXTI_LINE_INT_STS(pin_num)     (EXTI->PR & (1 << pin_num))
/* EXTI line clear pending interrupt */
#define __EXTI_LINE_CLR_INT(pin_num)     (EXTI->PR |= (1 << pin_num))

/* Enable specific IRQ number in NVIC */
#define __NVIC_ENABLE_IRQ(irq_num)       { *(NVIC_ISER0 + (irq_num/NUM_INT_PER_NVIC_ISER_REG)) \
	                                          |= (1 << (irq_num % NUM_INT_PER_NVIC_ISER_REG)); }
/* Disable specific IRQ number in NVIC */
#define __NVIC_DISABLE_IRQ(irq_num)      { *(NVIC_ICER0 + (irq_num/NUM_INT_PER_NVIC_ISER_REG)) \
	                                          |= (1 << (irq_num % NUM_INT_PER_NVIC_ISER_REG)); }
/* Set priority for a specific IRQ number in NVIC */
#define __NVIC_SET_PRIORITY(irq_num, irq_priority)    { uint8_t reg_idx = irq_num/NUM_INT_PRIO_PER_REG;\
	                                                    uint8_t pos = VALID_INT_PRIO_BIT_START_POS + \
														    ((irq_num % NUM_INT_PRIO_PER_REG) << 0x3); \
		                                                *(NVIC_IPR0 + reg_idx) &= ~(0xFF << pos); \
		                                                *(NVIC_IPR0 + reg_idx) |= ((irq_priority) << pos); \
	                                                  }
#endif
