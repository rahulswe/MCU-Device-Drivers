/*
 ******************************************************************************
 * @file      nrf52840.c
 * @author    Rahul Singh
 * @brief     This file includes nRF52840 device specific details such as
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

#ifndef _NRF52840_H_
#define _NRF52840_H_

#include <stdint.h>

/* Generic Macros */
#define _vo volatile
#define ENABLE  1
#define DISABLE 0

/* Base address of memory regions */
#define FLASH_BASEADDR    0x00000000U
#define RAM_BASEADDR      0x20000000U
#define ROM_BASEADDR      0x00000000U

/* AHB & APB bus peripherals base address */
#define PERIPH_BASEADDR          0x40000000U
#define APB_PERIPH_BASEADDR      PERIPH_BASEADDR
#define AHB_PERIPH_BASEADDR      0x50000000U

/* AHB peripherals base address */
#define GPIO_P0_BASEADDR         AHB_PERIPH_BASEADDR
#define GPIO_P1_BASEADDR         (AHB_PERIPH_BASEADDR + 0x00000300U)

/* APB perhipherals base address */
#define CLOCK_BASEADDR           APB_PERIPH_BASEADDR 

#define SPI_MASTER0_BASEADDR     (APB_PERIPH_BASEADDR + 0x00000300U)
#define SPI_MASTER1_BASEADDR     (APB_PERIPH_BASEADDR + 0x00000400U)

#define SPI_SLAVE0_BASEADDR      SPI_MASTER0_BASEADDR
#define SPI_SLAVE1_BASEADDR      SPI_MASTER1_BASEADDR

#define TWI_MASTER0_BASEADDR     (APB_PERIPH_BASEADDR + 0x00000300U)
#define TWI_MASTER1_BASEADDR     (APB_PERIPH_BASEADDR + 0x00000400U)

#define TWI_SLAVE0_BASEADDR      TWI_MASTER0_BASEADDR
#define TWI_SLAVE1_BASEADDR      TWI_MASTER1_BASEADDR

#define UARTE0_BASEADDR          (APB_PERIPH_BASEADDR + 0x00000200U)


/* GPIO peripheral registers */
typedef struct{
  _vo uint32_t reserve[321];
  _vo uint32_t OUT;            /* Address Offset : 0x504U */
  _vo uint32_t OUTSET;         /* Address Offset : 0x508U */
  _vo uint32_t OUTCLR;         /* Address Offset : 0x50CU */
  _vo uint32_t IN;             /* Address Offset : 0x510U */
  _vo uint32_t DIR;            /* Address Offset : 0x514U */
  _vo uint32_t DIRSET;         /* Address Offset : 0x518U */
  _vo uint32_t DIRCLR;         /* Address Offset : 0x51CU */
  _vo uint32_t LATCH;          /* Address Offset : 0x520U */
  _vo uint32_t DETECTMODE;     /* Address Offset : 0x524U */
  _vo uint32_t reserved[118];
  _vo uint32_t PINCNF[32];     /* Address Offset : 0x700U */
}GPIO_RegDef_t;

#define GPIO_P0    ((GPIO_RegDef_t*)(GPIO_P0_BASEADDR))
#define GPIO_P1    ((GPIO_RegDef_t*)(GPIO_P1_BASEADDR))

/* CLOCK peripheral registers */
typedef struct{
  _vo uint32_t TASKS_HFCLKSTART;            /* Address Offset : 0x00U */
  _vo uint32_t TASKS_HFCLKSTOP;             /* Address Offset : 0x04U */
  _vo uint32_t TASKS_LFCLKSTART;            /* Address Offset : 0x04U */
  _vo uint32_t TASKS_LFCLKSTOP;             /* Address Offset : 0x04U */
  _vo uint32_t TASKS_CAL;                   /* Address Offset : 0x04U */
  _vo uint32_t TASKS_CSTART;                /* Address Offset : 0x04U */
  _vo uint32_t TASKS_CSTOP;                 /* Address Offset : 0x04U */
  _vo uint32_t TASKS_HFCLKSTARTED;          /* Address Offset : 0x04U */
  _vo uint32_t TASKS_LFCLKSTARTED;          /* Address Offset : 0x04U */
  _vo uint32_t EVENTS_DONE;                 /* Address Offset : 0x04U */
  _vo uint32_t EVENTS_CTIO;                 /* Address Offset : 0x04U */
  _vo uint32_t EVENTS_CTSTARTED;            /* Address Offset : 0x04U */
  _vo uint32_t EVENTS_CTSTOPPED;            /* Address Offset : 0x04U */
  _vo uint32_t INTENSET;                    /* Address Offset : 0x04U */
  _vo uint32_t INTENCLR;                    /* Address Offset : 0x04U */
  _vo uint32_t HFCLKRUN;                    /* Address Offset : 0x04U */
  _vo uint32_t HFCLKSTAT;                   /* Address Offset : 0x04U */
  _vo uint32_t LFCLKRUN;                    /* Address Offset : 0x04U */
  _vo uint32_t LFCLKSTAT;                   /* Address Offset : 0x04U */
  _vo uint32_t LFCLKSRCOPY;                 /* Address Offset : 0x04U */
  _vo uint32_t LFCLKSRC;                    /* Address Offset : 0x04U */
  _vo uint32_t HFXODEBOUNCE;                /* Address Offset : 0x04U */
  _vo uint32_t CTIV;                        /* Address Offset : 0x04U */
  _vo uint32_t TRACECONFIG;                 /* Address Offset : 0x04U */
  _vo uint32_t LFRCMODE;                    /* Address Offset : 0x04U */
}CLOCK_RefDef_t;

#define CLOCK      ((CLOCK_RegDef_t*)(CLOCK_BASEADDR))

#endif