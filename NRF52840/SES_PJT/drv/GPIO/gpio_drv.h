/*
 ******************************************************************************
 * @file      gpio_drv.h
 * @author    Rahul Singh
 * @brief     Header file for nRF52840 GPIO driver
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

#ifndef _GPIO_DRV_H_
#define _GPIO_DRV_H_

#include <nrf52840.h>

/*
 This structure holds GPIO pin configuration
*/
typedef struct{
    uint8_t pinNumber;
    uint8_t pinDirection;  /*!< possible values from @GPIO_PIN_MODES >*/
    uint8_t pinInput;      /*!< possible values from @PIN_CONNECT_OR_DISCONNECT_INPUT_BUFFER >*/
    uint8_t pinPull;       /*!< possible values from @GPIO_PIN_PULL_CONFIGURATIONS >*/
    uint8_t pinSense;      /*!< possible values from @GPIO_PIN_SENSING_MECHANISM >*/
    uint8_t pinDrive;      /*!< possible values from @GPIO_PIN_DRIVE_CONFIGURATION >*/
}GPIO_PinConfig_t;

/*
 This is a handle structure for a GPIO pin
*/
typedef struct{
    GPIO_RegDef_t *pGPIOx; /*!< this holds the base address of the GPIO port to which the pin belongs >*/
    GPIO_PinConfig_t GpioPinConfig;
}GPIO_handle_t;

/*
  @GPIO_PIN_MODES
  GPIO pin possible modes
*/
#define GPIO_PIN_MODE_IN      0  // Input Mode
#define GPIO_PIN_MODE_OUT     1  // Output Mode

/*
  @GPIO_PIN_PULL_CONFIGURATIONS
  GPIO pin possible pull configurations
*/
#define GPIO_PIN_PULL_DISABLED  0 
#define GPIO_PIN_PULL_UP        3
#define GPIO_PIN_PULL_DOWN      1

/*
  @GPIO PIN_CONNECT_OR_DISCONNECT_INPUT_BUFFER
  GPIO pin connect or disconnect input buffer
*/
#define GPIO_PIN_IN_BUFF_CONNECT    0
#define GPIO_PIN_IN_BUFF_DISCONNECT 1

/*
  @GPIO_PIN_SENSING_MECHANISM
  GPIO pin possible drive mechanism
*/
#define GPIO_PIN_SENSE_DISABLED   0
#define GPIO_PIN_SENSE_HIGH       2
#define GPIO_PIN_SENSE_LOW        3

/*
  @GPIO_PIN_DRIVE_CONFIGURATION
  GPIO pin possible drive configurations
*/
#define GPIO_PIN_DRIVE_S0S1   0
#define GPIO_PIN_DRIVE_H0S1   1
#define GPIO_PIN_DRIVE_S0H1   2
#define GPIO_PIN_DRIVE_H0H1   3
#define GPIO_PIN_DRIVE_D0S1   4
#define GPIO_PIN_DRIVE_D0H1   5
#define GPIO_PIN_DRIVE_S0D1   6
#define GPIO_PIN_DRIVE_H0D1   7

/*
  @GPIO_PIN_NUMBERS 
*/
#define GPIO_PIN_0    0
#define GPIO_PIN_1    1
#define GPIO_PIN_2    2
#define GPIO_PIN_3    3
#define GPIO_PIN_4    4
#define GPIO_PIN_5    5
#define GPIO_PIN_6    6
#define GPIO_PIN_7    7
#define GPIO_PIN_8    8
#define GPIO_PIN_9    9
#define GPIO_PIN_10   10
#define GPIO_PIN_11   11
#define GPIO_PIN_12   12
#define GPIO_PIN_13   13
#define GPIO_PIN_14   14
#define GPIO_PIN_15   15
#define GPIO_PIN_16   16
#define GPIO_PIN_17   17
#define GPIO_PIN_18   18
#define GPIO_PIN_19   19
#define GPIO_PIN_20   20
#define GPIO_PIN_21   21
#define GPIO_PIN_22   22
#define GPIO_PIN_23   23
#define GPIO_PIN_24   24
#define GPIO_PIN_25   25
#define GPIO_PIN_26   26
#define GPIO_PIN_27   27
#define GPIO_PIN_28   28
#define GPIO_PIN_29   29
#define GPIO_PIN_30   30
#define GPIO_PIN_31   31

/*
  @GPIO_PIN_CNF_REG_BIT_POSITIONS
*/
#define BITP_GPIO_PIN_CNF_REG_DIR    0
#define BITP_GPIO_PIN_CNF_REG_INPUT  1
#define BITP_GPIO_PIN_CNF_REG_PULL   2
#define BITP_GPIO_PIN_CNF_REG_DRIVE  8
#define BITP_GPIO_PIN_CNF_REG_SENSE  16

/*
  LED Control Macros
*/
#define LEDS_ACTIVE_STATE 0 /*< refer LEDS_ACTIVE_STATE in nrf SDK >*/
#define LED_ON  0
#define LED_OFF 1

/*
 Button Macros
*/
#define BUTTON_PRESSED      0  /*< if button pressed, it gets connected to GND >*/
#define BUTTON_NOT_PRESSSED 1

/*********** GPIO Peripheral APIs ************/

/*
  Peripheral Clock Setup
*/
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t enable);

/*
  Init & De-Init 
*/
void GPIO_Init(GPIO_handle_t* pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t* pGpiox);

/*
  Read & Write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGpiox, uint8_t pinNumber);
uint32_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGpiox);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGpiox, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGpiox, uint32_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGpiox, uint8_t pinNumber);

/*
  IRQ & ISR Handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enable);
void GPIO_IRQHandler(uint8_t pinNumber);

#endif