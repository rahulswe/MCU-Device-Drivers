/*
 ******************************************************************************
 * @file      gpio_drv.c
 * @author    Rahul Singh
 * @brief     Source file for nRF52840 GPIO driver
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

#include <gpio_drv.h>

/*
  Init & De-Init 
*/
void GPIO_Init(GPIO_handle_t* pGPIOHandle)
{
  pGPIOHandle->pGPIOx->PINCNF[pGPIOHandle->GpioPinConfig.pinNumber] = 
                                  (pGPIOHandle->GpioPinConfig.pinDirection << BITP_GPIO_PIN_CNF_REG_DIR)
                                  | (pGPIOHandle->GpioPinConfig.pinInput << BITP_GPIO_PIN_CNF_REG_INPUT)
                                  | (pGPIOHandle->GpioPinConfig.pinPull << BITP_GPIO_PIN_CNF_REG_PULL)
                                  | (pGPIOHandle->GpioPinConfig.pinSense << BITP_GPIO_PIN_CNF_REG_SENSE)
                                  | (pGPIOHandle->GpioPinConfig.pinDrive << BITP_GPIO_PIN_CNF_REG_DRIVE);
}

void GPIO_Deinit(GPIO_RegDef_t* pGpiox)
{
  uint8_t nTotalPins = 0;
  if(pGpiox == GPIO_P1) {
    nTotalPins = 32U;
  }else{
    nTotalPins = 16U;
  }

  for(uint8_t nIdx = 0; nIdx < nTotalPins; nIdx++){
    pGpiox->PINCNF[nIdx] = 0;
  }
}

/*
  Read & Write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGpiox, uint8_t pinNumber)
{
  uint8_t val = ((pGpiox->IN >> pinNumber) & 0x1U);
  return val;
}

uint32_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGpiox)
{
  return pGpiox->IN;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGpiox, uint8_t pinNumber, uint8_t value)
{
  if(value == 0) {
    pGpiox->OUTCLR = (1U << pinNumber);
  } else {
    pGpiox->OUTSET = (1U << pinNumber);
  }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGpiox, uint32_t value)
{
  pGpiox->OUT = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGpiox, uint8_t pinNumber)
{
  uint32_t pins_state = pGpiox->OUT;
  pGpiox->OUTSET = (~pins_state & (1UL << pinNumber));
  pGpiox->OUTCLR = (pins_state & (1UL << pinNumber));
}

/*
  IRQ & ISR Handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enable){

}

void GPIO_IRQHandler(uint8_t pinNumber){

}