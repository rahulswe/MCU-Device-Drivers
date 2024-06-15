#include <stdio.h>
#include <gpio_drv.h>

#define TEST_LED_BUTTON_TOGGLE

/* Soft delay */
void delay(){
  uint32_t i = 0;
  do{
    i++;
  }while(i<500000);
}

volatile uint8_t val = 0;
int main()
{
#ifdef TEST_LED_BLINK_APP
  GPIO_handle_t GPIOHandle;
  GPIOHandle.pGPIOx = GPIO_P0;
  GPIOHandle.GpioPinConfig.pinDirection = GPIO_PIN_MODE_OUT;
  GPIOHandle.GpioPinConfig.pinNumber = GPIO_PIN_13; // LED1
  GPIOHandle.GpioPinConfig.pinPull = GPIO_PIN_PULL_DISABLED;
  GPIOHandle.GpioPinConfig.pinInput = GPIO_PIN_IN_BUFF_DISCONNECT;
  GPIOHandle.GpioPinConfig.pinDrive = GPIO_PIN_DRIVE_S0S1;
  GPIOHandle.GpioPinConfig.pinSense = GPIO_PIN_SENSE_DISABLED;
  
  GPIO_Init(&GPIOHandle);
  
  for(uint8_t i = 1; i < 4; i++) {
    ++GPIOHandle.GpioPinConfig.pinNumber;
    GPIO_Init(&GPIOHandle);   
  }
  
  /* Turn OFF LEDs after initialization */
  GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_13, LED_OFF);
  GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_14, LED_OFF);
  GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_15, LED_OFF);
  GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_16, LED_OFF);
  
  /* Blinky App */
  while(1) {
#if 1
    GPIO_ToggleOutputPin(GPIO_P0, GPIO_PIN_13);
    GPIO_ToggleOutputPin(GPIO_P0, GPIO_PIN_14);
    GPIO_ToggleOutputPin(GPIO_P0, GPIO_PIN_15);
    GPIO_ToggleOutputPin(GPIO_P0, GPIO_PIN_16);
    delay();
#else
/*
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_13, 1);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_14, 1);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_15, 1);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_16, 1);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_13, 0);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_14, 0);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_15, 0);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_16, 0);
    delay();
    */
#endif
  }
#endif
#ifdef TEST_LED_BUTTON_TOGGLE
  GPIO_handle_t GPIOHandle1, GPIOHandle2;

  GPIOHandle1.pGPIOx = GPIO_P0;
  GPIOHandle1.GpioPinConfig.pinDirection = GPIO_PIN_MODE_OUT;
  GPIOHandle1.GpioPinConfig.pinNumber = GPIO_PIN_13; // LED1
  GPIOHandle1.GpioPinConfig.pinPull = GPIO_PIN_PULL_DISABLED;
  GPIOHandle1.GpioPinConfig.pinInput = GPIO_PIN_IN_BUFF_DISCONNECT;
  GPIOHandle1.GpioPinConfig.pinDrive = GPIO_PIN_DRIVE_S0S1;
  GPIOHandle1.GpioPinConfig.pinSense = GPIO_PIN_SENSE_DISABLED;
  
  GPIO_Init(&GPIOHandle1);
  
  for(uint8_t i = 1; i < 4; i++) {
    ++GPIOHandle1.GpioPinConfig.pinNumber;
    GPIO_Init(&GPIOHandle1);   
  }
  
  GPIOHandle2.pGPIOx = GPIO_P0;
  GPIOHandle2.GpioPinConfig.pinDirection = GPIO_PIN_MODE_IN;
  GPIOHandle2.GpioPinConfig.pinNumber = GPIO_PIN_11; // BUTTON1
  GPIOHandle2.GpioPinConfig.pinPull = GPIO_PIN_PULL_UP; /* If button pressed, it is connected to GND */
  GPIOHandle2.GpioPinConfig.pinInput = GPIO_PIN_IN_BUFF_CONNECT;
  GPIOHandle2.GpioPinConfig.pinDrive = GPIO_PIN_DRIVE_S0S1;
  GPIOHandle2.GpioPinConfig.pinSense = GPIO_PIN_SENSE_DISABLED;
  
  GPIO_Init(&GPIOHandle2);

  /* Turn OFF LEDs after initialization */
  GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_13, LED_OFF);
  GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_14, LED_OFF);
  GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_15, LED_OFF);
  GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_16, LED_OFF);
  
  uint8_t button_press_cnt = 0;
  /* Blinky App */
  while(1) {
#if 1
    if(GPIO_ReadFromInputPin(GPIO_P0, GPIO_PIN_11) == BUTTON_PRESSED) {
      delay(); /* to avoid button debouncing */
      GPIO_ToggleOutputPin(GPIO_P0, GPIO_PIN_13);
      GPIO_ToggleOutputPin(GPIO_P0, GPIO_PIN_14);
      GPIO_ToggleOutputPin(GPIO_P0, GPIO_PIN_15);
      GPIO_ToggleOutputPin(GPIO_P0, GPIO_PIN_16);
    }

#else
/*
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_13, 1);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_14, 1);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_15, 1);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_16, 1);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_13, 0);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_14, 0);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_15, 0);
    delay();
    GPIO_WriteToOutputPin(GPIO_P0, GPIO_PIN_16, 0);
    delay();
    */
#endif
  }
#endif
  return 0;
}