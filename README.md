# MCU-Device-Drivers

## STM32F446xx MCU

Following peripheral drivers have been implemented for this MCU - 

### GPIO 

Driver files are located at the following location - 

```shell
STM32F446xx/src/Drivers/src/stm32f446xx_gpio.c
STM32F446xx/src/Drivers/inc/stm32f446xx_gpio.h
```

### SPI 

Driver files are located at the following location - 

```shell
STM32F446xx/src/Drivers/src/stm32f446xx_spi.c
STM32F446xx/src/Drivers/inc/stm32f446xx_spi.h
```

### I2C 

Driver files are located at the following location - 

```shell
STM32F446xx/src/Drivers/src/stm32f446xx_i2c.c
STM32F446xx/src/Drivers/inc/stm32f446xx_i2c.h
```

### USART (asynchronus mode)

Driver files are located at the following location - 

```shell
STM32F446xx/src/Drivers/src/stm32f446xx_usart.c
STM32F446xx/src/Drivers/inc/stm32f446xx_usart.h
```

### Target

Two ST NUCLEO-F446RE development boards were used to test these drivers for the STM32F446xx MCU by having them communicate with each other, with one acting as the master and the other as the slave.

### Debugging and Analysis

A Digilent logic analyzer was used for testing, debugging and analysis purpose.


## Nordic nRF52840 SOC

Following peripheral drivers have been implemented - 

### GPIO 

Driver files are located at the following location - 

```shell
NRF52840/SES_PJT/drv/GPIO/gpio_drv.c
NRF52840/SES_PJT/drv/GPIO/gpio_drv.h
```

### Target

Nordic nRF52840 development board was used to test the GPIO driver.

### Debugging and Analysis

A Digilent logic analyzer was used for testing, debugging and analysis purpose.