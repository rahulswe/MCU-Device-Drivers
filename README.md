# MCU-Device-Drivers

## STM32F446xx MCU

Following drivers have been implemented for this MCU - 

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

### Target

NUCLEOF446RE Development board was used for testing these drivers of STM32F446xx MCU. In case of SPI driver two of such boards were used to act as slave and master. 

### Debugging and Analysis

A Digilent logic analyzer was used for testing, debugging and analysis purpose.