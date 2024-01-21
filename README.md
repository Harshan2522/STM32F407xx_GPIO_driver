# GPIO Device driver for STM32F407xx
This is a simple and light weight device driver that is specifically developed for make use of the GPIO peripheral of STM32F407xx based discovery board.

## Overview ##
A concise and simple GPIO driver to operate some basic function's such like **GPIOx ENABLE**, **I/O TYPE**, **ALT FUNCTIONS** etc..,
These files are developed using STM32cubeIDE version 1.14.0 and testing using the STM32F4 discovery board.

>This GPIO device driver is been developed for the educational purpose, if in case of using for the project it is advised to go through the program implementation throughly.

## Folders and files
### Driver folder
  <details>
  <summary>inc</summary>
    1.stm32f407xx.h<br>
    2.stm32f407xx_gpio_driver.h
  </details>
  <details>
  <summary>src</summary>
    1.stm32f407xx_gpio_driver.c
  </details>


## Function discriptions

#### Mentioned functions are in 

```
  driver/inc/stm32f407xx.h
```
1.In this file the base address for all the GPIOx and peripheral suck like I2C, SPI, UART, Interrupt are definied.</br>
2.Clock enabling and disabling macro functions for above mentioned peripherals.</br>
3.The IRQ numbers and NVIC interrupt priority.</br>
4.RCC and GPIO register definition.</br>
 
These API are MCU specific functions
|    NAME   | Type     | Description                |
| :-------- | :------- | :------------------------- |
| `GPIOx_PCLK_EN` | `CLK enable` | Replace 'x' with Port name |
| `GOIOx_PCLK_DI` | `CLK disable`| Replace 'x' with Port name |
| `GPIOx_REG_RESET` | `De-initialize`| Replace 'x' with Port name |
| `I2Cx_PCLK_EN` | `CLK enable` | Replace 'x' with I2Cx name |
| `I2Cx_PCLK_DI` | `CLK disable`| Replace 'x' with I2Cx name |
| `SPIx_PCLK_EN` | `CLK enable` | Replace 'x' with SPIx name |
| `SPIx_PCLK_DI` | `CLK disable`| Replace 'x' with SPIx name |
| `USARTx_PCLK_EN` | `CLK enable` | Replace 'x' with USARTx name |
| `USARTx_PCLK_DI` | `CLK disable`| Replace 'x' with USARTX name |
| `EXTI_RegDef_t` | `Struct`| interrupt registers definition |
| `RCC_RegDef_t` | `Struct`| RCC registers definition |
| `GPIO_RegDef_t` | `Struct`| GPIO pin registers definition |
| `NVIC_enum_num` | `enum`| NVIC interrupt priority definition|


#### Mentioned functions are in 

```
   driver/inc/stm32f407xx_gpio_driver.h
```
1.This file holds the GPIO pin numbers, output modes, speed mode, push & pull configuration </br>
2.This file is specifics to the STM32F407VG discovery boards fuctions.</br>
3.It also consist of the most functions which need for the GPIO configurations.</br>
4.The GOIO handle structure is definied.</br>

| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `GPIO_Handle_t`      | `struct` | Holds the GOIPx base addresses & configurations |
| `GPIO_PinConfig_t`      | `struct` | configuration structure of GPIOx |
| `GPIO_PIN_NUM`      | `enum` | GPIOx pin number definitions |
| `GPIO_MODE`      | `enum` | GPIOx possible modes|
| `GPIO_OP_MODE`      | `enum` | GPIOx pin output mode |
| `GPIO_SPEED_MODE`      | `enum` | GPIOx pin speed modes |
| `GPIO_PUPD_MODE`      | `enum` | GPIOx push and pull up modes |

#### API reference for this drivers

```c
 /*
  * Peripheral clock setup
  */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx );

/*
 * Read,Write & toggling functions
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*
 * IRQ related functions
 */
void GPIO_IRQIntrruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQpPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
```
>[!NOTE]
>The detailed discription about every function is given inside the `driver/src/stm32f407xx_gpio_driver.c` file
>Refer the reference manual of this microcontroller for more information about the registers.



