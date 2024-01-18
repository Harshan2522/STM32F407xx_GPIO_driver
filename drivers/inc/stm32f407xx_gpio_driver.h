/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Dec 29, 2023
 *      Author: ASUS
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"



/*
 * configuration structure of GPIO
 */
typedef struct
{
	uint8_t GPIO_PinNumber;      /* to know the possible mode @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;        /* to know the possible modes @GPIO_PIN_MODE*/
	uint8_t GPIO_PinSpeed;       /* to know the possible mode @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl; /* to know the possible mode @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_pinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;             /* This hold the base address of the GPIO port for the pin */
	GPIO_PinConfig_t Gpio_PinConfig;   /* This hold the GPIO configuration setting*/
}GPIO_Handle_t;


/*
 * GPIO number available pin
 * @GPIO_PIN_NUMBER
 */
typedef enum
{
	GPIO_PIN_NO_0=0,
	GPIO_PIN_NO_1,
	GPIO_PIN_NO_2,
	GPIO_PIN_NO_3,
	GPIO_PIN_NO_4,
	GPIO_PIN_NO_5,
	GPIO_PIN_NO_6,
	GPIO_PIN_NO_7,
	GPIO_PIN_NO_8,
	GPIO_PIN_NO_9,
	GPIO_PIN_NO_10,
	GPIO_PIN_NO_11,
	GPIO_PIN_NO_12,
	GPIO_PIN_NO_13,
	GPIO_PIN_NO_14,
	GPIO_PIN_NO_15
}GPIO_PIN_NUM;
/*
 * GPIO pin possible mode
 * @GPIO_PIN_MODE
 */

typedef enum
{
	GPIO_MODE_IN=0,
	GPIO_MODE_OUT,
	GPIO_MODE_ALTFN,
	GPIO_MODE_ANALOG,
	GPIO_MODE_IT_FT,
	GPIO_MODE_IT_RT,
	GPIO_MODE_IT_RFT
}GPIO_MODE;

/*
 * GPIO output possible types
 */
typedef enum{
	GPIO_OP_TYPE_PP=0,
	GPIO_OP_TYPE_OD
}GPIO_OP_MODE;

/*
 * GPIO speed mode possible modes
 * @GPIO_PIN_SPEED
 */
typedef enum
{
	GPIO_SPEED_LOW=0,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_FAST,
	GPIO_SPEED_HIGH
}GPIO_SPEED_MODE;

/*
 * GPIO pull up and pull down modes
 * @GPIO_PIN_PUPD
 */
typedef enum
{
	GPIO_NO_PUPD=0,
	GPIO_PIN_PU,
	GPIO_PIN_PD
}GPIO_PUPD_MODE;
/*********************************************************************************
 *                             API supported by this driver
**********************************************************************************/
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
#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
