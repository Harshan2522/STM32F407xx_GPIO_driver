/*
 * 001LED_toggle.c
 *
 *  Created on: Jan 2, 2024
 *      Author: ASUS
 */


#include "stm32f407xx.h"

#define HIGH ENABLE
#define pressed HIGH
void delay(int data)
{
	for(int i=0;i<data;i++);
}

int main()
{
	GPIO_Handle_t GPIOx;
	GPIOx.pGPIOx = GPIOA;
	GPIOx.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOx.Gpio_PinConfig.GPIO_PinMode   = GPIO_MODE_OUT;
	GPIOx.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOx.Gpio_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
	GPIOx.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOx);

/*configuration for tht inbuild board push button */
	GPIO_Handle_t GPIOp;
	GPIOp.pGPIOx = GPIOD;
	GPIOp.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOp.Gpio_PinConfig.GPIO_PinMode   = GPIO_MODE_IN;
	GPIOp.Gpio_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
	GPIOp.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		GPIO_PeriClockControl(GPIOA, ENABLE);

		GPIO_Init(&GPIOp);
     //volatile uint8_t value;
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)== pressed){
		delay(500000);
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
		//
		//GPIO_WriteToOutputPin(GPIOx, GPIO_PIN_NO_12, value);

//		GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, 0);
	}
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);//give the GPIO pin number which is configred as a interrupt
}
