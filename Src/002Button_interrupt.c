/*
 * 002Button_interrupt.c
 *
 *  Created on: Jan 10, 2024
 *      Author: ASUS
 */


#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define HIGH ENABLE
#define pressed HIGH
void delay()
{
	//it will introduce 200ms of delay when system clock id 16MHz
	for(uint32_t i=0;i<500000/2;i++);
}


int main()
{

    /*configuration for the LED to toggle*/
	GPIO_Handle_t GPIOx , GPIOBUTx;

	//clear or set 0 to members of handle variable
	memset(&GPIOx,0,sizeof(GPIOx));
	memset(&GPIOBUTx,0,sizeof(GPIOBUTx));

	GPIOx.pGPIOx = GPIOD;
	GPIOx.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOx.Gpio_PinConfig.GPIO_PinMode   = GPIO_MODE_OUT;
	GPIOx.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOx.Gpio_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
	GPIOx.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOx);

    /*configuration for the push button */
	GPIOBUTx.pGPIOx = GPIOA;
	GPIOBUTx.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBUTx.Gpio_PinConfig.GPIO_PinMode   = GPIO_MODE_IT_FT;
	GPIOBUTx.Gpio_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;
	GPIOBUTx.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOBUTx);


	//IRQ interrupt priority
    GPIO_IRQpPriorityConfig(IRQ_NO_EXTI0, NVIC_PRI_15);

	//IRQ interrupt configuration
    GPIO_IRQIntrruptConfig(IRQ_NO_EXTI0, ENABLE);


    while(1);

}


void EXTI0_IRQHandler(void)
{
	delay();
	//IRQ interrupt handling
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	//Toggle the led
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);

}
