/*
 * stm32f407xx.h
 *
 *  Created on: Dec 28, 2023
 *      Author: ASUS
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define _vo volatile
/*ARM cortex Mx NVIC ISERx register base addresses*/
#define NVIC_ISER0    ((_vo uint32_t*)0xE000E100 )
#define NVIC_ISER1    ((_vo uint32_t*)0xE000E104 )
#define NVIC_ISER2    ((_vo uint32_t*)0xE000E108 )
#define NVIC_ISER3    ((_vo uint32_t*)0xE000E10C )

/*ARM cortex Mx NVIC ICERx register base addresses*/
#define NVIC_ICER0    ((_vo uint32_t*)0XE000E180 )
#define NVIC_ICER1    ((_vo uint32_t*)0XE000E184 )
#define NVIC_ICER2    ((_vo uint32_t*)0XE000E188 )
#define NVIC_ICER3    ((_vo uint32_t*)0XE000E18C )



/*NVIC interrupt priority definition of 0 - 15
 */
typedef enum {
	NVIC_PRI_0 =0,
	NVIC_PRI_1,
	NVIC_PRI_2,
	NVIC_PRI_3,
	NVIC_PRI_4,
	NVIC_PRI_5,
	NVIC_PRI_6,
	NVIC_PRI_7,
	NVIC_PRI_8,
	NVIC_PRI_9,
	NVIC_PRI_10,
	NVIC_PRI_11,
	NVIC_PRI_12,
	NVIC_PRI_13,
	NVIC_PRI_14,
	NVIC_PRI_15
}NVIC_enum_num;


/*ARM CORTEX M4 priority register base address
 */

#define NVIC_PR_BASE_ADDR   ((_vo uint32_t*)0xE000E400)


/*
 * ARM cortex M4 number of priority bits implemented in priority register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/* base addresses of FLASH and SRAM*/
#define FLASH_BASEADDR    0x08000000U   //stm32f4 has 1Mb of flash
#define SRAM1_BASEADDR    0x20000000U   //112kbytes
#define SRAM2_BASEADDR    0x2001C000U   //16kbytes
#define ROM_BASEADDR      0x1FFF0000U   //mentioned as system memory in ref.manual
#define SRAM              SRAM1_BASEADDR

/*Base addresses of Bus registers*/

#define PERI_BASE        0x40000000U
#define APB1_BASEADDR    PERI_BASE
#define APB2_BASEADDR    0x40010000U
#define AHB1_BASEADDR    0x40020000U
#define AHB2_BASEADDR    0x50000000U


/*defining the base address of all GPIO'S from AHB1 bus*/

#define  GPIOA_BASEADDR  (AHB1_BASEADDR+0x0000)
#define  GPIOB_BASEADDR  (AHB1_BASEADDR+0x0400)
#define  GPIOC_BASEADDR  (AHB1_BASEADDR+0x0800)
#define  GPIOD_BASEADDR  (AHB1_BASEADDR+0x0C00)
#define  GPIOE_BASEADDR  (AHB1_BASEADDR+0x1000)
#define  GPIOF_BASEADDR  (AHB1_BASEADDR+0x1400)
#define  GPIOG_BASEADDR  (AHB1_BASEADDR+0x1800)
#define  GPIOH_BASEADDR  (AHB1_BASEADDR+0x1C00)
#define  GPIOI_BASEADDR  (AHB1_BASEADDR+0x2000)
//#define  GPIOJ_BASEADDR  (AHB1_BASEADDR+0x2400)   //didn't have a RCC->AHB1EN register
//#define  GPIOK_BASEADDR  (AHB1_BASEADDR+0x2800)   //didn't have a RCC->AHB1EN register
#define  RCC_BASEADDR    (AHB1_BASEADDR+0x3800)

/*defining the base address of peripherals hanging on APB1 bus*/
#define  I2C1_BASEADDR   (APB1_BASEADDR+0X5400)
#define  I2C2_BASEADDR   (APB1_BASEADDR+0X5800)
#define  I2C3_BASEADDR   (APB1_BASEADDR+0X5C00)

#define  SPI2_BASEADDR   (APB1_BASEADDR+0X3800)
#define  SPI3_BASEADDR   (APB1_BASEADDR+0X3C00)

#define  USART2_BASEADDR (APB1_BASEADDR+0X4400)
#define  USART3_BASEADDR (APB1_BASEADDR+0X4800)

#define  UART4_BASEADDR  (APB1_BASEADDR+0X4C00)
#define  UART5_BASEADDR  (APB1_BASEADDR+0X5000)
//#define  UART7_BASEADDR  (APB1_BASEADDR+0x7800)  //didn't have a RCC->APB1EN register
//#define  UART8_BASEADDR  (APB1_BASEADDR+0X7C00)  //didn't have a RCC->APB1EN register

/*defining the base address of peripherals hanging on the APB2 bus */
#define EXTI_BASEADDR    (APB2_BASEADDR+0X3C00)
#define SPI1_BASEADDR    (APB2_BASEADDR+0X3000)
#define SYSCFG_BASEADDR  (APB2_BASEADDR+0X3800)
#define USART1_BASEADDR  (APB2_BASEADDR+0X1000)
#define USART6_BASEADDR  (APB2_BASEADDR+0X1400)

/* Defining a register for the GPIO peripheral
 * Note: Register of the peripheral will depended on the MCU
 */

typedef struct
{
	_vo uint32_t MODER;
	_vo uint32_t OTYPER;
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFR[2];

}GPIO_RegDef_t;

/* Defining a registers for the RCC*/


typedef struct
{
	_vo uint32_t CR;
	_vo uint32_t PLLCFGR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t AHB1RSTR;
	_vo uint32_t AHB2RSTR;
	_vo uint32_t AHB3RSTR;
	uint32_t RES1;
	_vo uint32_t APB1RSTR;
	_vo uint32_t APB2RSTR;
	uint32_t RES2[2];
	_vo uint32_t AHB1ENR;
	_vo uint32_t AHB2ENR;
	_vo uint32_t AHB3ENR;
	uint32_t RES4;
	_vo uint32_t APB1ENR;
	_vo uint32_t APB2ENR;
	uint32_t RES5[2];
	_vo uint32_t AHB1LPENR;
	_vo uint32_t AHB2LPENR;
	_vo uint32_t AHB3LPENR;
	uint32_t RES7;
	_vo uint32_t APB1LPENR;
	_vo uint32_t APB2LPENR;
	uint32_t RES8[2];
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
	uint32_t RES10[2];
	_vo uint32_t SSCGR;
	_vo uint32_t PLLI2SCFGR;
	_vo uint32_t PLLSAICFGR;
	_vo uint32_t DCKCFGR;
	_vo uint32_t CKGATENR;
	_vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

/*Register definition for the SYSCFG register*/
typedef struct
{
	_vo uint32_t MEMRMP; // SYSCFG memory remap register       : offset address 0x00
	_vo uint32_t PMC;    // peripheral mode configuration      : offset address 0x04
	_vo uint32_t EXTICR[4];// External interrupt configuration : offset address 0x08-0x14
  	uint32_t RESERVED1[2]; // reserved for other uses           : offset address 0x18-0x1c
	_vo uint32_t CMPCR;//compensation cell control register    : offset address 0x20
    uint32_t  RESERVED2[2];// reserved for the other purpose    : offset address 0x24-0x28
    _vo uint32_t CFGR;//                                       : offset address 0x2c
}SYSCFG_RegDef_t;

/* Register definition used to configure the interrupt in pin */

typedef struct
{
	_vo uint32_t IMR;/*Interrupt mask register            : offset address 0x00*/
	_vo uint32_t EMR;/*event mask register                : offset address 0x04*/
	_vo uint32_t RTSR;/*Rising trigger selection register : offset address 0x08*/
	_vo uint32_t FTSR;/*Falling trigger selection register: offset address 0x0c*/
	_vo uint32_t SWIER;/*software interrupt event register: offset address 0x10*/
	_vo uint32_t PR;/*pending register                    : offset address 0x14*/
}EXTI_RegDef_t;

/* peripheral definition (type casting to the structure of XXX_RegDeg_t)*/

#define GPIOA   ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB   ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC   ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD   ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE   ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF   ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG   ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH   ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI   ((GPIO_RegDef_t*)GPIOI_BASEADDR)
//#define GPIOJ   ((GPIO_RegDef_t*)GPIOJ_BASEADDR)
//#define GPIOK   ((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC     ((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI     ((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG   ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
/*
 * Macro to enable GPIOx peripheral clock
 */
#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()    (RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()    (RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()    (RCC->AHB1ENR |=(1<<8))

/*
 * Macro to enable I2Cx peripheral clock
 */
#define I2C1_PCLK_EN()     (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()     (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()     (RCC->APB1ENR |= (1<<23))

/*
 * Macros to enable SPIx peripheral clock
 */
#define  SPI1_PCLK_EN()    (RCC->APB2ENR |= (1<<12))
#define  SPI2_PCLK_EN()    (RCC->APB1ENR |= (1<<14))
#define  SPI3_PCLK_EN()    (RCC->APB1ENR |= (1<<15))

/*
 * Macros to enable USARTx peripheral clock
 */
#define  USART2_PCLK_EN()  (RCC->APB1ENR  |= (1<<17))
#define  USART3_PCLK_EN()  (RCC->APB1ENR  |= (1<<18))
#define  UART4_PCLK_EN()   (RCC->APB1ENR  |= (1<<19))
#define  UART5_PCLK_EN()   (RCC->APB1ENR  |= (1<<20))
#define  USART1_PCLK_EN()  (RCC->APB2ENR  |= (1<<4))
#define  USART6_PCLK_EN()  (RCC->APB2ENR  |= (1<<5))
/*
 * Macros to enable SYSCFG peripheral clock
 */
#define  SYSCFG_PCLK_EN()  (RCC->APB2ENR  |= (1<<14))
/*
 * Macro to Disable GPIOx peripheral clock
 */
#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI()    (RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI()    (RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &=~(1<<7))
#define GPIOI_PCLK_DI()    (RCC->AHB1ENR &=~(1<<8))
/*
 * Macro to Disable SPIx peripheral clock
 */
#define  SPI1_PCLK_DI()    (RCC->APB2ENR &= ~(1<<12))
#define  SPI2_PCLK_DI()    (RCC->APB1ENR &= ~(1<<14))
#define  SPI3_PCLK_DI()    (RCC->APB1ENR &= ~(1<<15))
/*
 * Macro to Disable I2Cx peripheral clock
 */
#define I2C1_PCLK_DI()     (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()     (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()     (RCC->APB1ENR &= ~(1<<23))
/*
 * Macro to Disable SYSCFG peripheral clock
 */
#define  SYSCFG_PCLK_DI()  (RCC->APB2ENR &= ~(1<<14))

/*
 * Macro to De-initialize the GPIO peripheral
 */
#define  GPIOA_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define  GPIOB_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<1));(RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define  GPIOC_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<2));(RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define  GPIOD_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<3));(RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define  GPIOE_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<4));(RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define  GPIOF_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<5));(RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define  GPIOG_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<6));(RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define  GPIOH_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<7));(RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define  GPIOI_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<8));(RCC->AHB1RSTR &= ~(1<<8));}while(0)
/*
 * some generic macros
 */
#define ENABLE  1
#define DISABLE 0
#define SET    ENABLE
#define RESET  DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

/*
 * IRQ(interrupt request) Number of stm32F407x MCU
 * NOTE: the IRQ numbers will be change with the MCU
 * verify with the MCU specific vector table
 */
#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4     10
#define IRQ_NO_EXTI15_10 40
#define IRQ_NO_EXTI9_5   23
//return the port code for given GPIOx base address
#define GPIO_BASEADDR_TO_CODE(x)     ((x == GPIOA)? 0:\
                                      (x == GPIOB)? 1:\
                                      (x == GPIOC)? 2:\
                                      (x == GPIOD)? 3:\
                                      (x == GPIOE)? 4:\
                                      (x == GPIOF)? 5:\
                                      (x == GPIOG)? 6:\
                                      (x == GPIOH)? 7:\
                                      (x == GPIOI)? 8:0)

#include "stm32f407xx_gpio_driver.h"
#endif /* INC_STM32F407XX_H_ */
