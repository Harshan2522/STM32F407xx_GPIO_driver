/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 29, 2023
 *      Author: ASUS
 */
#include "stm32f407xx_gpio_driver.h"

 /*
  * Peripheral clock setup
  */
/*************************************************
 * @fn        - GPIO_PeriClockControl
 * @brief     - This function enables or disables peripheral clock for the given GPIO port
 * @param[in] - Base address of the GPIO peripheral
 * @param[in] - Enable or Disable macros
 * @return    - None
 * @Note      - None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
        {
		GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
		GPIOC_PCLK_EN();
	    }else if(pGPIOx == GPIOD)
	    {
		GPIOD_PCLK_EN();
	    }else if(pGPIOx == GPIOE)
	    {
	    GPIOE_PCLK_EN();
	    }else if(pGPIOx == GPIOF)
	    {
		GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
		GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
		GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
		GPIOI_PCLK_EN();
		}
	    } else {
		       if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if(pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}else if(pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}else if(pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}
				else if(pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}
}

/*
 * Init and De-init
 */
/*************************************************
 * @fn        - GPIO_Init
 * @brief     - This function Initialize the selected GPIO port
 * @param[in] - Base address of the GPIO peripheral
 * @return    - None
 * @Note      - None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
   uint32_t temp=0;
   //1.configure the mode of gpio pin
   if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG)
   {
	   //the non interrupt mode
	   temp=((pGPIOHandle->Gpio_PinConfig.GPIO_PinMode)<<(2*pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
	   pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	   pGPIOHandle->pGPIOx->MODER |= temp;
   }
   else
   {
	   //the interrupt mode definition
	   if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
	   {
		  //1.Configure the interrupt falling edge or FTSR
		   EXTI->FTSR |= (1<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
		  //clear the RTSR bit to prevent any mis funtion
		   EXTI->RTSR &= ~(1<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	   }
	   else if(pGPIOHandle ->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
	   {
		   //1.Configure the interrupt falling edge or RTSR
		   EXTI->RTSR |= (1<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
		   //clear the RTSR bit to prevent any mis funtion
		   EXTI->FTSR &= ~(1<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	   }
	   else if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
	   {
		   //1.Configure the interrupt falling edge for RTSR
		   EXTI->RTSR |= (1<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
		   //2.Configure the interrupt falling edge for RTSR
		   EXTI->FTSR |=  (1<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
	   }
	   // 2.configure the GPIO port selection in SYSCFG_EXTICR
	     uint8_t temp1 = pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber /4;
	     uint8_t temp2 = pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber% 4;
	     uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	     SYSCFG_PCLK_EN();
         SYSCFG->EXTICR[temp1] |= portcode << (temp2*4);

	   //3. enable the exti interrupt delivery using IMR
	   EXTI->IMR |= (1<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);

   }
   temp=0;

   //2.configure the GPIO speed register
   temp=((pGPIOHandle->Gpio_PinConfig.GPIO_PinSpeed)<<(2*pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
   pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
   	   pGPIOHandle->pGPIOx->OSPEEDR |= temp;

   temp=0;
   //3. Configure the pupd settings
   temp=((pGPIOHandle->Gpio_PinConfig.GPIO_PinPuPdControl)<<(2*pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
   pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
      	   pGPIOHandle->pGPIOx->PUPDR |= temp;

   temp=0;

   //4. Configure the output type
   temp=((pGPIOHandle->Gpio_PinConfig.GPIO_PinOPType)<<(pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));
   pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
      	   pGPIOHandle->pGPIOx->OTYPER |= temp;

   temp=0;
   //5. Configure the alt functions
   if(pGPIOHandle->Gpio_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
      {
	   uint8_t temp1,temp2=0;
	   temp1=pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber / 8;                                             // To find which register the pin number is belongs
	   temp2=pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber % 8;                                             // TO calculate the bit position to start store the value
	   pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
	   pGPIOHandle->pGPIOx->AFR[temp1] = (pGPIOHandle->Gpio_PinConfig.GPIO_pinAltFunMode << (4*temp2));  //mul by 4 to determine the bit position on the concerned AFR register

      }

}
/*************************************************
 * @fn        - GPIO_DeInit
 * @brief     - This function DeInitialize the selected GPIO port
 * @param[in] - Base address of the GPIO peripheral
 * @return     - None
 * @Note      - None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx )
{
	        if(pGPIOx == GPIOA)
			{
	        	GPIOA_REG_RESET();
			}else if(pGPIOx == GPIOB)
	        {
				GPIOB_REG_RESET();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
		    }else if(pGPIOx == GPIOD)
		    {
		    	GPIOD_REG_RESET();
		    }else if(pGPIOx == GPIOE)
		    {
		    	GPIOE_REG_RESET();
		    }else if(pGPIOx == GPIOF)
		    {
		    	GPIOF_REG_RESET();
			}else if(pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}else if(pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}

/*
 * Read,Write & toggling functions
 */
/*************************************************
 * @fn        - GPIO_ReadFromInputPin
 * @brief     - This function Reads data form selected GPIO port of given GPIO pin
 * @param[in] - Base address of the GPIO peripheral
 * @param[in] - Pin number of the selected GPIO
 * @return    - 0 or 1
 * @Note      - None
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value=0;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber)& 0x00000001);
	return value;

}

/*************************************************
 * @fn        - GPIO_ReadFromInputPort
 * @brief     - This function Reads data form selected GPIO port
 * @param[in] - Base address of the GPIO peripheral
 * @return    - port value
 * @Note      - None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value=0;
	value = (uint16_t)pGPIOx->IDR ;
	return value;
}

/*************************************************
 * @fn        - GPIO_WriteToOutputPin
 * @brief     - This function Writes data form selected GPIO port of GPIO pin with the user given value
 * @param[in] - Base address of the GPIO peripheral
 * @param[in] - Pin number of the selected GPIO
 * @param[in] - 8 bit value
 * @return    - None
 * @Note      - None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		//write 1 to the pin
		pGPIOx->ODR |=(1<<PinNumber);
	}
	else
	{
		//write 0 to the pin
		pGPIOx->ODR &=~(1<<PinNumber);
	}

}

/*************************************************
 * @fn        - GPIO_WriteToOutputPort
 * @brief     - This function Writes data form selected GPIO port with the given value
 * @param[in] - Base address of the GPIO peripheral
 * @param[in] - 16 bit value
 * @return    - None
 * @Note      - None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
    pGPIOx->ODR = value;
}

/*************************************************
 * @fn        - GPIO_ToggleOutputPin
 * @brief     - This function toggles the state of selected GPIO port of GPIO pin
 * @param[in] - Base address of the GPIO peripheral
 * @param[in] - Pin number of the selected GPIO
 * @return    - None
 * @Note      - None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
     pGPIOx->ODR^=(1<<PinNumber);
}


/*
 * IRQ related functions
 */

/*************************************************
 * @fn        - GPIO_IRQConfig
 * @brief     - This function configures the enabling or disable for IRQ interrupt
 * @param[in] - Number of the IRQ
 * @param[in] - Priority of the selected IRQ
 * @param[in] - Set the value Enable or Disable
 * @return    - None
 * @Note      - None
 */
void GPIO_IRQIntrruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
//Enable / disable and set priority the corresponding NVIC register
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <=31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1<<(IRQNumber % 32));
		}
		else if(IRQNumber >63 && IRQNumber <96)
		{
			//program ISER3 register
			*NVIC_ISER2 |= (1<<(IRQNumber % 64));
		}
		}
    else
    {
    	if(IRQNumber <=31)
    			{
    				//Program ICER0 register:0-31
    				*NVIC_ICER0 |= (1<<IRQNumber);
    			}
    			else if(IRQNumber >31 && IRQNumber <64)
    			{
    				//program ICER1 register: 32-63
    				*NVIC_ICER1 |= (1<<(IRQNumber % 32));
    			}
    			else if(IRQNumber >63 && IRQNumber <96)
    			{
    				//program ICER3 register: 64-95
    				*NVIC_ICER2 |= (1<<(IRQNumber % 64));
    			}
    			}
}

/****************************************************************
 * @fn        - GPIO_IRQpPriorityConfig
 * @brief     - set the priority of the given IRQ number with given priority
 * @param     - IRQNumber and IRQPriority
 * @return    - NONE
 * @Note      - NONE
 */
void GPIO_IRQpPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
   uint8_t iprx = IRQNumber /4;// to find the offset address
   uint8_t iprx_section = IRQNumber %4; // to find the bit field to set the priority
   uint8_t shift_amount = (8*iprx_section) +(8-NO_PR_BITS_IMPLEMENTED);// the lower 4 bit are reserved so shift to higher 4 bit
   *(NVIC_PR_BASE_ADDR + (iprx) ) |=(IRQPriority<< shift_amount);
}

/*************************************************
 * @fn        - GPIO_IRQHandling
 * @brief     - This function handles the selected IRQ interrupt
 * @param[in] - Number of the pin that enabled with IRQ
 * @return    - None
 * @Note      - This function will be called in the main.c and
 *              ISR function name will be found at startup code
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
  if(EXTI->PR & (1<<PinNumber))
  {
	  //clear the corresponding pin number by setting the pin
	  EXTI->PR |=(1<<PinNumber);
  }


}
