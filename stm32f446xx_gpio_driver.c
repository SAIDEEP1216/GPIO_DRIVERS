/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Aug 8, 2024
 *      Author: saideep
 *      Contains Function Declarations
 */
#include "stm32f446xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	  if (EnorDi == ENABLE) {
	        if (pGPIOx == GPIOA) {
	            GPIOA_PCLK_EN(); //Macros
	        } else if (pGPIOx == GPIOB) {
	            GPIOB_PCLK_EN();
	        } else if (pGPIOx == GPIOC) {
	            GPIOC_PCLK_EN();
	        } else if (pGPIOx == GPIOD) {
	            GPIOD_PCLK_EN();
	        } else if (pGPIOx == GPIOE) {
	            GPIOE_PCLK_EN();
	        } else if (pGPIOx == GPIOF) {
	            GPIOF_PCLK_EN();
	        } else if (pGPIOx == GPIOG) {
	            GPIOG_PCLK_EN();
	        } else if (pGPIOx == GPIOH) {
	            GPIOH_PCLK_EN();
	        }
	    } else {
	        if (pGPIOx == GPIOA) {
	            GPIOA_PCLK_DI();
	        } else if (pGPIOx == GPIOB) {
	            GPIOB_PCLK_DI();
	        } else if (pGPIOx == GPIOC) {
	            GPIOC_PCLK_DI();
	        } else if (pGPIOx == GPIOD) {
	            GPIOD_PCLK_DI();
	        } else if (pGPIOx == GPIOE) {
	            GPIOE_PCLK_DI();
	        } else if (pGPIOx == GPIOF) {
	            GPIOF_PCLK_DI();
	        } else if (pGPIOx == GPIOG) {
	            GPIOG_PCLK_DI();
	        } else if (pGPIOx == GPIOH) {
	            GPIOH_PCLK_DI();
	        }
	    }


}

/*
 * Initialization Handler
 */
/*********************************************************************
 * @fn      		  - Initialization Handler
 *
 * @brief             - This function Initializes Configurations for the given GPIO port
 *                        Mode,PinNumber, Output Type, Speed, Alternate Functionality, Interrupt Functionality
 *
 * @param[in]         - GPIO Handle Structure
 * @param[in]         - GPIO_Handle_t *pGPIOHandle
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
//1.Configuring the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANLG){

		//Non Interrupt Mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing Register
		pGPIOHandle->pGPIOx->MODER |= temp; // Setting Register

	}
	//Interrupt Mode Initialization
	else{
		//Set Falling Trigger Register FTSR
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1.Configure Falling Trigger Service Routine (FTSR)
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Reset RTSR - Need to be Reset after Set
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1.Configure Rising Trigger Service Routine (RTSR)
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Reset FTSR
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_RFT){
			//1.Configure Falling and Rising Trigger Service Routine (FTSR and RSTR)
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}


//2. Configure the GPIO port Section in SYSCFG_EXTICR. This is for Enabling EXTI Lines with Respective GPIO Pins
			uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;
			uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG->SYSCFG_EXTICR[temp1] = portcode<<(4*temp2);




//3. Enable the EXTI interrupt  delivery using IMR
			EXTI->EXTI_IMR |=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;

	//2.Configuring the Speed of GPIO pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing Register
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;


    //3.Configuration of GPIO pin PUPD settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPDControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing Register
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;

    //4.Configuration GPIO pin Output Type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER&=~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing Register
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;

    //5.Configuration GPIO pin Alternate Functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANLG){
    	uint8_t temp1, temp2;
    	temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8; //This is to locate AFRL or AFRH gives quotient
    	temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8; //This is to locate Pin as this has 4bits example: AF1= 0000
    pGPIOHandle->pGPIOx->AFR[temp1]&=~(0xF<<temp2); // Clearing Register
    pGPIOHandle->pGPIOx->AFR[temp1]|= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));

    }

    else{
    	temp = 0;
    }



}
/*
 * D-Initialization Handler
 */
/*********************************************************************
 * @fn      		  - Initialization Handler
 *
 * @brief             - This function D-Initializes Configurations for the given GPIO port
 *                        Mode,PinNumber, Output Type, Speed, Alternate Functionality
 *
 * @param[in]         - GPIO Handle Structure
 * @param[in]         - GPIO_Handle_t *pGPIOHandle
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){ // D-initialization  can be done simply by setting RCC AHB1 Reset of Particular GPIO


		        if (pGPIOx == GPIOA) {
		            GPIOA_REG_RESET();
		        } else if (pGPIOx == GPIOB) {
		            GPIOB_REG_RESET();
		        } else if (pGPIOx == GPIOC) {
		            GPIOC_REG_RESET();
		        } else if (pGPIOx == GPIOD) {
		            GPIOD_REG_RESET();
		        } else if (pGPIOx == GPIOE) {
		            GPIOE_REG_RESET();
		        } else if (pGPIOx == GPIOF) {
		            GPIOF_REG_RESET();
		        } else if (pGPIOx == GPIOG) {
		            GPIOG_REG_RESET();
		        } else if (pGPIOx == GPIOH) {
		            GPIOH_REG_RESET();
		        }
}
/*
 * Read Data From Input Pins
 */
/*********************************************************************
 * @fn      		  - Read Data From Input Pins
 *
 * @brief             - This function  is used to Read data from GPIO Pins, Uses GPIO IDR
 *
 * @param[in]         - GPIO Register Definition Structure
 * @param[in]         - GPIO_RegDef_t
 * @param[in]         -
 *
 * @return            -  Value  of the Pin 0/1
 *
 * @Note              -  none

 */

uint8_t GPIO_ReadFrmInpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR>>PinNumber) & 0x00000001); //Right shifted and &operation
	return value;

}


/*
 * Read Data From Input Port
 */
/*********************************************************************
 * @fn      		  - Read Data From Input Pins
 *
 * @brief             - This function  is used to Read data from GPIO Port, Uses GPIO IDR
 *
 * @param[in]         - GPIO Register Definition Structure
 * @param[in]         - GPIO_RegDef_t
 * @param[in]         -
 *
 * @return            -  Value  of the Port 16bit data
 *
 * @Note              -  none

 */


uint16_t GPIO_ReadFrmInpPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}

void GPIO_WrtToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value){
	if(Value == GPIO_PIN_SET)
	  pGPIOx->ODR|=(1<<PinNumber);
	else
	  pGPIOx->ODR&=~(1<<PinNumber);
}

void GPIO_WrtToOutPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){

		  pGPIOx->ODR = Value;
}

void GPIO_ToggleOutPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR^=(1<<PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 *
 */

/* GPIO_IRQInterruptCnfg
 *
 */
/*********************************************************************
 * @fn      		  -  Configure Interrupt Set Enable Register(ISER) and Interrupt Clear Enable Register (ICER)
 *                        of ARM CORTEX M4 NVIC Registers
 *
 * @brief             - ISER0 - ISER7 Total 7-registers each 32bits
 *                      ISER0 - 0-31 IRQ's and ISER1 for 32 - 63 IRQ's
 *
 * @param[in]         - Interrupt Request Number and Enable/Disable
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_IRQInterruptCnfg(uint8_t IRQNumber, uint8_t EnorDi){
 if(EnorDi == ENABLE){
	if(IRQNumber <= 31){
		//Program ISER0 Register
		 *NVIC_ISER0 |= (1<<IRQNumber);
	}
	else if(IRQNumber>31 && IRQNumber <64){
		//Program ISER1 Register
		*NVIC_ISER1 |= (1<<(IRQNumber % 32));
	}
	else if(IRQNumber>=64 && IRQNumber < 96){
		//Program ISER2 Register
		*NVIC_ISER1 |= (1<<(IRQNumber % 64));

	}


}else{
	if(IRQNumber <= 31){
		//Program ISER0 Register
		 *NVIC_ICER0 |= (1<<IRQNumber);
	}
	else if(IRQNumber>31 && IRQNumber <64){
		//Program ISER1 Register
		*NVIC_ICER1 |= (1<<(IRQNumber % 32));
	}
	else if(IRQNumber>=64 && IRQNumber < 96){
		//Program ISER2 Register
		*NVIC_ICER1 |= (1<<(IRQNumber % 64));

	}
}

}


/* GPIO_ GPIO_IRQPriorityCnfg
 *
 */
/*********************************************************************
 * @fn      		  -  Configure  Interrupt Priority Registers IPR of ARM CORTEX M4 NVIC Registers
 *
 * @brief             - IPR0 - IPR59 Total 60 Registers of 32 bit each and can represent  4 Priorities |PRI3|PRI2|PRI1|PR10|
 * 						example to set a Priority 1 to IRQ 20 => 20/4 =>5 IPR5 and 20%4 gives IPR section which is 0
 * 						So, IPR5 Section 0 and Value will be 0x00
 *
 * @param[in]         - Interrupt Request Number and Priority Value
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_IRQPriorityCnfg(uint8_t IRQNumber, uint8_t IRQPriority)
{

uint8_t iprx = IRQNumber/4;
uint8_t iprx_section = IRQNumber%4;

uint8_t shift_amount = (iprx_section*8) + (8 - NO_PR_BITS_IMPLEMENTED); //8bits so Multiply by 8

*(NVIC_PR_BASE_ADDR + (iprx*4)) |=  (IRQPriority << shift_amount);

}

// Every Time an Interrupt Occurs a bit is Set in NVIC Pending Register we need to clear this bit as Handling
void GPIO_IRQHandling(uint8_t PinNumber){

if(EXTI->EXTI_PR & (1<< PinNumber)){

	//Clear Pending Register Bit
	EXTI->EXTI_PR |= (1<< PinNumber);

}
}
