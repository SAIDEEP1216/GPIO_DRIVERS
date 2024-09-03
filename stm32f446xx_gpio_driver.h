/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Aug 8, 2024
 *      Author: saideep
 *      GPIO Driver Header File
 *      Contains Configuration Macros.
 *      Contains GPIO Configuration and Handling Structure.
 *      Provides API's and Handler for Driver.c.
 *
 */
#include "stm32f446xx.h" //Device Specific Header File

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_


//GPIO Pin Configuration structure for GPIO Pin
typedef struct{
	uint8_t  GPIO_PinNumber;
	uint8_t  GPIO_PinMode;          /*!< Possible values from @GPIO_PIN_MODES >*/
	uint8_t  GPIO_PinSpeed;         /*!< Possible values from @GPIO_SPEED >*/
	uint8_t  GPIO_PinPuPDControl;
	uint8_t  GPIO_PinOPType;
	uint8_t  GPIO_PinAltFunMode;


} GPIO_PinCongig_t;


/*
 * GPIO Handling Structure pointer to base address.
 */
typedef struct{
	GPIO_RegDef_t  *pGPIOx;          // *pGPIOX = GPIOA=>((GPIO_RegDef_t*)GPIOA_BASEADDR)) returns Base Address of GPIOA or whatever is assigned;
	GPIO_PinCongig_t  GPIO_PinConfig;  //  This will

}GPIO_Handle_t;


/* @GPIO_PIN_Numbers
 * GPIO PinNumber macro
 */

#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15





/* @GPIO_PIN_MODES
 * GPIO Configuration Mode Macros
 */

/* GPIO Possible Modes*/

#define GPIO_MODE_IN     		0                //Input Mode
#define GPIO_MODE_OUT   		1				 //Output Mode
#define GPIO_MODE_ALTFN   	    2                //Alternate Function Mode
#define GPIO_MODE_ANLG    		3                //Analog Function Mode
#define GPIO_MODE_IT_FT         4                //Interrupt mode with Falling Edge Trigger
#define GPIO_MODE_IT_RT         5                //Interrupt mode with Rising  Edge Trigger
#define GPIO_MODE_RFT       	6				 //Interrupt mode with Falling  and Rising Edge Trigger



/*
 * GPIO Configuration OutputType Macros
 */
#define GPIO_OP_TYPE_PP    		0      /*PushPull configuration*/
#define GPIO_OP_TYPE_OD  		1      /* Open Drain output, Pull up Mode need to be used*/


/* @GPIO_SPEED
 * GPIO Configuration Output Speeds
 */
#define GPIO_SPEED_LOW   		    0      /*SPEED LOW*/
#define GPIO_SPEED_MEDIUM  			1
#define GPIO_SPEED_FAST   		    2      /*SPEED FAST*/
#define GPIO_SPEED_HIGH   			3


/*
 * GPIO PIN  PullUp and PullDown configuration macros
 */
#define GPIO_NO_PUPD 		        0      /*NO PUPD*/
#define GPIO_PIN_PU				    1
#define GPIO_PIN_PD  		   	    2





/**********************************************************************************************************
 Driver API definitions/ Prototypes
 Initialize GPIO port
 *********************************************************************************************************/

/*Peripheral Clock Setup */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);



/*  Initialization  and D-Init */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // D-initialization  can be done simply by setting RCC AHB1 Reset of Particular GPIO


uint8_t GPIO_ReadFrmInpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFrmInpPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WrtToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
void GPIO_WrtToOutPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptCnfg(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityCnfg(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
