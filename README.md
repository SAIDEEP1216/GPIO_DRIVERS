This GPIO  Bare-metal drivers  offer high-level APIs for GPIO configuration and handling, which abstract away the direct interaction with hardware registers. These drivers simplify development by providing a standardized interface, but at the cost of increased overhead. This is  generally suitable for non-real-time applications where performance and precision are not critical concerns. In contrast, low-level drivers provide minimal abstraction, allowing direct access to hardware registers for more precise control. which are ideal for real-time applications where low latency and fine-grained timing are essential.
***********************************************************************

General Overview:
![WhatsApp Image 2024-09-08 at 1 49 56 AM (1)](https://github.com/user-attachments/assets/7db64caa-e52a-4d16-a7a7-5b84ad54b12b)

**********************************************************************
## Project file structure

**Drivers** : The Driver Layer consisits of  **Device Specific Header** , **Driver Header**  and  **Driver Source** files.

- **Device Specific Header** :  **stm32f446xx.h** This file contains Base Address Macros of FLASH and SRAM memories and SFR's, processor Specific Register Addresses.
  
- **Driver Header** : **stm32f446xx-gpio.h** Contains Configuration Macros, GPIO Configuration and Handling Structure, provides API's and Handler for Driver.c.

- **Driver Header** : **stm32f446xx-gpio.c** Contains all the driver API's the Application file uses.

# Driver API's Usage

All the examples and code have been tested on [STM32F446RE NUCLEO Board]([https://www.st.com/en/evaluation-tools/stm32f4discovery.html](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html)

# Possible Modes of GPIO
GPIO's can be configured as Input, Output, ALternate Functionality based on their Application.
  #### Non Interrupt Modes
  - Input Mode
  - Output Mode
  - Analog Mode
  - Alternate Function Mode (for SPI, I2C, USART etc).
    
  #### Interrupt Modes
  - Interrupt Mode with Falling Edge Trigger
  - Interrupt Mode with Rising Edge Trigger
  - Interrupt Mode with Falling and Rising Edge Triggers.

## Output Modes
1) Push-Pull Configuration
2) Open Drain with Pull-UP Configuration
   
![WhatsApp Image 2024-09-08 at 1 49 54 AM](https://github.com/user-attachments/assets/86724d9c-6223-4244-b806-44da40caac77)

## Required Pin Configurations for the mode configurations for Applications. 
 - PinNumber
 - PinMode, PinSpeed
 - PinPuPDControl(Pullup or PullDown)
 - PinOPType(Pin Output Type)
 - PinAltFunMode(Alternate Function Mode)


# Driver Structures

These are the 2 major Important Structures used for Pin Configuration and Handling in stm32f446xx-gpio.h file.
```
typedef struct{
	uint8_t  GPIO_PinNumber;
	uint8_t  GPIO_PinMode;          /*!< Possible values from @GPIO_PIN_MODES >*/
	uint8_t  GPIO_PinSpeed;         /*!< Possible values from @GPIO_SPEED >*/
	uint8_t  GPIO_PinPuPDControl;
	uint8_t  GPIO_PinOPType;
	uint8_t  GPIO_PinAltFunMode;

} GPIO_PinCongig_t;


 // GPIO Handling Structure pointer to base address.
 
typedef struct{
	GPIO_RegDef_t  *pGPIOx;          // *pGPIOX = GPIOA=>((GPIO_RegDef_t*)GPIOA_BASEADDR)) returns Base Address of GPIOA or whatever is assigned;
	GPIO_PinCongig_t  GPIO_PinConfig;  

}GPIO_Handle_t;


```


# Driver API's

## GPIO_PeriClkCtrl()- **Peripheral Clock Control API**

Used to Enable and Disable Clock for the GPIO Peripherals 
_Parameters_

- `GPIO_RegDef_t *pGPIOx `: Base address of Gpio Port . base address for Ports are defined in **Stm32f446xx.h** header file as **`GPIOA`** ,**`GPIOB`** ,**`GPIOC`** .... ,**`GPIOH`**

- `uint8_t EnorDi`: Enable or Disable . macros used are: **`ENABLE`** , **`DISABLE`** .

## GPIO_Init() - **GPIO Peripheral Initialization API**

Initializes a GPIO port's pin as per the configuration given to `Gpio_Handle_t` . 

_Parameters_

- `GPIO_Handle_t* pGPIOHandle ` : Pointer to `Gpio_Handle_t` configuration structure . `Gpio_Handle_t` has member elements which serves as configuration parameters to` GPIO_Init` function
- `GPIO_RegDef_t * pGPIOx` Set this with base addr of the port who's pin is to be configured
- `GPIO_PinConfig_t|GPIO_PinConfig` This stores the configuration which gets applied to the pin which is to be configured
  
**GPIO_Handle_t member elements**
  
 - uint8_t  GPIO_PinNumber
 - uint8_t  GPIO_PinMode, PinSpeed
 - uint8_t  GPIO_PinPuPDControl(Pullup or PullDown)
 - uint8_t  GPIO_PinOPType(Pin Output Type)
 - uint8_t  GPIO_PinAltFunMode(Alternate Function Mode)


## GPIO_ReadFrmInputPin() - **API for Reading Input Pins**

Return type : `uint8_t`

Return the value of a gpio pin of a certain PORT (A,BC,....H) which is either 0 or 1 . when a pin configured in input mode and depending on where the pin is connected either to GND or VDD the function returns 0 or 1 .


_Parameters_

- `GPIO_RegDef_t *pGPIOx ` : Base address of Gpio Port . base address for Ports are defined in **Stm32f407xx.h** header file as **`GPIOA`** ,**`GPIOB`** ,**`GPIOC`** .... ,**`GPIOH`**

- `uint8_t PinNumber` : Pin number to read data from . for most stm32 micros each port has 16 pins from 0 to 15 .PinNumber definition macros are defined in **Stm32f407xx.h** header file as \*PinNumber**\*x** ( **x** : 0 ,1 ,2 ,3 .... 15 ).


## GPIO_ReadFrmInputPort() - **API for Reading Input Port**

Return type : `uint8_t`

Return the value of a gpio pin of a certain PORT (A,BC,....K) which is either 0 or 1 . when a pin configured in input mode and depending on where the pin is connected either to GND or VDD the function returns 0 or 1 .


_Parameters_

- `GPIO_RegDef_t *pGPIOx ` : Base address of Gpio Port . base address for Ports are defined in **Stm32f407xx.h** header file as **`GPIOA`** ,**`GPIOB`** ,**`GPIOC`** .... ,**`GPIOH`**




## GPIO_WriteToOutputPin() - **API for Write Output Pin**

Return type : `Void`

Writes the Data value to a gpio pin of a certain PORT (A,BC,....H) which is either 0 or 1 . when a pin configured in Output mode and if the `PinOPType`is set to ` GPIO_OP_TYPE_PP` then a pin can have 2 states either **0** or **1** .when `PinOPType`is set to ` GPIO_OP_TYPE_OD` then a pin can only connect to only GND or floating state

_Parameters_

- `GPIO_RegDef_t *pGPIOx ` : Base address of Gpio Port . base address for Ports are defined in **Stm32f407xx.h** header file as **`GPIOA`** ,**`GPIOB`** ,**`GPIOC`** .... ,**`GPIOH`**

- `uint8_t PinNumber` : Pin number to write data to . Most stm32 micros each port has 16 pins from 0 to 15 .PinNumber definition macros are defined in **Stm32f407xx.h** header file as \*PinNumber**\*x** ( **x** : 0 ,1 ,2 ,3 .... 15 ) .

- `uint8_t Value` : The value / Data to be written to a gpio pin . eiither 0 or 1 can be written to the output pin . any value other than **0** is considered as **1**

  
 ## GPIO_WriteToOutputPin() - **API for Write to Output Pin**

Return type : `Void`

Writes the Data value to a gpio pin of a certain PORT (A,BC,....H) which is either 0 or 1 . when a pin configured in Output mode and if the `PinOPType`is set to ` GPIO_OP_TYPE_PP` then a pin can have 2 states either **0** or **1** .when `PinOPType`is set to ` GPIO_OP_TYPE_OD` then a pin can only connect to only GND or floating state

_Parameters_

- `GPIO_RegDef_t *pGPIOx ` : Base address of Gpio Port . base address for Ports are defined in **Stm32f407xx.h** header file as **`GPIOA`** ,**`GPIOB`** ,**`GPIOC`** .... ,**`GPIOH`**

- `uint8_t Value` : The value / Data to be written to a gpio pin . eiither 0 or 1 can be written to the output pin . any value other than **0** is considered as **1**

  

## GPIO_ToggleOutputPin()- **API to Toggle LED**

Return type : `Void`

Toggles the output of a gpio pin of a certain PORT (A,BC,....H) which is from **0** or **1** or from **1** to **0** when a pin configured in Output mode and if the `PinOPType`is set to ` GPIO_OP_TYPE_PP` then a pin can have 2 states either **0** or **1** .when `PinOPType`is set to ` GPIO_OP_TYPE_OD` then a pin can only connect to only GND or floating state

_Parameters_

- `GPIO_RegDef_t *pGPIOx ` : Base address of Gpio Port . base address for Ports are defined in **Stm32f407xx.h** header file as **`GPIOA`** ,**`GPIOB`** ,**`GPIOC`** .... ,**`GPIOH`**

- `uint8_t PinNumber` : Pin number to write data to . Most stm32 micros each port has 16 pins from 0 to 15 .PinNumber definition macros are defined in **Stm32f407xx.h** header file as \*PinNumber**\*x** ( **x** : 0 ,1 ,2 ,3 .... 15 ) .

- `uint8_t Value` : The value / Data to be written to a gpio pin . eiither 0 or 1 can be written to the output pin . any value other than **0** is considered as **1**



# Interrupt Modes
--To be Continued.

# Example1: Toggle On-Board LED with Open Drain PullUp Configuration.
--To be Continued




**********************************************************************

Note: This Code is Part of Learning STM32 Bare Metal Driver Development from Udemy! I have written these drivers from scratch by learning from my course only for Learnig Purpose.
Any Errors, Suggestions please correct me.
Thank you.
