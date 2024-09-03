/*
 * stm32f446xx.h
 *
 *  Created on: Aug 3, 2024
 *      Author: saideep Pajjuri
 *      //This is Device Specific Header File for STM32F446RE
 *      Contains Base Address Macros of FLASH and SRAM memories and SFR's
 *      Contains Processor Specific Register Addresses
 */

#include<stdint.h> //Need to removed or placed somewhere

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#define __vo volatile



/*********** Processor Specific Details *******************/
/*
 * ARM Cortex M4 Processor NVIC ISERx Register Addresses
 * Source: ARM Cortex M4 Generic User Guide
 * NVIC Register Addresses
 */

//Interrupt Set-enable Registers We can have max of 7 Registers
#define  NVIC_ISER0  ((__vo uint32_t*)0xE000E100)
#define  NVIC_ISER1  ((__vo uint32_t*)0xE000E104)
#define  NVIC_ISER2  ((__vo uint32_t*)0xE000E108)
#define  NVIC_ISER3  ((__vo uint32_t*)0xE000E10C)


//Interrupt Clear-enable Registers To Reset/ Clear the NVOC Registers
#define NVIC_ICER0   ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1   ((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2   ((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3   ((__vo uint32_t*)0XE000E18C)


//Interrupt Priority Registers
#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0XE000E400) // Base Address of IPR


/*
 * No of Priority Bits Implemented for ARM CORTEX M4 NVIC Priority Registers
 */
#define NO_PR_BITS_IMPLEMENTED     4


/****************************************************************/

/* Base Addresses of FLASH and SRAM Memories*/ /* */

#define Flash_BASEADDR      0x00000000U    				  /* Flash Address starts from here*//* Here "U" indicates Unsigned Integer (uint32_t) */
#define SRAM1_BASEADDR  	0x20000000U	   				  /* SRAM1 Address starts here*/
#define SRAM2_BASEADDR  	0x2001C000U   				  /* SRAM2 Address starts here*/
#define SRAM_BASEADDR 		SRAM1_BASEADDR   		 	  /* SRAM Address starts here*/
#define ROM_BASEADDR 		0x1FFF0000U	  				  /* ROM or System Memory  Address starts here*/


/* Base Addresses of Bus Peripherals APB1,APB2,AHB1,AHB2,AHB3*/
#define PHERI_BASEADDR 		    0x40000000U
#define APB1_PHERI_BASEADDR     PHERI_BASEADDR                 /*  APB1 Address starts from here*/
#define APB2_PHERI_BASEADDR     0x40010000U                    /*  APB2 Address starts from here*/
#define AHB1_PHERI_BASEADDR     0x40020000U                    /*  AHB1 Address starts from here*/
#define AHB2_PHERI_BASEADDR     0x50000000U                    /*  AHB2 Address starts from here*/
#define AHB3_PHERI_BASEADDR     0xA0001000U                    /*  AHB3 Address starts from here*/


/* Base Addresses of Peripherals  Hanging on APB1,APB2,AHB1,AHB2,AHB3*/
//AHB1 Peripherals <AHB1_BASE + OFFSET>
#define GPIOA_BASEADDR       (AHB1_PHERI_BASEADDR + 0x0000)
#define GPIOB_BASEADDR       (AHB1_PHERI_BASEADDR + 0x0400)
#define GPIOC_BASEADDR       (AHB1_PHERI_BASEADDR + 0x0800)
#define GPIOD_BASEADDR       (AHB1_PHERI_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR       (AHB1_PHERI_BASEADDR + 0x1000)
#define GPIOF_BASEADDR       (AHB1_PHERI_BASEADDR + 0x1400)
#define GPIOG_BASEADDR       (AHB1_PHERI_BASEADDR + 0x1800)
#define GPIOH_BASEADDR       (AHB1_PHERI_BASEADDR + 0x1C00)

#define RCC_BASEADDR       (AHB1_PHERI_BASEADDR + 0x3800) //RCC BaseAddress

//APB1 Peripherals <APB1_BASE + OFFSET>

#define I2C1_BASEADDR         (APB1_PHERI_BASEADDR + 0x5400)
#define I2C2_BASEADDR         (APB1_PHERI_BASEADDR + 0x5800)
#define I2C3_BASEADDR         (APB1_PHERI_BASEADDR + 0x5C00)

#define SPI2_BASEADDR         (APB1_PHERI_BASEADDR + 0x3800)
#define SPI3_BASEADDR         (APB1_PHERI_BASEADDR + 0x3C00)

#define USART2_BASEADDR       (APB1_PHERI_BASEADDR + 0x4400)
#define USART3_BASEADDR       (APB1_PHERI_BASEADDR + 0x4800)
#define UART4_BASEADDR        (APB1_PHERI_BASEADDR + 0x4C00)
#define UART5_BASEADDR        (APB1_PHERI_BASEADDR + 0x5000)




//APB2 Peripherals <APB2_BASE + OFFSET>

#define USART1_BASEADDR       (APB2_PHERI_BASEADDR + 0x1000)
#define USART6_BASEADDR       (APB2_PHERI_BASEADDR + 0x1400)

#define SPI1_BASEADDR         (APB2_PHERI_BASEADDR + 0x3000)
#define SPI4_BASEADDR         (APB2_PHERI_BASEADDR + 0x3400)
/*
 * SYSCNFG BASE ADDRESS
 */
#define SYSCFG_BASEADDR       (APB2_PHERI_BASEADDR + 0x3800)

#define EXTI_BASEADDR         (APB2_PHERI_BASEADDR + 0x3C00)


//Peripheral GPIO Registers Definition Structures
//Instead of Creating a Base address macro take a pointer and of this structure and assign base address of GPIO_x ex: Reg_ptr->MODER = 25;
typedef struct{

	__vo uint32_t MODER;         /*!<GPIO port mode register              OFFSET: 0x00>  */
	__vo uint32_t OTYPER;	     /*!<GPIO port output type register       OFFSET: 0x04>   */
	__vo uint32_t OSPEEDR;       /*!<GPIO port output speed register      OFFSET: 0x08>   */
	__vo uint32_t PUPDR;         /*!<GPIO port pull-up/pull-down register OFFSET: 0x0C>   */
	__vo uint32_t IDR;           /*!<GPIO port input data register        OFFSET: 0x10>   */
	__vo uint32_t ODR;           /*!<GPIO port output data register               0x14> */
	__vo uint32_t BSRR;          /*!<GPIO port bit set/reset register             0x18> */
	__vo uint32_t LCKR;          /*!<GPIO port configuration lock register        0x1C> */
	__vo uint32_t AFR[2];        /*!<GPIO alternate function low register AFR[0] -> AFR Low Register*/
								   /* AFR[1] -> AFR High Register                 0x20> */

}GPIO_RegDef_t;

//To access these structure for GPIOA we need a pointer example GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t*)(GPIOA_BASEADDR)=>GPIOA
//pointer type-casting macros for GPIO's

#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR) /*<(GPIOA=>GPIO_RegDef_t*)(GPIOA_BASEADDR)>*/
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR)

//EXTI Peripheral Definition
#define EXTI   ((EXTI_RegDef_t*)EXTI_BASEADDR)

//SYSCFG Peripheral Definition
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)




//Peripheral Register Definitions for RCC
typedef struct{
	__vo uint32_t    RCC_CR;  /*<RCC clock control register>*/
	__vo uint32_t    RCC_PLLCFGR;		/*<RCC PLL configuration register (RCC_PLLCFGR)>*/
	__vo uint32_t    RCC_CFGR;			/*<RCC clock configuration register (RCC_CFGR>*/
	__vo uint32_t    RCC_CIR;			/*<Reference Manual Page 175>*/
	__vo uint32_t    RCC_AHB1RSTR;		/*<AHB1 Registers Reset>*/
	__vo uint32_t    RCC_AHB2RSTR;		/*<>*/
	__vo uint32_t    RCC_AHB3RSTR;		/*<>*/
	__vo uint32_t    Reserved0;			/*<>*/
	__vo uint32_t    RCC_APB1RSTR;		/*<APB1 Registers Reset >*/
	__vo uint32_t    RCC_APB2RSTR;		/*<>*/
	__vo uint32_t    Reserved1;			/*<>*/
	__vo uint32_t    Reserved2;			/*<>*/
	__vo uint32_t    RCC_AHB1ENR;		/*<AHB1 Clock enable Register 0x30>*/
	__vo uint32_t    RCC_AHB2ENR;		/*<>*/
	__vo uint32_t    RCC_AHB3ENR;		/*<>*/
	__vo uint32_t    Reserved3;			/*<>*/
	__vo uint32_t    RCC_APB1ENR;       /*<RCC APB1 peripheral clock enable register 0x40>*/
	__vo uint32_t    RCC_APB2ENR;
	__vo uint32_t    Reserved4;
	__vo uint32_t    Reserved5;
	__vo uint32_t    RCC_AHB1LPENR;
	__vo uint32_t    RCC_AHB2LPENR;
	__vo uint32_t    RCC_AHB3LPENR;
	__vo uint32_t    Reserved6;
	__vo uint32_t    RCC_APB1LPENR;
	__vo uint32_t    RCC_APB2LPENR;
	__vo uint32_t    Reserved7;
	__vo uint32_t    Reserved8;
	__vo uint32_t    RCC_BDCR;
	__vo uint32_t    RCC_CSR ;
	__vo uint32_t    Reserved9;
	__vo uint32_t    Reserved10;
	__vo uint32_t    RCC_SSCGR;
	__vo uint32_t    RCC_PLLI2SCFGR;
	__vo uint32_t    RCC_PLLSAICFGR;
    __vo uint32_t    RCC_DCKCFGR;
    __vo uint32_t    RCC_CKGATENR;
    __vo uint32_t    RCC_DCKCFGR2;



}RCC_RegDef_t;




/*
 * Peripheral Register Structure for EXTI External Interrupt Controller
 * Based on Interrupt Type these Registers can be Configured
 */
typedef struct{
	__vo uint32_t EXTI_IMR;         /*!<Interrupt mask register (EXTI_IMR)  OFFSET:     0x00> */
	__vo uint32_t EXTI_EMR;	     	/*!<Event mask register (EXTI_EMR)                  0x04> */
	__vo uint32_t EXTI_RTSR;        /*!<Rising trigger selection register (EXTI_RTSR)   0x08> */
	__vo uint32_t EXTI_FTSR;        /*!<Falling trigger selection register (EXTI_FTSR)  0x0C> */
	__vo uint32_t EXTI_SWIER;       /*!<Software interrupt event register (EXTI_SWIER)  0x10> */
	__vo uint32_t EXTI_PR;          /*!<Pending register (EXTI_PR)                      0x14> */

}EXTI_RegDef_t;

/*
 * Peripheral Registers Structure for SYSCFG
 * These Registers are used to Establish Connection b/w GPIO and EXTI Lines
 */
typedef struct{
	__vo uint32_t SYSCFG_MEMRMP;         /*!<SYSCFG memory remap register (SYSCFG_MEMRMP)  OFFSET:                        0x00> */
	__vo uint32_t SYSCFG_PMC;	     	 /*!<SYSCFG peripheral mode configuration register (SYSCFG_PMC)                   0x04> */
	__vo uint32_t SYSCFG_EXTICR[4];        /*!<SYSCFG external interrupt configuration register 1 (SYSCFG_EXTICR1) 0x08 - 0x14> */
	__vo uint32_t RESERVED1[2];          /*Reserved 1                                                             0x18 to 0x1C> */
	__vo uint32_t SYSCFG_CMPCR;          /*!<Compensation cell control register (SYSCFG_CMPCR)                            0x20> */
	__vo uint32_t RESERVED2[2];          /*Reserved 2                                                              0x24 - 0x28> */
	__vo uint32_t SYSCFG_CFGR;           /*!<SYSCFG configuration register (SYSCFG_CFGR)                                  0x2C> */

}SYSCFG_RegDef_t;




//Clock Enable macros for GPIO's using RCC registers
//All GPIOS's are hanging on AHB1 Bus  //EN for enable and DI for disable
#define  GPIOA_PCLK_EN()   (RCC->RCC_AHB1ENR|=(1<<0))
#define  GPIOB_PCLK_EN()   (RCC->RCC_AHB1ENR|=(1<<1))
#define  GPIOC_PCLK_EN()   (RCC->RCC_AHB1ENR|=(1<<2))
#define  GPIOD_PCLK_EN()   (RCC->RCC_AHB1ENR|=(1<<3))
#define  GPIOE_PCLK_EN()   (RCC->RCC_AHB1ENR|=(1<<4))
#define  GPIOF_PCLK_EN()   (RCC->RCC_AHB1ENR|=(1<<5))
#define  GPIOG_PCLK_EN()   (RCC->RCC_AHB1ENR|=(1<<6))
#define  GPIOH_PCLK_EN()   (RCC->RCC_AHB1ENR|=(1<<7))


//Clock Enable macros for I2C hanging on APB1 Bus
#define I2C1_PCLK_EN()      (RCC->RCC_APB1ENR|=(1<<21))
#define I2C2_PCLK_EN()      (RCC->RCC_APB1ENR|=(1<<22))
#define I2C3_PCLK_EN()      (RCC->RCC_APB1ENR|=(1<<23))


//Clock Enable macros for SPI and SYSCNFG hanging on APB2 Buses
//APB1
#define SP12_PCLK_EN()       (RCC->RCC_APB1ENR|=(1<<14))
#define SP13_PCLK_EN()       (RCC->RCC_APB1ENR|=(1<<15))
//APB2
#define SP11_PCLK_EN()       (RCC->RCC_APB2ENR|=(1<<12))
#define SP14_PCLK_EN()       (RCC->RCC_APB2ENR|=(1<<13))
#define SYSCFG_PCLK_EN()     (RCC->RCC_APB2ENR|=(1<<14))


//Clock Enable macros for USART and UART hanging on APB1 and APB2 Buses
//APB1
#define USART2_PCLK_EN()     (RCC->RCC_APB1ENR|=(1<<17))
#define USART3_PCLK_EN()     (RCC->RCC_APB1ENR|=(1<<18))
#define UART4_PCLK_EN()      (RCC->RCC_APB1ENR|=(1<<19))
#define UART5_PCLK_EN()      (RCC->RCC_APB1ENR|=(1<<20))
//APB2
#define USART1_PCLK_EN()     (RCC->RCC_APB2ENR|=(1<<4))
#define USART6_PCLK_EN()     (RCC->RCC_APB2ENR|=(1<<5))





//Clock Disable macros for GPIO's using RCC registers
//All GPIOS's are hanging on AHB1 Bus < "DI" for disable>

#define  GPIOA_PCLK_DI()   (RCC->RCC_AHB1ENR &=~(1<<0))
#define  GPIOB_PCLK_DI()   (RCC->RCC_AHB1ENR&=~(1<<1))
#define  GPIOC_PCLK_DI()  (RCC->RCC_AHB1ENR&=~(1<<2))
#define  GPIOD_PCLK_DI()   (RCC->RCC_AHB1ENR&=~(1<<3))
#define  GPIOE_PCLK_DI()  (RCC->RCC_AHB1ENR&=~(1<<4))
#define  GPIOF_PCLK_DI()  (RCC->RCC_AHB1ENR&=~(1<<5))
#define  GPIOG_PCLK_DI()  (RCC->RCC_AHB1ENR&=~(1<<6))
#define  GPIOH_PCLK_DI()  (RCC->RCC_AHB1ENR&=~(1<<7))


//Clock Enable macros for I2C hanging on APB1 Bus
#define I2C1_PCLK_DI()     (RCC->RCC_APB1ENR&=~(1<<21))
#define I2C2_PCLK_DI()     (RCC->RCC_APB1ENR&=~(1<<22))
#define I2C3_PCLK_DI()     (RCC->RCC_APB1ENR&=~(1<<23))


//Clock Enable macros for SPI and SYSCNFG hanging on APB2 Buses
//APB1
#define SP12_PCLK_DI()      (RCC->RCC_APB1ENR&=~(1<<14))
#define SP13_PCLK_DI()      (RCC->RCC_APB1ENR&=~(1<<15))
//APB2
#define SP11_PCLK_DI()      (RCC->RCC_APB2ENR&=~(1<<12))
#define SP14_PCLK_DI()      (RCC->RCC_APB2ENR&=~(1<<13))
#define SYSCFG_PCLK_DI()    (RCC->RCC_APB2ENR&=~(1<<14))


//Clock Enable macros for USART and UART hanging on APB1 and APB2 Buses
//APB1
#define USART2_PCLK_DI()    (RCC->RCC_APB1ENR&=~(1<<17))
#define USART3_PCLK_DI()    (RCC->RCC_APB1ENR&=~(1<<18))
#define UART4_PCLK_DI()     (RCC->RCC_APB1ENR&=~(1<<19))
#define UART5_PCLK_DI()     (RCC->RCC_APB1ENR&=~(1<<20))
//APB2
#define USART1_PCLK_DI()    (RCC->RCC_APB2ENR&=~(1<<4))
#define USART6_PCLK_DI()    (RCC->RCC_APB2ENR&=~(1<<5))


// Macros to Reset for GPIOA-GPIOH Peripherals using RCC Register Reset and Set
#define  GPIOA_REG_RESET()   do{ (RCC->RCC_AHB1ENR|=(1<<0)); (RCC->RCC_AHB1ENR &=~(1<<0));}while(0)
#define  GPIOB_REG_RESET()   do{ (RCC->RCC_AHB1ENR|=(1<<1)) ;(RCC->RCC_AHB1ENR &=~(1<<1));}while(0)
#define  GPIOC_REG_RESET()   do{ (RCC->RCC_AHB1ENR|=(1<<2)) ;(RCC->RCC_AHB1ENR &=~(1<<2));}while(0)
#define  GPIOD_REG_RESET()   do{ (RCC->RCC_AHB1ENR|=(1<<3)) ;(RCC->RCC_AHB1ENR &=~(1<<3));}while(0)
#define  GPIOE_REG_RESET()   do{ (RCC->RCC_AHB1ENR|=(1<<4)) ;(RCC->RCC_AHB1ENR &=~(1<<4));}while(0)
#define  GPIOF_REG_RESET()   do{ (RCC->RCC_AHB1ENR|=(1<<5)) ;(RCC->RCC_AHB1ENR &=~(1<<5));}while(0)
#define  GPIOG_REG_RESET()   do{ (RCC->RCC_AHB1ENR|=(1<<6)) ;(RCC->RCC_AHB1ENR &=~(1<<6));}while(0)
#define  GPIOH_REG_RESET()   do{ (RCC->RCC_AHB1ENR|=(1<<7)) ;(RCC->RCC_AHB1ENR &=~(1<<7));}while(0)

/*
 * Macro to convert GPIO_BASEADDR TO CODE
 */
#define GPIO_BASEADDR_TO_CODE(x)    ( (x==GPIOA) ? 0 :\
									  (x==GPIOB) ? 1 :\
									  (x==GPIOC) ? 2 :\
									  (x==GPIOD) ? 3 :\
									  (x==GPIOE) ? 4 :\
									  (x==GPIOF) ? 5 :\
									  (x==GPIOG) ? 6 :\
									  (x==GPIOH) ? 7 :0 )




/*
 * IRQ(Interrupt Request) Numbers of STM32F446RE MCU
 * As per given NVIC Table Priority of the following
 */

#define IRQ_NO_EXTI0         	 6
#define IRQ_NO_EXTI1         	 7
#define IRQ_NO_EXTI2          	 8
#define IRQ_NO_EXTI3             9
#define IRQ_NO_EXTI4             10
#define IRQ_NO_EXTI9_5           23
#define IRQ_NO_EXTI15_10         40


//Some Generic Macros
#define ENABLE 			 1
#define DIAABLE 	   	 0
#define SET    			 ENABLE
#define RESET   		 DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET

#include "stm32f446xx_gpio_driver.h"


#endif /* INC_STM32F446XX_H_ */
