/*
 * LED_Toggle_001.c
 *
 *  Created on: Aug 13, 2024
 *      Author: saideep
 */


#include "stm32f446xx.h"
#include <stdint.h>

void delay(){
	for(uint32_t i=0; i <500000;i++);
}

int main(void)
{
   GPIO_Handle_t GPIO_Led;
  /*
   * This is to talk to GPIO Registers
   */
     GPIO_Led.pGPIOx = GPIOA;
   //GPIO_RegDef_t *pGPIOx = GPIOA; Instead of Initializing this separately included in handle structure

  /*
   * This is for GPIO Configuration
   */
   GPIO_Led.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_5;
   GPIO_Led.GPIO_PinConfig.GPIO_PinMode   		= GPIO_MODE_OUT;
   GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed  		= GPIO_SPEED_MEDIUM;
   GPIO_Led.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;
   GPIO_Led.GPIO_PinConfig.GPIO_PinPuPDControl  = GPIO_NO_PUPD;

   GPIO_PeriClkCtrl(GPIOA, ENABLE);
   GPIO_Init(&GPIO_Led);

   while(1){

   GPIO_ToggleOutPin(GPIOA,GPIO_PIN_NO_5);
   delay();
   }


   return 0;
}
