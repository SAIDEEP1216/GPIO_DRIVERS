/*
 * LED_Toggle_OD_002.c
 *
 *  Created on: Aug 23, 2024
 *      Author: saideep
 */

// This is the Application to Toggle LED with Open Drain Configuration
// On-board Led Pin connected to PA5


#include "stm32f446xx.h"
#include <stdint.h>

void delay(){
	for(uint32_t i=0;i<500000; i++);
}

int main(){
	GPIO_Handle_t GPIO_LED_OD;

	GPIO_LED_OD.pGPIOx = GPIOA;



	// Initialization of GPIO
	GPIO_LED_OD.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_LED_OD.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_LED_OD.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIO_LED_OD.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIO_LED_OD.GPIO_PinConfig.GPIO_PinPuPDControl = GPIO_PIN_PU;


	//Enable GPIO A Clock
	GPIO_PeriClkCtrl(GPIOA, ENABLE);

	//Initialize the GPIO A
	GPIO_Init(&GPIO_LED_OD);

	while(1){
	//Toggle the LED

	GPIO_ToggleOutPin(GPIOA, GPIO_PIN_NO_5);
	delay();


	}

	return 0;
}

