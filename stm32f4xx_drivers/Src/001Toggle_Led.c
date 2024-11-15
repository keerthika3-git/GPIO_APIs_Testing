/*
 * 001Toggle_Led.c
 *
 *  Created on: Nov 14, 2024
 *      Author: keerthika.m
 */


#include "stm32f411xx.h"
void delay(void){
	for(uint8_t i=0;i<500000;i++);
}

int main(void){
	GPIO_Handle_t Led;
	Led.pGPIOx=GPIOA;
	Led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	Led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	Led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	Led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	Led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&Led);

	while(1){
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
