/*
 * 003.Led_Button_External.c
 *
 *  Created on: Nov 15, 2024
 *      Author: keerthika.m
 */


/*
 * 002Led_Button.c
 *
 *  Created on: Nov 15, 2024
 *      Author: keerthika.m
 */

#include "stm32f411xx.h"

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}

int main(void){
	GPIO_Handle_t Led,GPIOBtn;
		Led.pGPIOx=GPIOA;
		Led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_8;
		Led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
		Led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
		Led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
		Led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

		GPIO_PeriClockControl(GPIOA,ENABLE);

		GPIO_Init(&Led);


		GPIOBtn.pGPIOx=GPIOB;
		GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
		GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
//		GPIOBtn.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP; //this is only applicable for output mode
		GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
		GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;

		GPIO_PeriClockControl(GPIOB,ENABLE);

		GPIO_Init(&GPIOBtn);

		while(1){
			if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_5) == BTN_PRESSED){
			delay(); //added to prevent de-bouncing issues
 			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
			}

		}
		return 0;
}
