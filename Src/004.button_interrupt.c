/*
 * 004.button_interrupt.c
 *
 *  Created on: Nov 18, 2024
 *      Author: keerthika.m
 */


#include "stm32f411xx.h"
#include<string.h>

#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
void EXTI15_10_IRQHandler(void);

int main(void){
	GPIO_Handle_t Led,GPIOBtn;
	memset(&Led,0,sizeof(Led));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));
		Led.pGPIOx=GPIOA;
		Led.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_9;
		Led.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
		Led.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
		Led.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
		Led.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

		GPIO_PeriClockControl(GPIOA,ENABLE);

		GPIO_Init(&Led);


		GPIOBtn.pGPIOx=GPIOA;
		GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_10;
		GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
		GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
		GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;

		GPIO_PeriClockControl(GPIOA,ENABLE);

		GPIO_Init(&GPIOBtn);


		//IRQ CONFIGURATIONS
		GPIO_IRQPriority(IRQ_NO_EXTI15_10,NVIC_IRQ_PRI15);
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,ENABLE);


		while(1);
}

void EXTI15_10_IRQHandler(void){
	delay();//200ms
	GPIO_IRQHandling(GPIO_PIN_NO_10);//clear pending event from exti line
    GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_9);

}
