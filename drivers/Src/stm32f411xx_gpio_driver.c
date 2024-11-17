/*
 * stm32f411xx.c
 *
 *  Created on: Nov 13, 2024
 *      Author: keerthika.m
 */


#include "stm32f411xx_gpio_driver.h"


/* APIs SUPPORTED BY THIS DRIVER  */

/* Peripheral Clock setup*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else{
			GPIOH_PCLK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else{
			GPIOH_PCLK_DI();
		}

	}
}


/* Init and DeInit  */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp=0;

	//1.configure modes
	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG ){
       //non interrupt mode
       temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
       pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing 2bits before setting
       pGPIOHandle->pGPIOx->MODER |=temp; //setting
	 }
	 else{
		 //this for interrupt mode
	 }
	 temp=0;

	 //2.configure speed
	 temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing 2bits before setting and 0x3 means to enable first 2bits - 0011(3)
	 pGPIOHandle->pGPIOx->OSPEEDR |=temp; //setting

	 temp=0;

	 //3.configure output types
	 temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing 1bit before setting
	 pGPIOHandle->pGPIOx->OTYPER |=temp; //setting

	 temp=0;

	 //4. configure pull up/pull down
	 temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing 2bits before setting and 0x3 means to enable first 2bits - 0011(3)
     pGPIOHandle->pGPIOx->PUPDR |=temp;

	 temp=0;

	 //5.configure alternate functionality
	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN ){
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFRL[temp1] &= ~(0xFF <<(4*temp2));
		pGPIOHandle->pGPIOx->AFRL[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber << (4 * temp2));
	 }

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

			if(pGPIOx == GPIOA){
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC){
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD){
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE){
				GPIOE_REG_RESET();
			}
			else{
				GPIOH_REG_RESET();
			}

}

/* Data read and write  */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value =(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); //right shifting any pin number to least significant bit by setting everything 0
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value){
	if(Value == GPIO_PINSET){
		pGPIOx->ODR |= (1<<PinNumber);
	}else{
		pGPIOx->ODR &= ~(1<<PinNumber);

	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR =Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}


/* TRQ Configuration and ISR Handling */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDis);//In IRQconfig need IRQ number,priority,enable or disable

void GPIO_IRQHandling(uint8_t PinNumber);// Handling function will be called for specific pin number


