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

//	//Enable Peripheral clock
//	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1.configure modes
	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG ){

       //non interrupt mode
       temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
       pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing 2bits before setting
       pGPIOHandle->pGPIOx->MODER |=temp; //setting
	 }
	 else{


		 //this for interrupt mode
		 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			 //1. configure Falling Trigger Selection Register
			 EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 //clearing RTSR bit to avoid previous configuration issues
			 EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
		 else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			 //1. configure Rising Trigger Selection Register
			 EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clearing FTSR bit to avoid previous configuration issues
             EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
		 else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			 //1. configure both FTSR and RTSR
			 EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
		 //2.configure GPIO port selection in SYSCFG_EXTICR
		 uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		 uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		 uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		 SYSCFG_PCLK_EN();
		 SYSCFG->EXTICR[temp1]=portcode << (temp2*4);

		 //3.enable EXTI interrupt delivery using IMR
		 EXTI->IMR =(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	 }

	 temp=0;
	 //2.configure speed
//	 if(pGPIOHandle->pGPIOx == GPIOA){
//
//	 pGPIOHandle->pGPIOx->OSPEEDR= 0x00000000;}
	 temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing 2bits before setting and 0x3 means to enable first 2bits - 0011(3)
	 pGPIOHandle->pGPIOx->OSPEEDR |=temp; //setting

	 temp=0;

	 //3.configure output types
//	 if(pGPIOHandle->pGPIOx == GPIOA){
//	 pGPIOHandle->pGPIOx->OTYPER=0x00000000;}
	 temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing 1bit before setting
	 pGPIOHandle->pGPIOx->OTYPER |=temp; //setting

	 temp=0;

	 //4. configure pull up/pull down
//	 if(pGPIOHandle->pGPIOx == GPIOA){
//	 pGPIOHandle->pGPIOx->PUPDR=0x00000000;}
	 temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	 pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing 2bits before setting and 0x3 means to enable first 2bits - 0011(3)
     pGPIOHandle->pGPIOx->PUPDR |=temp;

	 temp=0;

	 //5.configure alternate functionality
	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN ){

		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFRL[temp1] &= ~(0xFF <<(4*temp2));
		pGPIOHandle->pGPIOx->AFRL[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
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


/* IRQ Configuration and ISR Handling */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis){
	if (EnorDis == ENABLE) {
	        // Enable interrupt
	        if (IRQNumber <= 31) {
	            *NVIC_ISER0 |= (1 << IRQNumber); // Enable interrupt in ISER0
	        } else if (IRQNumber >= 32 && IRQNumber < 64) {
	            *NVIC_ISER1 |= (1 << (IRQNumber % 32)); // Enable interrupt in ISER1

	        } else if (IRQNumber >=64 && IRQNumber <96) {
	            *NVIC_ISER2 |= (1 << (IRQNumber % 64)); // Enable interrupt in ISER2
	        }
	    } else if (EnorDis == DISABLE) {
	       //  Disable interrupt
	        if (IRQNumber <= 31) {
	            *NVIC_ICER0 |= (1 << IRQNumber); // Disable interrupt in ICER0
	        } else if (IRQNumber >= 32 && IRQNumber < 64) {
	            *NVIC_ICER1 |= (1 << (IRQNumber % 32)); // Disable interrupt in ICER1
	        } else if (IRQNumber > 64 && IRQNumber < 96) {
	            *NVIC_ICER2 |= (1 << (IRQNumber % 64)); // Disable interrupt in ICER2
	        }
	    }
}

void GPIO_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority){

	//1.find out ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount  = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the exti pr register to this pin number
   if(EXTI->PR & (1<<PinNumber)){
	   EXTI->PR |= (1<<PinNumber);//clear
   }
}


