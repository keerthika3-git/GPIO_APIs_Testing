/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Nov 20, 2024
 *      Author: keerthika.m
 */


#include "stm32f411xx_spi_driver.h"

/* APIs SUPPORTED BY THIS DRIVER  */

/* Peripheral Clock setup*/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
		else{
			SPI5_PCLK_EN();
		}
	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
		else{
			SPI5_PCLK_DI();
		}
	}
}

/* Init and DeInit  */

void SPI_Init(SPI_Handle_t *pSPIHandle){


	  uint32_t tempreg = 0;
	  //1. Configure Device Mode
	  tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	  //2.Configure Bus Config
	  if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		  //clear bidi mode
		  tempreg &= ~(1<<15);
	  }
	  else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		  //set bidi mode
		  tempreg |= (1<<15);
	  }
	  else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		  //clear bidi mode
		  tempreg &= ~(1<<15);
		  //set RXONLY bit
		  tempreg |= (1<<10);
	  }

	  //3. configure clk speed
	  tempreg |= pSPIHandle->SPIConfig.SPI_SClkSpeed << SPI_CR1_BR;

	  //4.Configure DFF
	  tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	  //5. Configure CPOL
	  tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	  //6.Configure CPHA
	  tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	  pSPIHandle->pSPIx->CR1 = tempreg;
}


void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4){
		SPI4_REG_RESET();
	}
	else{
		SPI5_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/* Data Send and Receive  */
// This is Blocking Call

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	 while(Len>0){
		 //wait until tx buffer is set (set 1 denotes empty)
		 while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET); //polling for TXE flag to set

		 //check DFF bit in CR1
		 if((pSPIx->CR1 & (1<<SPI_CR1_DFF))){
			 //16bit DFF
			 pSPIx->DR = *((uint16_t*)pTxBuffer); // buffer is type casting to store 16bit
			 Len--;
			 Len--; // two time decrement -> 2bytes
			 (uint16_t*)pTxBuffer++; //inc pointer
		 }
		 else{
			 //8bit DFF
			 pSPIx->DR = *pTxBuffer;
			 Len--; // one time decrement -> 1byte
			 pTxBuffer++;
		 }
	 }
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);



/* IRQ CONFIGURATION AND ISR HANDLING */

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDis);//In IRQconfig need IRQ number,enable or disable

void SPI_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority);//priority

void SPI_IRQHandling(SPI_Handle_t *pHandle);
