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

      //Enable peripheral clock
	  SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	  uint32_t tempreg = 0;
	  //1. Configure Device Mode
	  tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	  //2.Configure Bus Config
	  if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		  //clear bidi mode
		  tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	  }
	  else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		  //set bidi mode
		  tempreg |= (1<<SPI_CR1_BIDIMODE);
	  }
	  else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		  //clear bidi mode
		  tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		  //set RXONLY bit
		  tempreg |= (1<<SPI_CR1_RXONLY);
	  }

	  //3. configure clk speed
	  tempreg |= pSPIHandle->SPIConfig.SPI_SClkSpeed << SPI_CR1_BR;

	  //4.Configure DFF
	  tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	  //5. Configure CPOL
	  tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	  //6.Configure CPHA
	  tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	  //7.configure SSM
	  tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

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
	while (Len>0) {
	        // Wait until TX buffer is empty
	        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

	        // Check DFF bit in CR1 for 8-bit or 16-bit data mode
	        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
	            // 16-bit DFF: Send 2 bytes
//	            pSPIx->DR =*pString ;
//	            pString++; // Move to the next character
	            pSPIx->DR =   *((uint16_t*)pTxBuffer);
	            Len--;
	            Len--;
	            (uint16_t*)pTxBuffer++;
	        } else {
	            // 8-bit DFF: Send 1 byte
//	        	pSPIx->DR =*pString ;
//	            pString++; // Move to the next character
				pSPIx->DR =   *pTxBuffer;
				Len--;
				pTxBuffer++;

	        }
	    }

	while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG) == FLAG_SET);


}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);



/* IRQ CONFIGURATION AND ISR HANDLING */

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDis);//In IRQconfig need IRQ number,enable or disable

void SPI_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority);//priority

void SPI_IRQHandling(SPI_Handle_t *pHandle);



void SPI_Peripheralcontrol(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
	pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else if(EnorDi == DISABLE){
	pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);

	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
		}
		else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);

		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
		}
	else if(EnorDi == DISABLE){
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);

		}
}


