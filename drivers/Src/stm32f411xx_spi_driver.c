/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Nov 20, 2024
 *      Author: keerthika.m
 */


#include "stm32f411xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);

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
	            pSPIx->DR =   *((uint16_t*)pTxBuffer);
	            Len--;
	            Len--;
	            (uint16_t*)pTxBuffer++;// Move to the next character
	        } else {
	            // 8-bit DFF: Send 1 byte
				pSPIx->DR =   *pTxBuffer;
				Len--;
				pTxBuffer++;// Move to the next character

	        }
	    }

//	while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG) == FLAG_SET);//cnfrms SPI is not busy


}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len){
	while (Len>0) {
	        // Wait until RXNE buffer is set
	        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

	        // Check DFF bit in CR1 for 8-bit or 16-bit data mode
	        if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
	            // 16-bit DFF: Send 2 bytes
	            *((uint16_t*)pRxBuffer)=pSPIx->DR; //Load data from DR to RXbuffer address
	            Len--;
	            Len--;
	            (uint16_t*)pRxBuffer++;// Move to the next character
	        } else {
	            // 8-bit DFF: Send 1 byte
				*pRxBuffer = pSPIx->DR;
				Len--;
				pRxBuffer++;// Move to the next character

	        }
	    }




}



/* IRQ CONFIGURATION AND ISR HANDLING */


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state =pSPIHandle->TxState;
	if(state !=SPI_BUSY_IN_TX ){

	//1. save Tx buffer address and len information in some global variables
	pSPIHandle->pTxBuffer=pTxBuffer;
	pSPIHandle->TxLen=Len;

	//2. Mark SPI state as busy in transmission so that
	 // no other cpde can take over same spi peripheral until transmisssion is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3.Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return state;

	//4.Data Transmission will be handled by ISR code
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state =pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX ){

	//1. save Tx buffer address and len information in some global variables
	pSPIHandle->pRxBuffer=pRxBuffer;
	pSPIHandle->RxLen=Len;

	//2. Mark SPI state as busy in transmission so that
	 // no other cpde can take over same spi peripheral until transmisssion is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	//3.Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	return state;
}


void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDis){
	if(EnorDis == ENABLE){
			if(IRQNumber <= 31){
				//program Interrupt Set Enable Register0 register
				*NVIC_ISER0 |= (1<<IRQNumber); //holds 32bits

			}else if (IRQNumber <= 63 ){
				//program ISER1 register
				*NVIC_ISER1 |= (1<<IRQNumber % 32);


			}else if(IRQNumber <= 95){
				//program ISER2 register
				*NVIC_ISER2 |= (1<<IRQNumber % 64);

			}
		}else if(EnorDis == DISABLE){
			if(IRQNumber <= 31){
				//program Interrupt Clear Enable Register0 register
				*NVIC_ICER0 |= (1<<IRQNumber);// holds 32 bits

			}else if (IRQNumber <= 63 ){
				//program ICER1 register
				*NVIC_ICER1 |= (1<<IRQNumber % 32);

			}else if(IRQNumber < 95){
				//program ICER2 register
				*NVIC_ICER2 |= (1<<IRQNumber % 64);

			}

		}
}

void SPI_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority){

	//1.find out ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount  = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pHandle){
    uint8_t temp1,temp2;

    //checks for TXE
    temp1= pHandle->pSPIx->SR & (1<<SPI_SR_TXE);
    temp2= pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

    if(temp1 && temp2){
    	spi_txe_interrupt_handle(pHandle);
    }

    //checks for RXNE
    temp1= pHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
    temp2= pHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);

    if(temp1 && temp2){
    	spi_rxne_interrupt_handle(pHandle);
    }

    //checks for overrun
    temp1= pHandle->pSPIx->SR & (1<<SPI_SR_OVR);
    temp2= pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);

    if(temp1 && temp2){
    	spi_ovr_err_interrupt_handle(pHandle);
    }



}

/*
 * some helper function implementation
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle){

    // Check DFF bit in CR1 for 8-bit or 16-bit data mode
    if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
        // 16-bit DFF: Send 2 bytes
    	pHandle->pSPIx->DR =  *((uint16_t*)pHandle->pTxBuffer);
    	pHandle->TxLen--;
    	pHandle->TxLen--;
        (uint16_t*)pHandle->pTxBuffer++;// Move to the next character
    } else {
        // 8-bit DFF: Send 1 byte
    	pHandle->pSPIx->DR = *pHandle->pTxBuffer;
    	pHandle->TxLen--;
		pHandle->pTxBuffer++;// Move to the next character

    }

    //TxLen is zero, so close the spi transmission and inform the application that TX is over
    if(!pHandle->TxLen){

    	SPI_CloseTransmission(pHandle);
    	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);
    }
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle){

    // Check DFF bit in CR1 for 8-bit or 16-bit data mode
    if (pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
        // 16-bit DFF: Send 2 bytes
    	*((uint16_t*)pHandle->pRxBuffer) =(uint16_t) pHandle->pSPIx->DR;
    	pHandle->RxLen--;
    	pHandle->RxLen--;
        pHandle->pRxBuffer++;// Move to the next character
        pHandle->pRxBuffer++;
    } else {
        // 8-bit DFF: Send 1 byte
    	*(pHandle->pRxBuffer)=(uint8_t)pHandle->pSPIx->DR;
    	pHandle->RxLen--;
		pHandle->pRxBuffer++;// Move to the next character
    }

    //TxLen is zero, so close the spi transmission and inform the application that TX is over
    if(!pHandle->RxLen){

    	SPI_CloseReception(pHandle);
    	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_RX_CMPLT);
    }

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle){

	uint8_t temp;
	//1.clear ovr flag
	if(pHandle->TxState != SPI_BUSY_IN_TX){
		temp= pHandle->pSPIx->DR;
		temp= pHandle->pSPIx->SR;
	}
    (void)temp;
	//2.inform application
    SPI_ApplicationEventCallback(pHandle,SPI_EVENT_OVR_ERR);
}
void SPI_CloseTransmission(SPI_Handle_t *pHandle){
	//this prevents interrupt from setting up TXE flag
	pHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pHandle->pTxBuffer=NULL;
	pHandle->TxLen =0;
	pHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pHandle){
	//this prevents interrupt from setting up TXE flag
	pHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pHandle->pRxBuffer=NULL;
	pHandle->RxLen =0;
	pHandle->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	 uint8_t temp;
	 temp=pSPIx->DR;
	 temp=pSPIx->SR;
	 (void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle,uint8_t AppEvent){
	//this is weak implementation. the application may override this function
}



/*
 * peripheral control APIs
 */

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


