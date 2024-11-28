/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Nov 27, 2024
 *      Author: keerthika.m
 */


#include "stm32f411xx_i2c_driver.h"

uint16_t AHB_PresScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PresScaler[4] = {2,4,8,16};

static void  I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void  I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void  I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);    //slave adress + lsb is read/write bit which must be set to 0 for write
	pI2Cx->DR = SlaveAddr;

}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;    //slave adress + lsb is read/write bit which must be set to 1 for read
	pI2Cx->DR = SlaveAddr;

}


static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx){
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;

}

static void  I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}


/* APIs SUPPORTED BY THIS DRIVER  */

/* Peripheral Clock setup*/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}

	}
	else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}

	}
}
uint32_t RCC_GetPLLOutputClock(){
	return 0;
}

uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1;

	clksrc = ((RCC->CFGR >> 2) &0x3); //mask all bits in cfgr register except 2 and 3 bits
	if(clksrc==0){
		SystemClk = 16000000;
	}
	else if(clksrc==1){
		SystemClk = 8000000;
	}
	else if(clksrc==2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	//AHB
	temp=((RCC->CFGR >> 4) &0xF);
	if(temp<8){
		ahbp=1;
	}else{
		ahbp=AHB_PresScaler[temp-8];
	}

	//APB1
	temp=((RCC->CFGR >> 10) &0x7);
	if(temp<4){
		apb1=1;
	}else{
		apb1=APB1_PresScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp ) / apb1;

	return pclk1;
}

/* Init and DeInit  */

void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}


void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr){
	//1.Generate START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.confirms start generation is completed by checking SB flag in SR1
	//Note: Until sb is cleared SCL will be stretched (pulled to low)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	//3.send address of the slave with read/write bit to 0 (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4.confirm address phase is completed by checking ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	//5.clear ADDR flag according to its software sequence
	//NOte: Until ADDR is cleared SCL will be stretched
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6.send data until len is 0
	while(Len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7.when Len becomes 0 wait for TXE=1 and BTF=1 before generating STOP
	//means that both SR and DR are empty and next transmission should begin
	//when BTF=1, SCL will be stretched
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );


	//8.Generate STOP and master need not to wait for completion of STOP
	//Note: generating STOP, automatically clears BTF

	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr){

	//1.generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2.confirms start generation is completed by checking SB flag in SR1
	//Note: Until sb is cleared SCL will be stretched (pulled to low)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	//3.send address of the slave with read/write bit to 1 (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

//	//4.confirm address phase is completed by checking ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	//procedure to read only 1 byte from slave
	if(Len == 1){
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

		//generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;




	}

	//procedure to read data from slave when Len>1
	if(Len>1){
		//clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for(uint32_t i=Len;i>0;i--){
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

			if(i==2)//if last 2 bytes are remaining
			{
				//clear ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


                //generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		    }

			//read data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment buffer address
			pRxBuffer++;
		}
	}
	//re enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
	I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}




/*
 * peripheral control APIs
 */

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == I2C_ACK_ENABLE){
		//enable the ack
		pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
	}
	else{
		//disable the ack
		pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
	}
}

void I2C_Peripheralcontrol(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
	pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}
	else if(EnorDi == DISABLE){
	pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);

	}
}



void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDis){
	if(EnorDis == ENABLE){
		if(IRQNumber <= 31){
			//program Interrupt Set Enable Register0 register
			*NVIC_ISER0 |= (1<<IRQNumber); //holds 32bits

		}else if (IRQNumber > 31 && IRQNumber < 64 ){
			//program ISER1 register
			*NVIC_ISER1 |= (1<<IRQNumber % 32);

		}else if(IRQNumber >=64 && IRQNumber < 96){
			//program ISER2 register
			*NVIC_ISER2 |= (1<<IRQNumber % 64);

		}
	}else if(EnorDis == DISABLE){
		if(IRQNumber <= 31){
			//program Interrupt Clear Enable Register0 register
			*NVIC_ICER0 |= (1<<IRQNumber);// holds 32 bits

		}else if (IRQNumber > 31 && IRQNumber < 64 ){
			//program ICER1 register
			*NVIC_ICER1 |= (1<<IRQNumber % 32);

		}else if(IRQNumber >=64 && IRQNumber < 96){
			//program ICER2 register
			*NVIC_ICER2 |= (1<<IRQNumber % 64);

		}

	}
}

void I2C_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority){

	//1.find out ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount  = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}



__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle,uint8_t AppEvent){
	//this is weak implementation. the application may override this function
}
