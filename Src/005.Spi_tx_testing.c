/*
 * 005.Spi_tx_testing.c
 *
 *  Created on: Nov 20, 2024
 *      Author: keerthika.m
 */
/*
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 -> SPI_NSS
 * ALT function mode : A5
 * PA5 -> SPI1 SCLK PIN D13
 * PA7 -> SPI1 MOSI PIN D11
 */
#include "stm32f411xx.h"
#include<stdio.h>
#include<string.h>

void SPI2_PinsInit(){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(SPIPins.pGPIOx, ENABLE);
	SPIPins.pGPIOx->MODER = 0x00000000;
	SPIPins.pGPIOx->OSPEEDR= 0x00000000;
	SPIPins.pGPIOx->OTYPER=0x00000000;
	SPIPins.pGPIOx->PUPDR=0x00000000;
	SPIPins.pGPIOx->AFRL[0]=0x00000000;
	SPIPins.pGPIOx->AFRL[1]=0x00000000;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

//	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
//
//	//NSS
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);


}

void SPI2_Init(void){

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EI;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&SPI2Handle);
}

int main(void){
	char user_data[] ="Hello World";

	SPI2_PinsInit();   // to initialize GPIO pins to behave as SPI2 pins

    SPI2_Init();	   // To Initialize SPI2 peripheral parameters


    SPI_SSIConfig(SPI2,ENABLE); // to make NSS signal internally high avoid MODF error


    SPI_Peripheralcontrol(SPI2,ENABLE); // To enable SPI2 peripheral


    SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

//    while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));//If this RETURN 1,SPI is busy
    SPI_Peripheralcontrol(SPI2,DISABLE);
    while(1);
    return 0;
}
