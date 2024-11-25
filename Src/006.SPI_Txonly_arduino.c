/*
 * 006.SPI_Txonly_arduino.c
 *
 *  Created on: Nov 21, 2024
 *      Author: keerthika.m
 */


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
 */
#include "stm32f411xx.h"

#include<string.h>

#define LOW 0
#define BTN_PRESSED LOW
void delay(void){
	for(uint32_t i=0;i<500000;i++);
}

void SPI2_PinsInit(void){
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

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

//	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
//


}

void SPI2_Init(void){

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV8; //generate clk of 2MHz
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //hardware slave disabled for NSS pin
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void){
	GPIO_Handle_t GPIOBtn;
	GPIOBtn.pGPIOx=GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE);

	GPIOBtn.pGPIOx->MODER = 0x00000000;
	GPIOBtn.pGPIOx->OSPEEDR= 0x00000000;
//	GPIOBtn.pGPIOx->OTYPER=0x00000000;
	GPIOBtn.pGPIOx->PUPDR=0x00000000;
	GPIOBtn.pGPIOx->AFRL[0]=0x00000000;
	GPIOBtn.pGPIOx->AFRL[1]=0x00000000;

	GPIO_Init(&GPIOBtn);

}

int main(void){
	char user_data[] ="Hello world";

	GPIO_ButtonInit();

	SPI2_PinsInit();   // to initialize GPIO pins to behave as SPI2 pins

    SPI2_Init();	   // To Initialize SPI2 peripheral parameters

    SPI_SSOEConfig(SPI2, ENABLE); // setting SSOE enable NSS output  and when SPE=1, NSS will pulled to low and NSS pin will high when SPE=0
while(1){
    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED);

    delay();
    SPI_Peripheralcontrol(SPI2,ENABLE); // To enable SPI2 peripheral

    uint8_t dataLen = strlen(user_data);
    SPI_SendData(SPI2,&dataLen,1); //length information for arduino to receive one  byte

    SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));


    SPI_Peripheralcontrol(SPI2,DISABLE);
}
    return 0;
}
