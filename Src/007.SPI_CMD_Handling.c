/*
 * 007.SPI_CMD_Handling.c
 *
 *  Created on: Nov 25, 2024
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

//command codes which slave recognizes
#define COMMAND_LED_CTRL         0x50
#define COMMAND_SENSOR_READ      0x51
#define COMMAND_LED_READ         0x52
#define COMMAND_PRINT            0x53
#define COMMAND_ID_READ          0x54

#define LED_ON      1
#define LED_OFF     0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4


//arduino led
#define LED_PIN   8

#include<string.h>

#define LOW 0
#define BTN_PRESSED LOW
void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
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

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);



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
	GPIO_Handle_t GPIOBtn,GpioLed;
	GPIOBtn.pGPIOx=GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE);
	GPIOBtn.pGPIOx->MODER = 0x00000000;
	GPIOBtn.pGPIOx->OSPEEDR= 0x00000000;
//	GpioLed.pGPIOx->OTYPER=0x00000000;
	GPIOBtn.pGPIOx->PUPDR=0x00000000;
	GPIOBtn.pGPIOx->AFRL[0]=0x00000000;
	GPIOBtn.pGPIOx->AFRL[1]=0x00000000;


	GPIO_Init(&GPIOBtn);



	//this is led gpio configuration
		GpioLed.pGPIOx = GPIOD;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GpioLed.pGPIOx,ENABLE);
	GpioLed.pGPIOx->MODER = 0x00000000;
	GpioLed.pGPIOx->OSPEEDR= 0x00000000;
	GpioLed.pGPIOx->OTYPER=0x00000000;
	GpioLed.pGPIOx->PUPDR=0x00000000;
	GpioLed.pGPIOx->AFRL[0]=0x00000000;
	GpioLed.pGPIOx->AFRL[1]=0x00000000;

	GPIO_Init(&GpioLed);

}

uint8_t SPI_VerifyResponse(uint8_t ackbyte){
	if(ackbyte ==(uint8_t) 0xF5){
		//ack
		return 1;
	}
	return 0;
}

int main(void){
//	char user_data[] ="Hello world";

	uint8_t dummy_write = 0xff;
    uint8_t dummy_read;
	GPIO_ButtonInit();

	SPI2_PinsInit();   // to initialize GPIO pins to behave as SPI2 pins

    SPI2_Init();	   // To Initialize SPI2 peripheral parameters

    SPI_SSOEConfig(SPI2, ENABLE); // setting SSOE enable NSS output  and when SPE=1, NSS will pulled to low and NSS pin will high when SPE=0
while(1){
	//wait till button is pressed
    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED);

    delay();//avoid button de-bouncing issues 200ms of delay

    SPI_Peripheralcontrol(SPI2,ENABLE); // To enable SPI2 peripheral

    //1.  CMD_LED_CTRL  <PIN NO> <VALUE(1)>
    uint8_t commandcode = COMMAND_LED_CTRL;
    uint8_t ackbyte;
    uint8_t args[2];

    //send command
    SPI_SendData(SPI2,&commandcode,1);

    //do dummy read to clear RXNE
    SPI_ReceiveData(SPI2,&dummy_read,1);

    //send some dummy bits(1byte) to fetch response from slave
    SPI_SendData(SPI2,&dummy_write,1);

    SPI_ReceiveData(SPI2,&ackbyte,1);// return response from slave to read ack byte received

    if(SPI_VerifyResponse(ackbyte)){
    	//send arguments
    	args[0]=LED_PIN;
    	args[1]=LED_ON;
    	SPI_SendData(SPI2,args,2);
    }



    //2.CMD_SENSOR_READ <analog pin number(1)>

    //wait till button is pressed
    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED);

    delay();//avoid button de-bouncing issues 200ms of delay

    commandcode = COMMAND_SENSOR_READ;

    //send command
    SPI_SendData(SPI2,&commandcode,1);

    //do dummy read to clear RXNE
    SPI_ReceiveData(SPI2,&dummy_read,1);

    //send some dummy bits(1byte) to fetch response from slave
    SPI_SendData(SPI2,&dummy_write,1);

    SPI_ReceiveData(SPI2,&ackbyte,1);// return response from slave to read ack byte received

    if(SPI_VerifyResponse(ackbyte)){

    	args[0]=ANALOG_PIN0;

    	//send arguments
    	SPI_SendData(SPI2,args,1); //sending 1byte

        //do dummy read to clear RXNE
        SPI_ReceiveData(SPI2,&dummy_read,1);

        //insert delay so that slave can ready with data
        delay();

        //send some dummy bits(1byte) to fetch response from slave
        SPI_SendData(SPI2,&dummy_write,1);

        uint8_t analog_read;
        SPI_ReceiveData(SPI2,&analog_read,1);

    }


    //3. COMMAND_LED_READ   <PIN NO(1)>
    //wait till button is pressed
    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED);

    delay();//avoid button de-bouncing issues 200ms of delay

    commandcode = COMMAND_LED_READ;

    //send command
    SPI_SendData(SPI2,&commandcode,1);

    //do dummy read to clear RXNE
    SPI_ReceiveData(SPI2,&dummy_read,1);

    //send some dummy bits(1byte) to fetch response from slave
    SPI_SendData(SPI2,&dummy_write,1);

    SPI_ReceiveData(SPI2,&ackbyte,1);// return response from slave to read ack byte received

    if(SPI_VerifyResponse(ackbyte)){

    	args[0]=LED_PIN;

    	//send arguments
    	SPI_SendData(SPI2,args,1); //sending 1byte

        //do dummy read to clear RXNE
        SPI_ReceiveData(SPI2,&dummy_read,1);

        //insert delay so that slave can ready with data
        delay();

        //send some dummy bits(1byte) to fetch response from slave
        SPI_SendData(SPI2,&dummy_write,1);

        uint8_t led_status;
        SPI_ReceiveData(SPI2,&led_status,1);

    }


    //4. CMD_PRINT   <len(2)> <message(len)>

    //wait till button is pressed
    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED);

    delay();//avoid button de-bouncing issues 200ms of delay

    commandcode = COMMAND_PRINT;

    //send command
    SPI_SendData(SPI2,&commandcode,1);

    //do dummy read to clear RXNE
    SPI_ReceiveData(SPI2,&dummy_read,1);

    //send some dummy bits(1byte) to fetch response from slave
    SPI_SendData(SPI2,&dummy_write,1);

    SPI_ReceiveData(SPI2,&ackbyte,1);// return response from slave to read ack byte received

    uint8_t msg[] = "Hello !,How are you";
	if(SPI_VerifyResponse(ackbyte)){

    	args[0]=strlen((char*)msg);

    	//send arguments
    	SPI_SendData(SPI2,args,1); //sending 1byte

        //do dummy read to clear RXNE
        SPI_ReceiveData(SPI2,&dummy_read,1);

        //insert delay so that slave can ready with data
        delay();

        //send message
        for(int i=0;i<args[0];i++){
        	SPI_SendData(SPI2,&msg[i],1);
        	SPI_ReceiveData(SPI2,&dummy_read,1);
        }

    }

    //5. CMD_ID_READ
	   //wait till button is pressed
	    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED);

	    delay();//avoid button de-bouncing issues 200ms of delay

	    commandcode = COMMAND_ID_READ;

	    //send command
	    SPI_SendData(SPI2,&commandcode,1);

	    //do dummy read to clear RXNE
	    SPI_ReceiveData(SPI2,&dummy_read,1);

	    //send some dummy bits(1byte) to fetch response from slave
	    SPI_SendData(SPI2,&dummy_write,1);

	    SPI_ReceiveData(SPI2,&ackbyte,1);// return response from slave to read ack byte received

	    uint8_t id[11];
	    uint32_t i=0;
		if(SPI_VerifyResponse(ackbyte)){
	        //read 10 bytes id from slave
	        for( i=0;i<10;i++){
	        	SPI_SendData(SPI2,&dummy_write,1);
	        	SPI_ReceiveData(SPI2,&id[i],1);
	        }
	        id[10]='\0';

	    }




    while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) == FLAG_SET);

    SPI_Peripheralcontrol(SPI2,DISABLE); //disable spi2 peripheral
}
    return 0;
}
