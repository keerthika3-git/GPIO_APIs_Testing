/*
 * 011.I2C_master_rx_testingIT.c
 *
 *  Created on: Nov 29, 2024
 *      Author: keerthika.m
 */



/*
 * PB6 -> SCL   Pin name: D10
 * PB9 -> SDA   pin name: D14
 */
#include "stm32f411xx.h"
#include "string.h"
#include<stdio.h>

uint8_t rxcmplt=RESET;

extern void initialise_monitor_handles(void);

#define MY_ADDR   0x61
#define SlaveAddr 0x68

#define LOW 0
#define BTN_PRESSED LOW

void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}

I2C_Handle_t I2C1Handle;

//receive buffer
uint8_t rcv_buf[32];


void I2C1_PinsInit(void){
     GPIO_Handle_t I2CPins;

     I2CPins.pGPIOx=GPIOB;
     I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
     I2CPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
     I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PU;
     I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
     I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

     GPIO_PeriClockControl(GPIOB, ENABLE);
     I2CPins.pGPIOx->MODER = 0x00000000;
     I2CPins.pGPIOx->OSPEEDR= 0x00000000;
     I2CPins.pGPIOx->OTYPER=0x00000000;
     I2CPins.pGPIOx->PUPDR=0x00000000;
     I2CPins.pGPIOx->AFRL[0]=0x00000000;
     I2CPins.pGPIOx->AFRL[1]=0x00000000;
     //SCL
     I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
     GPIO_Init(&I2CPins);

     //SDA
     I2CPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_9;
     GPIO_Init(&I2CPins);


}

void I2C1_Inits(void){

	I2C1Handle.pI2Cx=I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl =I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_DeviceAddress=MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;

	I2C_Init(&I2C1Handle);

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
	GPIOBtn.pGPIOx->PUPDR=0x00000000;
	GPIOBtn.pGPIOx->AFRL[0]=0x00000000;
	GPIOBtn.pGPIOx->AFRL[1]=0x00000000;


	GPIO_Init(&GPIOBtn);



}

int main(void){
	initialise_monitor_handles();

	printf("Application is running\n");
	uint8_t commandcode;
	uint8_t len;

	GPIO_ButtonInit();

	//i2c pin ints
	I2C1_PinsInit();

	//i2c peripheral config
	I2C1_Inits();

	//I2C IRQ config
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);


	//enable clock for i2c peripheral
	I2C_Peripheralcontrol(I2C1,ENABLE);

	//ACK bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1){

	//wait till button is pressed
	while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED);

	delay();//avoid button de-bouncing issues 200ms of delay

	commandcode = 0x51;
    while( I2C_MasterSendDataIT(&I2C1Handle, &commandcode,1, SlaveAddr,I2C_ENABLE_SR) != I2C_READY);

    while(I2C_MasterReceiveDataIT(&I2C1Handle,&len, 1, SlaveAddr,I2C_ENABLE_SR)!= I2C_READY);

    commandcode = 0x52;
    while( I2C_MasterSendDataIT(&I2C1Handle, &commandcode,1, SlaveAddr,I2C_ENABLE_SR) != I2C_READY);

    while(I2C_MasterReceiveDataIT(&I2C1Handle,rcv_buf, len, SlaveAddr,I2C_DISABLE_SR)!= I2C_READY){}

	rxcmplt = RESET;

    //wait till Rx completes
    while(rxcmplt != SET){}

    rcv_buf[len+1]='\0';

	printf("Data: %s",rcv_buf);

	rxcmplt = RESET;
	}


	return 0;
}


void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&I2C1Handle);

}
void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle,uint8_t AppEvent){
	if(AppEvent == I2C_EV_TX_CMPLT){
		printf("Tx is completed\n");
	}
	else if(AppEvent == I2C_EV_RX_CMPLT){
		printf("Rx is completed\n");

		rxcmplt =SET;
	}
	else if(AppEvent == I2C_ERROR_AF){
		printf("ERROR: ACK failure\n");
		//in master ack failure happens when slave fails to send ack for the byte
	    I2C_CloseSendData(pHandle);

	    //generate stop condition to release the bus
	    I2C_GenerateStopCondition(I2C1);

	    //Hang in infinite loop
	    while(1);
	}
}

