/*
 * 015uart_case.c
 *
 *  Created on: Dec 11, 2024
 *      Author: keerthika.m
 */

#include<stdio.h>
#include<string.h>
#include "stm32f411xx.h"

extern void initialise_monitor_handles(void);

#define LOW 0
#define BTN_PRESSED LOW

//we have 3 different messages that we transmit to arduino
//you can by all means add more messages
char *msg[3] = {"hihihihihihi123", "Hello How are you ?" , "Today is Monday !"};

//reply from arduino will be stored here
char rx_buf[1024] ;

USART_Handle_t usart2_handle;


//This flag indicates reception completion
uint8_t rxCmplt = RESET;

uint8_t g_data = 0;



void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART1;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOB;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	usart_gpios.pGPIOx->MODER = 0x00000000;
	usart_gpios.pGPIOx->OSPEEDR= 0x00000000;
	usart_gpios.pGPIOx->OTYPER=0x00000000;
	usart_gpios.pGPIOx->PUPDR=0x00000000;
	usart_gpios.pGPIOx->AFRL[0]=0x00000000;
	usart_gpios.pGPIOx->AFRL[1]=0x00000000;

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_6; // pin no d10
	GPIO_Init(&usart_gpios);

	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;// In CN7(21 pin)
	GPIO_Init(&usart_gpios);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE);
	GPIOBtn.pGPIOx->MODER = 0x00000000;
	GPIOBtn.pGPIOx->OSPEEDR= 0x00000000;
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

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main(void)
{
	initialise_monitor_handles();

    printf("Application is running\n");
	uint32_t cnt = 0;

	GPIO_ButtonInit();
	USART2_GPIOInit();
    USART2_Init();

    USART_IRQInterruptConfig(IRQ_NO_USART1,ENABLE);

    USART_Peripheralcontrol(USART1,ENABLE);

    printf("All Peripheral and GPIO initializations are done\n");

    //do forever
    while(1)
    {
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) == BTN_PRESSED );

		printf("Communication Started....\n");
		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		// Next message index ; make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		//First lets enable the reception in interrupt mode
		//this code enables the receive interrupt
		while ( USART_ReceiveDataIT(&usart2_handle,rx_buf,strlen(msg[cnt])) != USART_READY );

		//Send the msg indexed by cnt in blocking mode
    	USART_SendData(&usart2_handle,(uint8_t*)msg[cnt],strlen(msg[cnt]));

    	printf("Transmitted : %s\n",msg[cnt]);


    	//Now lets wait until all the bytes are received from the arduino .
    	//When all the bytes are received rxCmplt will be SET in application callback
    	while(rxCmplt != SET);

    	//just make sure that last byte should be null otherwise %s fails while printing
    	rx_buf[strlen(msg[cnt])+ 1] = '\0';

    	//Print what we received from the arduino
    	printf("Received    : %s\n",rx_buf);

    	//invalidate the flag
    	rxCmplt = RESET;

    	//move on to next message indexed in msg[]
    	cnt ++;
    }


	return 0;
}


void USART1_IRQHandler(void)
{
	USART_IRQHandling(&usart2_handle);
}





void USART_ApplicationEventCallback( USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}
