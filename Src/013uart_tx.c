
#include<stdio.h>
#include<string.h>
#include "stm32f411xx.h"

extern void initialise_monitor_handles(void);

#define LOW 0
#define BTN_PRESSED LOW

char msg[1024] = "UART Tx testing\n\r";

USART_Handle_t usart2_handle;


void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART1;
	usart2_handle.USART_Config.USART_Baud =USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
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

	//USART1 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_6; // D10(CN9)
	GPIO_Init(&usart_gpios);

	//USART1 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7; // 21 pin(CN7)
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



	GPIO_PeriClockControl(GPIOC,ENABLE);

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

	GpioLed.pGPIOx->MODER = 0x00000000;
	GpioLed.pGPIOx->OSPEEDR= 0x00000000;
	GpioLed.pGPIOx->OTYPER=0x00000000;
	GpioLed.pGPIOx->PUPDR=0x00000000;
	GpioLed.pGPIOx->AFRL[0]=0x00000000;
	GpioLed.pGPIOx->AFRL[1]=0x00000000;

	GPIO_Init(&GpioLed);


}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

//	initialise_monitor_handles();
//
//	printf("Application is running\n");

	GPIO_ButtonInit();

	USART2_GPIOInit();

    USART2_Init();

    USART_Peripheralcontrol(USART1,ENABLE);

    while(1)
    {
		//wait till button is pressed
    	while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));
//		printf("Transmitted : %s\n",msg);

    }

	return 0;
}
