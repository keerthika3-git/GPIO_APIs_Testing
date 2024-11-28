/*
 * 010.i2c_master_rx_testing.c
 *
 *  Created on: Nov 28, 2024
 *      Author: keerthika.m
 */


/*
 * PB6 -> SCL   Pin name: D10
 * PB9 -> SDA   pin name: D14
 */
#include "stm32f411xx.h"
#include "string.h"

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
//	GpioLed.pGPIOx->OTYPER=0x00000000;
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

	//enable clock for i2c peripheral
	I2C_Peripheralcontrol(I2C1,ENABLE);

	//ACK bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1){

	//wait till button is pressed
	while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)== BTN_PRESSED);

	delay();//avoid button de-bouncing issues 200ms of delay

	commandcode = 0x51;
    I2C_MasterSendData(&I2C1Handle, &commandcode,1, SlaveAddr);

    I2C_MasterReceiveData(&I2C1Handle,&len, 1, SlaveAddr);

    commandcode = 0x52;
    I2C_MasterSendData(&I2C1Handle, &commandcode,1, SlaveAddr);

    I2C_MasterReceiveData(&I2C1Handle,rcv_buf, len, SlaveAddr);

    rcv_buf[len+1]='\0';

	printf("Data: %s",rcv_buf);
	}


	return 0;
}
