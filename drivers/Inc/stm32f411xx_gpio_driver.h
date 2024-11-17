/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Nov 13, 2024
 *      Author: keerthika.m
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"




/*CONFIGURATION STRUCTURE FOR GPIO PIN*/
typedef struct{
	uint8_t GPIO_PinNumber; /* possible values from @GPIO_PIN_NO*/
	uint8_t GPIO_PinMode;  /* possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed; /* possible values from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl; /* possible values from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType; /* possible values from @GPIO_PIN_OUT_TYPE*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/* Handle structure for GPIO */

typedef struct{
	GPIO_RegDef_t *pGPIOx; // pointer to hold base address of GPIO peripheral registers
	GPIO_PinConfig_t GPIO_PinConfig; // holds GPIO pin configuration settings
}GPIO_Handle_t;


/*@GPIO_PIN_NO*/
/* GPIO PIN NUMBERS*/
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15





/*@GPIO_PIN_MODES*/
/* GPIO PIN POSSIBLE MODES*/
#define GPIO_MODE_IN         0
#define GPIO_MODE_OUT        1
#define GPIO_MODE_ALTFN      2
#define GPIO_MODE_ANALOG     3
#define GPIO_MODE_IT_RT      4
#define GPIO_MODE_IT_FT      5
#define GPIO_MODE_IT_RFT     6


/*@GPIO_PIN_OUT_TYPE*/
/* GPIO PIN POSSIBLE OUTPUT TYPES   */
#define GPIO_OP_TYPE_PP     0
#define GPIO_OP_TYPE_OD     1


/*@GPIO_PIN_SPEED*/
/* GPIO PIN POSSIBLE OUTPUT SPEEDS  */
#define GPIO_SPEED_LOW       0
#define GPIO_SPEED_MEDIUM    1
#define GPIO_SPEED_FAST      2
#define GPIO_SPEED_HIGH      3


/*@GPIO_PIN_PUPD*/
/* GPIO PULL UP AND PULL DOWN CONFIGURATION MACROS  */
#define GPIO_NO_PUPD     0
#define GPIO_PU          1
#define GPIO_PD          2



/* APIs SUPPORTED BY THIS DRIVER  */

/* Peripheral Clock setup*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi); //first parameter point to base address of GPIOx register
                                                                  //second parameter variable to hold enable or disable clock value

/* Init and DeInit  */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);   //parameter point to handle structure
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);      //parameter point to base address of GPIOx register

/* Data read and write  */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber); //first parameter point to base address of GPIOx register
                                                                        //second parameter variable to hold pin number

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);//first parameter point to base address of GPIOx register //uint16_t means port has 16bits

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value); //first parameter point to base address of GPIOx register
                                                                                    //second parameter variable to hold pin number
                                                                                    // third parameter to write value in specific pin

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);//first parameter point to base address of GPIOx register
                                                                   // second parameter to write value in specific pin

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);//first parameter point to base address of GPIOx register
                                                                   //second parameter variable to hold pin number

/* TRQ Configuration and ISR Handling */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDis);//In IRQconfig need IRQ number,priority,enable or disable

void GPIO_IRQHandling(uint8_t PinNumber);// Handling function will be called for specific pin number

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */

