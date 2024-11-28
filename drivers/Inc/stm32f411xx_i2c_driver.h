/*
 * stm32f411xx_i2c_driver.h
 *
 *  Created on: Nov 27, 2024
 *      Author: keerthika.m
 */

#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_

#include "stm32f411xx.h"


/***** CONFIGURATION STRUCTURE FOR I2C PIN ******/
typedef struct{
	uint32_t I2C_SCLSpeed;       /* possible values from @I2C_SCLSpeed */
	uint8_t I2C_DeviceAddress;   //Defined by user
	uint8_t I2C_ACKControl;      /* possible values from @I2C_ACKControl */
	uint8_t I2C_FMDutyCycle;    /* possible values from @I2C_FMDutyCycle */

}I2C_Config_t;


/* Handle structure for I2C */
typedef struct{
	I2C_RegDef_t *pI2Cx;       // pointer to hold base address of I2Cx
	I2C_Config_t  I2C_Config;   // holds I2C pin configuration
 }I2C_Handle_t;


 /*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM    100000    //100kHz
#define I2C_SCL_SPEED_FM4K  400000    //400kHz
#define I2C_SCL_SPEED_FM2K  200000    //200kHz


 /*
  * @I2C_ACKControl
  */
#define I2C_ACK_ENABLE       1
#define I2C_ACK_DISABLE      0

/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE    (1<<I2C_SR1_TXE)
#define I2C_FLAG_RXNE   (1<<I2C_SR1_RXNE)
#define I2C_FLAG_ADDR   (1<<I2C_SR1_ADDR)
#define I2C_FLAG_BTF    (1<<I2C_SR1_BTF)
#define I2C_FLAG_STOPF  (1<<I2C_SR1_STOPF)
#define I2C_FLAG_ADD10  (1<<I2C_SR1_ADD10)
#define I2C_FLAG_AF     (1<<I2C_SR1_AF)
#define I2C_FLAG_BERR   (1<<I2C_SR1_BERR)
#define I2C_FLAG_ARLO   (1<<I2C_SR1_ARLO)
#define I2C_FLAG_OVR    (1<<I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT (1<<I2C_SR1_TIMEOUT)
#define I2C_FLAG_SB      (1<<I2C_SR1_SB)


 /* APIs SUPPORTED BY THIS DRIVER  */

 /* Peripheral Clock setup*/

 void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi); //first parameter point to base address of I2COx register
                                                                   //second parameter variable to hold enable or disable clock value


 /* Init and DeInit  */

 void I2C_Init(I2C_Handle_t *pI2CHandle);   //parameter point to handle structure

 void I2C_DeInit(I2C_RegDef_t *pI2Cx);      //parameter point to base address of I2COx register



 /* Data Send and Receive  */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t Len,uint8_t SlaveAddr);



 /* IRQ CONFIGURATION AND ISR HANDLING */

 void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDis);//In IRQconfig need IRQ number,enable or disable

 void I2C_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority);//priority



 /*
  * Peripheral Control APIs
  */
void I2C_Peripheralcontrol(I2C_RegDef_t *pI2Cx, uint8_t EnorDi); //to enable I2C Peripheral

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);



 /*
  * Application callback
  */
 void I2C_ApplicationEventCallback(I2C_Handle_t *pHandle,uint8_t AppEvent);














#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */
