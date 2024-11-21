/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Nov 20, 2024
 *      Author: keerthika.m
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_


#include "stm32f411xx.h"




/***** CONFIGURATION STRUCTURE FOR SPI PIN ******/
typedef struct{
	uint8_t SPI_DeviceMode;  /* possible values from @SPI_DeviceMode */
	uint8_t SPI_BusConfig;   /* possible values from @SPI_BusConfig */
	uint8_t SPI_SClkSpeed;   /* possible values from @SPI_SClkSpeed */
	uint8_t SPI_DFF;         /* possible values from @SPI_DFF */
	uint8_t SPI_CPHA;        /* possible values from @SPI_CPHA */
	uint8_t SPI_CPOL;        /* possible values from @SPI_CPOL */
	uint8_t SPI_SSM;         /* possible values from @SPI_SSM */
}SPI_Config_t;


/* Handle structure for SPI */
typedef struct{
	SPI_RegDef_t *pSPIx;      // pointer to hold base address of SPIx
	SPI_Config_t SPIConfig;   // holds SPI pin configuration
}SPI_Handle_t;


/*
@SPI_DeviceMode
*/
#define SPI_DEVICE_MODE_MASTER    1
#define SPI_DEVICE_MODE_SLAVE     0


/*
 @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD                  1
#define SPI_BUS_CONFIG_HD                  2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY      3

/*
 @SPI_SClkSpeed
 */
#define SPI_SCLK_SPEED_DIV2         0
#define SPI_SCLK_SPEED_DIV4         1
#define SPI_SCLK_SPEED_DIV8         2
#define SPI_SCLK_SPEED_DIV16        3
#define SPI_SCLK_SPEED_DIV32        4
#define SPI_SCLK_SPEED_DIV64        5
#define SPI_SCLK_SPEED_DIV128       6
#define SPI_SCLK_SPEED_DIV256       7

/*
 @SPI_DFF
 */
#define SPI_DFF_8BITS      0
#define SPI_DFF_16BITS     1

/*
 @SPI_CPHA
 */
#define SPI_CPHA_LOW    0
#define SPI_CPHA_HIGH   1

/*
 @SPI_CPOL
 */
#define SPI_CPOL_LOW    0
#define SPI_CPOL_HIGH   1

/*
 @SPI_SSM
 */
#define SPI_SSM_EI     1
#define SPI_SSM_DI     0

/*
 SPI Status Flags definitions
 */
#define SPI_TXE_FLAG       (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG      (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG      (1 << SPI_SR_BSY)
#define SPI_CHSIDE_FLAG    (1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG       (1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG    (1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG      (1 << SPI_SR_MODF)
#define SPI_OVR_FLAG       (1 << SPI_SR_OVR)
#define SPI_BSY_FLAG       (1 << SPI_SR_BSY)
#define SPI_FRE_FLAG       (1 << SPI_SR_FRE)


/* APIs SUPPORTED BY THIS DRIVER  */

/* Peripheral Clock setup*/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi); //first parameter point to base address of SPIOx register
                                                                  //second parameter variable to hold enable or disable clock value


/* Init and DeInit  */

void SPI_Init(SPI_Handle_t *pSPIHandle);   //parameter point to handle structure

void SPI_DeInit(SPI_RegDef_t *pSPIx);      //parameter point to base address of SPIOx register



/* Data Send and Receive  */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);



/* IRQ CONFIGURATION AND ISR HANDLING */

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDis);//In IRQconfig need IRQ number,enable or disable

void SPI_IRQPriority(uint8_t IRQNumber,uint32_t IRQPriority);//priority

void SPI_IRQHandling(SPI_Handle_t *pHandle);


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
