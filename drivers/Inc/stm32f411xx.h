/*
 * stm32f411xx.h
 *
 *  Created on: Nov 13, 2024
 *      Author: keerthika.m
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stdint.h>

#define __vo volatile



/*********** PROCESSOR SPECIFIC DETAILS ****************/
//ARM CORTEX Mx Processor NVIC Interrupt Set Enable Register x register Addresses

#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)

//ARM CORTEX Mx Processor NVIC Interrupt Clear Enable Register x register Addresses

#define NVIC_ICER0          ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1          ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2          ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3          ((__vo uint32_t*)0xE000E18C)


/* ARM Cortex Mx Processor Priority Register Address Calculation */
#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED     4     //

//DEFINE BASE ADDRESSES OF MEMORY IN MCU
#define FLASH_BASEADDR       0x08000000U //base adress of flash memory or main memory
#define SRAM1_BASEADDR       0x20000000U //base address of SRAM1 if available mention SRAM2
#define ROM_BASEADDR         0x1FFF0000U //base address of System memory
#define SRAM                 SRAM1_BASEADDR



//DEFINE BASE ADDRESSES OF PERIPHERALS BUSES
#define PERIPHERAL_BASEADDR          0x40000000U
#define APB1PERIPHERAL_BASEADDR          PERIPHERAL_BASEADDR
#define APB2PERIPHERAL_BASEADDR          0x40010000U
#define AHB1PERIPHERAL_BASEADDR          0x40020000U
#define AHB2PERIPHERAL_BASEADDR          0x50000000U

//DEFINE BASE ADDRESSES OF PERIPHERAL CONNECTED TO AHB1 BUS
#define GPIOA_BASEADDR    (AHB1PERIPHERAL_BASEADDR+0x0000)
#define GPIOB_BASEADDR    (AHB1PERIPHERAL_BASEADDR+0x0400)
#define GPIOC_BASEADDR    (AHB1PERIPHERAL_BASEADDR+0x0800)
#define GPIOD_BASEADDR    (AHB1PERIPHERAL_BASEADDR+0x0C00)
#define GPIOE_BASEADDR    (AHB1PERIPHERAL_BASEADDR+0x1000)
#define GPIOH_BASEADDR    (AHB1PERIPHERAL_BASEADDR+0x1C00)
#define RCC_BASEADDR      (AHB1PERIPHERAL_BASEADDR+0x3800)

//DEFINE BASE ADDRESSES OF PERIPHERAL CONNECTED TO APB1 BUS
#define I2C1_BASEADDR       (APB1PERIPHERAL_BASEADDR+0X5400)
#define I2C2_BASEADDR       (APB1PERIPHERAL_BASEADDR+0X5800)
#define I2C3_BASEADDR       (APB1PERIPHERAL_BASEADDR+0X5C00)

#define SPI2_BASEADDR       (APB1PERIPHERAL_BASEADDR+0X3800)
#define SPI3_BASEADDR       (APB1PERIPHERAL_BASEADDR+0X3C00)

#define USART2_BASEADDR     (APB1PERIPHERAL_BASEADDR+0X4400)

//DEFINE BASE ADDRESSES OF PERIPHERAL CONNECTED TO APB2 BUS
#define SPI1_BASEADDR    (APB2PERIPHERAL_BASEADDR+0x3000)

#define USART1_BASEADDR  (APB2PERIPHERAL_BASEADDR+0x1000)
#define USART6_BASEADDR  (APB2PERIPHERAL_BASEADDR+0x1400)

#define EXTI_BASEADDR    (APB2PERIPHERAL_BASEADDR+0x3C00)

#define SYSCFG_BASEADDR  (APB2PERIPHERAL_BASEADDR+0x3800)


/* PERIPHERAL  REGISTER  DEFINITION STRUCTURES FOR GPIO */
typedef struct{
	__vo uint32_t MODER;   //port mode register offset:0x00
	__vo uint32_t OTYPER;  //output type register offset:0x04
	__vo uint32_t OSPEEDR; //..
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL[2];   //..every register in GPIO defined in structure for effecient access
	                   // refer REFERNCE MANUAL for name of GPIO registers.
}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVED4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t RESERVED7;
	__vo  uint32_t DCKCFGR;

}RCC_RegDef_t; // rcc controls clocks and here you can use this as registers


/* PERIPHERAL  REGISTER  DEFINITION STRUCTURES FOR EXTI */
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

/* PERIPHERAL  REGISTER  DEFINITION STRUCTURES FOR SYSCFG */
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*port selection for GPIOx*/
#define GPIO_BASEADDR_TO_CODE(x)    ((x==GPIOA)? 0:\
		                            (x==GPIOB)? 1:\
		                            (x==GPIOC)? 2:\
		                            (x==GPIOD)? 3:\
		                            (x==GPIOE)? 4:\
		                            (x==GPIOH)? 7:0)

/*INTERRUPT REQUEST NUMBER*/
#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40
#define IRQ_NO_EXTI16     1
#define IRQ_NO_EXTI17     41
#define IRQ_NO_EXTI18     42
#define IRQ_NO_EXTI21     2
#define IRQ_NO_EXTI22     3

/* MACROS for all possible priority levels */
#define NVIC_IRQ_PRI0     0
#define NVIC_IRQ_PRI1     1
#define NVIC_IRQ_PRI2     2
#define NVIC_IRQ_PRI3     3
#define NVIC_IRQ_PRI4     4
#define NVIC_IRQ_PRI5     5
#define NVIC_IRQ_PRI6     6
#define NVIC_IRQ_PRI7     7
#define NVIC_IRQ_PRI8     8
#define NVIC_IRQ_PRI9     9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14
#define NVIC_IRQ_PRI15    15



/* PERIPHERAL DEFINITIONS (peripheral base addresses type casted to xxxx_RegDef_t ) */

#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR) //need this to enable and disable clock

#define EXTI   ((EXTI_RegDef_t*)EXTI_BASEADDR) // deliver interrupts from peripheral side

#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)// interrupt pin port is decided by SYSCFG

/* CLOCK ENABLE MACROS FOR GPIOX PERIPHERALS */
#define GPIOA_PCLK_EN()  (RCC -> AHB1ENR |= (1<<0)) // |= means setting bit (enable)
#define GPIOB_PCLK_EN()  (RCC -> AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()  (RCC -> AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()  (RCC -> AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()  (RCC -> AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()  (RCC -> AHB1ENR |= (1<<7))

/* CLOCK ENABLE MACROS FOR IRCX PERIPHERALS */
#define I2C1_PCLK_EN()    (RCC -> APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()    (RCC -> APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()    (RCC -> APB1ENR |= (1<<23))

/* CLOCK ENABLE MACROS FOR SPIX PERIPHERALS */
#define SPI2_PCLK_EN()    (RCC -> APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()    (RCC -> APB1ENR |= (1<<15))
#define SPI1_PCLK_EN()    (RCC -> APB2ENR |= (1<<12))

/* CLOCK ENABLE MACROS FOR USARTX PERIPHERALS */
#define USART2_PCLK_EN()    (RCC -> APB1ENR |= (1<<17))
#define USART1_PCLK_EN()    (RCC -> APB2ENR |= (1<<4))
#define USART6_PCLK_EN()    (RCC -> APB2ENR |= (1<<5))

/* CLOCK ENABLE MACROS FOR SYSCFG  PERIPHERALS */
#define SYSCFG_PCLK_EN()    (RCC -> APB2ENR |= (1<<14))




/* CLOCK DISABLE MACROS FOR GPIOX PERIPHERALS */
#define GPIOA_PCLK_DI()  (RCC -> AHB1ENR &= ~(1<<0)) // &= ~ means resetting bit (clear bit)
#define GPIOB_PCLK_DI()  (RCC -> AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()  (RCC -> AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()  (RCC -> AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()  (RCC -> AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()  (RCC -> AHB1ENR &= ~(1<<7))

/* CLOCK DISABLE MACROS FOR IRCX PERIPHERALS */
#define I2C1_PCLK_DI()    (RCC -> APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()    (RCC -> APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()    (RCC -> APB1ENR &= ~(1<<23))

/* CLOCK DISABLE MACROS FOR SPIX PERIPHERALS */
#define SPI2_PCLK_DI()    (RCC -> APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()    (RCC -> APB1ENR &= ~(1<<15))
#define SPI1_PCLK_DI()    (RCC -> APB2ENR &= ~(1<<12))

/* CLOCK DISABLE MACROS FOR USARTX PERIPHERALS */
#define USART2_PCLK_DI()    (RCC -> APB1ENR &= ~(1<<17))
#define USART1_PCLK_DI()    (RCC -> APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI()    (RCC -> APB2ENR &= ~(1<<5))

/* CLOCK DISABLE MACROS FOR SYSCFG  PERIPHERALS */
#define SYSCFG_PCLK_DI()    (RCC -> APB2ENR &= ~(1<<14))


/* Macros to reset GPIOX peripherals  */
#define GPIOA_REG_RESET()    do{(RCC -> AHB1ENR |= (1<<0)); (RCC -> AHB1ENR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()    do{(RCC -> AHB1ENR |= (1<<1)); (RCC -> AHB1ENR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()    do{(RCC -> AHB1ENR |= (1<<2)); (RCC -> AHB1ENR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()    do{(RCC -> AHB1ENR |= (1<<3)); (RCC -> AHB1ENR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()    do{(RCC -> AHB1ENR |= (1<<4)); (RCC -> AHB1ENR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()    do{(RCC -> AHB1ENR |= (1<<7)); (RCC -> AHB1ENR &= ~(1<<7));}while(0)

//some generic macros

#define ENABLE               1
#define DISABLE              0
#define SET                ENABLE
#define RESET              DISABLE
#define GPIO_PINSET         SET
#define GPIO_PINRESET       RESET

#include"stm32f411xx_gpio_driver.h"

#endif /* INC_STM32F411XX_H_ */