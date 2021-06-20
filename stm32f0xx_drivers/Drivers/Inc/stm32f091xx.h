/*
 * stm32f091xx..h
 *
 *  Created on: Jun 5, 2021
 *      Author: Badri Ramesh
 *
 *  Device(MCU) specific header file (Driver level) for STM32F091xx containing MCU specific data.
 *  Contains base addressses of various memories, various bus domains, for various peripherals (GPIO,I2C,SPI,USART), and the required peripheral register definition structures
 *  Clock enable and disable macros.
 *
 *  TO DO: IRQ macros
 */

#ifndef INC_STM32F091XX_H_
#define INC_STM32F091XX_H_

#include <stdint.h>

#define __vol volatile

/**************************************************START : Processor Specific details************************************************/

/*ARM Cortex M0 Processor NVIC ICER & ISER register base addresses*/

#define NVIC_ISER				((__vol uint32_t *)0xE000E100)
#define NVIC_ICER				((__vol uint32_t *)0xE000E180)

/*ARM Cortex M0 Processor priority register base addresses*/

#define NVIC_IPR_BASEADDR		((__vol uint32_t *)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED	2


/**************************************************START : MCU Specific details************************************************/

/*Macros for base addresses of memory domains like SRAM & FLASH*/

#define DRV_FLASH_BASEADDR 			0x08000000U             //aka MAIN FLASH MEMORY
#define DRV_SRAM1_BASEADDR			0x20000000U			    //aka SRAM
//#define DRV_SRAM2_BASEADDR

#define DRV_SYSTEM_BASEADDR			0x1FFFD800U			    //aka ROM
#define DRV_SRAM 					DRV_SRAM1_BASEADDR
#define DRV_ROM 					DRV_SYSTEM_BASEADDR  		/* explain these macors breifly while documentation*/


/*Macros for Peripheral bus domains (OR) AHBx & APBx Bus peripheral base addresses*/         //AHB1,AHB2,APB (APB1 OR APB2)

#define DRV_PERIPH_BASEADDR			0x40000000U
#define DRV_APBPERIPH_BASEADDR		DRV_PERIPH_BASEADDR
#define DRV_AHB1PERIPH_BASEADDR		0x40020000U
#define DRV_AHB2PERIPH_BASEADDR		0x48000000U


/*Base addresses of peripherals which are hanging on AHB1 bus
 * TO DO: Complete for all other peripherals
 */

#define DRV_RCC_BASEADDR 			(DRV_AHB1PERIPH_BASEADDR + 0x1000)


/*Base addresses of peripherals which are hanging on AHB2 bus
 * TO DO: Complete for all other peripherals
 */

#define DRV_GPIOA_BASEADDR			(DRV_AHB2PERIPH_BASEADDR + 0x0000)          //0x48000000U
#define DRV_GPIOB_BASEADDR			(DRV_AHB2PERIPH_BASEADDR + 0x0400)			//0x48000400U
#define DRV_GPIOC_BASEADDR			(DRV_AHB2PERIPH_BASEADDR + 0x0800)			//0x48000800U
#define DRV_GPIOD_BASEADDR			(DRV_AHB2PERIPH_BASEADDR + 0x0C00)			//0x48000C00U
#define DRV_GPIOE_BASEADDR			(DRV_AHB2PERIPH_BASEADDR + 0x1000)			//0x48001000U
#define DRV_GPIOF_BASEADDR			(DRV_AHB2PERIPH_BASEADDR + 0x1400)			//0x48001400U

//you can also define like #define GPIOA_BASEADDR    (AHB2PERIPH_BASEADDR + OFFSET(0x0000))
//you can also define like #define GPIOB_BASEADDR    (AHB2PERIPH_BASEADDR + OFFSET(0x0400))


/*Base addresses of peripherals which are hanging on APB bus
 * TO DO: Complete for all other peripherals
 */

#define DRV_USART1_BASEADDR			0x40013800U									//(DRV_APBPERIPH_BASEADDR + 0x0000)	0x40013800U
#define DRV_USART2_BASEADDR			(DRV_APBPERIPH_BASEADDR + 0x4400)			//0x40004400U
#define DRV_USART3_BASEADDR			(DRV_APBPERIPH_BASEADDR + 0x4800)			//0x40004800U
#define DRV_USART4_BASEADDR			(DRV_APBPERIPH_BASEADDR + 0x4C00)			//0x40004C00U
#define DRV_USART5_BASEADDR			(DRV_APBPERIPH_BASEADDR + 0x5000)			//0x40005000U
#define DRV_USART6_BASEADDR			0x40011400U									//(DRV_APBPERIPH_BASEADDR + 0x0000) 0x40011400U
#define DRV_USART7_BASEADDR			0x40011800U									//(DRV_APBPERIPH_BASEADDR + 0x0000) 0x40011800U
#define DRV_USART8_BASEADDR			0x40011C00U									//(DRV_APBPERIPH_BASEADDR + 0x0000) 0x40011C00U

#define DRV_I2C1_BASEADDR			(DRV_APBPERIPH_BASEADDR + 0x5400)			//0x40005400U
#define DRV_I2C2_BASEADDR			(DRV_APBPERIPH_BASEADDR + 0x5800)			//0x40005800U

#define DRV_SYSCFG_BASEADDR			0x40010000U									//(DRV_APBPERIPH_BASEADDR + 0x0000)	0x40010000U

#define DRV_SPI1_BASEADDR			0x40013000U									//(DRV_APBPERIPH_BASEADDR + 0x0000) 0x40013000U
#define DRV_SPI2_BASEADDR			(DRV_APBPERIPH_BASEADDR + 0x3800)			//0x40003800U

#define DRV_EXTI_BASEADDR			0x40010400U									//(DRV_APBPERIPH_BASEADDR + 0x0000) 0x40010400U

#define DRV_SPI1					I2S1


/*******************************Creating structures for peripheral registers************************************************/
//Peripheral register definition structures
//Note : No of register for each peripheral are specific to MCU. It may vary so please check RM.
/*structure for GPIOx peripheral */


typedef struct{
	uint32_t __vol MODER;                     	/*!< Give a short description,               Addr offset: 0x00 */  //i.e address of MODER will be 0x48000000U for GPIOA
	uint32_t __vol OTYPER;						/*!< Give a short description,               Addr offset: 0x04 */  //address of OTYPER will be 0x48000000U + 0x04, since mem allocated for uint32_t is 4 bytes
	uint32_t __vol OSPEEDR;						/*!< Give a short description,               Addr offset: 0x08 */
	uint32_t __vol PUPDR;							/*!< Give a short description,               Addr offset: 0x0C */
	uint32_t __vol IDR;							/*!< Give a short description,               Addr offset: 0x10 */
	uint32_t __vol ODR;							/*!< Give a short description,               Addr offset: 0x14 */
	uint32_t __vol BSRR;							/*!< Give a short description,               Addr offset: 0x18 */
	uint32_t __vol LCKR;							/*!< Give a short description,               Addr offset: 0x1A */
	uint32_t __vol AFR[2];						/*!< Give a short description,               Addr offset: 0x1A */
	//uint32_t AFRL;						/*!< Give a short description,               Addr offset: 0x1C */
	//uint32_t AFRH;						/*!< Give a short description,               Addr offset: 0x20 */
	uint32_t __vol BRR;							/*!< Give a short description,               Addr offset: 0x24 */
}GPIO_RegDef_t;

//we can create a pointer GPIO_RegDef_t *pGPIOA = (GPIO_RegDef_t *) DRV_GPIOA_BASEADDR;
//we can modify values by pGPIOA->MODER = 25;
//alternate : #define GPIOA  ((GPIO_RegDef_t *)DRV_GPIOA_BASEADDR)


/*structure for RCC peripheral */

typedef struct{
	uint32_t __vol RCC_CR;
	uint32_t __vol RCC_CFGR;
	uint32_t __vol RCC_CIR;
	uint32_t __vol RCC_APB2RSTR;
	uint32_t __vol RCC_APB1RSTR;
	uint32_t __vol RCC_AHBENR;
	uint32_t __vol RCC_APB2ENR;
	uint32_t __vol RCC_APB1ENR;
	uint32_t __vol RCC_BDCR;
	uint32_t __vol RCC_CSR;
	uint32_t __vol RCC_AHBRSTR;
	uint32_t __vol RCC_CFGR2;
	uint32_t __vol RCC_CFGR3;
	uint32_t __vol RCC_CR2;
}RCC_RegDef_t;


//Look at register map of RCC peripheral because there might be some reserved locations in between.

/*structure for SPIx peripheral */

typedef struct{
	uint32_t __vol SPI_CR1;
	uint32_t __vol SPI_CR2;
	uint32_t __vol SPI_SR;
	uint32_t __vol SPI_DR;
	uint32_t __vol SPI_CRCPR;
	uint32_t __vol SPI_RXCRCR;
	uint32_t __vol SPI_TXCRCR;
	uint32_t __vol SPI_I2SCFGR;
	uint32_t __vol SPI_I2SPR;
}SPI_RegDef_t;

/*structure for USARTx peripheral */

typedef struct{
	uint32_t __vol USART_CR1;
	uint32_t __vol USART_CR2;
	uint32_t __vol USART_CR3;
	uint32_t __vol USART_BRR;
	uint32_t __vol USART_GTPR;
	uint32_t __vol USART_RTOR;
	uint32_t __vol USART_RQR;
	uint32_t __vol USART_ISR;
	uint32_t __vol USART_ICR;
	uint32_t __vol USART_RDR;
	uint32_t __vol USART_TDR;
}USART_RegDef_t;

/*structure for I2Cx peripheral */

typedef struct{
	uint32_t __vol I2C_CR1;
	uint32_t __vol I2C_CR2;
	uint32_t __vol I2C_OAR1;
	uint32_t __vol I2C_OAR2;
	uint32_t __vol I2C_TIMINGR;
	uint32_t __vol I2C_TIMEOUTR;
	uint32_t __vol I2C_ISR;
	uint32_t __vol I2C_ICR;
	uint32_t __vol I2C_PECR;
	uint32_t __vol I2C_RXDR;
	uint32_t __vol I2C_TXDR;
}I2C_RegDef_t;

/*peripheral register structure definition for EXTI  */

typedef struct{
	uint32_t __vol EXTI_IMR;
	uint32_t __vol EXTI_EMR;
	uint32_t __vol EXTI_RTSR;
	uint32_t __vol EXTI_FTSR;
	uint32_t __vol EXTI_SWIER;
	uint32_t __vol EXTI_PR;
}EXTI_RegDef_t;

/*peripheral register structure definition for SYSCFG peripheral */

typedef struct{
	uint32_t __vol SYSCFG_CFGR1;
	uint32_t RESERVED0;
	uint32_t __vol SYSCFG_EXTICR[4];
	uint32_t __vol SYSCFG_CFGR2;
	uint8_t RESERVED1;
	uint32_t RESERVED2[18];      //check the register map !!!!
	uint32_t __vol SYSCFG_ITLINE[30];
}SYSCFG_RegDef_t;


/*
 * peripheral definitions for peripherals like GPIO,SPI,I2C,USART,CLOCK
 */

#define GPIOA 	((GPIO_RegDef_t *)DRV_GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t *)DRV_GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t *)DRV_GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t *)DRV_GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t *)DRV_GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t *)DRV_GPIOF_BASEADDR)

#define RCC 	((RCC_RegDef_t *)DRV_RCC_BASEADDR)
#define EXTI	((EXTI_RegDef_t *)DRV_EXTI_BASEADDR)    //EXTI peripheral defenition
#define SYSCFG  ((SYSCFG_RegDef_t *)DRV_SYSCFG_BASEADDR)
/*
 * Clock ENABLE macros for GPIOx peripheral
 */

#define GPIOA_PCLK_EN()	    (RCC->RCC_AHBENR |= (0x1 << 17))
#define GPIOB_PCLK_EN()		(RCC->RCC_AHBENR |= (0x1 << 18))
#define GPIOC_PCLK_EN()		(RCC->RCC_AHBENR |= (0x1 << 19))
#define GPIOD_PCLK_EN()		(RCC->RCC_AHBENR |= (0x1 << 20))
#define GPIOE_PCLK_EN()		(RCC->RCC_AHBENR |= (0x1 << 21))
#define GPIOF_PCLK_EN()		(RCC->RCC_AHBENR |= (0x1 << 22))


/*
 * Clock ENABLE macros for I2Cx peripheral
 */


#define I2C1_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x1 << 21))
#define I2C2_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x1 << 22))

/*
 * Clock ENABLE macros for SPIx peripheral
 */

#define SPI1_PCLK_EN()		(RCC->RCC_APB2ENR |= (0x1 << 12))
#define SPI2_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x1 << 14))

/*
 * Clock ENABLE macros for USARTx peripheral
 */

#define USART1_PCLK_EN()	(RCC->RCC_APB2ENR |= (0x1 << 14))
#define USART2_PCLK_EN()	(RCC->RCC_APB1ENR |= (0x1 << 17))
#define USART3_PCLK_EN()	(RCC->RCC_APB1ENR |= (0x1 << 18))
#define USART4_PCLK_EN()	(RCC->RCC_APB1ENR |= (0x1 << 19))
#define USART5_PCLK_EN()	(RCC->RCC_APB1ENR |= (0x1 << 20))
#define USART6_PCLK_EN()	(RCC->RCC_APB2ENR |= (0x1 << 5))
#define USART7_PCLK_EN()	(RCC->RCC_APB2ENR |= (0x1 << 6))
#define USART8_PCLK_EN()	(RCC->RCC_APB2ENR |= (0x1 << 7))


/*
 * Clock ENABLE macros for SYS_CFG peripheral
 */

#define SYSCFG_PCLK_EN()	(RCC->RCC_APB2ENR |= (0x1 << 0))


/*
 * Clock DISABLE macros for GPIOx peripheral
 */


#define GPIOA_PCLK_DI()	    (RCC->RCC_AHBENR &= ~(0x1 << 17))
#define GPIOB_PCLK_DI()		(RCC->RCC_AHBENR &= ~(0x1 << 18))
#define GPIOC_PCLK_DI()		(RCC->RCC_AHBENR &= ~(0x1 << 19))
#define GPIOD_PCLK_DI()		(RCC->RCC_AHBENR &= ~(0x1 << 20))
#define GPIOE_PCLK_DI()		(RCC->RCC_AHBENR &= ~(0x1 << 21))
#define GPIOF_PCLK_DI()		(RCC->RCC_AHBENR &= ~(0x1 << 22))


/*
 * Clock DISABLE macros for I2Cx peripheral
 */

#define I2C1_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(0x1 << 21))
#define I2C2_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(0x1 << 22))


/*
 * Clock DISABLE macros for SPIx peripheral
 */

#define SPI1_PCLK_DI()		(RCC->RCC_APB2ENR &= ~(0x1 << 12))
#define SPI2_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(0x1 << 14))


/*
 * Clock DISABLE macros for USARTx peripheral
 */

#define USART1_PCLK_DI()	(RCC->RCC_APB2ENR &= ~(0x1 << 14))
#define USART2_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(0x1 << 17))
#define USART3_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(0x1 << 18))
#define USART4_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(0x1 << 19))
#define USART5_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(0x1 << 20))
#define USART6_PCLK_DI()	(RCC->RCC_APB2ENR &= ~(0x1 << 5))
#define USART7_PCLK_DI()	(RCC->RCC_APB2ENR &= ~(0x1 << 6))
#define USART8_PCLK_DI()	(RCC->RCC_APB2ENR &= ~(0x1 << 7))


/*
 * Clock DISABLE macros for SYS_CFG peripheral
 */

#define SYSCFG_PCLK_DI()	(RCC->RCC_APB2ENR &= ~(0x1 << 0))


/*
 * Macros to RESET GPIOx peripheral
 */

#define GPIOA_REG_RESET()	do { (RCC->RCC_AHBRSTR |= (1 << 17));  (RCC->RCC_AHBRSTR &= ~(1 << 17));} while(0)
#define GPIOB_REG_RESET()	do { (RCC->RCC_AHBRSTR |= (1 << 18));  (RCC->RCC_AHBRSTR &= ~(1 << 18));} while(0)
#define GPIOC_REG_RESET()	do { (RCC->RCC_AHBRSTR |= (1 << 19));  (RCC->RCC_AHBRSTR &= ~(1 << 19));} while(0)
#define GPIOD_REG_RESET()	do { (RCC->RCC_AHBRSTR |= (1 << 20));  (RCC->RCC_AHBRSTR &= ~(1 << 20));} while(0)
#define GPIOE_REG_RESET()	do { (RCC->RCC_AHBRSTR |= (1 << 21));  (RCC->RCC_AHBRSTR &= ~(1 << 21));} while(0)
#define GPIOF_REG_RESET()	do { (RCC->RCC_AHBRSTR |= (1 << 22));  (RCC->RCC_AHBRSTR &= ~(1 << 22));} while(0)


/*
 * Macros - returns a code (b/w 0 to 7) for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x)   ( (x == GPIOA) ? 0 : \
									 (x == GPIOB) ? 1 : \
				                     (x == GPIOC) ? 2 : \
				                     (x == GPIOD) ? 3 : \
				                     (x == GPIOE) ? 4 : \
									 (x == GPIOF) ? 5 : 0)


/*
 * Macros for IRQ numbers of STM32F091xx MCU
 * TODO : complete for other peripherals as well
 *
 */

#define IRQ_NUM_EXTI0_1		5
#define IRQ_NUM_EXTI2_3		6
#define IRQ_NUM_EXTI4_15	7
/*
 * Some Generic Macros
 */

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#include "stm32f091xx_gpio_driver.h"





#endif /* INC_STM32F091XX_H_ */

