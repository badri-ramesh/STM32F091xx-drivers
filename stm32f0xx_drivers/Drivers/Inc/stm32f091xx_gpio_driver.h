/*
 * stm32f091xx_gpio_driver.h
 *
 *  Created on: 06-Jun-2021
 *      Author: Badri Ramesh
 *
 *  The driver layer should give a configuration structure to the user application.
 *  Contains driver specific data (driver APIs for the user application) i.e GPIO handle and configuration structures for working with the GPIO peripheral
 *  Also has the prototypes of the driver APIs which are supported by this driver file based on requirements. Prototypes are defined first. Prototype parameters are defined next.
 */

#ifndef INC_STM32F091XX_GPIO_DRIVER_H_
#define INC_STM32F091XX_GPIO_DRIVER_H_

#include "stm32f091xx.h" 					/*Include MCU specific header file*/

/*
 * This is a configuration structure for a GPIO pin. Contains the configurable items as member elements
 */

typedef struct{
	uint8_t GPIO_PinNumber;				/*!<Possible values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;				/*!<Possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;				/*!<Possible values from @GPIO_PIN_SPEED>*/
	uint8_t GPIO_PinPuPdControl;		/*!<Possible values from @GPIO_PULLUP_PULLDOWN>*/
	uint8_t GPIO_PinOPType;				/*!<Possible values from @GPIO_PIN_OP_TYPE>*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * This is a handle structure for a GPIO pin
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx; 				/*!x<pointer to hold the base address of the GPIO peripheral  OR this holds the base address of the GPIO port to which the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!<This holds th GPIO pin configuration settings>*/
}GPIO_Handle_t;


/**********************************************************************************************************************************
 * 													APIs supported by this driver
 * 								For more information about implementation of the APIs check the corr. function definitions
 *
**********************************************************************************************************************************/


/*Peri clock setup*/

void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);						//To enable or disable the peripheral clock

/*Init and DeInit*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);                              		//To initialize the GPIO port & pin. Initially parameters are not considered. Then as we implement they are defined.
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);							  			//Change GPIO settings to reset state or reset value collectively of a peripheral

/*Data Read & Write*/

uint8_t GPIO_ReadFrom_InputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);   	//since return will be either 0 or 1->uint8_t
uint16_t GPIO_ReadFrom_InputPort(GPIO_RegDef_t *pGPIOx);						//each port is 16pins so return type is uint16_t. This will return the content of input data register
void GPIO_WriteInto_OutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteInto_OutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
uint8_t GPIO_ReadFrom_OutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);   	//since return will be either 0 or 1->uint8_t
uint16_t GPIO_ReadFrom_OutputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ configuration and ISR handling*/

void GPIO_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t ENorDI);	//TODO: IRQ grouping; For interrupts; to configure the IRQ number of the GPIO pin. Like EN, setting priority
void GPIO_IRQPriority_Config(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);										//process interrupts; since IRQ handling functions should know from which pin the interrupt is triggerd.


/*******************************************************peripheral specific macros*******************************************************/

//Written seeing the registers

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers possible
 */

#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15


/*
 * @GPIO_PIN_MODES
 * GPIO Modes possible for a pin
 */

#define GPIO_MODE_INPUT  0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4       //Since in input mode we can config the GPIO pin to deliver interrupt to the processor, so there are more modes for which we can create macros
#define GPIO_MODE_IT_RT  5
#define GPIO_MODE_IT_RFT 6

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO OUTPUT Types possible for a pin
 */

#define GPIO_OP_TYPE_PP  0
#define GPIO_OP_TYPE_OD  1

/*
 * @GPIO_PIN_SPEED
 * GPIO OUTPUT SPEEDS possible for a pin
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_HIGH		2

/*
 * @GPIO_PULLUP_PULLDOWN
 * GPIO PULLUP or PULLDOWN config possible for a pin
 */

#define GPIO_NO_PUPD 		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2







#endif /* INC_STM32F091XX_GPIO_DRIVER_H_ */
