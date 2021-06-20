/*
 * stm32f091xx_gpio_driver.c
 *
 *  Created on: 06-Jun-2021
 *      Author: Badri Ramesh
 *
 *  GPIO Driver APIs are implemented here.
 *  User application takes the help of driver APIs to configure/initialize various settings.
 *
 *  Driver API requirements:
 *  	A GPIO driver should be able to give the user application all these APIs like an API for GPIO initialization, EN/DI GPIO port clock, Read from GPIO pin/port,
 *  	Write into a GPIO pin/port, Config Alt functionality, Interrupt handling & config..etc.,`
 *  	When writing into physical registers use |=
 *  	Before setting any bit fields clear them first
 */


#include "stm32f091xx_gpio_driver.h"			/*Driver header file*/

/**************************************************Function/API Definitions*************************************************************/

/*
 * Peripheral clock setup
 */

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)					//To enable or disable the peripheral clock
{
	if(ENorDI == ENABLE)
	{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
	}
}


/*
 * Init and DeInit
 */

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function is to initialize the given GPIO port and the given GPIO pin.
 * 						Configure its mode, speed, output type, Pullup or pulldown config, alt func..etc
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)                              		//To initialize the GPIO port & pin. Initially parameters are not considered. Then as we implement they are defined.
{
		uint32_t temp = 0;   //temp register

		//1. Configure mode of GPIO pin			//we have to shift the appropriate value into the appropriate bit pos acc to pin number of the MODE REGISTER

		if(pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));   //why we multiply by 2, beacuse each pin takes 2 bit fields
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//clearing
			pGPIOHandle->pGPIOx->MODER |= temp;		//setting
		}else
		{
			//Configure registers asso. with interrupts on MCU & MCU Peripheral side like EXTI,

			if(pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

				//configure the FTSR. So FTSR is a reg. of EXTI so lets dereferene the EXTI peripheral & config. that
				EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}
			else if(pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){

				//configure the RTSR
				EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}else if(pGPIOHandle-> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){

				//configure the RTSR & FTSRx
				EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}

			//2. configure the GPIO port selection in SYSCFG_EXTICR

			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->SYSCFG_EXTICR[temp1] |= (portcode << (4 * temp2));

			//3. Enable EXTI interrupt delivery using IMR (mask reg).

			EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		temp = 0;

		//2. Configure speed

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <= GPIO_SPEED_HIGH)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->OSPEEDR &=  ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		}

		temp = 0;

		//3. Configure PU, PD settings

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <= GPIO_PIN_PD)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->PUPDR &=  ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->PUPDR |= temp;
		}
		temp = 0;

		//4. Configure Output type

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <= GPIO_OP_TYPE_OD)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
			pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber	);
			pGPIOHandle->pGPIOx->OTYPER |= temp;
		}
		temp=0;

		//5. Configure Alt. funct.

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN )
		{
			uint8_t temp1=0, temp2=0;
			temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
			temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));;
			//temp1 = 0, temp2 = 0;
		}
}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)							  			//Change GPIO settings to reset state or reset value collectively of a peripheral
{
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
}

/*
 * Data Read & Write
 */

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

uint8_t  GPIO_ReadFrom_InputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)  	//since return will be either 0 or 1->uint8_t
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

uint16_t GPIO_ReadFrom_InputPort(GPIO_RegDef_t *pGPIOx)						//each port is 16pins so return type is uint16_t. This will return the content of input data register
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

void 	 GPIO_WriteInto_OutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET){
	pGPIOx->ODR |= (0x1 << PinNumber);
	}
	else{
	pGPIOx->ODR |= ~(0x1 << PinNumber);
	}
}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

void 	 GPIO_WriteInto_OutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;		   // only = because you are just writing 	into the whole port of the given GPIO peripheral
}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

uint8_t  GPIO_ReadFrom_OutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)   	//since return will be either 0 or 1->uint8_t
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->ODR >> PinNumber) & 0x00000001);
	return value;
}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

uint16_t GPIO_ReadFrom_OutputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value;
	Value = pGPIOx->ODR;
	return Value;
}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

void 	 GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration and ISR handling
 */

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

void GPIO_IRQInterrupt_Config(uint8_t IRQNumber, uint8_t ENorDI)	//TODO: IRQ grouping; For interrupts; to configure the IRQ number of the GPIO pin. Like EN, setting priority
{
	//Configure registers asso. with interrupts on Processor & Processor Peripheral side like NVIC
	if(ENorDI == ENABLE){
		if(IRQNumber <=31){
			//program ISER
			*NVIC_ISER |= (1 << IRQNumber);

		}
	}
	else if(ENorDI == DISABLE){
		if(IRQNumber <=31){
			//program ICER
			*NVIC_ICER |= (1 << IRQNumber);

		}
	}

}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

void GPIO_IRQPriority_Config(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	//Find IPR register
	uint8_t shift_amount = ((8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED));
	*(NVIC_IPR_BASEADDR + (iprx * 4)) |= (IRQPriority << shift_amount);    //8 bit values
}

/* API purpose, parameters explanation, return value explanation
 * @fn 				-	GPIO_PCLK_Control
 *
 * @brief			-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-	Base address of the GPIO peripheral
 * @param[in]		-	ENABLE or DISABLE macros
 *
 * @return			-	None
 *
 * @Note			-	None
 */

void GPIO_IRQHandling(uint8_t PinNumber)										//process interrupts; since IRQ handling functions should know from which pin the interrupt is triggerd.
{

}








