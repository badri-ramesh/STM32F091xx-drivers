/*
 * 001Led_toggle.c
 *
 *  Created on: 07-Jun-2021
 *      Author: Badri Ramesh
 */


#include <stdio.h>
#include <stdint.h>


#include <stm32f091xx.h>

void delay(void);

void delay(void)
{
	for(uint32_t i=0 ; i<500000/2 ; i++);
}


int main(void)
{

	//1. Pushpull config toggle led
	//2. Opendrain config toggle led

	GPIO_Handle_t Gpioled;

	Gpioled.pGPIOx = GPIOA;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PCLK_Control(GPIOA, ENABLE);     	//Enable PCLK for GPIOA
	GPIO_Init(&Gpioled);					//Config registers

	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
