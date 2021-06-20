/*
 * 002Led_button.c
 *
 *  Created on: Jun 7, 2021
 *      Author: Badri Ramesh
 */

#include <stdio.h>
#include <stdint.h>

#include "stm32f091xx.h"

#define LOW	0
#define BTN_PRESSED LOW    //Nucleo F091XX

void delay(void);

void delay(void)
{
	for(uint32_t i=0 ; i<500000/2 ; i++);
}

int main(void)
{
	//create a handle structure variable
	GPIO_Handle_t Gpiobutton, Gpioled;

	//configure the external LED - PA5 as o/p and external Button - PC13 as input

	Gpioled.pGPIOx = GPIOA;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;			//Make sure after reset the port & pin you used in not one of the Serial wire debug/JTAG debug pins because they will be in AF mode and they will not be a free I/O. So dont use those pins PA14, PA13. Other than that we can use any pin
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PCLK_Control(GPIOA, ENABLE);
	GPIO_Init(&Gpioled);

	Gpiobutton.pGPIOx = GPIOD;
	Gpiobutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	Gpiobutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	Gpiobutton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	Gpiobutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PCLK_Control(GPIOD, ENABLE);
	GPIO_Init(&Gpiobutton);

	while(1)
	{
		if(GPIO_ReadFrom_InputPin(GPIOD, GPIO_PIN_NO_5) == BTN_PRESSED){
			delay();									//compensate for button debouncing. Without this delay there will be button debouncing and hence the loop will execute multiple times. Because the read from input pin will be true multiple times
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
		else{
			;
		}
	}
	return 0;
}


