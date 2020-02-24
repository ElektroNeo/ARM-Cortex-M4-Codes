
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   SysTick Delay Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"

void SysTickDelayMs(uint16_t time) {
	// Configure SysTick
	SysTick->LOAD = 16000-1;	// 1ms @16MHz
	SysTick->VAL = 0;			// Clear current value register
	SysTick->CTRL = 5;			// Enable SysTick

	for(uint16_t i = 0; i < time; i++) {
		// Wait until the COUNT flag is set
		while(((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) >> 16) == 0);
	}

	SysTick->CTRL = 0;
}

int main(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

  while(1) {
	SysTickDelayMs(100);
	GPIOA->ODR ^= GPIO_ODR_OD5;
  }
}
