
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   SysTick IRQ Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"

void SysTick_Handler(void) {
	GPIOA->ODR ^= GPIO_ODR_OD5;
}

int main(void)
{
	__disable_irq();

	// Configure SysTick
	SysTick->LOAD = 16000000-1;	// 1s @16MHz
	SysTick->VAL = 0;			// Clear current value register
	// Enable SysTick and SysTick Interrupt
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk;

  __enable_irq();
	
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  // Set PA5 as output
	GPIOA->MODER |= GPIO_MODER_MODER5_0;

  while(1) {

  }
}
