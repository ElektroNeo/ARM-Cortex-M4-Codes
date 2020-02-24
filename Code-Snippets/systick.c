
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   SysTick Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"

int main(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

	// Configure SysTick
	SysTick->LOAD = 1600000-1;	// 1ms @16MHz
	SysTick->VAL = 0;
	SysTick->CTRL = 5;


  while(1) {
	  // Check if count flag is set
	  if((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) >> 16) {
		  GPIOA->ODR ^= GPIO_ODR_OD5;
	  }
  }
}
