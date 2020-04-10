
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   SysTick Delay Example
  ******************************************************************************
*/

#include "stm32f1xx.h"
#include "stdint.h"

void delayMs(uint16_t time) {
	/* Configure SysTick */
	SysTick->LOAD = 8000-1;		/* 1ms @8MHz */
	SysTick->VAL = 0;			/* Clear current value register */
	SysTick->CTRL = 5;			/* Enable SysTick */

	for(uint16_t i = 0; i < time; i++) {
		/* Wait until the COUNT flag is set */
		while(((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) >> 16) == 0);
	}

	SysTick->CTRL = 0;
}

void delayUs(uint16_t time) {
	/* Configure SysTick */
	SysTick->LOAD = 8-1;		/* 1us @8MHz */
	SysTick->VAL = 0;			/* Clear current value register */
	SysTick->CTRL = 5;			/* Enable SysTick */

	for(uint16_t i = 0; i < time; i++) {
		/* Wait until the COUNT flag is set */
		while(((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) >> 16) == 0);
	}

	SysTick->CTRL = 0;
}

int main(void)
{
	/* Enable GPIOC clocks. */
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	/* Set PC13 as push-pull output */
	GPIOC->CRH |= GPIO_CRH_MODE13_1;
	GPIOC->CRH &= ~GPIO_CRH_MODE13_0;
	GPIOC->CRH &= ~GPIO_CRH_CNF13_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF13_0;

  while(1) {
		GPIOC->ODR ^= GPIO_ODR_ODR13;
		delayMs(200);
  }
}
