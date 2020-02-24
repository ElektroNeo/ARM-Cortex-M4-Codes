
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   General Purpose Timer Example
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

	// Timer Configuration
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 1600-1;		// 16 000 000 / 1600 = 10 000
	TIM2->ARR = 10000-1;	// 10 000 / 10 000 = 1
	TIM2->CNT = 0;			// Clear counter
	TIM2->CR1 = TIM_CR1_CEN;	// Counter Enable

  while(1) {
	while(!(TIM2->SR & TIM_SR_UIF));
	TIM2->SR &= ~TIM_SR_UIF;
	GPIOA->ODR ^= GPIO_ODR_OD5;
  }
}
