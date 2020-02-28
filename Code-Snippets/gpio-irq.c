
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   GPIO Interrupt Example
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

void EXTI15_10_IRQHandler(void) {
	GPIOA->ODR |= GPIO_ODR_OD5;
	SysTickDelayMs(500);
	GPIOA->ODR &= ~GPIO_ODR_OD5;
	SysTickDelayMs(500);
	GPIOA->ODR |= GPIO_ODR_OD5;
	SysTickDelayMs(500);
	GPIOA->ODR &= ~GPIO_ODR_OD5;
	SysTickDelayMs(500);

	EXTI->PR = EXTI_PR_PR13;
}

int main(void)
{
	__disable_irq();

	// LED: RA5
	// BUTTON: PC13

	// PORTA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

	// PORTC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER &= (GPIO_MODER_MODE13_0 | GPIO_MODER_MODE13_0);

	// Enable SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// Select PORTC for EXTI13
	SYSCFG->EXTICR[3] = SYSCFG_EXTICR4_EXTI13_PC;
	// Unmask EXTI13
	EXTI->IMR |= EXTI_IMR_MR13;
	// Select falling edge trigger
	EXTI->FTSR |= EXTI_FTSR_TR13;

	NVIC_EnableIRQ(EXTI15_10_IRQn);

	__enable_irq();

  while(1) {

  }
}
