
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   Keypad Step Motor Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"
#include "stdio.h"

void SysTickDelayMs(uint16_t time);

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

int8_t keys[16] = {13, 15, 0, 14, 12, 9, 8, 7, 11, 6, 5, 4, 10, 3, 2, 1};

void blinkLED(int8_t num) {
	for(uint8_t i = 0; i < num; i++) {
		SysTickDelayMs(500);
		GPIOA->ODR ^= GPIO_ODR_OD5;
		SysTickDelayMs(500);
		GPIOA->ODR ^= GPIO_ODR_OD5;
	}
}

void keypadInit(void) {
	// Connect Pin 1 of keypad to PC0 --- Pin 8 of keypad to PC7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	// Set PC0, PC1, PC2 and PC3 as output
	GPIOC->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0;
	// Set PC4, PC5, PC6 and PC7 as input and pull-down resistor
	GPIOC->PUPDR |= GPIO_PUPDR_PUPD4_1 | GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1;
}

int8_t keypadGetKey(void) {
	for(uint8_t i = 1, j = 0; i <= 0b1000; i <<= 1, j++) {
		GPIOC->ODR |= i;
		if(GPIOC->IDR & (1<<4)) {
			GPIOC->ODR &= ~i;
			return keys[j];
		}
		else if(GPIOC->IDR & (1<<5)) {
			GPIOC->ODR &= ~i;
			return keys[j+4];
		}
		else if(GPIOC->IDR & (1<<6)) {
			GPIOC->ODR &= ~i;
			return keys[j+8];
		}
		else if(GPIOC->IDR & (1<<7)) {
			GPIOC->ODR &= ~i;
			return keys[j+12];
		}

		GPIOC->ODR &= ~i;
	}
	return -1;
}

void TIM2_IRQHandler() {
	if(TIM2->SR & TIM_SR_UIF) {

		GPIOA->ODR ^= GPIO_ODR_OD5;
		
		// Clear interrupt flag
		TIM2->SR &= ~TIM_SR_UIF;
	}
}

void stepMotor(uint16_t spd, uint16_t dir, uint16_t delay) {
	TIM2->CR1 &= ~TIM_CR1_CEN;	// Counter Disable
	TIM2->CNT = 0;
	if(spd < 2)	spd = 2;
	if(spd > 5000)	spd = 5000;
	TIM2->ARR = spd;
	if(dir == 1) {
		GPIOA->ODR &= ~GPIO_ODR_OD6;
	} else if(dir == 0) {
		GPIOA->ODR |= GPIO_ODR_OD6;
	}
	TIM2->CR1 |= TIM_CR1_CEN;	// Counter Enable
	SysTickDelayMs(delay);
	TIM2->CR1 &= ~TIM_CR1_CEN;	// Counter Disable
}

int main(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	GPIOA->MODER |= GPIO_MODER_MODE6_0;

	keypadInit();

	__disable_irq();
	// Timer Configuration
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	// The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
	TIM2->PSC = 80-1;		// 16 000 000 / (79+1) = 200 000 Hz
	TIM2->ARR = 50;	// 200 000 / 30 = 0.0001 second
	TIM2->CNT = 0;			// Clear counter
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 &= ~TIM_CR1_CEN;	// Counter Disable

	NVIC_EnableIRQ(TIM2_IRQn);

	__enable_irq();
	int8_t key = -1;

  while(1) {
	  key = keypadGetKey();
	  if(key == 1) {
		  stepMotor(5, 1, 5000);
		  stepMotor(5, 0, 2000);
		  stepMotor(30, 1, 1000);
		  key = -1;
	  }

	  if(key == 2) {
		  for(uint16_t i = 100; i > 2; i-=1) {
			  stepMotor(i, 1, 100);
		  }

		  stepMotor(2, 1, 5000);

		  for(uint16_t i = 2; i < 100; i+=1) {
			  stepMotor(i, 1, 100);
		  }
		  key = -1;
	  }

	  if(key == 3) {
		  stepMotor(100, 1, 5000);
	  }
  }
}
