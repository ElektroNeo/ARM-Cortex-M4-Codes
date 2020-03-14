
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   4x4 Keypad Example
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

int main(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

	keypadInit();

	int8_t key = -1;
	GPIOC->ODR |= 1;
  while(1) {
	  key = keypadGetKey();
	  if(key != -1) {
		  blinkLED(key);
		  key = -1;
	  }
  }
}