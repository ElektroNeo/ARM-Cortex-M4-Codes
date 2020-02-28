
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   USART RX IRQ Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"

void USART2_IRQHandler(void);
void USART2_Init(void);
void USART2_WriteChar(char ch);
void USART2_WriteString(char *str);

void USART2_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  // Set alternate function on PA2 pin to USART1..2
  GPIOA->AFR[0] |= GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_2;
  // Set alternate function on PA3 pin to USART1..2
  GPIOA->AFR[0] |= GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_2;
  GPIOA->MODER |= GPIO_MODER_MODE2_1;	// Enable alternate function on PA2
  GPIOA->MODER |= GPIO_MODER_MODE3_1;	// Enable alternate function on PA3

  USART2->BRR = 0x0683;           	// 9600 @16MHz
  //USART2->CR1 |= USART_CR1_PCE;		// Parity enable
  //USART2->CR1 |= USART_CR1_PS;		// Odd parity
  USART2->CR1 |= USART_CR1_TE;     	// Enable TX
  USART2->CR1 |= USART_CR1_RE;		// Enable RX
  USART2->CR1 |= USART_CR1_UE;    	// Enable USART
}

void USART2_WriteChar(char ch) {
  // Wait while TX bugger is empty
  while(!((USART2->SR & USART_SR_TXE) >> 7));
  USART2->DR = ch;
}

void USART2_IRQHandler(void) {
	if(USART2->SR & USART_SR_RXNE) {
		USART2->SR &= ~USART_SR_RXNE;
		char c = USART2->DR;
		USART2_WriteChar(c);
	}
}

int main(void)
{
	__disable_irq();

  USART2_Init();
  USART2->CR1 |= USART_CR1_RXNEIE;
  NVIC_EnableIRQ(USART2_IRQn);

  __enable_irq();

  while(1) {

  }
}
