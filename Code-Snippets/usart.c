
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   USART Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"

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

char USART2_ReadChar() {
	while(!((USART2->SR & USART_SR_RXNE) >> 5));	// Wait until RX ready.
	return USART2->DR;
}

void USART2_WriteString(char *str) {
  for(uint8_t i = 0; str[i] != '\0'; i++) {
	  USART2_WriteChar(str[i]);
  }
}

// 1ms delay for 16MHz oscillator.
void delayMs(uint32_t delay) {
  for(; delay > 0; delay--)
      for(uint32_t i = 0; i < 3195; i++);
}

void LED_Blink(uint8_t value) {
	for(uint8_t i = 0; i < value; i++) {
		GPIOA->BSRR |= GPIO_BSRR_BS5;
		delayMs(200);
		GPIOA->BSRR |= GPIO_BSRR_BR5;
		delayMs(200);
	}
}

int main(void)
{

  USART2_Init();
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  // Set PA5 as output
	GPIOA->MODER |= GPIO_MODER_MODER5_0;
	char ch = 's';
	
  while(1) {
    delayMs(100);
		
    ch = USART2_ReadChar();
    
	  USART2_WriteString("Readed Char: ");
		USART2_WriteChar(ch);
		USART2_WriteString("\r\n");
	  if( ch == 'a') {
		  LED_Blink(3);
	  }
  }
}
