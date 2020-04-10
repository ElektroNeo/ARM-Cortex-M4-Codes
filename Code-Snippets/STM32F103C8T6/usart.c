
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

void usart2Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  GPIOA->CRL |= GPIO_CRL_CNF2_0;	/* Enable alternate function on PA2 TX */
  GPIOA->CRL |= GPIO_CRL_CNF2_1;	/* Enable alternate function on PA2 */
  /*GPIOA->CRL |= GPIO_CRL_CNF3_0;*/	/* Enable alternate function on PA3 RX */
  /*GPIOA->CRL |= GPIO_CRL_CNF3_1;*/	/* Enable alternate function on PA3 */

  GPIOA->CRL |= GPIO_CRL_MODE2_0;
  /*GPIOA->CRL |= GPIO_CRL_MODE3_0;*/

  /*USART2->BRR = 0x0683;*/           	/* 9600 @16MHz */
  USART2->BRR = 0x0341;           		/* 9600 @8MHz */
  /*USART2->CR1 |= USART_CR1_PCE;*/		/* Parity enable */
  /*USART2->CR1 |= USART_CR1_PS;*/		/* Odd parity */
  USART2->CR1 |= USART_CR1_TE;     	/* Enable TX */
  USART2->CR1 |= USART_CR1_RE;		/* Enable RX */
  USART2->CR1 |= USART_CR1_UE;    	/* Enable USART */
}

void usart2WriteChar(char ch) {
  /* Wait while TX bugger is empty */
  while(!((USART2->SR & USART_SR_TXE) >> 7));
  USART2->DR = ch;
}

char usart2ReadChar() {
	while(!((USART2->SR & USART_SR_RXNE) >> 5));	/* Wait until RX ready. */
	return (USART2->DR);
}

void usart2WriteString(char *str) {
  for(uint8_t i = 0; str[i] != '\0'; i++) {
	  usart2WriteChar(str[i]);
  }
}

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

void ledBlink(uint8_t value) {
	for(uint8_t i = 0; i < value; i++) {
		GPIOC->ODR ^= GPIO_ODR_ODR13;
		delayMs(200);
	}
}

int main(void)
{

  usart2Init();
  
	/* Enable GPIOC clocks. */
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	/* Set PC13 as push-pull output */
	GPIOC->CRH |= GPIO_CRH_MODE13_1;
	GPIOC->CRH &= ~GPIO_CRH_MODE13_0;
	GPIOC->CRH &= ~GPIO_CRH_CNF13_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF13_0;
	
	char ch = 's';
	
  while(1) {
    delayMs(100);
		
    ch = usart2ReadChar();
    
	  usart2WriteString("Readed Char: ");
		usart2WriteChar(ch);
		usart2WriteString("\r\n");
	  if( ch == 'a') {
		  ledBlink(3);
	  }
  }
}
