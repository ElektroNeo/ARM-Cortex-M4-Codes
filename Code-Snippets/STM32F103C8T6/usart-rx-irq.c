
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   USART RX IRQ Example
  ******************************************************************************
*/
#include "stm32f1xx.h"
#include "stdint.h"
#include "stdio.h"

#define COMMAND_MODE		1
#define COMMAND_READ_MODE	2
#define GET_VALUE_MODE		3

uint16_t g_temp = 5;
uint16_t g_tempGoal = 50;
char g_command[4];
uint8_t g_mode = 1, g_cmd, g_cmdCounter = 0, g_numValues[5];
uint32_t g_number = 0;
uint8_t g_ssrControle = 0, g_ssrState = 0;

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

void ledBlink(uint8_t value) {
	for(uint8_t i = 0; i < value; i++) {
		GPIOC->ODR ^= GPIO_ODR_ODR13;
		delayMs(200);
	}
}

/* Function to compare array elements */
uint8_t compareArray(char a[], char b[], uint8_t size)	{
	for(uint8_t i=0; i<size; i++){
		if(a[i]!=b[i])
			return (0);
	}
	return (1);
}

void USART2_IRQHandler(void) {
	if(USART2->SR & USART_SR_RXNE) {
		USART2->SR &= ~USART_SR_RXNE;
		uint8_t data = USART2->DR;
		if(data == '$' && g_mode == COMMAND_MODE) {
			g_mode = COMMAND_READ_MODE;
		} else if(g_mode == COMMAND_READ_MODE) {
			g_command[g_cmdCounter] = data;
			g_cmdCounter++;
			if(g_cmdCounter == 3) {
				if(compareArray(g_command, "HSD", 3)) {
					g_mode = GET_VALUE_MODE;
				} else {
					if(compareArray(g_command, "IAK", 3)) {
						g_ssrControle = 1;
					} else if(compareArray(g_command, "IKK", 3)) {
						g_ssrControle = 0;
					}
					g_mode = COMMAND_MODE;
				}
				g_cmdCounter = 0;
			}
		} else if(g_mode == GET_VALUE_MODE) {
			g_numValues[g_cmdCounter] = data;
			g_cmdCounter++;
			if(g_cmdCounter == 4) {
				g_number = (g_numValues[3] << 24) | (g_numValues[2] << 16) | (g_numValues[1] << 8) | g_numValues[0];
				if(compareArray(g_command, "HSD", 3)) {
					g_tempGoal = g_number;
				}
				g_mode = COMMAND_MODE;
				g_cmdCounter = 0;
			}
		}
	}
}

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

  __disable_irq();
    USART2->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
    __enable_irq();

  USART2->CR1 |= USART_CR1_UE;    	/* Enable USART */
}

void usart2WriteChar(char ch) {
  /* Wait while TX bugger is empty */
  while(!((USART2->SR & USART_SR_TXE) >> 7));
  USART2->DR = ch;
}

void usart2WriteString(char *str) {
  for(uint8_t i = 0; str[i] != '\0'; i++) {
	  usart2WriteChar(str[i]);
  }
}

void max6675Init(void) {
	/* Configure clocks and pins */
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	/* NSS pin */
	GPIOA->CRL |= GPIO_CRL_MODE4_1;
	GPIOA->CRL &= ~GPIO_CRL_MODE4_0;
	GPIOA->CRL &= ~GPIO_CRL_CNF4_0;
	GPIOA->CRL &= ~GPIO_CRL_CNF4_1;
	/* SCK pin */
	GPIOA->CRL |= GPIO_CRL_MODE5_1;
	GPIOA->CRL &= ~GPIO_CRL_MODE5_0;
	GPIOA->CRL &= ~GPIO_CRL_CNF5_0;
	GPIOA->CRL &= ~GPIO_CRL_CNF5_1;
	/* MISO pin */
	GPIOA->CRL &= ~GPIO_CRL_MODE6_0;
	GPIOA->CRL &= ~GPIO_CRL_MODE6_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF6_1;
	GPIOA->CRL |= GPIO_CRL_CNF6_0;

	/* Set CS pin to high */
	GPIOA->ODR |= GPIO_ODR_ODR4;
}

uint16_t max6675Read(void) {
	uint16_t data = 0;
	/* Set CS pin to low */
	GPIOA->ODR &= ~GPIO_ODR_ODR4;

	delayMs(1);

	for(uint8_t i = 16; i > 0; i--) {
		/* Set SCLK pin to low */
		GPIOA->ODR &= ~GPIO_ODR_ODR5;

		data |= (GPIOA->IDR & GPIO_IDR_IDR6) >> 6;
		delayMs(1);
		data <<= 1;
		delayMs(1);
		/* Set SCLK pin to high */
		GPIOA->ODR |= GPIO_ODR_ODR5;
		delayMs(1);
	}
	data >>= 1;
	/* Set CS pin to high */
	GPIOA->ODR |= GPIO_ODR_ODR4;

	return (data);
}

void nextionSend(char *data) {
	for(uint8_t i = 0; data[i] != '\0'; i++) {
		usart2WriteChar(data[i]);
	}
	usart2WriteChar(0xFF);
	usart2WriteChar(0xFF);
	usart2WriteChar(0xFF);
}

void sssInit(void) {
	/* Enable GPIOC clocks. */
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	/* Set PC13 as push-pull output */
	GPIOC->CRH |= GPIO_CRH_MODE15_0;
	GPIOC->CRH &= ~GPIO_CRH_MODE15_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF15_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF15_0;
}

void sssEnable(void) {
	if(g_ssrControle) {
		GPIOC->ODR |= GPIO_ODR_ODR15;
		GPIOC->ODR |= GPIO_ODR_ODR13;
	} else {
		GPIOC->ODR &= ~GPIO_ODR_ODR15;
		GPIOC->ODR &= ~GPIO_ODR_ODR13;
	}
}

void sssDisable(void) {
	GPIOC->ODR &= ~GPIO_ODR_ODR15;
	GPIOC->ODR &= ~GPIO_ODR_ODR13;
}

int main(void)
{
	max6675Init();
	sssInit();
	/*SPI1_Init(SPI_MASTER);*/
	usart2Init();
	/* Enable RX interrupt */
	/*__disable_irq();
	NVIC_EnableIRQ(SPI1_IRQn);
	__enable_irq();*/
	char str[50];

	/* Enable GPIOC clocks. */
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	/* Set PC13 as push-pull output */
	GPIOC->CRH |= GPIO_CRH_MODE13_0;
	GPIOC->CRH &= ~GPIO_CRH_MODE13_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF13_1;
	GPIOC->CRH &= ~GPIO_CRH_CNF13_0;
	
	while(1) {
		delayMs(500);

		g_temp = max6675Read();
		g_temp = g_temp >> 3;
		g_temp *= 0.25;

		if(!g_ssrControle) {
			sssEnable();
		} else {
			if(g_temp < (g_tempGoal - 10)) {
				sssDisable();
				g_ssrState = 0;
			} else if(g_temp > (g_tempGoal + 10)) {
				sssEnable();
				g_ssrState = 1;
			} else {
				if(g_ssrState) {
					sssEnable();
				} else {
					sssDisable();
				}
			}
		}
		sprintf(str, "anaSayfa.n0.val=%d", g_temp/*(g_temp/5)*5*/);

		nextionSend(str);

		/*USART2_WriteString("Temp: ");*/
		/*USART2_WriteString(str);*/
		/*USART2_WriteString("\r\n");*/
	}
}
