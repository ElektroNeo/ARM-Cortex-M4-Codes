
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   ADC Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"
#include "stdio.h"

void USART2_Init(void);
void USART2_WriteChar(char ch);
char USART2_ReadChar();
void USART2_WriteString(char *str);
void delayMs(uint32_t delay);
void ADC1_Init(void);
uint16_t ADC1_Read(void);

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

void ADC1_Init(void)
{
	// Enable clock for ADC1 and GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// Set PA4 as analog input
	GPIOA->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1);

	// Initialize ADC1
	// Set to single conversion mode
	ADC1->CR2 &= ~ADC_CR2_CONT;
	// Right alignment
	ADC1->CR2 &= ~ADC_CR2_ALIGN;
	// Disable external trigger detection
	ADC1->CR2 &= ~ADC_CR2_EXTEN;
	// 1 conversion
	ADC1->SQR1 &= ~ADC_SQR1_L;
	// Set resolution to 12 bit
	ADC1->CR1 |= ADC_CR1_RES_1 | ADC_CR1_RES_0;
	// Scan mode is disabled
	ADC1->CR1 &= ~ADC_CR1_SCAN;
	// Enable ADC
	ADC1->CR2 |= ADC_CR2_ADON;

	// 1 conversion in one conversion mode
	// Reset 1st conversion
	ADC1->SQR3 &= ~ADC_SQR3_SQ1;
	// Set 1st conversion to Channel 4
	ADC1->SQR3 |= (ADC_SQR3_SQ1_2);
	// Set sample time to 84 cycle
	ADC1->SMPR2 |= ADC_SMPR2_SMP4_2;
}

uint16_t ADC1_Read(void)
{
  // Start ADC conversion
	ADC1->CR2 |= ADC_CR2_SWSTART;
	// Wait until conversion is finish
	while(!(ADC1->SR & ADC_SR_EOC));

	return ADC1->DR;
}

int main(void)
{
	USART2_Init();
	ADC1_Init();
	
	uint16_t adc;
	char data[15];
	
  while(1) {
	adc = ADC1_Read();
	//adc = (adc*100)/4095;
	sprintf (data, "%d\r\n", adc);
	USART2_WriteString(data);
	delayMs(100);
  }
}
