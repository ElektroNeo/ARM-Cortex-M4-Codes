
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   RTC with LSI Clock Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"
#include "stdio.h"

// Set to 1 for configuration
#define CONFIGURE 0

void USART2_Init(void);
void USART2_WriteChar(char ch);
char USART2_ReadChar();
void USART2_WriteString(char *str);
void delayMs(uint32_t delay);
void RTC_Init(void);
void RTC_Config(void);

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

void RTC_Init(void) {
	// Power interface clock enable
	RCC->APB1ENR |= RCC_APB1LPENR_PWRLPEN;
	// Enable DBP bit in PWR_CR
	PWR->CR |= PWR_CR_DBP;
	// Enable LSI RC oscillator
	RCC->CSR |= RCC_CSR_LSION;
	// Wait for LSI RC ready
	while(!(RCC->CSR & RCC_CSR_LSIRDY));
	// LSI oscillator clock used as the RTC clock
	RCC->BDCR &= ~RCC_BDCR_RTCSEL_0;
	RCC->BDCR |= RCC_BDCR_RTCSEL_1;
	// Configure prescaler register for 32kHz LSI clock
	// PREDIV_A[6:0] = 127, PREDIV_S[14:0] = 249
	RTC->PRER = (127 << 16) + 249;
	// Enable RTC clock
	RCC->BDCR |= RCC_BDCR_RTCEN;
	//while(!(RTC->ISR & RTC_ISR_RSF));
}

void RTC_Config(void)
{
	// Check for calendar initialization
	// If not initialized then initialize it.
	if(!(RTC->ISR & RTC_ISR_INITS) || CONFIGURE) {
		// Disable the RTC registers write protection
		RTC->WPR = 0xCA;
		RTC->WPR = 0x53;

		// Enter Initialization mode
		RTC->ISR |= RTC_ISR_INIT;
		while(!((RTC->ISR & RTC_ISR_INITF)>>6));
		// 24 Hour Format
		RTC->CR &= ~RTC_CR_FMT;
		// Time Register
		RTC->TR = 1 << RTC_TR_PM_Pos | (0 << RTC_TR_HT_Pos) | (1 << RTC_TR_HU_Pos) |
						 (4 << RTC_TR_MNT_Pos) | (5 << RTC_TR_MNU_Pos) | (0 << RTC_TR_ST_Pos) |
						 (0 << RTC_TR_SU_Pos);
		// Data Register
		RTC->DR = (2 << RTC_DR_YT_Pos) | (0 << RTC_DR_YU_Pos) | 6 << RTC_DR_WDU_Pos |
							(0 << RTC_DR_MT_Pos) | (3 << RTC_DR_MU_Pos) | (0 << RTC_DR_DT_Pos) |
							(1 << RTC_DR_DU_Pos);
				
		// Free running mode
		RTC->ISR &= ~RTC_ISR_INIT;
		// Enable write protection
		RTC->WPR=0xFF;
	}
}

int main(void)
{
	USART2_Init();
	RTC_Init();
	RTC_Config();

	char data[30];
	uint8_t lastTime;
	
  while(1) {
		
		// Save last second unit
		lastTime = RTC->TR & RTC_TR_SU;
		// Wait 1 second
		while(lastTime == (RTC->TR & RTC_TR_SU));
		// Save time and date to data array
		sprintf (data, "20%ld%ld/%ld%ld/%ld%ld %ld%ld:%ld%ld:%ld%ld\r\n", (RTC->DR & RTC_DR_YT) >> 20, (RTC->DR & RTC_DR_YU) >> 16,
				(RTC->DR & RTC_DR_MT) >> 12, (RTC->DR & RTC_DR_MU) >> 8, (RTC->DR & RTC_DR_DT) >> 4, RTC->DR & RTC_DR_DU,
				(RTC->TR & RTC_TR_HT) >> 20, (RTC->TR & RTC_TR_HU) >> 16, (RTC->TR & RTC_TR_MNT) >> 12, (RTC->TR & RTC_TR_MNU) >> 8,
				(RTC->TR & RTC_TR_ST) >> 4, RTC->TR & RTC_TR_SU);
		// Write data to USART
		USART2_WriteString(data);
	
  }
}
