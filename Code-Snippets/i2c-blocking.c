
/**
  ******************************************************************************
  * @file    main.c
  * @author  Bahtiyar Bayram
  * @version V1.0
  * @brief   I2C Blocking Example
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stdint.h"
#include "stdio.h"

// Address of EEPROM
// 1 0 1 0 A2 A1 A0 R/W
#define EEPROM_SLAVE_ADD	0b1010000

void I2C1_Init(void);
void I2C1_ReadData16bit(uint8_t slaveAddr, uint16_t regAddr, uint8_t *buf, uint16_t len);
void I2C1_ReadData8bit(uint8_t slaveAddr, uint8_t regAddr, uint8_t *buf, uint16_t len);
void I2C1_ReadBuf(uint8_t slaveAddr, uint8_t *buf, uint16_t len);
void I2C1_Read(uint8_t *data);
void I2C1_WriteData16bit(uint8_t slaveAddr, uint16_t regAddr, uint8_t data);
void I2C1_WriteData8bit(uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
void I2C1_Write(uint8_t data);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_WriteSlaveAddr(uint8_t slaveAddr, uint8_t rw);

void SysTickDelayMs(uint16_t time);

void I2C1_Init(void) {
	// Enable PORTB clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// Enable I2C clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// Set PB6 and PB7 mode to alternate function
	GPIOB->MODER |= GPIO_MODER_MODE6_1;
	GPIOB->MODER &= ~GPIO_MODER_MODE6_0;
	GPIOB->MODER |= GPIO_MODER_MODE7_1;
	GPIOB->MODER &= ~GPIO_MODER_MODE7_0;

	// Reset alternate function
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL6;
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL7;
	// Set alternate function as I2C
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL6_2;
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL7_2;

	// Set PB6 and PB7 as open drain
	GPIOB->OTYPER |= GPIO_OTYPER_OT6;
	GPIOB->OTYPER |= GPIO_OTYPER_OT7;

	// No pull-up and pull-down register on PB6 and PB7 pins
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD6;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD7;

	// I2C Configuration
	// Clear control registers
	I2C1->CR1 = 0;
	I2C1->CR2 = 0;
	I2C1->CCR = 0;
	// I2C Peripheral under reset state
	I2C1->CR1 |= I2C_CR1_SWRST;
	// I2C Peripheral not under reset
	I2C1->CR1 &= ~I2C_CR1_SWRST;
	// Set peripheral clock frequency to 8MHz (FPCLK)
	I2C1->CR2 |= I2C_CR2_FREQ_3;
	// 100kHz I2C clock
	/*
	   TSCL = 1/FSCL = T_high + T_low
	   T_high = T_low -> T_high = TSCL/2
	   FSCL = 100kHz -> TSCL = 10us
	   T_high = 10us/2 = 5000ns
	   TPCLK1 = 1/FPCLK = 1/8MHz = 125ns
	   CCR = T_high/TPCLK1 = 5000ns/125ns = 40 = 0x28
	*/
	I2C1->CCR = 0x28;
	// Standard mode (Sm) I2C
	I2C1->CCR &= ~I2C_CCR_FS;
	// Max rise time for Sm; 1000ns / 125ns = 8 + 1 = 9
	// TRISE = T_maxrise/TPCLK1 + 1 = 1000ns/125ns + 1 = 9 = 0x9
	I2C1->TRISE = 0x9;
	// I2C Peripheral enable
	I2C1->CR1 |= I2C_CR1_PE;
	// Event interrupt enable
	//I2C1->CR2 |= I2C_CR2_ITEVTEN;
	// Buffer interrupt enable
	//I2C1->CR2 |= I2C_CR2_ITBUFEN;
	//NVIC_EnableIRQ(I2C1_EV_IRQn);
	//__enable_irq();

	// Acknowledge returned after a byte is received
	I2C1->CR1 |= I2C_CR1_ACK;
}

void I2C1_ReadData16bit(uint8_t slaveAddr, uint16_t regAddr, uint8_t *buf, uint16_t len) {
	// Sent 16 bit register address
	I2C1_Start();
	I2C1_WriteSlaveAddr(slaveAddr, 0);
	// Read SR2 to clear ADDR flag
	(void)I2C1->SR2;
	I2C1->DR = (regAddr >> 8) & 0x1F;
	// Wait for transmitter data register empty
	while(!(I2C1->SR1 & I2C_SR1_TXE));

	I2C1->DR = regAddr & 0xFF;
	// Wait for transmitter data register empty
	while(!(I2C1->SR1 & I2C_SR1_TXE));

	// Start reading
	I2C1_ReadBuf(slaveAddr, buf, len);

	// Wait until no communication on the bus
	while(I2C1->SR2 & I2C_SR2_BUSY);
}

void I2C1_ReadData8bit(uint8_t slaveAddr, uint8_t regAddr, uint8_t *buf, uint16_t len) {
	// Wait until no communication on the bus
	while(I2C1->SR2 & I2C_SR2_BUSY);

	// Sent 16 bit register address
	I2C1_Start();
	I2C1_WriteSlaveAddr(slaveAddr, 0);
	// Read SR2 to clear ADDR flag
	(void)I2C1->SR2;
	I2C1->DR = regAddr;
	// Wait for transmitter data register empty
	while(!(I2C1->SR1 & I2C_SR1_TXE));

	// Start reading
	I2C1_ReadBuf(slaveAddr, buf, len);

	// Wait until no communication on the bus
	while(I2C1->SR2 & I2C_SR2_BUSY);
}

void I2C1_ReadBuf(uint8_t slaveAddr, uint8_t *buf, uint16_t len) {
	I2C1_Start();
	// Write slave address with read command
	I2C1->DR = (slaveAddr << 1) + 1;
	// Wait for end of address transmission
	// To clear ADDR flag read SR1 and SR2 register
	// In while loop SR1 is already read
	while(!(I2C1->SR1 & I2C_SR1_ADDR));

	//We are going to read only 1 byte
	if(len == 1) {
		//Before Clearing Addr bit by reading SR2, we have to cancel ack.
		// No acknowledge returned
		I2C1->CR1 &= ~I2C_CR1_ACK;
		// Read SR2 to clear ADDR flag
		(void)I2C1->SR2;
		// Stop generation after the current byte transfer or
		// after the current Start condition is sent.
		I2C1->CR1 |= I2C_CR1_STOP;

		I2C1_Read(buf);

		// Wait until no communication on the bus
		while(I2C1->SR2 & I2C_SR2_BUSY);

		// Enable the Acknowledgement
		I2C1->CR1 |= I2C_CR1_ACK;
	}
	// We are going to read 2 bytes
	else if(len == 2) {
		// Before Clearing Addr, reset ACK, set POS
		I2C1->CR1 &= ~I2C_CR1_ACK;
		I2C1->CR1 |= I2C_CR1_POS;
		//Read the SR2 to clear ADDR
		(void)I2C1->SR2;
		//Wait for the data on the shift register to be transmitted completely
		while(!(I2C1->SR1 & I2C_SR1_BTF));

		// Stop generation after the current byte transfer or
		// after the current Start condition is sent.
		I2C1->CR1 |= I2C_CR1_STOP;

		I2C1_Read(buf++);
		I2C1_Read(buf);

		// Wait until no communication on the bus
		while(I2C1->SR2 & I2C_SR2_BUSY);

		I2C1->CR1 |= I2C_CR1_ACK;
		I2C1->CR1 &= ~I2C_CR1_POS;
	}
	// We have more than 2 bytes
	else if(len > 2) {
		// Read SR2 to clear ADDR flag
		(void)I2C1->SR2;

		while((len--) > 3) {
			I2C1_Read(buf++);
		}

		// Wait for the next 2 bytes to be received (1st in the DR, 2nd in the shift register)
		while(!(I2C1->SR1 & I2C_SR1_BTF));

		// Reset ACK
		I2C1->CR1 &= ~I2C_CR1_ACK;

		// Read N-2
		I2C1_Read(buf++);
		// Once we read this, N is going to be read to the shift register and NACK is generated

		// Wait for the next 2 bytes to be received (1st in the DR, 2nd in the shift register)
		while(!(I2C1->SR1 & I2C_SR1_BTF)); // N-1 is in DR, N is in shift register
		// Here the clock is stretched

		// Stop generation after the current byte transfer or
		// after the current Start condition is sent.
		I2C1->CR1 |= I2C_CR1_STOP;


		// Read the last two bytes (N-1 and N)
		// Read the next two bytes
		I2C1_Read(buf++);
		I2C1_Read(buf);

		// Wait until no communication on the bus
		while(I2C1->SR2 & I2C_SR2_BUSY);

		I2C1->CR1 |= I2C_CR1_ACK;
	}
}

void I2C1_Read(uint8_t *data) {
	// Wait for receiver data register not empty
	while(!(I2C1->SR1 & I2C_SR1_RXNE));
	*data = I2C1->DR;
}

void I2C1_WriteData16bit(uint8_t slaveAddr, uint16_t regAddr, uint8_t data) {
	// Send 16 bit register address
	I2C1_Start();
	I2C1_WriteSlaveAddr(slaveAddr, 0);
	// Read SR2 to clear ADDR flag
	(void)I2C1->SR2;

	I2C1_Write((regAddr >> 8) & 0x1F);

	I2C1_Write(regAddr & 0xFF);

	I2C1_Write(data);

	I2C1_Stop();
	// Wait until no communication on the bus
	while(I2C1->SR2 & I2C_SR2_BUSY);
}

void I2C1_WriteData8bit(uint8_t slaveAddr, uint8_t regAddr, uint8_t data) {
	// Send 16 bit register address
	I2C1_Start();
	I2C1_WriteSlaveAddr(slaveAddr, 0);
	// Read SR2 to clear ADDR flag
	(void)I2C1->SR2;

	I2C1_Write(regAddr);

	I2C1_Write(data);

	I2C1_Stop();
	// Wait until no communication on the bus
	while(I2C1->SR2 & I2C_SR2_BUSY);
}

void I2C1_Write(uint8_t data) {
	// Write data
	I2C1->DR = data;
	// Wait for byte transfer complete
	while(!(I2C1->SR1 & I2C_SR1_TXE));
}

void I2C1_Start(void) {
	// Start generation
	I2C1->CR1 |= I2C_CR1_START;
	// Wait for start condition generation
	while(!(I2C1->SR1 & I2C_SR1_SB));
}

void I2C1_Stop(void) {
	// Stop generation after the current byte transfer or
	// after the current Start condition is sent.
	I2C1->CR1 |= I2C_CR1_STOP;
}

// 0 for write, 1 for read
void I2C1_WriteSlaveAddr(uint8_t slaveAddr, uint8_t rw) {
	if(rw == 1) {
		// Write slave address with read command
		I2C1->DR = (slaveAddr << 1) + 1;
		// Wait for end of address transmission
		// To clear ADDR flag read SR1 and SR2 register
		// In while loop SR1 is already read
		while(!(I2C1->SR1 & I2C_SR1_ADDR));
	} else if(rw == 0) {
		// Write slave address with write command
		I2C1->DR = slaveAddr << 1;
		// Wait for end of address transmission
		// To clear ADDR flag read SR1 and SR2 register
		// In while loop SR1 is already read
		while(!(I2C1->SR1 & I2C_SR1_ADDR));
	}
}

/*
void I2C1_EV_IRQHandler(void) {

}
*/

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
	I2C1_Init();

	uint8_t datas[25];
	uint8_t str[25] = "Scordion Electronics";

	for(uint8_t i = 0; i < 20; i++) {
		I2C1_WriteData16bit(EEPROM_SLAVE_ADD, i, str[i]);
		SysTickDelayMs(2);
	}

	I2C1_ReadData16bit(EEPROM_SLAVE_ADD, 0, datas, 20);

	int8_t temp[20];
	for(int8_t i = 0; i < 20; i++) {
		temp[i] = datas[19-i];
	}
	for(int8_t i = 0; i < 20; i++) {
		I2C1_WriteData16bit(EEPROM_SLAVE_ADD, i+50, temp[i]);
		SysTickDelayMs(2);
	}

	I2C1_ReadData16bit(EEPROM_SLAVE_ADD, 50, datas, 20);

  while(1) {

  }
}
