# How to Use CMSIS Library for Cortex M4 Projects

1. Download latest version of CMSIS source files https://github.com/ARM-software/CMSIS_5/releases
2. Download STM32CubeF4 files https://github.com/STMicroelectronics/STM32CubeF4 (or which uC you are use)
3. Create STM32 project on STM32CubeIDE File>New>STM32 Project (Targeted Language > C, Targeted Binary Type > Executable, Targeted Project Type > Empty).
4. Copy-Paste "cmsis_compiler.h, cmsis_gcc.h, cmsis_version.h, core_cm4.h, mpu_armv7.h" from "CMSIS_5-5.6.0\CMSIS\Core\Include" file and Copy-Paste "stm32f401xe.h (or which you are use), stm32f4xx.h, system_stm32f4xx.h" from "Device\ST\STM32F4xx\Include" to your project Inc file.

## Debug Configuration
1. Click Run > Debug Configuration. 
2. Then double click to "STM32 Cortex-M C/C++ Application". 
3. In the Main section make sure that is "C/C++ Application" is "Debug\ProjectName.elf".
4. Select "Build Configuration" to Debug (Optional).
5. Click Apply button.
6. Select Debugger section.
7. (For Nucleo Board tested) Set debug probe to ST-LINK (ST-LINK GDB server).
8. Tick "Use spesific ST-LINK S/N" and click scan for your ST-LINK.
9. Click Apply and then Close.

## Test Code

You can test your project with code below.

```c
#include "stm32f4xx.h"
#include "stdint.h"

void delayMs(uint32_t delay) {
	for(; delay > 0; delay--)
		for(uint32_t i = 0; i < 3195; i++);
}

int main(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	// Set PB15 as output
	GPIOA->MODER |= GPIO_MODER_MODER5_0;
	// Set PC13 as input
	GPIOC->MODER &= ~(GPIO_MODER_MODER13);

	while(1) {
		if(GPIOC->IDR & GPIO_IDR_IDR_13) {
			// Turn off PA5
			GPIOA->BSRR |= GPIO_BSRR_BR5;
		} else {
			// Turn on PA5
			//GPIOA->ODR |= GPIO_ODR_ODR_5;
			GPIOA->BSRR |= GPIO_BSRR_BS5;
			delayMs(200);
			// Turn off PA5
			//GPIOA->ODR &= ~(GPIO_ODR_ODR_5);
			GPIOA->BSRR |= GPIO_BSRR_BR5;
			delayMs(200);
		}
	}
}
```
