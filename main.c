#include "stm32f4xx_hal_conf.h"
#include "UART_CONFIG.h"
int main()
{
	RCC_OscInitTypeDef rcc_osc;
	rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	rcc_osc.HSEState = RCC_HSE_ON;
	rcc_osc.PLL.PLLState = RCC_PLL_ON;
	rcc_osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	rcc_osc.PLL.PLLM = 8;
	rcc_osc.PLL.PLLN = 336;
	rcc_osc.PLL.PLLP = 4;	
	rcc_osc.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&rcc_osc);
	
	RCC_ClkInitTypeDef rcc_clock;
	rcc_clock.ClockType =(RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	rcc_clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	rcc_clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
	rcc_clock.APB1CLKDivider = RCC_HCLK_DIV4;
	rcc_clock.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&rcc_clock,FLASH_LATENCY_5);
	
	 int sys_clock = SystemCoreClock;
	RCC_AHB1ENR = GPIOA_CLOCK_ENABLE;
	GPIOA_MODER = PA2_AF_TX | PA3_IO_RX;
	GPIOA_PUPDR = PA2_PULL_UP | PA3_PULL_DOWN;
	GPIOA_AFRL = PA2_AF7 | PA3_AF7;
	
	RCC_APB1ENR = UART2_CLOCK_ENABLE;
	UART2_BRR = UART_MANTIASA | UART_FRACTION;
	
	
}


void USART2_IRQHandler(){

}