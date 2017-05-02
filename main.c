#include "stm32f4xx_hal_conf.h"
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
}