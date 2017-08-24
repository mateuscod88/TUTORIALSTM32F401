#include "stm32f4xx_hal_conf.h"
#include "UART_CONFIG.h"

typedef struct UART_DATAS{
	uint8_t * pTxBuffer;
	uint16_t Size;
	uint16_t SizeCounter;
}Uart_data;
Uart_data * data;
void sendData( uint8_t * pTxData, uint16_t Size);
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
	UART2_CR1 |= UART_UE;
	UART2_CR1 |= UART_WORD_8B;
	UART2_CR2 |= UART_STOP_BIT_0;
	UART2_BRR |= UART_MANTIASA | UART_FRACTION;
	UART2_CR1 |= UART_TCIE_EN;
	
	uint8_t dataToSend[3]={'h','u','j'};
	
	Uart_Transmit_Interrupt(dataToSend,3);
	
	
	
}


void USART2_IRQHandler()
{
	Uart_Handler(data);
}

void Uart_Transmit_Interrupt(uint8_t * pTxData, uint16_t Size)
{
	if(UART2_SR & UART_TXE_FLAG_SET)
	{
		data->pTxBuffer = pTxData;
		data->Size = Size;
		data->SizeCounter = Size;
		UART2_CR1|= UART_TXEIE_EN;
		return;
	}
}
void Uart_Handler(Uart_data * data)
{
	//Receiver handler here 
	if(UART2_SR & UART_TXE_FLAG_SET && UART2_CR1 & UART_TXEIE_EN)
	{
		Send_Data(data);
	}
	if(UART2_SR & UART_TC_FLAG_SET && UART2_CR1 & UART_TCIE_EN) 
	{
		UART2_CR1 &= UART_TCIE_CLEAR;
		//Callback TX
	}
}
void Send_Data(Uart_data * data)
{
	if(UART2_SR & UART_TXE_FLAG_SET)
	{
		UART2_DR = (uint8_t)(*data->pTxBuffer++ & (uint8_t)0x00FFU);
		data->SizeCounter--;
	}
	if(data->SizeCounter == 0)
	{
		UART2_CR1 &= UART_TXEIE_CLEAR;
		UART2_CR1 |= UART_TCIE_EN;
	}
	return;
}
// send Data in pooling mode 
void sendData( uint8_t * pTxData, uint16_t Size)
{
	uint16_t counter = Size;
	while(counter > 0)
	{
		UART2_CR1 |= UART_TE_SET;
		while(UART2_SR & UART_TXE_FLAG_SET);
		UART2_DR = (uint8_t)(*pTxData++ & (uint8_t)0x00FFU);
		pTxData++;
		counter--;
	}
	while(UART2_SR & UART_TC_FLAG_SET);
	UART2_CR1 |= UART_TE_CLEAR;
}