#include "stm32f4xx_hal_conf.h"
#include "UART_CONFIG.h"

typedef struct UART_DATAS{
	uint8_t * pTxBuffer;
	uint8_t * pRxBuffPtr;
	uint16_t Size;
	uint16_t SizeCounter;
	
}Uart_data;
Uart_data * data;
void Uart_Handler(Uart_data * data);
void Uart_Transmit_Interrupt(uint8_t * pTxData, uint16_t Size);
void Send_Data(Uart_data * data);
void Receive_Data();
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
	GPIOA_MODER = PA9_AF_TX | PA10_IO_RX;
	GPIOA_PUPDR = PA9_PULL_UP | PA10_PULL_DOWN;
	GPIOA_AFRH = PA9_AF7 | PA9_AF7;
	
	RCC_APB2ENR = UART1_CLOCK_ENABLE;
	UART1_CR1 |= UART_UE;
	UART1_CR1 |= UART_WORD_8B;
	UART1_CR2 |= UART_STOP_BIT_0;
	UART1_BRR |= UART_MANTIASA | UART_FRACTION;
	UART1_CR1 |= UART_TCIE_EN;
	
	uint8_t dataToSend[3]={'h','u','j'};
	
	Uart_Transmit_Interrupt(dataToSend,3);
	
	
	
}


void USART1_IRQHandler()
{
	Uart_Handler(data);
}

void Uart_Transmit_Interrupt(uint8_t * pTxData, uint16_t Size)
{
	if(UART1_SR & UART_TXE_FLAG_SET)
	{
		data->pTxBuffer = pTxData;
		data->Size = Size;
		data->SizeCounter = Size;
		UART1_CR1|= UART_TXEIE_EN;
		return;
	}
}
void Uart_Receive_Interrupt(uint8_t * pRxData)
{
	UART1_CR1 |= UART_RXNEIE_SET;
	data->pRxBuffPtr = pRxData;
}
void Uart_Handler(Uart_data * data)
{
	//Receiver handler here 
	if(UART1_SR & UART_RXNE_FLAG && UART1_CR1 & UART_RXNEIE_SET)
	{
		Receive_Data();
	}
	if(UART1_SR & UART_TXE_FLAG_SET && UART1_CR1 & UART_TXEIE_EN)
	{
		Send_Data(data);
	}
	if(UART1_SR & UART_TC_FLAG_SET && UART1_CR1 & UART_TCIE_EN) 
	{
		UART1_CR1 &= UART_TCIE_CLEAR;
		//Callback TX
	}
}
void Receive_Data()
{
	
	*data->pRxBuffPtr++ = (UART1_DR) & (uint8_t)0x00FFU;
	//(uint8_t)((*(uint8_t)UART2_DR) & (uint8_t)0x00FFU);
	/*if(UART2_SR & UART_RXNE_FLAG)
	{
		UART2_CR1 &= UART_RXNEIE_CLEAR;
	}
	For future purposes
	*/
    
}
void Send_Data(Uart_data * data)
{
	if(UART1_SR & UART_TXE_FLAG_SET)
	{
		UART1_DR = (uint8_t)(*data->pTxBuffer++ & (uint8_t)0x00FFU);
		data->SizeCounter--;
	}
	if(data->SizeCounter == 0)
	{
		UART1_CR1 &= UART_TXEIE_CLEAR;
		UART1_CR1 |= UART_TCIE_EN;
	}
	return;
}
// send Data in pooling mode 
void sendData( uint8_t * pTxData, uint16_t Size)
{
	uint16_t counter = Size;
	while(counter > 0)
	{
		UART1_CR1 |= UART_TE_SET;
		while(UART1_SR & UART_TXE_FLAG_SET);
		UART1_DR = (uint8_t)(*pTxData++ & (uint8_t)0x00FFU);
		pTxData++;
		counter--;
	}
	while(UART1_SR & UART_TC_FLAG_SET);
	UART1_CR1 |= UART_TE_CLEAR;
}