#include "UART_CONFIG.h"

void uart2_config(Uart_Instance * uart){
	
	uart->UART_CR1 |= UART_UE;
	uart->UART_CR1 |= UART_WORD_8B;
	uart->UART_CR2 |= UART_STOP_BIT_0;
	uart->UART_BRR |= UART_MANTIASA | UART_FRACTION;
	uart->UART_CR1 |= UART_TE_SET | UART_RE_SET;
	uart->UART_CR1 |= UART_RXNEIE_SET;
}