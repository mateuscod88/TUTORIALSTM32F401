#define UART_CONFIG
#ifdef UART_CONFIG
#define UART2_BASE 0x40004400
#define UART2_SR (*(unsigned int*)(0x40004400))
#define UART2_DR (*(unsigned int*)(0x40004404))
#define UART2_SR (*(unsigned int*)(0x40004408))
#define UART2_CR1 (*(unsigned int*)(0x4000440C))
#define UART2_CR2 (*(unsigned int*)(0x40004410))
#define UART2_CR3 (*(unsigned int*)(0x40004414))

	
#endif