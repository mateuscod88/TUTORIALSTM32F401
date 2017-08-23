#define UART_CONFIG
#ifdef UART_CONFIG
#define UART2_BASE 0x40004400
#define UART2_SR (*(unsigned int*)(0x40004400))
#define UART2_DR (*(unsigned int*)(0x40004404))
#define UART2_BRR (*(unsigned int*)(0x40004408))
#define UART2_CR1 (*(unsigned int*)(0x4000440C))
#define UART2_CR2 (*(unsigned int*)(0x40004410))
#define UART2_CR3 (*(unsigned int*)(0x40004414))
#define RCC_APB1ENR (*(unsigned int*)(0x40023840))
#define UART2_CLOCK_ENABLE 1U << 17



#define UART_WORD_8 0U << 12
#define UART_STOP_BIT_0  0x00 << 12
#define UART_MANTIASA 0x0222 << 4
#define UART_FRACTION 0x0E << 0 

#define WORD_LENGHT_8B 0 << 12

//PA2 tx  pa3 rx

#define GPIOA_MODER (*(unsigned int*)(0x40020000))
#define PA2_AF_TX  2U << 4
#define PA3_IO_RX  0U << 6

#define GPIOA_PUPDR (*(unsigned int*)(0x4002000C))
#define PA2_PULL_UP 1U << 4
#define PA3_PULL_DOWN 2U << 6

#define GPIOA_AFRL (*(unsigned int*)(0x40020020))
#define GPIOA_AFRH (*(unsigned int*)(0x40020024))	
#define PA2_AF7 0x0111 << 8
#define PA3_AF7 0x0111 << 12

#define RCC_AHB1ENR  (*(unsigned int*)(0x40023830))
#define GPIOA_CLOCK_ENABLE 1U << 0


	
#endif