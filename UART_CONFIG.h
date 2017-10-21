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
//pc6 uart6 tx pc7 uart6rx
//pa9 uart1tx pa10 uart1rx

#define UART1_BASE  0x40011000
#define UART1_SR (*(unsigned int*)(0x40011000))
#define UART1_DR (*(unsigned int*)(0x40011004))
#define UART1_BRR (*(unsigned int*)(0x40011008))
#define UART1_CR1 (*(unsigned int*)(0x4001100C))
#define UART1_CR2 (*(unsigned int*)(0x40011010))
#define UART1_CR3 (*(unsigned int*)(0x40011014))
#define RCC_APB2ENR (*(unsigned int*)(0x40023844))
#define UART1_CLOCK_ENABLE 1U << 4
	
#define UART_UE 1U << 13
#define UART_TE_SET 1U  << 3
#define UART_TE_CLEAR 0U<< 3

#define UART_WORD_8B 0U << 12
#define UART_STOP_BIT_0  0x00 << 12
#define UART_MANTIASA 0x0222 << 4
#define UART_FRACTION 0x0E << 0 

#define UART_TCIE_EN 1U << 6
#define UART_TCIE_CLEAR ~(1U << 6)
#define UART_TXEIE_EN 1U << 7
#define UART_TXEIE_CLEAR ~(1U << 7)
#define UART_TC_FLAG_SET 1U << 6
#define UART_TXE_FLAG_SET 1U << 7
#define UART_RXNEIE_SET 1U << 5
#define UART_RXNEIE_CLEAR ~(1U << 5)
#define UART_RXNE_FLAG 1U << 5

//PA2 tx  pa3 rx

#define GPIOA_MODER (*(unsigned int*)(0x40020000))
#define PA2_AF_TX  2U << 4
#define PA3_IO_RX  0U << 6

#define PA9_AF_TX  2U << 18
#define PA10_IO_RX  0U << 20

#define GPIOA_PUPDR (*(unsigned int*)(0x4002000C))
#define PA2_PULL_UP 1U << 4
#define PA3_PULL_DOWN 2U << 6
#define PA9_PULL_UP 1U << 18
#define PA10_PULL_DOWN 2U << 20

#define GPIOA_AFRL (*(unsigned int*)(0x40020020))
#define GPIOA_AFRH (*(unsigned int*)(0x40020024))	
#define PA2_AF7 0x0111 << 8
#define PA3_AF7 0x0111 << 12
#define PA9_AF7 0x0111 << 4
#define PA10_AF7 0x0111 << 8

#define RCC_AHB1ENR  (*(unsigned int*)(0x40023830))
	
#define GPIOA_CLOCK_ENABLE 1U << 0




	
#endif