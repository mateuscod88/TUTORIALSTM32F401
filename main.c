#include "stm32f4xx_hal.h"

#include "UART_CONFIG.h"
#include "gpio.h"

typedef struct UART_DATAS{
	uint8_t * pTxBuffer;
	uint8_t * pRxBuffPtr;
	volatile uint16_t Size;
	volatile uint16_t SizeCounter;
	
}Uart_data;
Uart_data dataA;
volatile Uart_data * data;
enum LOCK {lock,unlock};
enum LOCK uart_lock = lock;
uint8_t dataToSend[3]={'h','u','j'};
Uart_Instance * uart1;
GPIO_Instance * LEDs;
void delay_ms_m();
void Uart_Handler(volatile Uart_data * data);
void Uart_Transmit_Interrupt(uint8_t * pTxData, uint16_t Size);
void Send_Data(volatile Uart_data * data);
void Receive_Data();
void sysConfig();
int main()
{	
	sysConfig();
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
	GPIO_InitTypeDef gpio_push_button;
  gpio_push_button.Mode = GPIO_MODE_IT_FALLING;
  gpio_push_button.Pull = GPIO_NOPULL;
  gpio_push_button.Pin = GPIO_PIN_0;
  
	HAL_GPIO_Init(GPIOA,&gpio_push_button);
	
	RCC_AHB1ENR |= GPIOA_CLOCK_ENABLE;
	GPIO_Instance * push_button = (GPIO_Instance*)GPIOA_BASE;
	push_button->GPIOx_MODER |= PA2_AF_TX |PA3_AF_RX;
	push_button->GPIOx_SPEED |= 2U << 4 | 2U << 6;
	push_button->GPIOx_AFRL |= PA2_AF7 | PA3_AF7;
	
	
	RCC_AHB1ENR |= GPIOD_CLOCK_ENABLE;
	
	LEDs = (GPIO_Instance *)GPIOD_addr;
	LEDs->GPIOx_PUPDR |= GPIOx_PUPDR_pull_up(GPIOx_PIN_12) 
										| GPIOx_PUPDR_pull_up(GPIOx_PIN_13) 
										| GPIOx_PUPDR_pull_up(14)
										| GPIOx_PUPDR_pull_up(15);
	LEDs->GPIOx_MODER |= GPIOx_MODER_output(GPIOx_PIN_12) 
										| GPIOx_MODER_output(GPIOx_PIN_13) 
										| GPIOx_MODER_output(14)
										| GPIOx_MODER_output(15);
	LEDs->GPIOx_TYPER &= GPIOx_TYPER_push_pull(GPIOx_PIN_12);
	LEDs->GPIOx_ODR |= GPIOx_ODR_bit_set(GPIOx_PIN_12) 
									| GPIOx_ODR_bit_set(GPIOx_PIN_13) 
									| GPIOx_ODR_bit_set(14)
									| GPIOx_ODR_bit_set(15);
	NVIC_SetPriority(EXTI0_IRQn,1);
	NVIC_EnableIRQ(EXTI0_IRQn);
	//GPIO_Instance * push_button = (GPIO_Instance*)GPIOA_BASE;
	/*push_button->GPIOx_PUPDR |= 0U << 0;//pull down
	push_button->GPIOx_MODER |= 0U << 0;//Input mode
	push_button->GPIOx_TYPER |= 0U << 0;//Push pull output type
	
	RCC_APB2ENR |= SYSCFG_CLCK_ENABLE;
	SYSCFG_Instance * push_button_SYSCFG = (SYSCFG_Instance * )SYSCFG_addr;
	push_button_SYSCFG->SYSCFG_EXTICR1 |= 0x0000 << 0;
	
	EXTI_Instance * push_button_EXTI = (EXTI_Instance * )EXTI_addr;
	push_button_EXTI->EXT_IMR |= 1U << 0; // interrupt masked
	push_button_EXTI->EXT_RTSR |= 1U << 0;
	*/
	//push_button_EXTI->EXT_FTSR |= 1U << 0;
	
	
	  
	
	
	 
	RCC_APB1ENR = UART2_CLOCK_ENABLE;
	uart1 = (Uart_Instance *)UART2_BASE;
	uart1->UART_CR1 |= UART_UE;
	uart1->UART_CR1 |= UART_WORD_8B;
	uart1->UART_CR2 |= UART_STOP_BIT_0;
	uart1->UART_BRR |= UART_MANTIASA | UART_FRACTION;
	uart1->UART_CR1 |=   UART_TE_SET;
	NVIC_SetPriority(USART2_IRQn,4);
	NVIC_EnableIRQ(USART2_IRQn);
	
	uart_lock = unlock;
  
	Uart_Transmit_Interrupt(dataToSend,3U);
	LEDs->GPIOx_ODR &= ~GPIOx_ODR_bit_set(13);
	while(1){
		delay_ms_m();
		LEDs->GPIOx_ODR |= GPIOx_ODR_bit_set(GPIOx_PIN_12) 
										| GPIOx_ODR_bit_set(GPIOx_PIN_13) 
										| GPIOx_ODR_bit_set(14)
										| GPIOx_ODR_bit_set(15);
	}

}

void USART2_IRQHandler()
{
	Uart_Handler(data);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(){	
	/*while(LEDs->GPIOx_IDR & 1U << 0){}
	 EXTI_PENDING = 1U<<0;
	*/
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(uart_lock == 1){
		LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(12));
		NVIC_DisableIRQ(EXTI0_IRQn);
		Uart_Transmit_Interrupt(dataToSend,3U);
	}
	
}

void Uart_Transmit_Interrupt(uint8_t * pTxData, uint16_t Size)
{
	if(uart1->UART_SR & UART_TXE_FLAG_SET)
	{
		
		data = &dataA;
		data->pTxBuffer = pTxData;
		if((*data->pTxBuffer) == 'h' || (*data->pTxBuffer) == 'u' ||(*data->pTxBuffer) == 'j'){
			LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(15));
		}
		data->Size = 3U;
		data->SizeCounter = 3U;
		uart1->UART_CR1 |= UART_TXEIE_EN;
		return;
	}
}
void Uart_Receive_Interrupt(uint8_t * pRxData)
{
	uart1->UART_CR1 |= UART_RXNEIE_SET;
	data->pRxBuffPtr = pRxData;
}
void Uart_Handler(volatile Uart_data * data)
{
	//Receiver handler here 
	if(uart1->UART_SR & UART_RXNE_FLAG && uart1->UART_CR1 & UART_RXNEIE_SET)
	{
		Receive_Data();
	}
	if(uart1->UART_SR & UART_TXE_FLAG_SET && uart1->UART_CR1 & UART_TXEIE_EN)
	{
		Send_Data(data);
	}
	if(uart1->UART_SR & UART_TC_FLAG_SET && uart1->UART_CR1 & UART_TCIE_EN) 
	{
		uart1->UART_CR1 &= UART_TCIE_CLEAR;
		//Callback TX
	}
}
void Receive_Data()
{
	
	*data->pRxBuffPtr++ = (uart1->UART_DR) & (uint8_t)0x00FFU;
	//(uint8_t)((*(uint8_t)UART2_DR) & (uint8_t)0x00FFU);
	/*if(UART2_SR & UART_RXNE_FLAG)
	{
		UART2_CR1 &= UART_RXNEIE_CLEAR;
	}
	For future purposes
	*/
    
}
void sysConfig(){
	HAL_Init();
	 __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
void delay_ms_m(){
	for(int i=0;i < 1000;i++){
		for(int j = 0;j < 15000;j++){
		}
	}
}
void Send_Data(volatile Uart_data * data)
{
	if(uart1->UART_SR & UART_TXE_FLAG_SET)
	{
		LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(13));
		if(data->SizeCounter < 0 || data->SizeCounter > 3){
			LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(14));
		}
		uart1->UART_DR = (uint8_t)(*data->pTxBuffer++ & (uint8_t)0x00FFU);
		data->SizeCounter--;
	}
	if(data->SizeCounter == 0)
	{
		LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(13));
		uart1->UART_CR1 &= UART_TXEIE_CLEAR;
		uart1->UART_CR1 |= UART_TCIE_EN;
		uart_lock = unlock;
	}
	return;
}
void Gpio_Init(GPIO_Instance * gpio){

}
void SysTick_Handler()
{
}
