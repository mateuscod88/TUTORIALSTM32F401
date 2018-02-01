#include "stm32f4xx_hal.h"

#include "UART_CONFIG.h"
#include "gpio.h"
#include "ADC.h"

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
uint8_t dataToSendADC[3]={'a','d','c'};
volatile int * dataFromADC;
volatile int temp = 0;
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
	dataFromADC = &temp;
	
	RCC_AHB1ENR |= GPIOA_CLOCK_ENABLE;
	gpio_0_falling_edge();
	NVIC_SetPriority(EXTI0_IRQn,1);
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	gpio_uart2_config();
	
	RCC_AHB1ENR |= GPIOD_CLOCK_ENABLE;
	LEDs = (GPIO_Instance *)GPIOD_addr;
	leds_On(LEDs);
	
	RCC_APB1ENR |= UART2_CLOCK_ENABLE;
	uart1 = (Uart_Instance *)UART2_BASE;
	uart2_config(uart1);
	NVIC_SetPriority(USART2_IRQn,4);
	NVIC_EnableIRQ(USART2_IRQn);
	
	GPIO_InitTypeDef gpio_adc_in5;
  gpio_adc_in5.Mode = GPIO_MODE_ANALOG;
  gpio_adc_in5.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOA,&gpio_adc_in5);
	
	RCC_APB2ENR |= ADC_CLOCK_ENABLE;
	init_ADC();
	//temperatureSensorOn();
	ADC_in5on();
	NVIC_SetPriority(ADC_IRQn,5);
	NVIC_EnableIRQ(ADC_IRQn);
	
	start_Conversion_ADC();
	uart_lock = unlock;
  uint8_t adc[3];
	adc[0] = 'e';
	int ads = *dataFromADC;
	adc[1] = (uint8_t)*dataFromADC;
	adc[2] = 'd';
	Uart_Transmit_Interrupt(dataToSend,3U);
	delay_ms_m();
	Uart_Transmit_Interrupt(adc,3U);
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
void ADC_IRQHandler()
{
	read_ADC(dataFromADC);
	
	
	/*&uint8_t temp_adc[3];
	temp_adc[0] = 'e';
	temp_adc[1] = (uint8_t)c;
	temp_adc[2] = 'h';
	Uart_Transmit_Interrupt(temp_adc,2U);
*/
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
		start_Conversion_ADC();
		uint8_t  adc[3];
		int c = *dataFromADC;
		adc[0] = 'e';
		adc[1] = (uint8_t)*dataFromADC;
		adc[2] = 'd';
		Uart_Transmit_Interrupt(adc,3U);
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
