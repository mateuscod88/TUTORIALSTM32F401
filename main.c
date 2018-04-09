#include "stm32f4xx_hal.h"

#include "UART_CONFIG.h"
#include "gpio.h"
#include "ADC.h"
#include "SPI.h"
#include "MEMS.h"

#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
uint8_t aTxBuffer[] = " ";

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];
typedef struct UART_DATAS{
	uint8_t * pTxBuffer;
	uint8_t * pRxBuffPtr;
	volatile uint16_t Size;
	volatile uint16_t SizeCounter;
	
}Uart_data;
uint8_t rx_bfr[10];
uint8_t tx_bfr[10];
SPI_Data * ptr_spi_data;
SPI_Data spi_data;
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
	data = &dataA;
	RCC_AHB1ENR |= GPIOA_CLOCK_ENABLE;
	gpio_0_falling_edge();
	NVIC_SetPriority(EXTI0_IRQn,1);
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	gpio_uart2_config();
	//LEDS INIT
	RCC_AHB1ENR |= GPIOD_CLOCK_ENABLE;
	LEDs = (GPIO_Instance *)GPIOD_addr;
	leds_On(LEDs);
	//UART2 INIT
	RCC_APB1ENR |= UART2_CLOCK_ENABLE;
	uart1 = (Uart_Instance *)UART2_BASE;
	uart2_config(uart1);
	NVIC_SetPriority(USART2_IRQn,4);
	NVIC_EnableIRQ(USART2_IRQn);
	//ADC INIT
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
	//Uart_Transmit_Interrupt(dataToSend,3U);
	delay_ms_m();
	//Uart_Transmit_Interrupt(adc,3U);
	LEDs->GPIOx_ODR &= ~GPIOx_ODR_bit_set(ORANGE_LED_ON);
	LEDs->GPIOx_ODR &= ~GPIOx_ODR_bit_set(BLUE_LED_ON);
//SPI Init
	gpio_spi_config();
	SPI_Init();
	
	aTxBuffer[0] = READ_FROM_ONE(WHO_AM_I_REG);
	tx_bfr[0] = READ_FROM_ONE(WHO_AM_I_REG);
	tx_bfr[1] ='m';
	tx_bfr[2] ='a';
	tx_bfr[3] ='l';
	rx_bfr[0] = 't';
	rx_bfr[1] = 'h';
	rx_bfr[2] = 'r';
	//SPI Data INIT
	spi_data.ptTxBuffer = tx_bfr;
	spi_data.ptRxBuffer = rx_bfr;
	spi_data.Size = 1;
	spi_data.SizeCounter = 1;
	ptr_spi_data = &spi_data;
	
	while(1){
		
		SPI_Send(ptr_spi_data);
		
		delay_ms_m();
		
		LEDs->GPIOx_ODR |= GPIOx_ODR_bit_set(GREEN_LED_ON) ;
		LEDs->GPIOx_ODR |= GPIOx_ODR_bit_set(ORANGE_LED_ON) ;
		LEDs->GPIOx_ODR	|= GPIOx_ODR_bit_set(RED_LED_ON);
		
	  //Uart_Transmit_Interrupt(ptr_spi_data->ptRxBuffer,1U);
		
		delay_ms_m();
	}

}

void USART2_IRQHandler()
{
	LEDs->GPIOx_ODR |= GPIOx_ODR_bit_set(BLUE_LED_ON);
	//LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(BLUE_LED_ON));
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
		LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(GREEN_LED_ON));
		NVIC_DisableIRQ(EXTI0_IRQn);
		start_Conversion_ADC();
		uint8_t  adc[3];
		int c = *dataFromADC;
		adc[0] = 'e';
		adc[1] = (uint8_t)*dataFromADC;
		adc[2] = 'd';
		//Uart_Transmit_Interrupt(adc,3U);
	}
	
}

void Uart_Transmit_Interrupt(uint8_t * pTxData, uint16_t Size)
{
	if(uart1->UART_SR & UART_TXE_FLAG_SET)
	{
		
		data = &dataA;
		data->pTxBuffer = pTxData;
		data->Size = Size;
		data->SizeCounter = Size;
		//UART interrupt Enable
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
	LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(BLUE_LED_ON));
	//Receiver handler here 
	if(uart1->UART_SR & UART_RXNE_FLAG && uart1->UART_CR1 & UART_RXNEIE_SET)
	{
		LEDs->GPIOx_ODR &= ~(GPIOx_ODR_bit_set(GREEN_LED_ON));
		Receive_Data(data);
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
void Receive_Data(volatile Uart_data * dataa)
{
	
	aTxBuffer[0] = (uart1->UART_DR) & (uint8_t)0x00FFU;
	dataa->pRxBuffPtr = aTxBuffer;
	Uart_Transmit_Interrupt(dataa->pRxBuffPtr,1U);
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
		uart1->UART_DR = (uint8_t)(*data->pTxBuffer++ & (uint8_t)0x00FFU);
		data->SizeCounter--;
	}
	if(data->SizeCounter == 0)
	{
		uart1->UART_CR1 &= UART_TXEIE_CLEAR;
		uart1->UART_CR1 |= UART_TCIE_EN;
	}
	return;
}
void Gpio_Init(GPIO_Instance * gpio){

}
void SysTick_Handler()
{
}

	
