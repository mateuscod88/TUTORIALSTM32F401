#include "SPI.h"
#include "gpio.h"
	/*
	CONFIG
	
	BR[2:0] CR1 baudrate
	CPOL CPHA
	DFF 8 or 16 bit dataframe
	LSBFIRST  when TI mode set not required
	NSS out SSOE should  be set  not requierd when TI mode  PE3
	FRF in CR2 to select TI protocol
	MSTR SPE must set when NSS connected to high level
	SPI1  AP2 fpclk = 42Mhz
	*/
	/*
PA5 SCK
PA6 MISO
PA7 MOSI
PE3 CS

*/
void SPI_Init(){
	RCC_APB2_EN |=  SPI1_CLOCK_EN;
	SPI_INSTANCE * spi_instance = (SPI_INSTANCE*)SPI1_addr;
	
	spi_instance->SPI_CR1 |= SPI1_BR;
	spi_instance->SPI_CR1 |= CPOL_SET;
	spi_instance->SPI_CR1 &= CPHA_RESET;
	spi_instance->SPI_CR1 &= SPI1_DFF_8B;
	spi_instance->SPI_CR1 &= SPI1_MSBFIRST;
	spi_instance->SPI_CR1 |= SPI1_SSM;
	spi_instance->SPI_CR1 |= SPI1_SSI;
	spi_instance->SPI_CR1 |= SPI1_MSTR;
	spi_instance->SPI_CR1 |= SPI1_SPE_EN;
	
} 
void SPI_Send(SPI_Data * data){
	
	SPI_INSTANCE * spi_instance = (SPI_INSTANCE*)SPI1_addr;
	GPIO_Instance * spi_pin_cs = (GPIO_Instance*)GPIOE_addr;
	GPIO_Instance * LEDs = (GPIO_Instance *)GPIOD_addr;
	if(data->SizeCounter == 0){
		spi_instance->SPI_CR1 &= SPI1_SPE_DIS;
	}
	if(spi_instance->SPI_SR & SPI_TXE_SET){
		spi_pin_cs->GPIOx_BSRR |= GPIO_CS_LOW;
		spi_instance->SPI_DR = (uint8_t)(*data->ptTxBuffer & (uint8_t)0x00FFU);
		
		
	}
	uint8_t counter = 0;
	
	while(spi_instance->SPI_SR & SPI_RXNE_SET && counter != 255)
	{
		counter++;
		LEDs->GPIOx_ODR |= GPIOx_ODR_bit_set(15);
	}
	LEDs->GPIOx_ODR &= ~GPIOx_ODR_bit_set(15);
		*data->ptRxBuffer = (uint8_t)spi_instance->SPI_DR; 
	spi_pin_cs->GPIOx_BSRR |= GPIO_CS_HIGH;
	
	
}

void SPI_Read(SPI_Data * data){
	SPI_INSTANCE * spi_instance = (SPI_INSTANCE*)SPI1_addr;
	GPIO_Instance * spi_pin_cs = (GPIO_Instance*)GPIOE_addr;
	GPIO_Instance * LEDs = (GPIO_Instance *)GPIOD_addr;
	
	if(spi_instance->SPI_SR & SPI_RXNE_SET)
	{
		LEDs->GPIOx_ODR |= GPIOx_ODR_bit_set(15);
		*data->ptRxBuffer = (uint8_t)spi_instance->SPI_DR; 
	}
	spi_pin_cs->GPIOx_BSRR |= GPIO_CS_HIGH;
}
void gpio_spi_config(){
	GPIO_Instance * spi_pin = (GPIO_Instance*)GPIOA_addr;
	GPIO_Instance * spi_pin_cs = (GPIO_Instance*)GPIOE_addr;
	RCC_AHB1ENR |= GPIOA_CLOCK | GPIOE_CLOCK;
	//SCK Config
	spi_pin->GPIOx_PUPDR &= ~(3U << 10);
	spi_pin->GPIOx_MODER &= ~(3U << 10);
	spi_pin->GPIOx_MODER |= GPIO_MODER(GPIO_MODER_PIN_5);
	spi_pin->GPIOx_SPEED |= GPIO_SPEED_FAST(GPIO_SPEED_PIN_5);
	spi_pin->GPIOx_AFRL |= GPIO_AFRL(GPIO_AFRL_PIN_5);
	spi_pin->GPIOx_PUPDR |=1U << 10;
	
	//MOSI Config
	spi_pin->GPIOx_PUPDR &= ~(3U << 14);
	spi_pin->GPIOx_MODER &= ~(3U << 14);
	spi_pin->GPIOx_MODER |= GPIO_MODER(GPIO_MODER_PIN_7);
	spi_pin->GPIOx_SPEED |= GPIO_SPEED_FAST(GPIO_SPEED_PIN_7);
	spi_pin->GPIOx_AFRL |= GPIO_AFRL(GPIO_AFRL_PIN_7);
	spi_pin->GPIOx_PUPDR |=1U << 14;
	
	//MISO Config
	spi_pin->GPIOx_PUPDR &= ~(3U << 12);
	spi_pin->GPIOx_MODER &= ~(3U << 12);
	spi_pin->GPIOx_MODER |= GPIO_MODER(GPIO_MODER_PIN_6);
	spi_pin->GPIOx_SPEED |= GPIO_SPEED_FAST(GPIO_SPEED_PIN_6);
	spi_pin->GPIOx_AFRL |= GPIO_AFRL(GPIO_AFRL_PIN_6);
	spi_pin->GPIOx_PUPDR |= 1U << 12;
	
	//CS
	spi_pin_cs->GPIOx_PUPDR &= ~(3U << 6);
	spi_pin_cs->GPIOx_MODER |= 1U << 6;
	spi_pin_cs->GPIOx_SPEED |= GPIO_SPEED_FAST(GPIO_SPEED_PIN_3);
	//spi_pin_cs->GPIOx_AFRL |= GPIO_AFRL(GPIO_AFRL_PIN_3);
	spi_pin_cs->GPIOx_PUPDR |= GPIO_PULL_UP(GPIO_PULL_UP_PIN_3);
	spi_pin_cs->GPIOx_BSRR |= GPIO_CS_HIGH;
}
/*
-----Transmit---
transmit start when byte written to Tx Buffer
TXE ustawione podczas transmisji
interrupt genereted when TXEIE in CR2 is set
1.SPI enable SPE = 1
2.Write data to DR this clears TXE
3.Wait until TXE = 1 and write next byte
then wait RXNE = 1 and read DR this clears RXNE and repeat
4.Wait until RXNE = 1 and TXE = 1 and BSY = 0 before disabling SPI

*/
/*
------Receive---
when data in RX buffer RXNE is setR
RXNEIE set in CR2 to generate interrupt
RXNE jest czyszczony po odczycie DR

*/
/*
----TI protcol----
FRF in CR2  set TI protocol
NSS set in CR1 and CR2 SSM SSI SSOE

*/
