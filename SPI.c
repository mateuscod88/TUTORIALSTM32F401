#include "SPI.h"

void SPI_Init(){
	/*
	CONFIG
	
	BR[2:0] CR1 baudrate
	CPOL CPHA
	DFF 8 or 16 bit dataframe
	LSBFIRST  when TI mode set not required
	NSS out SSOE should  be set  not requierd when TI mode
	FRF in CR2 to select TI protocol
	MSTR SPE must set when NSS connected to high level
	
	*/
	SPI_INSTANCE * spi_instance = (SPI_INSTANCE*)SPI1_addr;
	spi_instance->SPI_CR1 |= NSS_BY_SOFT;
	spi_instance->SPI_CR1 |= 


} 
/*
-----Transmit---
transmit start when byte written to Tx Buffer
TXE ustawione podczas transmisji
interrupt genereted when TXEIE in CR2 is set

*/
/*
------Receive---
when data in RX buffer RXNE is setR
RXNEIE set in CR2 to generate interrupt
RXNE jest czyszczony po odczycie DR

*/