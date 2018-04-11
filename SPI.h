
#define  SPI_H	
#include "stm32f4xx_hal.h"
#ifdef SPI_H
#define SPI1_addr 0x40013000
#define SPI2_addr 0x40003800
#define SPI3_addr 0x40003C00 
#define SPI4_addr 0x40013400 
/*
PA5 SCK
PA6 MISO
PA7 MOSI
PE3 CS

*/
#define CPOL_SET 1U << 1  //Idle High
#define CPOL_RESET ~(1U << 1)
#define CPHA_RESET  ~(1U << 0) // First Edge
#define CPHA_SET 1U << 0
#define RCC_APB2_EN   (*(unsigned int*)(0x40023844))
#define SPI1_CLOCK_EN 1U << 12
#define SPI1_BR  7U << 3
#define SPI1_DFF_8B ~(1U << 11)
#define SPI1_DFF_16B 1U << 11
#define SPI1_MSBFIRST  ~(1U << 7)
#define SPI1_LSBFIRST  1U << 7
#define SPI1_SPE_EN  1U << 6
#define SPI1_SPE_DIS  ~(1U << 6)
#define SPI1_SSM 1U << 9
#define SPI1_SSI 1U << 8
#define SPI1_MSTR 1U << 2
#define SPI_RXNE_SET 1U << 0
#define SPI_TXE_SET 1U << 1
#define GPIO_CS_LOW 1U << 19
#define GPIO_CS_HIGH 1U << 3
typedef struct{
	int SPI_CR1;
	int SPI_CR2;
	int SPI_SR;
	int SPI_DR;
	int SPI_CPCPR;
	int SPI_RXCRCR;
	int SPI_TXCRCR;
	int SPI_I2SCFGR;
	int SPI_I2SSPR;
}SPI_INSTANCE;

typedef struct{
	uint16_t *ptTxBuffer;
	uint8_t *ptRxBuffer;
	uint16_t Size;
	uint16_t SizeCounter;
}SPI_Data;

void SPI_Init();
void SPI_Send(SPI_Data * data);
void SPI_Read(SPI_Data * data);
void gpio_spi_config();

#endif