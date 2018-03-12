#define  SPI_H	

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
#define CPHA_RESET  ~(1U << 0) // First Edge
#define RCC_APB2_EN   (*(unsigned int*)(0x40023844))
#define SPI1_CLOCK_EN 1U << 12
#define SPI1_BR  0U << 3
#define SPI1_DFF_8B ~(1U << 11)
#define SPI1_LSBFIRST  ~(1U << 7)
#define SPI1_SPE_EN  1U << 6
typedef struct{
	int SPI_CR1;
	int SPI_SR;
	int SPI_DR;
	int SPI_CPCPR;
	int SPI_RXCRCR;
	int SPI_TXCRCR;
	int SPI_I2SCFGR;
	int SPI_I2SSPR;
}SPI_INSTANCE;

void SPI_Init();
void SPI_Send();
void SPI_Read();

#endif