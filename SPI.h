#define  SPI_H	

#ifdef SPI_H
#define SPI1_addr 0x40013000
#define SPI2_addr 0x40003800
#define SPI3_addr 0x40003C00 
#define SPI4_addr 0x40013400 

#define CPOL_SET 1<<  //Idle High
#define CPHA_RESET  ~(1<<) // First Edge
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