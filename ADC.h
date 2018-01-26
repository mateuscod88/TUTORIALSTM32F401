#define __ADC_H
#ifdef __ADC_H

//ADC_CR2 = ADON turn on ADC
//Conversion start SWSTART
// ADC_CLK = APB2 /2/4/8
//DIGITAL CLK  RCC_APB2ENR
/* -----Single Conversion------
CONT=0
set SWSTART or JSWSTART
set External Triger
after conversion done
--Regular channel
data in ADC_DR
flag EOC set
interrupt genereted when EOCIE set
--Injected channel
data in ADC_JDR1
flag JEOC set
interrupt genereted when JEOCIE set
ADC STOP

*/
#define ADC1_BASE_ADDR 0x40012000
#define ADC_RES_12Bit 0U << 24
#define ADC_RES_10Bit 1U<< 24
#define ADC_RES_8Bit 	2U<< 24
#define ADC_RES_6Bit 	3U<< 24

#define ADC_EOCIE_ENABLE 1U<<5
#define ADC_EOCIE_DISABLE ~(1U<<5)
typedef struct{
	int ADC_SR;
	int ADC_CR1;
	int ADC_CR2;
	int ADC_SMPR1;
	int ADC_SMPR2;
	int ADC_JOFR1;
	int ADC_JOFR2;
	int ADC_JOFR3;
	int ADC_JOFR4;
	int ADC_HTR;
	int ADC_LTR;
	int ADC_SQR1;
	int ADC_SQR2;
	int ADC_JSQR;
	int ADC_JDR1;
	int ADC_JDR2;
	int ADC_JDR3;
	int ADC_JDR4;
	int ADC_DR;
}ADC_INSTANCE;

typedef struct
{
	int ADC_CCR;
}ADC_COMMON_INSTANCE;

void init_ADC();
void start_Conversion_ADC();
void read_ADC();

#endif