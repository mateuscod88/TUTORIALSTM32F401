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
#define ADC1_COMMON_BASE_ADDR 0x40012304
#define ADC_PRESC_2 	  0U << 16
#define ADC_PRESC_4 	  1U << 16
#define ADC_PRESC_6 	  2U << 16
#define ADC_PRESC_8 	  3U << 16

#define ADC_VBAT 			1U << 22 
#define ADC_TSFRFEE 	1U << 23

#define ADC_RES_12Bit 0U << 24
#define ADC_RES_10Bit 1U << 24
#define ADC_RES_8Bit 	2U << 24
#define ADC_RES_6Bit 	3U << 24

#define ADC_SCANMODE_SET 1U << 8
#define ADC_DATA_ALIGNMENT_LEFT 1U << 11
#define ADC_CCONVERSION_CLR ~(1U << 1)
#define ADC_CCONVERSION_SET 1U << 1

#define ADC_DISCONT_SET 1U << 11
#define ADC_DISCONT_CLR ~(1U << 11)

#define ADC_NUM_OF_CONV_L ~(16U << 20) 
#define ADC_CHANEL_FIRST 18U << 0
#define ADC_EOCIE 1U << 5
#define ADC_SWSTART 1U << 30
#define ADC_SAMPLING_15C 1U << 18
#define ADC_SAMPLING_IN5_15C 1U << 15
#define ADC_SEQUENCE_FIRST 16U << 0
#define ADC_SEQUENCE_FIRST_IN5 5U << 0
#define ADC_EOCIE_ENABLE 1U<<5
#define ADC_EOCIE_DISABLE ~(1U<<5)
#define ADC_CLOCK_ENABLE 1U << 8
#define ADC_ADON  1U << 0 
typedef struct
{
	int ADC_CCR;
}ADC_COMMON_INSTANCE;

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
	int ADC_SQR3;
	int ADC_JSQR;
	int ADC_JDR1;
	int ADC_JDR2;
	int ADC_JDR3;
	int ADC_JDR4;
	int ADC_DR;
	
}ADC_INSTANCE;



void init_ADC();
void temperatureSensorOn();
void ADC_in5on();
void start_Conversion_ADC();
void read_ADC(volatile int * data);
void delay_milisec();

#endif