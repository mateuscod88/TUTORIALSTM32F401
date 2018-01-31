#include "ADC.h"

void init_ADC(){
	ADC_INSTANCE * ADC_Instance = (ADC_INSTANCE*)ADC1_BASE_ADDR;
	ADC_COMMON_INSTANCE* ADC_COMMON_Instance = (ADC_COMMON_INSTANCE*)ADC1_COMMON_BASE_ADDR;
	// Prescaler divided 2
	ADC_Instance->ADC_CR2 |= ADC_ADON;
	ADC_COMMON_Instance->ADC_CCR |= ADC_PRESC_2;
	//Scan Mode off
	ADC_Instance->ADC_CR1 &= ~ADC_SCANMODE_SET;
	// Resolution 12bit
	ADC_Instance->ADC_CR1 |= ADC_RES_12Bit;
	// Data Alignment Right
	ADC_Instance->ADC_CR2 &= ~(ADC_DATA_ALIGNMENT_LEFT);
	//Continous Conversion Off
	ADC_Instance->ADC_CR2 &= ADC_CCONVERSION_CLR;
	//Discontinous Conversion On
	ADC_Instance->ADC_CR1 |= ADC_DISCONT_SET;
	//Number of Conversion Lenght 1 
	ADC_Instance->ADC_SQR1 &= ADC_NUM_OF_CONV_L;
	//First conversion is 18channel
	//ADC_Instance->ADC_SQR3 |= ADC_SEQUENCE_FIRST;
	ADC_Instance->ADC_SQR3 |= ADC_SEQUENCE_FIRST_IN5;
	
}
void temperatureSensorOn(){
	ADC_INSTANCE * ADC_Instance = (ADC_INSTANCE*)ADC1_BASE_ADDR;
	ADC_Instance->ADC_SMPR1 |= ADC_SAMPLING_15C;
	ADC_COMMON_INSTANCE* ADC_COMMON_Instance = (ADC_COMMON_INSTANCE*)ADC1_COMMON_BASE_ADDR;
	ADC_COMMON_Instance->ADC_CCR |=ADC_TSFRFEE;
	delay_milisec();
	
}
void ADC_in5on(){
	ADC_INSTANCE * ADC_Instance = (ADC_INSTANCE*)ADC1_BASE_ADDR;
	ADC_Instance->ADC_SMPR2 |= ADC_SAMPLING_IN5_15C;
}
void start_Conversion_ADC(){
	ADC_INSTANCE * ADC_Instance = (ADC_INSTANCE*)ADC1_BASE_ADDR;
	ADC_Instance->ADC_CR1 |= ADC_EOCIE;
	ADC_Instance->ADC_CR2 |= ADC_SWSTART;
	
}
void read_ADC(volatile int * data){
 ADC_INSTANCE * ADC_Instance = (ADC_INSTANCE*)ADC1_BASE_ADDR;
	*data = ADC_Instance->ADC_DR;
}
void delay_milisec(){
	for(int i=0;i < 10000;i++)
	{
		for(int j=0;j <5000; j++)
		{
		}
	}
}