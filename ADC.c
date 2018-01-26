#include "ADC.h"

void init_ADC(){
	ADC_INSTANCE * ADC_Instance = (ADC_INSTANCE*)ADC1_BASE_ADDR;
	ADC_Instance->ADC_CR1 |= ADC_RES_12Bit;
}