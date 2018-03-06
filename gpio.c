#include "gpio.h"
#include "UART_CONFIG.h"
void gpio_0_falling_edge(){
	
  /* Configure PA0 pin as input floating */
	EXTI_Instance * exti_push_button = (EXTI_Instance*)EXTI_addr;
	exti_push_button->EXT_IMR |= EXTI_GPIOA_0_INTERRUPT;
	exti_push_button->EXT_FTSR |= EXTI_FALLING_EDGE;
}
void gpio_uart2_config(){
	GPIO_Instance * push_button = (GPIO_Instance*)GPIOA_addr;
	push_button->GPIOx_MODER |= PA2_AF_TX |PA3_AF_RX;
	push_button->GPIOx_SPEED |= 2U << 4 | 2U << 6;
	push_button->GPIOx_AFRL |= PA2_AF7 | PA3_AF7;
}
void leds_On(GPIO_Instance * LEDs){
	
	LEDs->GPIOx_PUPDR |= GPIOx_PUPDR_pull_up(GPIOx_PIN_12) 
										| GPIOx_PUPDR_pull_up(GPIOx_PIN_13) 
										| GPIOx_PUPDR_pull_up(14)
										| GPIOx_PUPDR_pull_up(15);
	LEDs->GPIOx_MODER |= GPIOx_MODER_output(GPIOx_PIN_12) 
										| GPIOx_MODER_output(GPIOx_PIN_13) 
										| GPIOx_MODER_output(14)
										| GPIOx_MODER_output(15);
	LEDs->GPIOx_TYPER &= GPIOx_TYPER_push_pull(GPIOx_PIN_12);
	LEDs->GPIOx_ODR |= GPIOx_ODR_bit_set(GPIOx_PIN_12) 
									| GPIOx_ODR_bit_set(GPIOx_PIN_13) 
									| GPIOx_ODR_bit_set(14)
									| GPIOx_ODR_bit_set(15);
}
	