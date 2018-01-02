#define __GPIO_H
#ifdef __GPIO_H

#define GPIOH_addr 0x40021C00 
#define GPIOE_addr 0x40021000 
#define GPIOD_addr 0x40020C00 
#define GPIOC_addr 0x40020800
#define GPIOB_addr 0x40020400 
#define GPIOA_addr 0x40020000

#define GPIOx_PUPDR_pull_up(position) 1U << position
#define GPIOx_MODER_output(pin) 1U << (pin*2)
#define GPIOx_TYPER_push_pull(pin) ~(1U << pin)
#define GPIOx_PIN_12  12
#define GPIOx_PIN_13  13
#define GPIOx_ODR_bit_set(pin) 1U << pin


typedef struct{
	unsigned int GPIOx_MODER;
	unsigned int GPIOx_TYPER;
	unsigned int GPIOx_SPEED;
	unsigned int GPIOx_PUPDR;
	unsigned int GPIOx_IDR;
	unsigned int GPIOx_ODR;
	unsigned int GPIOx_BSRR;
	unsigned int GPIOx_LCKR;
	unsigned int GPIOx_AFRL;
	unsigned int GPIOx_AFRH;	
}GPIO_Instance;

#define SYSCFG_addr (unsigned int *)0x40013800	
#define SYSCFG_CLCK_ENABLE 1U << 14
typedef struct{
	unsigned int SYSCFG_MEMRMP;
	unsigned int SYSCFG_PMC;
	unsigned int SYSCFG_EXTICR1;
	unsigned int SYSCFG_EXTICR2;
	unsigned int SYSCFG_EXTICR3;
	unsigned int SYSCFG_EXTICR4;
	unsigned int SYSCFG_CMPCR;
}SYSCFG_Instance;

#define EXTI_addr (unsigned int *) 0x40013C00
typedef struct{
	unsigned int EXT_IMR;
	unsigned int EXT_EMR;
	unsigned int EXT_RTSR;
	unsigned int EXT_FTSR;
	unsigned int EXT_SWIER;
	unsigned int EXT_PR;
}EXTI_Instance;
#endif 