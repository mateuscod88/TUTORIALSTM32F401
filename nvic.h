#ifdef __NVIC_H
#define ICTR_addr (unsigned int*)0xE000E004
typedef struct{
	unsigned int INTR_SET_ENABLE;
	unsigned int INTR_CLEAR_ENABLE;
	unsigned int INTR_SET_PENDING;
	unsigned int INTR_CLEAR_PENDING;
	unsigned int INTR_ACTIVE_BIT;
	unsigned int INTR_PIORITY;
}ICTR_Instance;
#endif