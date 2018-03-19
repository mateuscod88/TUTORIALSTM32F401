#define _MEMS_H
#ifdef _MEMS_H
#define READ_CMD 1U << 7
#define WRITE_CMD ~(1U << 7)
#define INCREMET_ADRS 1U << 6
#define NOT_INCREMENT_ADRS ~(1U << 6)
#define WHO_AM_I_REG 0x0F
#define READ_FROM_ONE(reg) ((READ_CMD | reg) & NOT_INCREMENT_ADRS)
#endif