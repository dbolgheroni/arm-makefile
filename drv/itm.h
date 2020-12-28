#include <stm32f103xb.h>
#include <core_cm3.h>

void enable_itm(void);
void enable_dwt(void);
void ITM_Print(int, const char *);
void ITM_SendValue(int, uint32_t);
