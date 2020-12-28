#include <timer.h>

volatile static uint32_t t;

void delay_init(void) {
    //SysTick_Config(SystemCoreClock / 1000000); /* for us precision */
    SysTick_Config(SystemCoreClock / 1000); /* for ms precision */
}

void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000);
    }

    //t = ms;
    //while(t > 0);
}

/* not precise with low < ~50 us values */
void delay_us(uint32_t us) {
    t = us;
    while(t);
    //while(t > 0);
}

void SysTick_Handler(void) {
    HAL_IncTick();
}
