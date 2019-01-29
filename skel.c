#include <stm32f103xb.h>

int main() {
        int i; 

        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; /* enable clock for GPIOC port */
        GPIOC->CRH |= GPIO_CRH_MODE13_1; /* output 2 MHz */

        /* blink */
        for (;;) {
                if (GPIOC->ODR & GPIO_ODR_ODR13) {
                        GPIOC->BSRR |= GPIO_BSRR_BR13; /* turn on led */
                } else {
                        GPIOC->BSRR |= GPIO_BSRR_BS13; /* turn off led */
                }

                for (i = 0; i < 800000; i++) {
                        asm("nop");
                }
        }

        return 0;
}
