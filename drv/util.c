#include <util.h>

volatile static uint32_t t;

void wait_cycles(uint32_t c) {
    int i = 0;

    for (; i < c; i++) {
        __asm__("nop");
    }
}
