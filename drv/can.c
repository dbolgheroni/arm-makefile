#include <stm32f103xb.h>
#include <can.h>

void can_init(CAN_TypeDef *can) {
    RCC->APB2ENR |= RCC_APB1ENR_CAN1EN;

    /* enter initialization mode */
    can->MCR |= CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK));
}

void candata_init(candata_t *d) {
    int i = 0;
    d->size = 0;

    /* zero out */
    for (i = 0; i < 8; i++) {
        (d->data)[i] = 0;
    }
}

void candata_add(candata_t *d, uint8_t v) {
    if (d->size > 7) {
        return;
    }

    (d->data)[d->size] = v;
    (d->size)++;
}

