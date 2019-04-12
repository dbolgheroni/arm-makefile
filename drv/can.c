#include <stm32f103xb.h>
#include <stdlib.h>
#include <can.h>

void can_init(CAN_TypeDef *can) {
    RCC->APB2ENR |= RCC_APB1ENR_CAN1EN;

    /* enter initialization mode */
    can->MCR |= CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK));

    /* set bit timing */

    /* enable interrupts */

    /* enter normal mode */
    can->MCR &= ~CAN_MCR_INRQ;
    while (CAN1->MSR & CAN_MSR_INAK);
}

struct can_frame *can_frame_init() {
    int i;
    can_frame_t *f = malloc(sizeof(struct can_frame));

    f->dlc = 0;
    for (i = 0; i < 8; i++) {
        (f->data)[i] = 0;
    }

    return f;
}

void can_frame_add(struct can_frame *d, uint8_t v) {
    if (d->dlc >= 8) {
        return;
    }

    (d->data)[(d->dlc)++] = v;
}
