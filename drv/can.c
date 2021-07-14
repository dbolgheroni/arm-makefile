#include <stm32f103xb.h>
#include <stdlib.h>
#include <can.h>

#include "FreeRTOS.h"

const struct canconf {
    uint8_t br;
    uint8_t sp;
    uint32_t can_btr;
} canconfbits[] = {
    { BR_10, SP_75, 0x003a00e0 },
    { BR_50, SP_75, 0x003a002c },
    { BR_125, SP_75, 0x003a0011 },
    { BR_250, SP_75, 0x003a0008 },
    { BR_10, SP_875, 0x001c00e0 },
    { BR_50, SP_875, 0x001c002c },
    { BR_125, SP_875, 0x001c0011 },
    { BR_250, SP_875, 0x001c0008 },
    { 0, 0, 0 },
};

/* init */
struct can_frame *can_frame_init() {
    int i;
    can_frame_t *f = pvPortMalloc(sizeof(struct can_frame));

    f->dlc = 0;
    for (i = 0; i < 8; i++) {
        (f->data)[i] = 0;
    }

    return f;
}

void can_frame_init1(struct can_frame *f) {
    f->dlc = 0;
    for (int i = 0; i < 8; i++) {
        (f->data)[i] = 0;
    }
}

/* add */
void can_frame_add(struct can_frame *d, uint8_t v) {
    if (d->dlc >= 8) {
        return;
    }

    (d->data)[(d->dlc)++] = v;
}
