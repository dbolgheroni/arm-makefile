#include <stddef.h>

struct can_frame {
    size_t dlc;
    uint8_t data[8];
};

typedef struct can_frame can_frame_t;

void can_init(CAN_TypeDef *);

struct can_frame *can_frame_init();
void can_frame_add(struct can_frame *, uint8_t);
