#include <stddef.h>

/* baud rate (Kbps) */
#define BR_5    0x1
#define BR_10   0x2
#define BR_20   0x3
#define BR_50   0x4
#define BR_100  0x5
#define BR_125  0x6
#define BR_250  0x7
#define BR_500  0x8
#define BR_1000 0x9

/* sample points (%) */
#define SP_75   0x1    /* ARINC 825 */
#define SP_875  0x2    /* DeviceNet, CANopen */
#define SP_XX   0x3

/* frametype */
#define STDF    0x1
#define EXTF    0x2

struct can_frame {
    size_t dlc;
    uint8_t data[8];
};

typedef struct can_frame can_frame_t;

/* init */
struct can_frame *can_frame_init1();
void can_frame_init(struct can_frame *);

/* add */
void can_frame_add(struct can_frame *, uint8_t);
