typedef struct {
    unsigned int size;
    uint8_t data[8];
} candata_t;

void can_init(CAN_TypeDef *);
void candata_init(candata_t *);
void candata_add(candata_t *, uint8_t);
