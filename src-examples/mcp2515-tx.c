#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>

#include <spi.h>
#include <mcp2515.h>

#include <init.h>

int main() {
    init();
    //spi_init1();

    struct can_frame *f1;
    struct can_frame *f2;

    f1 = can_frame_init();
    f2 = can_frame_init();

    if (mcp2515_init(OSC_8, BR_250, SP_75) == -1) {
        Error_Handler();
    };

    can_frame_add(f1, 't');
    can_frame_add(f1, 'x');
    can_frame_add(f1, 0x55);

    can_frame_add(f2, 'T');
    can_frame_add(f2, 'X');
    can_frame_add(f2, 0xAA);

    for (;;) {
        mcp2515_send(STDF, 0x74, LOW, f1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(450);

        mcp2515_send(EXTF, 0x5454, HIGHEST, f2);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(450);
    }

    return 0;
}

void mcp2515_int_rx(can_frame_t *rxb) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
}
