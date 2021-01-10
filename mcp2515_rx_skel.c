#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>

#include <stdio.h>
#include <stdlib.h>

#include <gpio.h>
#include <spi.h>
#include <util.h>
#include <mcp2515.h>

void error_handler();

int main() {
    HAL_Init();

    /* configure GPIO PC13 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef pc13;
    pc13.Pin = GPIO_PIN_13;
    pc13.Mode = GPIO_MODE_OUTPUT_PP;
    pc13.Pull = GPIO_PULLUP;
    pc13.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &pc13);

    /* mcp2515 init */
    if (mcp2515_init(OSC_8, BR_50, SP_75) == -1) {
        error_handler();
    };

    /* simple test to check if HAL* functions work outside weak ref function
    for (;;) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(500);
    } */
    for (;;);

    return 0;
}

void mcp2515_int_rx(can_frame_t *rxb) {
    /* XXX: HAL* functions * and even delay_ms() have no effect here.
     * Initialize PC13 with HAL and use gpio_toggle() for now.
    HAL_GPIO_TogglePin(GPIOC, 13); */
    gpio_toggle(GPIOC, 13);
}

void error_handler(void) {
    for (;;) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(1000);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(100);
    }
}
