#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>

#include <spi.h>
#include <mcp2515.h>

#include "FreeRTOS.h"
#include "task.h"

#include "init.h"

extern SPI_HandleTypeDef hspi1;

#if 1
void can_probe1() {
    struct can_frame f1;
    struct can_frame f2;

    can_frame_init1(&f1);
    can_frame_init1(&f2);

    can_frame_add(&f1, 't');
    can_frame_add(&f1, 'x');
    can_frame_add(&f1, 0x55);

    can_frame_add(&f2, 'T');
    can_frame_add(&f2, 'X');
    can_frame_add(&f2, 0xAA);

    for (;;) {
        mcp2515_send(STDF, 0x74, LOW, &f1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(450);

        mcp2515_send(EXTF, 0x5454, HIGHEST, &f2);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(450);
    }
}
#endif

#if 0
void can_probe(void *pvParameters) {
    struct can_frame f1;
    struct can_frame f2;

    can_frame_init1(&f1);
    can_frame_init1(&f2);

    can_frame_add(&f1, 't');
    can_frame_add(&f1, 'x');
    can_frame_add(&f1, 0x55);

    can_frame_add(&f2, 'T');
    can_frame_add(&f2, 'X');
    can_frame_add(&f2, 0xAA);

    for (;;) {
        mcp2515_send(STDF, 0x74, LOW, &f1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        vTaskDelay(50/portTICK_PERIOD_MS);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        vTaskDelay(450/portTICK_PERIOD_MS);

        mcp2515_send(EXTF, 0x5454, HIGHEST, &f2);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        vTaskDelay(50/portTICK_PERIOD_MS);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        vTaskDelay(450/portTICK_PERIOD_MS);
    }
}
#endif

void led14_probe(void *pvParameters) {
    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

int main() {
    init();

    if (mcp2515_init(OSC_8, BR_250, SP_75) == -1) {
        Error_Handler();
    };

    can_probe1();

#if 0
    xTaskCreate(can_probe, (char *)"can_probe", configMINIMAL_STACK_SIZE,
            (void *)NULL, 1, NULL);
    xTaskCreate(led14_probe, (char *)"led14_probe", configMINIMAL_STACK_SIZE,
            (void *)NULL, 1, NULL);
    
    vTaskStartScheduler();

    for (;;);
#endif

    return 0;
}

void mcp2515_int_rx(can_frame_t *rxb) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
}