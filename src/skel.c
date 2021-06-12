#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_spi.h>

#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "init.h"

void blink1(void *pvParameters) {
    for (;;) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        vTaskDelay(200/portTICK_PERIOD_MS);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

int main() {
    init();

    xTaskCreate(blink1, (char *)"blink1", configMINIMAL_STACK_SIZE,
            (void *)NULL, 1, NULL);

    vTaskStartScheduler();

    for(;;);

    return 0;
}
