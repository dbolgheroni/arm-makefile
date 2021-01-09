#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>

#include <stdio.h>
#include <stdlib.h>

#include <stm32f1xx_hal_spi.h>

void error_handler(void);

/*
 * XXX: the STM32Cube example shows this as global, and being global is the
 * only way to make it work with HAL_SPI_MspInit, and more debugging is needed
 */
SPI_HandleTypeDef spi1;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    /* 
     * The pins used for SPI should be configured as this, as of Reference
     * manual RM0008 Rev 20 p. 167:
     * MOSI=PA7 full duplex / master -> alternate function push-pull
     * MISO=PA6 full duplex / master -> input floating / input pull-up
     * SCK=PA5 master -> alternate function push-pull
     * NSS=PA4 software -> not used, can be used as a GPIO
     */

    /* enable gpio and spi clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* configure SPI pins */
    /* configure PA7 */
    GPIO_InitTypeDef pa7;
    pa7.Pin = GPIO_PIN_7;
    pa7.Mode = GPIO_MODE_AF_PP;
    pa7.Pull = GPIO_PULLDOWN;
    pa7.Speed = GPIO_SPEED_FREQ_LOW;

    /* configure PA6 */
    GPIO_InitTypeDef pa6;
    pa6.Pin = GPIO_PIN_6;
    pa6.Mode = GPIO_MODE_INPUT;
    pa6.Speed = GPIO_SPEED_FREQ_LOW;

    /* configure PA5 */
    GPIO_InitTypeDef pa5;
    pa5.Pin = GPIO_PIN_5;
    pa5.Mode = GPIO_MODE_AF_PP;
    pa5.Pull = GPIO_PULLDOWN;
    pa5.Speed = GPIO_SPEED_FREQ_LOW;

    /* configure PA4 */
    GPIO_InitTypeDef pa4;
    pa4.Pin = GPIO_PIN_4;
    pa4.Mode = GPIO_MODE_OUTPUT_PP;
    pa4.Pull = GPIO_PULLUP;
    pa4.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &pa7);
    HAL_GPIO_Init(GPIOA, &pa6);
    HAL_GPIO_Init(GPIOA, &pa5);
    HAL_GPIO_Init(GPIOA, &pa4);


}

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

    /* configure SPI itself */
    spi1.Instance = SPI1;

    uint8_t txdata[8] = { 0x55, 0x56, 0x57 };

    SPI_InitTypeDef spi1_init;
    spi1_init.Mode = SPI_MODE_MASTER;
    spi1_init.Direction = SPI_DIRECTION_2LINES;
    spi1_init.DataSize = SPI_DATASIZE_8BIT;
    spi1_init.CLKPolarity = SPI_POLARITY_LOW;
    spi1_init.CLKPhase = SPI_PHASE_1EDGE;
    spi1_init.NSS = SPI_NSS_SOFT;
    spi1_init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    spi1_init.FirstBit = SPI_FIRSTBIT_MSB;
    spi1_init.TIMode = SPI_TIMODE_DISABLE;
    spi1_init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi1_init.CRCPolynomial = 0;

    spi1.Init = spi1_init;

    if (HAL_SPI_Init(&spi1) != HAL_OK) {
        error_handler();
    }

    __HAL_SPI_ENABLE(&spi1);

    for (;;) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&spi1, txdata, 3, 50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(450);
    }

    return 0;
}

void error_handler(void) {
    for (;;) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(1000);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(100);
    }
}
