#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_hal_spi.h>

#include <stdio.h>
#include <stdlib.h>

void error_handler(void);

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef pin_init;
    I2C_InitTypeDef hi2c_init;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* configure SCL1 on PB6 */
    pin_init.Pin = GPIO_PIN_6;
    pin_init.Mode = GPIO_MODE_AF_OD;
    pin_init.Pull = GPIO_PULLUP;
    pin_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &pin_init);

    /* configure SDA1 on PB7 */
    pin_init.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &pin_init);

    /* configure I2C handle */
    hi2c->Instance = I2C1;
    hi2c_init.ClockSpeed = 400000;
    hi2c_init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c_init.OwnAddress1 = 0x8;
    hi2c_init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c_init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c_init.OwnAddress2 = 0xFF;
    hi2c_init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c_init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    hi2c->Init = hi2c_init;
}

int main() {
    I2C_HandleTypeDef i2c1;
    uint8_t dis_seqop[] = { 0x0A, 0x20 };
    uint8_t set_iodira[] = { 0x00, 0x00 };
    uint8_t set_gpioa[] = { 0x12, 0xFF };
    uint8_t unset_gpioa[] = { 0x12, 0x00 };

    uint8_t cmd2[] = { 0x00, 0x00 };

    HAL_Init();

    /* configure GPIO on PC13 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef pc13;
    pc13.Pin = GPIO_PIN_13;
    pc13.Mode = GPIO_MODE_OUTPUT_PP;
    pc13.Pull = GPIO_PULLUP;
    pc13.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &pc13);

    if (HAL_I2C_Init(&i2c1) != HAL_OK) {
        error_handler();
    }

    for (;;) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_I2C_Master_Transmit(&i2c1, (uint16_t)0x40, dis_seqop, 2, 1000);
        HAL_I2C_Master_Transmit(&i2c1, (uint16_t)0x40, set_iodira, 2, 1000);
        HAL_I2C_Master_Transmit(&i2c1, (uint16_t)0x40, set_gpioa, 2, 1000);
        HAL_Delay(1000);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_I2C_Master_Transmit(&i2c1, (uint16_t)0x40, unset_gpioa, 2, 1000);
        HAL_Delay(1000);
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
