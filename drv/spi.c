/*
 * MIT License
 *
 * Copyright (c) 2019, Daniel Bolgheroni.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>

#include <stm32f1xx_hal_spi.h>

#include <spi.h>

/*
 * XXX: the STM32Cube example shows this as global, and being global is the
 * only way to make it work with HAL_SPI_MspInit, and more debugging is needed
 */
SPI_HandleTypeDef spi1;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    /* The pins used for SPI should be configured as this, as of Reference
     * manual RM0008 Rev 20 p. 167:
     * MOSI=PA7 full duplex / master -> alternate function push-pull
     * MISO=PA6 full duplex / master -> input floating / input pull-up
     * SCK=PA5 master -> alternate function push-pull
     * NSS=PA4 software -> not used, can be used as a GPIO */

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

void spi_set_mode(SPI_TypeDef *spi, uint8_t mode) {
    spi->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);

    if (mode == SPI_MODE0) {
        return;
    } else if (mode == SPI_MODE3) {
        spi->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
    } else {
        spi_set_mode(spi, SPI_MODE0);
    }
}

void spi_set_clkdiv(SPI_TypeDef *spi, uint8_t clkdiv) {
    spi->CR1 &= ~SPI_CR1_BR;

    if (clkdiv == SPI_BR2) {
        return;
    } else if (clkdiv == SPI_BR4) { 
        spi->CR1 |= SPI_CR1_BR_0;
    } else if (clkdiv == SPI_BR8) { 
        spi->CR1 |= SPI_CR1_BR_1;
    } else if (clkdiv == SPI_BR16) { 
        spi->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_0;
    } else if (clkdiv == SPI_BR32) { 
        spi->CR1 |= SPI_CR1_BR_2;
    } else if (clkdiv == SPI_BR64) { 
        spi->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;
    } else if (clkdiv == SPI_BR128) { 
        spi->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;
    } else if (clkdiv == SPI_BR256) { 
        spi->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;
    } else {
        spi_set_clkdiv(spi, SPI_BR128);
    }

    return;
}

void spi_set_bitorder(SPI_TypeDef *spi, uint8_t bitorder) {
    if (bitorder == SPI_LSB) {
        spi->CR1 |= SPI_CR1_LSBFIRST;
    } else {
        spi->CR1 &= ~SPI_CR1_LSBFIRST;
    }
}

void spi_master_init(SPI_TypeDef *spi,
        uint8_t mode, uint8_t clkdiv, uint8_t bitorder) {

    if (spi == SPI1) {
        /* The pins used for SPI should be configured as this, as of Reference
         * manual RM0008 Rev 20 p. 167:
         * MOSI=PA7 full duplex / master -> alternate function push-pull
         * MISO=PA6 full duplex / master -> input floating / input pull-up
         * SCK=PA5 master -> alternate function push-pull
         * NSS=PA4 software -> not used, can be used as a GPIO */

        /* enable clock for periphericals */
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;

        /* configure AF GPIO pins, CNF does not zero out on reset  */
        GPIOA->CRL |= GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0;
        GPIOA->CRL |= GPIO_CRL_CNF7_1;
        GPIOA->CRL &= ~GPIO_CRL_CNF7_0;

        GPIOA->CRL |= GPIO_CRL_CNF6_1;
        GPIOA->CRL &= ~GPIO_CRL_CNF6_0;

        GPIOA->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_MODE5_0;
        GPIOA->CRL |= GPIO_CRL_CNF5_1;
        GPIOA->CRL &= ~GPIO_CRL_CNF5_0;

        /* configure NSS GPIO as output 2 MHz (software NSS) */
        GPIOA->CRL |= GPIO_CRL_MODE4_1 | GPIO_CRL_MODE4_0;
        GPIOA->CRL &= ~(GPIO_CRL_CNF4_1 | GPIO_CRL_CNF4_0);

        /* 8-bit data format */
        spi->CR1 &= ~(SPI_CR1_DFF); 

        /* SPI master */
        spi->CR1 |= SPI_CR1_MSTR;

        /* software slave management */
        spi->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;

        /* SS output enable */
        spi->CR2 |= SPI_CR2_SSOE;

        spi_set_clkdiv(spi, clkdiv);
        spi_set_mode(spi, mode);
        spi_set_bitorder(spi, bitorder);

        /* enable SPI */
        spi->CR1 |= SPI_CR1_SPE;
    } else {
        return;
    }
}

/* api v0.2 */
void spi_init1() {
    spi1.Instance = SPI1;

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

    HAL_SPI_Init(&spi1);

    __HAL_SPI_ENABLE(&spi1);
}

uint8_t spi_send(SPI_TypeDef *spi, uint8_t c) {
    while (!(spi->SR & SPI_SR_TXE)); /* wait to send the last char */
    spi->DR = c;

    while (!(spi->SR & SPI_SR_RXNE)); /* wait to receive char */
    while (spi->SR & SPI_SR_BSY);

    c = spi->DR;

    return c;
}

/* api v0.2 */
void spi_send1(uint8_t *txd, uint8_t *rxd, uint16_t size) {
    HAL_SPI_TransmitReceive(&spi1, txd, rxd, size, 100);
}
