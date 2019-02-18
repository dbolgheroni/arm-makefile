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

#include <spi.h>

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
        /* 
         * MOSI=PA7 (afpp)
         * MISO=PA6 (if/ipu)
         * SCK=PA5 (afpp)
         * NSS=PA4 (if/ipu/ipd)
         */ 

        /* enable clock for periphericals */
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;

        /* configure AF GPIO pins, CNF does not zero out on reset  */
        GPIOA->CRL |= GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;
        GPIOA->CRL &= ~GPIO_CRL_CNF7_0;

        GPIOA->CRL |= GPIO_CRL_CNF6_1;
        GPIOA->CRL &= ~GPIO_CRL_CNF6_0;

        GPIOA->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1;
        GPIOA->CRL &= ~GPIO_CRL_CNF5_0;

        /* configure NSS GPIO as output 2 MHz (software NSS) */
        GPIOA->CRL |= GPIO_CRL_MODE4_1;
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

uint8_t spi_send(SPI_TypeDef *spi, uint8_t c) {
    while (!(spi->SR & SPI_SR_TXE)); /* wait to send the last char */
    spi->DR = c;

    while (!(spi->SR & SPI_SR_RXNE)); /* wait to receive char */
    while (spi->SR & SPI_SR_BSY);

    c = spi->DR;

    return c;
}

