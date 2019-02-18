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

#define SPI_MODE0       0x1
#define SPI_MODE3       0x2

#define SPI_BR2         0x1
#define SPI_BR4         0x2
#define SPI_BR8         0x3
#define SPI_BR16        0x4
#define SPI_BR32        0x5
#define SPI_BR64        0x6
#define SPI_BR128       0x7
#define SPI_BR256       0x8

#define SPI_MSB         0x1
#define SPI_LSB         0x2         

void spi_master_init(SPI_TypeDef *, uint8_t, uint8_t, uint8_t);
void spi_set_mode(SPI_TypeDef *, uint8_t);
void spi_set_clkdiv(SPI_TypeDef *, uint8_t);
void spi_set_bitorder(SPI_TypeDef *, uint8_t);
uint8_t spi_send(SPI_TypeDef *, uint8_t);
