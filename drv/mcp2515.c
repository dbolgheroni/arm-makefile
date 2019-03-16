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
#include <spi.h>
#include <gpio.h>
#include <spi.h>
#include <usart.h>
#include <util.h>

#include <mcp2515.h>

void _mcp2515_reset(void) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_RESET_INSTR);
    gpio_set(GPIOA, 4);
    wait_cycles(1000);
}

uint8_t _mcp2515_read(uint8_t reg) {
    uint8_t data;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_READ_INSTR);
    spi_send(SPI1, reg);
    data = spi_send(SPI1, 0x00);
    gpio_set(GPIOA, 4);

    return data;
}

uint8_t _mcp2515_read_rx_buffer(uint8_t buf) {
    uint8_t data;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_READRXB_INSTR | buf);
    data = spi_send(SPI1, 0x00);
    gpio_set(GPIOA, 4);

    return data;
}

void _mcp2515_write(uint8_t reg, uint8_t data) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_WRITE_INSTR);
    spi_send(SPI1, reg);
    spi_send(SPI1, data);
    gpio_set(GPIOA, 4);
}

void _mcp2515_load_tx_buffer(uint8_t buf, uint8_t data) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_LOADTXB_INSTR | buf);
    spi_send(SPI1, data);
    gpio_set(GPIOA, 4);
}

void _mcp2515_rts(uint8_t buf) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_RTS_INSTR | buf);
    gpio_set(GPIOA, 4);
}

uint8_t _mcp2515_read_status(void) {
    uint8_t status;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_READSTATUS_INSTR);
    status = spi_send(SPI1, 0x00);
    gpio_set(GPIOA, 4);

    return status;
}

uint8_t _mcp2515_rx_status(void) {
    uint8_t status;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_RXSTATUS_INSTR);
    status = spi_send(SPI1, 0x00);
    gpio_set(GPIOA, 4);

    return status;
}

void _mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_BITMODIFY_INSTR);
    spi_send(SPI1, reg);
    spi_send(SPI1, mask);
    spi_send(SPI1, data);
    gpio_set(GPIOA, 4);
}

const struct canconf {
    uint8_t osc;
    uint8_t br;
    uint8_t sp;
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} canconfbits[] = {
    OSC_8, BR_5, SP_75, 0x27, 0xb6, 0x04,
    OSC_8, BR_10, SP_75, 0x13, 0xb6, 0x04,
    OSC_8, BR_50, SP_75, 0x03, 0xb6, 0x04,
    OSC_8, BR_125, SP_75, 0x01, 0xac, 0x03,
    OSC_8, BR_250, SP_75, 0x00, 0xac, 0x03,
    OSC_8, BR_5, SP_875, 0x31, 0xb5, 0x01,
    OSC_8, BR_10, SP_875, 0x18, 0xb5, 0x01,
    OSC_8, BR_50, SP_875, 0x04, 0xb5, 0x01,
    OSC_8, BR_125, SP_875, 0x01, 0xb5, 0x01,
    OSC_8, BR_250, SP_875, 0xc0, 0xb5, 0x01,
    0, 0, 0, 0, 0, 0,
};

char mcp2515_init(uint8_t osc, uint8_t br, uint8_t sp) {
    const struct canconf *c;
    char i, canset = 0;
    uint8_t t;

    spi_master_init(SPI1, SPI_MODE0, SPI_BR32, SPI_MSB);
    gpio_set(GPIOA, 4);

    /* enter Configuration Mode */
    _mcp2515_reset();

    for (c = canconfbits; c->osc; c++) {
        if (osc == c->osc && br == c->br && sp == c->sp) {
            _mcp2515_write(CNF1, c->cnf1);
            _mcp2515_write(CNF2, c->cnf2);
            _mcp2515_write(CNF3, c->cnf3);

            /* check */
            if ((_mcp2515_read(CNF1) != c->cnf1) ||
                    (_mcp2515_read(CNF2) != c->cnf2) ||
                    (_mcp2515_read(CNF3) != c->cnf3)) {
                return -1;
            }

            canset = 1;
        }
    }

    if (!canset) {
        return -1;
    }

    /* enter Normal Mode */
    _mcp2515_bit_modify(CANCTRL0,
            CANCTRL_REQOP2 | CANCTRL_REQOP1 | CANCTRL_REQOP0, 0x00);
}

void mcp2515_putc(uint8_t ft, uint32_t id, candata_t *d) {
    uint8_t txbnsidl, txbnsidh, txbneid0, txbneid8;
    uint8_t txbn = 0;
    unsigned int i = 0;
    char const *p;

    /* try TXB0 */
    if (!(_mcp2515_read(TXB0CTRL) & TXB0CTRL_TXREQ)) {
        txbn = 0;
    /* try TXB1 */
    } else if (!(_mcp2515_read(TXB1CTRL) & TXB1CTRL_TXREQ)) {
        txbn = 1;
    /* try TXB2 */
    } else if (!(_mcp2515_read(TXB2CTRL) & TXB2CTRL_TXREQ)) {
        txbn = 2;
    }

    /* standard frame */
    if (ft == STDF) {
        txbnsidl = (uint8_t) ((id & TXBnSIDL_STD) << 5);
        txbnsidh = (uint8_t) ((id & TXBnSIDH_STD) >> 3);

        /* standard message id */
        _mcp2515_write(TXBnSIDL(txbn), txbnsidl);
        _mcp2515_write(TXBnSIDH(txbn), txbnsidh);
    } else if (ft == EXTF) {
        txbneid0 = (uint8_t) (id & TXBnEID0_EXT);
        txbneid8 = (uint8_t) ((id & TXBnEID8_EXT) >> 8);
        txbnsidl = (uint8_t) ((id & TXBnSIDL10_EXT) >> 16);
        txbnsidl = (uint8_t) (txbnsidl | ((id & TXBnSIDL75_EXT) >> 13));
        txbnsidh = (uint8_t) ((id & TXBnSIDH_EXT) >> 21);

        /* write id on the respective 8-bit registers */
        _mcp2515_write(TXBnEID0(txbn), txbneid0);
        _mcp2515_write(TXBnEID8(txbn), txbneid8);
        _mcp2515_write(TXBnSIDL(txbn), txbnsidl);
        _mcp2515_write(TXBnSIDH(txbn), txbnsidh);

        /* activate extended frame bit */
        _mcp2515_write(TXBnSIDL(txbn), txbnsidl | TXBnSIDL_EXIDE);
    }

    /* load data length code register */
    _mcp2515_write(TXBnDLC(txbn), d->size);

    /* load data byte registers */
    for (i = 0; i < d->size; i++) {
        _mcp2515_write(TXBnDm(txbn, i), (d->data)[i]);
    }

    /* request to send TXBn */
    _mcp2515_rts(MCP2515_RTS_TXBn(txbn));

    /* clear TXBn flag */
    _mcp2515_bit_modify(CANINTF, CANINTF_TXnIF(txbn), 0x00);
}
