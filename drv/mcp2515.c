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
#include <stdlib.h>

#include <spi.h>
#include <gpio.h>
#include <spi.h>
#include <usart.h>
#include <util.h>

#include <mcp2515.h>

static void _mcp2515_reset(void) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_RESET_INSTR);
    gpio_set(GPIOA, 4);
    wait_cycles(10000);
}

static uint8_t _mcp2515_read(uint8_t reg) {
    uint8_t data;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_READ_INSTR);
    spi_send(SPI1, reg);
    data = spi_send(SPI1, 0x00);
    gpio_set(GPIOA, 4);

    return data;
}

static uint8_t _mcp2515_read_rx_buffer(uint8_t buf, struct can_frame *d) {
    uint8_t data;
    int i;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_READRXB_INSTR | buf);
    spi_send(SPI1, 0x00); /* ignore sidh */
    spi_send(SPI1, 0x00); /* ignore sidl */
    spi_send(SPI1, 0x00); /* ignore eid8 */
    spi_send(SPI1, 0x00); /* ignore eid0 */
    d->dlc = spi_send(SPI1, 0x00);

    for (i = 0; i < d->dlc; i++) {
        d->data[i] = spi_send(SPI1, 0x00);
    }

    gpio_set(GPIOA, 4);

    return data;
}

static void _mcp2515_write(uint8_t reg, uint8_t data) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_WRITE_INSTR);
    spi_send(SPI1, reg);
    spi_send(SPI1, data);
    gpio_set(GPIOA, 4);
}

static void _mcp2515_load_tx_buffer(uint8_t buf, txbconf_t *c, struct can_frame *d) {
    unsigned int i = 0;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_LOADTXB_INSTR | buf);
    spi_send(SPI1, c->txbnsidh);
    spi_send(SPI1, c->txbnsidl);
    spi_send(SPI1, c->txbneid8);
    spi_send(SPI1, c->txbneid0);

    spi_send(SPI1, d->dlc);
    for (i = 0; i < d->dlc; i++) {
        spi_send(SPI1, (d->data)[i]);
    }

    /* without optimized instructions (for testing purposes)
    _mcp2515_write(TXB0SIDH, c->txbnsidh);
    _mcp2515_write(TXB0SIDL, c->txbnsidl);
    _mcp2515_write(TXB0EID8, c->txbneid8);
    _mcp2515_write(TXB0EID0, c->txbneid0);
    _mcp2515_write(TXB0DLC, d->dlc);
    for (i = 0; i < d->dlc; i++) {
        _mcp2515_write(TXBnDm(0, i), (d->data)[i]);
    }
    */

    gpio_set(GPIOA, 4);
}

static void _mcp2515_rts(uint8_t buf) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_RTS_INSTR | buf);
    gpio_set(GPIOA, 4);
}

static uint8_t _mcp2515_read_status(void) {
    uint8_t status;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_READSTATUS_INSTR);
    status = spi_send(SPI1, 0x00);
    gpio_set(GPIOA, 4);

    return status;
}

static uint8_t _mcp2515_rx_status(void) {
    uint8_t status;

    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_RXSTATUS_INSTR);
    status = spi_send(SPI1, 0x00);
    gpio_set(GPIOA, 4);

    return status;
}

static void _mcp2515_bit_modify(uint8_t reg, uint8_t mask, uint8_t data) {
    gpio_reset(GPIOA, 4);
    spi_send(SPI1, MCP2515_BITMODIFY_INSTR);
    spi_send(SPI1, reg);
    spi_send(SPI1, mask);
    spi_send(SPI1, data);
    gpio_set(GPIOA, 4);
}

static uint8_t _mcp2515_read_tec(void) {
    uint8_t tec = 0;
    tec = _mcp2515_read(TEC);

    return tec;
}

const struct mcp2515_canconf {
    uint8_t osc;
    uint8_t br;
    uint8_t sp;
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} mcp2515_canconfbits[] = {
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

void _enable_pb10_int(void) {
    /* enable external interrupt line */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    EXTI->IMR |= EXTI_IMR_MR10;
    EXTI->FTSR |= EXTI_FTSR_TR10;

    /* PB10 as interrupt line */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRH &= ~GPIO_CRH_MODE10;
    GPIOB->CRH |= GPIO_CRH_CNF10_1;
    GPIOB->CRH &= ~GPIO_CRH_CNF10_0;
    GPIOB->ODR |= GPIO_ODR_ODR10;

    AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PB;

    __disable_irq();
    NVIC_SetPriorityGrouping(0U);

    NVIC_SetPriority(EXTI15_10_IRQn, 5U);
    NVIC_SetPriority(SysTick_IRQn, 6U);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    __enable_irq();

}

int mcp2515_init(uint8_t osc, uint8_t br, uint8_t sp) {
    const struct mcp2515_canconf *c;
    char canset = 0;

    spi_master_init(SPI1, SPI_MODE0, SPI_BR32, SPI_MSB);
    gpio_set(GPIOA, 4);

    /* enter Configuration Mode */
    _mcp2515_reset();

    for (c = mcp2515_canconfbits; c->osc; c++) {
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

    /* receives any message */
    _mcp2515_bit_modify(RXB0CTRL, RXB0CTRL_RXM1 | RXB0CTRL_RXM0 |
            RXB0CTRL_BUKT, 0xFF);

    /* enable external interrupts */
    _enable_pb10_int();

    /* enable MCP2515 interrupts */
    _mcp2515_bit_modify(CANINTE, CANINTE_RX0IE | CANINTE_RX1IE |
            //CANINTE_ERRIE | CANINTE_MERRE |
            CANINTE_TX0IE | CANINTE_TX1IE | CANINTE_TX2IE, 0xFF);

    /* enter Normal Mode */
    _mcp2515_bit_modify(CANCTRL0,
            CANCTRL_REQOP2 | CANCTRL_REQOP1 | CANCTRL_REQOP0, 0x00);
}

/* irq handler */
void EXTI15_10_IRQHandler(void) {
    uint8_t read_status;

    //1//NVIC_ClearPendingIRQ(EXTI15_10_IRQn); __DSB(); __ISB();
    //1//__disable_irq(); __DSB(); __ISB();

    //2//__set_BASEPRI(5 << (8U - __NVIC_PRIO_BITS));

    __disable_irq();

    read_status = _mcp2515_read_status();

    /* sent frame */
    if (read_status & MCP2515_READSTATUS_TX0IF) {
        _mcp2515_bit_modify(CANINTF, CANINTF_TX0IF, 0x00);
    }
    if (read_status & MCP2515_READSTATUS_TX1IF) {
        _mcp2515_bit_modify(CANINTF, CANINTF_TX1IF, 0x00);
    }
    if (read_status & MCP2515_READSTATUS_TX2IF) {
        _mcp2515_bit_modify(CANINTF, CANINTF_TX2IF, 0x00);
    }

    /* received frame */
    if (read_status & MCP2515_READSTATUS_RX0IF) {
        can_frame_t *rxb0 = can_frame_init();

        _mcp2515_bit_modify(CANINTF, CANINTF_RX0IF, 0x00);
        _mcp2515_read_rx_buffer(MCP2515_READRXB_RXB0SIDH, rxb0);

        mcp2515_int_rx(rxb0);
        free(rxb0);
    }
    if (read_status & MCP2515_READSTATUS_RX1IF) {
        can_frame_t *rxb1 = can_frame_init();

        _mcp2515_bit_modify(CANINTF, CANINTF_RX1IF, 0x00);
        _mcp2515_read_rx_buffer(MCP2515_READRXB_RXB1SIDH, rxb1);

        mcp2515_int_rx(rxb1);
        free(rxb1);
    }

    /* clear pending register */
    EXTI->PR |= EXTI_PR_PR10;

    //1//NVIC_EnableIRQ(EXTI15_10_IRQn);
    //1//__enable_irq(); __DSB(); __ISB();

    //2//__set_BASEPRI(0U);

    __enable_irq();

    return;
}

void mcp2515_send(uint8_t ft, const uint32_t id, const int prio,
        struct can_frame *d) {
    uint8_t txbnsidl;
    uint8_t txbn = 0;
    uint8_t read_status;
    txbconf_t c;

    //1//NVIC_ClearPendingIRQ(EXTI15_10_IRQn); __DSB(); __ISB();
    //1//__disable_irq(); __DSB(); __ISB();

    //2//__set_BASEPRI(5 << (8U - __NVIC_PRIO_BITS));

    __disable_irq();

    read_status = _mcp2515_read_status();

    /* try TXB0 */
    if (!(read_status & MCP2515_READSTATUS_TX0REQ)) {
        _mcp2515_bit_modify(TXB0CTRL, TXB0CTRL_TXP1 | TXB0CTRL_TXP0, prio);
        txbn = 0;
    /* try TXB1 */
    } else if (!(read_status & MCP2515_READSTATUS_TX1REQ)) {
        _mcp2515_bit_modify(TXB1CTRL, TXB1CTRL_TXP1 | TXB1CTRL_TXP0, prio);
        txbn = 1;
    /* try TXB2 */
    } else if (!(read_status & MCP2515_READSTATUS_TX2REQ)) {
        _mcp2515_bit_modify(TXB2CTRL, TXB2CTRL_TXP1 | TXB2CTRL_TXP0, prio);
        txbn = 2;
    }

    /* standard frame */
    if (ft == STDF) {
        c.txbnsidh = (uint8_t) ((id & TXBnSIDH_STD) >> 3);
        c.txbnsidl = (uint8_t) ((id & TXBnSIDL_STD) << 5);
        c.txbneid8 = 0x00;
        c.txbneid0 = 0x00;

        /* load TXB */
        _mcp2515_load_tx_buffer(MCP2515_LOADTXB_TXBnSIDH(txbn), &c, d);

        /* request to send TXBn */
        _mcp2515_rts(MCP2515_RTS_TXBn(txbn));
    } else if (ft == EXTF) {
        txbnsidl = (uint8_t) ((id & TXBnSIDL10_EXT) >> 16);
        txbnsidl = (uint8_t) (txbnsidl | ((id & TXBnSIDL75_EXT) >> 13));

        c.txbnsidh = (uint8_t) ((id & TXBnSIDH_EXT) >> 21);
        c.txbnsidl = txbnsidl;
        c.txbneid8 = (uint8_t) ((id & TXBnEID8_EXT) >> 8);
        c.txbneid0 = (uint8_t) (id & TXBnEID0_EXT);

        /* load TXB */
        _mcp2515_load_tx_buffer(MCP2515_LOADTXB_TXBnSIDH(txbn), &c, d);

        /* activate extended frame bit */
        _mcp2515_write(TXBnSIDL(txbn), c.txbnsidl | TXBnSIDL_EXIDE);

        /* request to send TXBn */
        _mcp2515_rts(MCP2515_RTS_TXBn(txbn));
    }

    //1//NVIC_EnableIRQ(EXTI15_10_IRQn);
    //1//__enable_irq(); __DSB(); __ISB();

    //2//__set_BASEPRI(0U);

    __enable_irq();
}
