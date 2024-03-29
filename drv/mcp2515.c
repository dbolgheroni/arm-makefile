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

#include "FreeRTOS.h"
#include "task.h"

#include <gpio.h>
#include <spi.h>
#include <usart.h>
#include <util.h>

#include <mcp2515.h>

extern SPI_HandleTypeDef hspi1;
TaskHandle_t can_recv_h;

//------------------------------------------------------------------------------
static void _mcp2515_reset1(void) {
    uint8_t instr[1];
    uint8_t rxd[1];

    instr[0] = MCP2515_RESET_INSTR;
    spi_send1(&hspi1, instr, rxd, 1);

    wait_cycles(10000);
}

//------------------------------------------------------------------------------
static uint8_t _mcp2515_read1(uint8_t reg) {
    uint8_t data;
    uint8_t instr[3];
    uint8_t rxd[3];

    instr[0] = MCP2515_READ_INSTR;
    instr[1] = reg;
    instr[2] = 0;

    spi_send1(&hspi1, instr, rxd, 3);
    data = rxd[2];

    return data;
}

//------------------------------------------------------------------------------
static void _mcp2515_read_rx_buffer1(uint8_t buf, struct can_frame *d) {
    uint8_t instr[14] = {0};
    uint8_t rxd[14];

    instr[0] = MCP2515_READRXB_INSTR | buf;

    spi_send1(&hspi1, instr, rxd, 14);

    d->dlc = rxd[5];

    for (unsigned int i = 0; i < d->dlc; i++) {
        d->data[i] = rxd[i+6];
    }
}

//------------------------------------------------------------------------------
static void _mcp2515_write1(uint8_t reg, uint8_t data) {
    uint8_t instr[3];
    uint8_t rxd[3];

    instr[0] = MCP2515_WRITE_INSTR;
    instr[1] = reg;
    instr[2] = data;

    spi_send1(&hspi1, instr, rxd, 3);
}

//------------------------------------------------------------------------------
static void _mcp2515_load_tx_buffer1(uint8_t buf, txbconf_t *c, struct can_frame *d) {
    uint8_t instr[14] = {0};
    uint8_t rxd[14];

    instr[0] = MCP2515_LOADTXB_INSTR | buf;
    instr[1] = c->txbnsidh;
    instr[2] = c->txbnsidl;
    instr[3] = c->txbneid8;
    instr[4] = c->txbneid0;
    instr[5] = d->dlc;

    for (unsigned int i = 0; i < d->dlc; i++) {
        instr[i+6] = d->data[i];
    }

    spi_send1(&hspi1, instr, rxd, 14);

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
}

//------------------------------------------------------------------------------
static void _mcp2515_rts1(uint8_t buf) {
    uint8_t instr[1];
    uint8_t rxd[1];

    instr[0] = MCP2515_RTS_INSTR | buf;

    spi_send1(&hspi1, instr, rxd, 1);
}

//------------------------------------------------------------------------------
static uint8_t _mcp2515_read_status1(void) {
    uint8_t instr[2];
    uint8_t rxd[2];

    instr[0] = MCP2515_READSTATUS_INSTR;
    instr[1] = 0;

    spi_send1(&hspi1, instr, rxd, 2);

    return rxd[1];
}

//------------------------------------------------------------------------------
static uint8_t _mcp2515_rx_status1(void) {
    uint8_t instr[1];
    uint8_t rxd[1];

    instr[0] = MCP2515_RXSTATUS_INSTR;

    spi_send1(&hspi1, instr, rxd, 1);

    return rxd[0];
}

//------------------------------------------------------------------------------
static void _mcp2515_bit_modify1(uint8_t reg, uint8_t mask, uint8_t data) {
    uint8_t instr[4];
    uint8_t rxd[1];

    instr[0] = MCP2515_BITMODIFY_INSTR;
    instr[1] = reg;
    instr[2] = mask;
    instr[3] = data;

    spi_send1(&hspi1, instr, rxd, 4);
}

//------------------------------------------------------------------------------
static uint8_t _mcp2515_read_tec1(void) {
    uint8_t tec = 0;
    tec = _mcp2515_read1(TEC);

    return tec;
}

//------------------------------------------------------------------------------
mcp2515_canconf_t mcp2515_confs[] = {
    { OSC_8, BR_5, SP_75, 0x27, 0xb6, 0x04 },
    { OSC_8, BR_10, SP_75, 0x13, 0xb6, 0x04 },
    { OSC_8, BR_50, SP_75, 0x03, 0xb6, 0x04 },
    { OSC_8, BR_125, SP_75, 0x01, 0xac, 0x03 },
    { OSC_8, BR_250, SP_75, 0x00, 0xac, 0x03 },
    { OSC_8, BR_5, SP_875, 0x31, 0xb5, 0x01 },
    { OSC_8, BR_10, SP_875, 0x18, 0xb5, 0x01 },
    { OSC_8, BR_50, SP_875, 0x04, 0xb5, 0x01 },
    { OSC_8, BR_125, SP_875, 0x01, 0xb5, 0x01 },
    { OSC_8, BR_250, SP_875, 0xc0, 0xb5, 0x01 },
    { 0, 0, 0, 0, 0, 0 },
};

//------------------------------------------------------------------------------
int mcp2515_init1(uint8_t osc, uint8_t br, uint8_t sp) {
    const mcp2515_canconf_t *c;
    char canset = 0;

    /* enter Configuration Mode */
    _mcp2515_reset1();

    for (c = mcp2515_confs; c->osc; c++) {
        if (osc == c->osc && br == c->br && sp == c->sp) {
            _mcp2515_write1(CNF1, c->cnf1);
            _mcp2515_write1(CNF2, c->cnf2);
            _mcp2515_write1(CNF3, c->cnf3);

            /* check */
            if ((_mcp2515_read1(CNF1) != c->cnf1) ||
                    (_mcp2515_read1(CNF2) != c->cnf2) ||
                    (_mcp2515_read1(CNF3) != c->cnf3)) {
                return -1;
            }

            canset = 1;
        }
    }

    if (!canset) {
        return -1;
    }

    /* receives any message */
    _mcp2515_bit_modify1(RXB0CTRL, RXB0CTRL_RXM1 | RXB0CTRL_RXM0 |
            RXB0CTRL_BUKT, 0xFF);

    /* enable MCP2515 interrupts */
    //_mcp2515_bit_modify1(CANINTE, CANINTE_RX0IE | CANINTE_RX1IE |
    //        CANINTE_TX0IE | CANINTE_TX1IE | CANINTE_TX2IE, 0xFF);
    _mcp2515_bit_modify1(CANINTE, CANINTE_RX0IE | CANINTE_RX1IE, 0xFF);

    /* enter Normal Mode */
    _mcp2515_bit_modify1(CANCTRL0,
            CANCTRL_REQOP2 | CANCTRL_REQOP1 | CANCTRL_REQOP0, 0x00);

    return 0;
}

//------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    vTaskNotifyGiveFromISR(can_recv_h, pdFALSE);
}

void can_recv(void *pvParameters) {
    uint32_t n;
    const TickType_t wait = pdMS_TO_TICKS(100);

    for (;;) {
        /* pdTRUE to act like a binary semaphore */
        n = ulTaskNotifyTake(pdTRUE, wait);

        if (n == 0x1) {
            uint8_t read_status;
            can_frame_t rxb;

            //vTaskSuspendAll();
            //taskENTER_CRITICAL();
            taskDISABLE_INTERRUPTS();

            read_status = _mcp2515_read_status1();

            /* sent frame */
            if (read_status & MCP2515_READSTATUS_TX0IF) {
                _mcp2515_bit_modify1(CANINTF, CANINTF_TX0IF, 0x00);
            }
            if (read_status & MCP2515_READSTATUS_TX1IF) {
                _mcp2515_bit_modify1(CANINTF, CANINTF_TX1IF, 0x00);
            }
            if (read_status & MCP2515_READSTATUS_TX2IF) {
                _mcp2515_bit_modify1(CANINTF, CANINTF_TX2IF, 0x00);
            }

            /* received frame */
            if (read_status & MCP2515_READSTATUS_RX0IF) {
                /* can_frame_init1() tries to malloc inside irq handler */
                can_frame_init(&rxb);

                _mcp2515_bit_modify1(CANINTF, CANINTF_RX0IF, 0x00);
                _mcp2515_read_rx_buffer1(MCP2515_READRXB_RXB0SIDH, &rxb);

                mcp2515_int_rx(&rxb);
            }
            if (read_status & MCP2515_READSTATUS_RX1IF) {
                /* can_frame_init1() tries to malloc inside irq handler */
                can_frame_init(&rxb);

                _mcp2515_bit_modify1(CANINTF, CANINTF_RX1IF, 0x00);
                _mcp2515_read_rx_buffer1(MCP2515_READRXB_RXB0SIDH, &rxb);

                mcp2515_int_rx(&rxb);
            }

            //xTaskResumeAll();
            //taskEXIT_CRITICAL();
            taskENABLE_INTERRUPTS();
        }
    }
}

//------------------------------------------------------------------------------
void mcp2515_send1(uint8_t ft, const uint32_t id, const int prio,
        struct can_frame *d) {
    uint8_t txbnsidl;
    uint8_t txbn = 0;
    uint8_t read_status;
    txbconf_t c;

    //vTaskSuspendAll();
    //taskENTER_CRITICAL();
    taskDISABLE_INTERRUPTS();

    read_status = _mcp2515_read_status1();

    /* try TXB0 */
    if (!(read_status & MCP2515_READSTATUS_TX0REQ)) {
        _mcp2515_bit_modify1(TXB0CTRL, TXB0CTRL_TXP1 | TXB0CTRL_TXP0, prio);
        txbn = 0;
    /* try TXB1 */
    } else if (!(read_status & MCP2515_READSTATUS_TX1REQ)) {
        _mcp2515_bit_modify1(TXB1CTRL, TXB1CTRL_TXP1 | TXB1CTRL_TXP0, prio);
        txbn = 1;
    /* try TXB2 */
    } else if (!(read_status & MCP2515_READSTATUS_TX2REQ)) {
        _mcp2515_bit_modify1(TXB2CTRL, TXB2CTRL_TXP1 | TXB2CTRL_TXP0, prio);
        txbn = 2;
    }

    /* standard frame */
    if (ft == STDF) {
        c.txbnsidh = (uint8_t) ((id & TXBnSIDH_STD) >> 3);
        c.txbnsidl = (uint8_t) ((id & TXBnSIDL_STD) << 5);
        c.txbneid8 = 0x00;
        c.txbneid0 = 0x00;

        /* load TXB */
        _mcp2515_load_tx_buffer1(MCP2515_LOADTXB_TXBnSIDH(txbn), &c, d);

        /* request to send TXBn */
        _mcp2515_rts1(MCP2515_RTS_TXBn(txbn));
    /* extended frame */
    } else if (ft == EXTF) {
        txbnsidl = (uint8_t) ((id & TXBnSIDL10_EXT) >> 16);
        txbnsidl = (uint8_t) (txbnsidl | ((id & TXBnSIDL75_EXT) >> 13));

        c.txbnsidh = (uint8_t) ((id & TXBnSIDH_EXT) >> 21);
        c.txbnsidl = txbnsidl;
        c.txbneid8 = (uint8_t) ((id & TXBnEID8_EXT) >> 8);
        c.txbneid0 = (uint8_t) (id & TXBnEID0_EXT);

        /* load TXB */
        _mcp2515_load_tx_buffer1(MCP2515_LOADTXB_TXBnSIDH(txbn), &c, d);

        /* activate extended frame bit */
        _mcp2515_write1(TXBnSIDL(txbn), c.txbnsidl | TXBnSIDL_EXIDE);

        /* request to send TXBn */
        _mcp2515_rts1(MCP2515_RTS_TXBn(txbn));
    }

    //xTaskResumeAll();
    //taskEXIT_CRITICAL();
    taskENABLE_INTERRUPTS();
}
