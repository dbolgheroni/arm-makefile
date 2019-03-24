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

#include <can.h>

/* MCP2515 instructions */
#define MCP2515_RESET_INSTR             0xC0

#define MCP2515_READ_INSTR              0x03

#define MCP2515_READRXB_INSTR           0x90
#define MCP2515_READRXB_RXB0SIDH        0x00
#define MCP2515_READRXB_RXB0D0          0x01
#define MCP2515_READRXB_RXB1SIDH        0x02
#define MCP2515_READRXB_RXB1D0          0x03

#define MCP2515_WRITE_INSTR             0x02

#define MCP2515_LOADTXB_INSTR           0x40
#define MCP2515_LOADTXB_TXB0SIDH        0x00
#define MCP2515_LOADTXB_TXB0D0          0x01
#define MCP2515_LOADTXB_TXB1SIDH        0x02
#define MCP2515_LOADTXB_TXB1D0          0x03
#define MCP2515_LOADTXB_TXB2SIDH        0x04
#define MCP2515_LOADTXB_TXB2D0          0x05

#define MCP2515_RTS_INSTR               0x80
#define MCP2515_RTS_TXB0                0x01
#define MCP2515_RTS_TXB1                0x02
#define MCP2515_RTS_TXB2                0x04

#define MCP2515_READSTATUS_INSTR        0xA0

#define MCP2515_RXSTATUS_INSTR          0xB0

#define MCP2515_BITMODIFY_INSTR         0x05

/* MCP2515 registers */
#define RXF0SIDH        0x00
#define RXF0SIDL        0x01
#define RXF0EID8        0x02
#define RXF0EID0        0x03
#define RXF1SIDH        0x04
#define RXF1SIDL        0x05
#define RXF1EID8        0x06
#define RXF1EID0        0x07
#define RXF2SIDH        0x08
#define RXF2SIDL        0x09
#define RXF2EID8        0x0A
#define RXF2EID0        0x0B
#define BFPCTRL         0x0C
#define TXRTSCTRL       0x0D
#define CANSTAT0        0x0E
#define CANCTRL0        0x0F

#define RXF3SIDH        0x10
#define RXF3SIDL        0x11
#define RXF3EID8        0x12
#define RXF3EID0        0x13
#define RXF4SIDH        0x14
#define RXF4SIDL        0x15
#define RXF4EID8        0x16
#define RXF4EID0        0x17
#define RXF5SIDH        0x18
#define RXF5SIDL        0x19
#define RXF5EID8        0x1A
#define RXF5EID0        0x1B
#define TEC             0x1C
#define REC             0x1D
#define CANSTAT1        0x1E
#define CANCTRL1        0x1F

#define RXM0SIDH        0x20
#define RXM0SIDL        0x21
#define RXM0EID8        0x22
#define RXM0EID0        0x23
#define RXM1SIDH        0x24
#define RXM1SIDL        0x25
#define RXM1EID8        0x26
#define RXM1EID0        0x27
#define CNF3            0x28
#define CNF2            0x29
#define CNF1            0x2A
#define CANINTE         0x2B
#define CANINTF         0x2C
#define EFLG            0x2D
#define CANSTAT2        0x2E
#define CANCTRL2        0x2F

#define TXB0CTRL        0x30
#define TXB0SIDH        0x31
#define TXB0SIDL        0x32
#define TXB0EID8        0x33
#define TXB0EID0        0x34
#define TXB0DLC         0x35
#define TXB0D0          0x36
#define TXB0D1          0x37
#define TXB0D2          0x38
#define TXB0D3          0x39
#define TXB0D4          0x3A
#define TXB0D5          0x3B
#define TXB0D6          0x3C
#define TXB0D7          0x3D
#define CANSTAT3        0x3E
#define CANCTRL3        0x3F

#define TXB1CTRL        0x40
#define TXB1SIDH        0x41
#define TXB1SIDL        0x42
#define TXB1EID8        0x43
#define TXB1EID0        0x44
#define TXB1DLC         0x45
#define TXB1D0          0x46
#define TXB1D1          0x47
#define TXB1D2          0x48
#define TXB1D3          0x49
#define TXB1D4          0x4A
#define TXB1D5          0x4B
#define TXB1D6          0x4C
#define TXB1D7          0x4D
#define CANSTAT4        0x4E
#define CANCTRL4        0x4F

#define TXB2CTRL        0x50
#define TXB2SIDH        0x51
#define TXB2SIDL        0x52
#define TXB2EID8        0x53
#define TXB2EID0        0x54
#define TXB2DLC         0x55
#define TXB2D0          0x56
#define TXB2D1          0x57
#define TXB2D2          0x58
#define TXB2D3          0x59
#define TXB2D4          0x5A
#define TXB2D5          0x5B
#define TXB2D6          0x5C
#define TXB2D7          0x5D
#define CANSTAT5        0x5E
#define CANCTRL5        0x5F

#define RXB0CTRL        0x60
#define RXB0SIDH        0x61
#define RXB0SIDL        0x62
#define RXB0EID8        0x63
#define RXB0EID0        0x64
#define RXB0DLC         0x65
#define RXB0D0          0x66
#define RXB0D1          0x67
#define RXB0D2          0x68
#define RXB0D3          0x69
#define RXB0D4          0x6A
#define RXB0D5          0x6B
#define RXB0D6          0x6C
#define RXB0D7          0x6D
#define CANSTAT6        0x6E
#define CANCTRL6        0x6F

#define RXB1CTRL        0x70
#define RXB1SIDH        0x71
#define RXB1SIDL        0x72
#define RXB1EID8        0x73
#define RXB1EID0        0x74
#define RXB1DLC         0x75
#define RXB1D0          0x76
#define RXB1D1          0x77
#define RXB1D2          0x78
#define RXB1D3          0x79
#define RXB1D4          0x7A
#define RXB1D5          0x7B
#define RXB1D6          0x7C
#define RXB1D7          0x7D
#define CANSTAT7        0x7E
#define CANCTRL7        0x7F

/* flex defs to support multiple TX buffers */
#define TXBnDm(n, m)                    ((TXB0D0) + ((n) * 10) + (m))
#define TXBnDLC(n)                      ((TXB0DLC) + ((n) * 10))
#define MCP2515_RTS_TXBn(n)             (1 << (n))
#define CANINTF_TXnIF(n)                (1 << ((n) + 2))
#define MCP2515_LOADTXB_TXBnD0(n)       ((MCP2515_LOADTXB_TXB0D0) + (2 * (n)))
#define TXBnSIDH(n)                     ((TXB0SIDH) + (10 * (n)))
#define TXBnSIDL(n)                     ((TXB0SIDL) + (10 * (n)))
#define TXBnEID8(n)                     ((TXB0EID8) + (10 * (n)))
#define TXBnEID0(n)                     ((TXB0EID0) + (10 * (n)))
#define MCP2515_LOADTXB_TXBnSIDH(n)     ((MCP2515_LOADTXB_TXB0SIDH) + 2 * (n))

/* MCP2515 registers flags */
#define BFPCTRL_B0BFM           (1 << 0)
#define BFPCTRL_B1BFM           (1 << 1)
#define BFPCTRL_B0BFE           (1 << 2)
#define BFPCTRL_B1BFE           (1 << 3)
#define BFPCTRL_B0BFS           (1 << 4)
#define BFPCTRL_B1BFS           (1 << 5)

#define TXRTSCTRL_B0RTSM        (1 << 0)
#define TXRTSCTRL_B1RTSM        (1 << 1)
#define TXRTSCTRL_B2RTSM        (1 << 2)
#define TXRTSCTRL_B0RTS         (1 << 0)
#define TXRTSCTRL_B1RTS         (1 << 1)
#define TXRTSCTRL_B2RTS         (1 << 2)

#define CANSTAT_ICOD0           (1 << 1)
#define CANSTAT_ICOD1           (1 << 2)
#define CANSTAT_ICOD2           (1 << 3)
#define CANSTAT_OPMOD0          (1 << 5)
#define CANSTAT_OPMOD1          (1 << 6)
#define CANSTAT_OPMOD2          (1 << 7)

#define CANCTRL_CLKPRE0         (1 << 0)
#define CANCTRL_CLKPRE1         (1 << 1)
#define CANCTRL_CLKEN           (1 << 2)
#define CANCTRL_OSM             (1 << 3)
#define CANCTRL_ABAT            (1 << 4)
#define CANCTRL_REQOP0          (1 << 5)
#define CANCTRL_REQOP1          (1 << 6)
#define CANCTRL_REQOP2          (1 << 7)

#define CNF3_PHSEG0             (1 << 0)
#define CNF3_PHSEG1             (1 << 1)
#define CNF3_PHSEG2             (1 << 2)
#define CNF3_WAKFIL             (1 << 6)
#define CNF3_SOF                (1 << 7)

#define CNF2_PRSEG0             (1 << 0)
#define CNF2_PRSEG1             (1 << 1)
#define CNF2_PRSEG2             (1 << 2)
#define CNF2_PHSEG10            (1 << 3)
#define CNF2_PHSEG11            (1 << 4)
#define CNF2_PHSEG12            (1 << 5)
#define CNF2_SAM                (1 << 6)
#define CNF2_BTLMODE            (1 << 7)

#define CNF1_BRP0               (1 << 0)
#define CNF1_BRP1               (1 << 1)
#define CNF1_BRP2               (1 << 2)
#define CNF1_BRP3               (1 << 3)
#define CNF1_BRP4               (1 << 4)
#define CNF1_BRP5               (1 << 5)
#define CNF1_SJW0               (1 << 6)
#define CNF1_SJW1               (1 << 7)

#define CANINTE_RX0IE           (1 << 0)
#define CANINTE_RX1IE           (1 << 1)
#define CANINTE_TX0IE           (1 << 2)
#define CANINTE_TX1IE           (1 << 3)
#define CANINTE_TX2IE           (1 << 4)
#define CANINTE_ERRIE           (1 << 5)
#define CANINTE_WAKIE           (1 << 6)
#define CANINTE_MERRE           (1 << 7)

#define CANINTF_RX0IF           (1 << 0)
#define CANINTF_RX1IF           (1 << 1)
#define CANINTF_TX0IF           (1 << 2)
#define CANINTF_TX1IF           (1 << 3)
#define CANINTF_TX2IF           (1 << 4)
#define CANINTF_ERRIF           (1 << 5)
#define CANINTF_WAKIF           (1 << 6)
#define CANINTF_MERRF           (1 << 7)

#define EFLG_EWARN              (1 << 0)
#define EFLG_RXWAR              (1 << 1)
#define EFLG_TXWAR              (1 << 2)
#define EFLG_RXEP               (1 << 3)
#define EFLG_TXEP               (1 << 4)
#define EFLG_TXBO               (1 << 5)
#define EFLG_RX0OVR             (1 << 6)
#define EFLG_RX1OVR             (1 << 7)

#define TXB0CTRL_TXP0           (1 << 0)
#define TXB0CTRL_TXP1           (1 << 1)
#define TXB0CTRL_TXREQ          (1 << 3)
#define TXB0CTRL_TXERR          (1 << 4)
#define TXB0CTRL_MLOA           (1 << 5)
#define TXB0CTRL_ABTF           (1 << 6)

#define TXB1CTRL_TXP0           (1 << 0)
#define TXB1CTRL_TXP1           (1 << 1)
#define TXB1CTRL_TXREQ          (1 << 3)
#define TXB1CTRL_TXERR          (1 << 4)
#define TXB1CTRL_MLOA           (1 << 5)
#define TXB1CTRL_ABTF           (1 << 6)

#define TXB2CTRL_TXP0           (1 << 0)
#define TXB2CTRL_TXP1           (1 << 1)
#define TXB2CTRL_TXREQ          (1 << 3)
#define TXB2CTRL_TXERR          (1 << 4)
#define TXB2CTRL_MLOA           (1 << 5)
#define TXB2CTRL_ABTF           (1 << 6)

#define RXB0CTRL_FILHIT0        (1 << 0)
#define RXB0CTRL_BUKT1          (1 << 1)
#define RXB0CTRL_BUKT           (1 << 2)
#define RXB0CTRL_RXRTR          (1 << 3)
#define RXB0CTRL_RXM0           (1 << 5)
#define RXB0CTRL_RXM1           (1 << 6)

#define RXB1CTRL_FILHIT0        (1 << 0)
#define RXB1CTRL_FILHIT1        (1 << 1)
#define RXB1CTRL_FILHIT2        (1 << 2)
#define RXB1CTRL_RXRTR          (1 << 3)
#define RXB1CTRL_RXM0           (1 << 5)
#define RXB1CTRL_RXM1           (1 << 6)

/* argument to mcp2515 putc */
#define STDF       0x1
#define EXTF       0x2

/* masks for handling standard and extended frames */
#define TXBnSIDL_STD       0x00000007
#define TXBnSIDH_STD       0x000007F8
#define TXBnEID0_EXT       0x000000FF
#define TXBnEID8_EXT       0x0000FF00
#define TXBnSIDL10_EXT     0x00030000
#define TXBnSIDL75_EXT     0x001C0000
#define TXBnSIDH_EXT       0x1FE00000
#define TXBnSIDL_EXIDE     0x08

/* oscillator (MHz) */
#define OSC_8   0x1
#define OSC_16  0x2

/* baud rate (Kbps) */
#define BR_5    0x1
#define BR_10   0x2
#define BR_50   0x3
#define BR_125  0x4
#define BR_250  0x5

/* sample points (%) */
#define SP_75   0x1    /* ARINC 825 */
#define SP_875  0x2    /* DeviceNet, CANopen */
#define SP_XX   0x3

/* conf */
typedef struct {
    uint8_t txbnsidh;
    uint8_t txbnsidl;
    uint8_t txbneid8;
    uint8_t txbneid0;
} txbconf_t;

void _mcp2515_reset(void);
uint8_t _mcp2515_read(uint8_t);
uint8_t _mcp2515_read_rx_buffer(uint8_t);
void _mcp2515_write(uint8_t, uint8_t);
void _mcp2515_load_tx_buffer(uint8_t, txbconf_t *, candata_t *);
void _mcp2515_rts(uint8_t);
uint8_t _mcp2515_read_status(void);
uint8_t _mcp2515_rx_status(void);
void _mcp2515_bit_modify(uint8_t, uint8_t, uint8_t);

int mcp2515_init(uint8_t, uint8_t, uint8_t);
void mcp2515_putc(uint8_t, const uint32_t, candata_t *);
