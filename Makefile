PROG = mcp2515-tx
OPENOCD_INTERFACE = stlink.cfg
OPENOCD_TARGET = stm32f103c8_blue_pill.cfg
PORT = /dev/ttyU0

.include "Makefile.arm"
