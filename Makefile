PROG = skel
OPENOCD_INTERFACE = stlink-v2.cfg
OPENOCD_TARGET = stm32f1x.cfg
PORT = /dev/ttyU0

.include "Makefile.arm"
