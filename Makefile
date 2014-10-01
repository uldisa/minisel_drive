include Arduino.mk

LIBS=core SPI minisel_lcd
#PORT=/dev/ttyACM0
#PORT=COM11
#CPU=atmega2560
#BR=115200
#CPPFLAGS+=-DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR
#VARIANT=mega
#PROGRAMMER=wiring
#PORT=/dev/ttyUSB0
PORT=COM10
CPU=atmega328p
BR=57600
CPPFLAGS+=-DARDUINO_AVR_PRO -DARDUINO_ARCH_AVR
VARIANT=standard
PROGRAMMER=arduino

USER_LIB_PATH=$(CURDIR)/libraries
WIRE_LIB_PATH=$(ARD_HOME)/hardware/arduino/avr/libraries/Wire
include lib.mk

ifeq ($(filter %-pc-cygwin,$(MAKE_HOST)),)
CPPFLAGS+=-I$(WIRE_LIB_PATH) -I$(WIRE_LIB_PATH)/utility
else
CPPFLAGS+=-I$(shell cygpath -m $(WIRE_LIB_PATH)) -I$(shell cygpath -m $(WIRE_LIB_PATH)/utility)
endif

CPPFLAGS+=-Wall -Wextra -I. -Os -fno-exceptions -ffunction-sections -fdata-sections

main.elf: main.o libcore.a libminisel_lcd.a libSPI.a
test.elf: test.o libcore.a


