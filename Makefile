include Arduino.mk

LIBS=core SPI minisel_lcd PID_v1 PID_AutoTune_v0
#PORT=/dev/ttyACM0
#PORT=COM11
#CPU=atmega2560
#BR=115200
#CPPFLAGS+=-DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR
#VARIANT=mega
#PROGRAMMER=wiring
#PORT=/dev/ttyUSB0
PORT=COM9
#CPU=atmega328p
CPU=atmega8
#BR=19200
BR=57600
#CPPFLAGS+=-DARDUINO_AVR_PRO -DARDUINO_ARCH_AVR -DF_CPU=4000000L
CPPFLAGS+=-DARDUINO_AVR_PRO -DARDUINO_ARCH_AVR -DF_CPU=8000000L
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

main.o: programSelector.h
#main.elf: main.o libminisel_lcd.a libSPI.a libPID_v1.a core/wiring_analog.o core/wiring_digital.o core/WInterrupts.o #libcore.a 
main.elf: main.o libminisel_lcd.a libSPI.a libPID_v1.a libcore.a 

test_LCD_atmega8.elf: test_LCD_atmega8.o libminisel_lcd.a libSPI.a libPID_v1.a libcore.a

test.elf: test.o libcore.a

ATmegaBOOT.o:CPPFLAGS+=-DF_CPU=4000000L 
ATmegaBOOT.hex:ATmegaBOOT.c
