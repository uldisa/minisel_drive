set -x
#CLOCK=4000000L
#BAUD=19200
CLOCK=8000000L
BAUD=57600
avr-gcc -g -Wall -Os -fno-inline-small-functions -fno-split-wide-types -mshort-calls -mmcu=atmega8 -DF_CPU=$CLOCK   '-DLED_START_FLASHES=3' -DBAUD_RATE=$BAUD   -c -o optiboot.o optiboot.c || exit $?
avr-gcc -g -Wall -Os -fno-inline-small-functions -fno-split-wide-types -mshort-calls -mmcu=atmega8 -DF_CPU=$CLOCK '-DLED_START_FLASHES=3' -DBAUD_RATE=$BAUD -Wl,--section-start=.text=0x1e00 -Wl,--section-start=.version=0x1ffe -Wl,--relax -Wl,--gc-sections -nostartfiles -nostdlib -o optiboot_atmega8.elf optiboot.o || exit $?
avr-objcopy -j .text -j .data -j .version --set-section-flags .version=alloc,load -O ihex optiboot_atmega8.elf optiboot_atmega8.hex || exit $?
avr-objdump -h -S optiboot_atmega8.elf > optiboot_atmega8.lst || exit $?
rm optiboot.o optiboot_atmega8.elf || exit $?
# ext osc
#avrdude  -P COM12 -b 19200 -c avrisp -p m8 -C "C:/Program Files/Arduino/hardware/tools/avr/etc/avrdude.conf" -u  -U lfuse:w:0xbf:m -U hfuse:w:0xcc:m -U lock:w:0x3f:m || exit $?
# int osc 8Mhz
avrdude  -P COM12 -b 19200 -c avrisp -p m8 -C "C:/Program Files/Arduino/hardware/tools/avr/etc/avrdude.conf" -u  -U lfuse:w:0xa4:m -U hfuse:w:0xcc:m -U lock:w:0x3f:m || exit $?
avrdude  -P COM12 -b 19200 -c avrisp -p m8 -C "C:/Program Files/Arduino/hardware/tools/avr/etc/avrdude.conf"  -U flash:w:optiboot_atmega8.hex -U lock:w:0x2f:m || exit $?
