.PHONY: all
all:
	avr-gcc -g -mmcu=attiny84 -c tach.c
	avr-gcc -g -mmcu=attiny84 -o tach.elf tach.o
	avr-objcopy -j .text -j .data -O ihex tach.elf tach.hex
	avr-size -C --mcu=attiny84 tach.elf 

.PHONY: clean
clean:
	rm tach.o
	rm tach.elf
	rm tach.hex

.PHONY: viewhex
viewhex:
	cat tach.hex

.PHONY: upload
upload:
	avrdude -c usbtiny -p t84 -U lfuse:w:0xFF:m
	avrdude -c usbtiny -p t84 -U flash:w:tach.hex
	
