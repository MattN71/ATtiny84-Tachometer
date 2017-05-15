program_name := tach
program_SRCS := $(wildcard *.c)
program_OBJS := ${program_SRCS:.c=.o}

CC := avr-gcc
cpu := attiny84
programmer := usbtiny

.PHONY: all
all:
	avr-gcc -g -O -mmcu=$(cpu) -c $(program_SRCS)
	avr-gcc -g -O -mmcu=$(cpu) -o $(program_name).elf $(program_OBJS)
	avr-objcopy -j .text -j .data -O ihex $(program_name).elf $(program_name).hex
	avr-size -C --mcu=$(cpu) $(program_name).elf 

.PHONY: clean
clean:
	rm $(program_OBJS)
	rm $(program_name).elf
	rm $(program_name).hex

.PHONY: viewhex
viewhex:
	cat $(program_name).hex

.PHONY: upload
upload:
	avrdude -c $(programmer) -p t84 -U flash:w:$(program_name).hex
	
.PHONY: fuse
fuse:
	avrdude -c $(programmer) -p t84 -U lfuse:w:0xFF:m
	
	
