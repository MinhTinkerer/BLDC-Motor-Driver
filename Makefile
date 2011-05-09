CC=avr-gcc
CFLAGS=-std=c99 -Os -mmcu=attiny861 -Wall
OBJCPY=avr-objcopy
PROG=avrdude

all: main.hex

clean:
	rm -f main.hex main.elf main.o

main.hex: main.elf
	$(OBJCPY) -j .text -O ihex main.elf main.hex

main.elf: main.o
	$(CC) -o main.elf main.o
  
.o:
	$(CC) $(CFLAGS) -c $*.c

flash:
	$(PROG) -c usbtiny -p t861 -U flash:w:main.hex
