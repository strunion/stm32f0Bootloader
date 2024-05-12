CC=arm-gcc-none-eabi

INCUDES += cmsis_device_f0/include/
INCUDES += system/

LDSCRIPT = stm32f0.ld

CFLAGS += -mcpu=cortex-m0 $(addprefix -I, $(INCUDES)) -O0 -g -Wall -Werror -std=c99

all: main.bin

main.bin: main.elf
	$(CC)-objcopy -O binary $< $@

main.elf: main.o startup.o system_stm32f0xx.o

clean:
	rm -f *.o *.elf *.bin
