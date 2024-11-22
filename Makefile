# Compiler and flags
CC = arm-none-eabi-gcc
CFLAGS = -Wall -Wextra -Os -nostartfiles -mcpu=cortex-m0 -mfloat-abi=soft
SRCDIRS = ./src
INCDIRS = ./include ./system/CMSIS
DEFS += STM32F030x6 STM32F0 F_CPU=8000000 PAGES=16


#LD flags
LDFLAGS = -T./system/boot16.ld

# Define flags
DEFINES = $(addprefix -D,$(DEFS))

# Include flags
INCLUDES = $(addprefix -I,$(INCDIRS))

# Source files
SOURCES = $(foreach dir,$(SRCDIRS),$(wildcard $(dir)/*.c))

# Object files
OBJECTS = $(SOURCES:.c=.o)
OBJECTS += startup_boot.o

# Output file
TARGET = bootloader.bin
TARGET_ELF = bootloader.elf

# Rules
all: $(TARGET)

$(TARGET): $(TARGET_ELF)
	arm-none-eabi-objcopy -O binary $< $@

$(TARGET_ELF): $(OBJECTS)
	$(CC) $(CFLAGS) $(INCLUDES) $(DEFINES) $(LDFLAGS) $^ -o $@

startup_boot.o: ./system/startup_boot.s
	$(CC) $(CFLAGS) $(INCLUDES) $(DEFINES) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) $(DEFINES) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET) $(TARGET_ELF)
