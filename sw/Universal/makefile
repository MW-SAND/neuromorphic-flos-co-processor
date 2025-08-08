# Application makefile.
# Use this makefile to configure all relevant CPU / compiler options.

# User's application sources (*.c, *.cpp, *.s, *.S); add additional files here
APP_SRC ?= vgg16_inf.c $(wildcard ./*.s) $(wildcard ./*.cpp) $(wildcard ./*.S)

# Override the default CPU ISA
MARCH = rv32im_zicsr_zifencei

# Override the default RISC-V GCC prefix
#RISCV_PREFIX ?= riscv-none-elf-

# Override default optimization goal
EFFORT = -Os # Default is 0s

# Add extended debug symbols
USER_FLAGS += -ggdb -gdwarf-3

# Adjust processor IMEM size
USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=32k

# Adjust processor DMEM size
USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=8k

# UART SIM
USER_FLAGS+=-DUART0_SIM_MODE

# Adjust maximum heap size
#USER_FLAGS += -Wl,--defsym,__neorv32_heap_size=1k

# Additional sources
#APP_SRC += $(wildcard ./*.c)
#APP_INC += -I .

# Set path to NEORV32 root directory
NEORV32_HOME ?= /home/matti/Projects/neorv32/

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk
