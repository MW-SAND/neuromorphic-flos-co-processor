# Application makefile.
# Use this makefile to configure all relevant CPU / compiler options.

# Override the default CPU ISA
MARCH = rv32i_zicsr_zifencei

# Override the default RISC-V GCC prefix
#RISCV_PREFIX ?= riscv32-unknown-elf-

# Override default optimization goal
EFFORT = -Os

# Add extended debug symbols
USER_FLAGS += -ggdb -gdwarf-3

# Adjust processor IMEM size
USER_FLAGS += -Wl,--defsym,__neorv32_rom_size=16k

# Adjust processor DMEM size
USER_FLAGS += -Wl,--defsym,__neorv32_ram_size=1k

# Adjust maximum heap size
#USER_FLAGS += -Wl,--defsym,__neorv32_heap_size=1k

# Sim mode for UART0
# USER_FLAGS+=-DUART0_SIM_MODE

# Additional sources
#APP_SRC += $(wildcard ./*.c)
#APP_INC += -I .

# Additional clean and clean_all commands
ADDITIONAL_CLEAN_COMMANDS = @rm -f verification.out neorv32_dmem_image_*.hex
ADDITIONAL_CLEAN_ALL_COMMANDS = @rm -rf .venv

# Set path to NEORV32 root directory
NEORV32_HOME ?= ../../..

# Include the main NEORV32 makefile
include $(NEORV32_HOME)/sw/common/common.mk

venv: .venv/touchfile

.venv/touchfile: requirements.txt
	test -d .venv || python3 -m venv .venv
	. .venv/bin/activate; pip install -Ur requirements.txt
	touch .venv/touchfile

dmem: neorv32_dmem_image_*.hex verification.out

neorv32_dmem_image_*.hex verification.out: .venv/touchfile model.py
	. .venv/bin/activate; python3 model.py

install-dmem: neorv32_dmem_image_*.hex verification.out
	for i in neorv32_dmem_image_*.hex; do \
		cp "$$i" "$(NEORV32_HOME)/rtl/core/"; \
	done
	cp verification.out $(NEORV32_HOME)/sim/verification.txt

install-hex: neorv32_raw_exe.hex
	cp neorv32_raw_exe.hex $(NEORV32_HOME)/rtl/core/neorv32_imem_image_0.hex
