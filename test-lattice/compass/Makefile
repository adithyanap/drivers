# ============================================================================
#  RadioLink SE100 V2.0 (IST8310) compass firmware for CDAC VEGA "THEJAS32"
#
#  The SE100 compass port is connected to the ARIES v3 I2C bus. Magnetic field
#  X/Y/Z readings are printed to UART0 at 115200 baud.
#
#  Targets
#    make            -> build  (ELF + BIN + HEX + MAP + listings)
#    make build      -> same as `make`
#    make flash      -> upload the .bin through the ARIES XMODEM loader
#    make run        -> build + flash
#    make monitor    -> build + flash + open picocom
#    make serial     -> open picocom without reflashing
#    make size       -> print section sizes
#    make disasm     -> regenerate disassembly listing
#    make info       -> print resolved toolchain / board configuration
#    make clean      -> remove build artefacts
#
#  Everything is overridable from the command line, e.g.
#    make SERIAL_PORT=/dev/ttyUSB0 MONITOR_BAUD=115200
# ============================================================================

# ---------------------------------------------------------------------------
# Board / toolchain
# ---------------------------------------------------------------------------
TARGET            ?= THEJAS32
CROSS_COMPILE     ?= riscv32-vega-elf

# vega-tools-rv32 installation.
VEGA_TOOLS        ?= $(HOME)/project/riscv_toolchain/riscv_toolchain
TOOLCHAIN_BIN     ?= $(VEGA_TOOLS)/toolchain/bin

# The vega gcc 13.x cc1 needs libisl.so.22. Point this at a directory that
# contains libisl.so.22 (or leave empty to use the system loader).
TOOLCHAIN_LIB     ?= $(HOME)/vega-toolchain-compat/install/lib

CC        := $(TOOLCHAIN_BIN)/$(CROSS_COMPILE)-gcc
OBJCOPY   := $(TOOLCHAIN_BIN)/$(CROSS_COMPILE)-objcopy
OBJDUMP   := $(TOOLCHAIN_BIN)/$(CROSS_COMPILE)-objdump
SIZE      := $(TOOLCHAIN_BIN)/$(CROSS_COMPILE)-size
GDB       := $(TOOLCHAIN_BIN)/$(CROSS_COMPILE)-gdb

# THEJAS32 is a 32-bit RV32I core with Zicsr; all floating point is soft-float.
ARCH      ?= -march=rv32i_zicsr
ABI       ?= -mabi=ilp32
CMODEL    ?= -mcmodel=medany

# ---------------------------------------------------------------------------
# Project layout
# ---------------------------------------------------------------------------
ROOT          := $(CURDIR)
BUILD_DIR     := $(ROOT)/build
TARGET_NAME   ?= riscv_compass

# Shared code lives alongside compass/ in the repository root.
C_SOURCES := \
  main.c \
  ../driver/src/magnetometer/ist8310.c \
  ../common/src/i2c.c \
  ../common/src/uart.c

ASM_SOURCES := \
  startup.S

# Map shared sources into the local build tree so every object stays under
# compass/build/ rather than being written next to the sources.
OBJECTS := \
  $(BUILD_DIR)/main.o \
  $(BUILD_DIR)/driver/src/magnetometer/ist8310.o \
  $(BUILD_DIR)/common/src/i2c.o \
  $(BUILD_DIR)/common/src/uart.o \
  $(ASM_SOURCES:%.S=$(BUILD_DIR)/%.o)

DEPS := $(OBJECTS:.o=.d)

LDSCRIPT ?= $(ROOT)/linker.ld

# ---------------------------------------------------------------------------
# Flags
# ---------------------------------------------------------------------------
INCLUDES := \
  -I../common/include \
  -I../common/src \
  -I../driver/include \
  -I../driver/include/magnetometer \
  -I../driver/src

CFLAGS := \
  $(ARCH) $(ABI) $(CMODEL) \
  -std=c11 -Os -g -Wall -Wextra \
  -ffunction-sections -fdata-sections -fno-common \
  -fno-unwind-tables -fno-asynchronous-unwind-tables \
  $(INCLUDES)

ASMFLAGS := $(ARCH) $(ABI) $(CMODEL)

LDFLAGS := \
  $(ARCH) $(ABI) $(CMODEL) \
  -T$(LDSCRIPT) \
  -nostartfiles \
  -Wl,--gc-sections \
  -Wl,--no-warn-rwx-segments \
  -Wl,-Map=$(BUILD_DIR)/$(TARGET_NAME).map \
  -Wl,--print-memory-usage

LDLIBS := \
  --specs=nano.specs \
  --specs=nosys.specs \
  -Wl,--start-group -lgcc -lc -lm -Wl,--end-group

# ---------------------------------------------------------------------------
# Flash / serial console
# ---------------------------------------------------------------------------
SERIAL_PORT    ?= /dev/ttyUSB0
FLASH_TOOL     ?= $(HOME)/Desktop/vega-flasher/xmodemflasher
FLASH_BAUD     ?= 115200
MONITOR_BAUD   ?= 115200
PICOCOM        ?= picocom

# ---------------------------------------------------------------------------
# Artifacts
# ---------------------------------------------------------------------------
ELF := $(BUILD_DIR)/$(TARGET_NAME).elf
BIN := $(BUILD_DIR)/$(TARGET_NAME).bin
HEX := $(BUILD_DIR)/$(TARGET_NAME).hex
LST := $(BUILD_DIR)/$(TARGET_NAME).lst
DMP := $(BUILD_DIR)/$(TARGET_NAME).dump
MAP := $(BUILD_DIR)/$(TARGET_NAME).map

# ---------------------------------------------------------------------------
# Toolchain environment (libisl.so.22 shim)
# ---------------------------------------------------------------------------
ifneq ($(strip $(TOOLCHAIN_LIB)),)
  export LD_LIBRARY_PATH := $(TOOLCHAIN_LIB)$(if $(LD_LIBRARY_PATH),:$(LD_LIBRARY_PATH))
endif

# ---------------------------------------------------------------------------
# Rules
# ---------------------------------------------------------------------------
.PHONY: all build clean distclean flash run monitor serial size disasm info help check-toolchain check-serial

all: build

# Compile a throwaway file so a missing cc1 runtime dependency is caught
# before make emits a wall of confusing per-file errors.
check-toolchain:
	@command -v $(CC) >/dev/null 2>&1 || { \
	    echo "error: compiler not found: $(CC)"; \
	    echo "       set TOOLCHAIN_BIN=/path/to/vega-tools-rv32/toolchain/bin"; \
	    exit 1; }
	@echo 'int main(void){return 0;}' | $(CC) $(ARCH) $(ABI) $(CMODEL) \
	    -x c -c - -o /dev/null 2>/tmp/.vega-tc-check.log || { \
	    echo "error: toolchain cannot compile for $(ARCH) $(ABI)."; \
	    sed 's/^/       /' /tmp/.vega-tc-check.log; \
	    if grep -q 'libisl' /tmp/.vega-tc-check.log; then \
	      echo "       vega gcc 13.x needs libisl.so.22 (distros ship only .23)."; \
	      echo "       point TOOLCHAIN_LIB at an isl-0.22 install, e.g."; \
	      echo "           make TOOLCHAIN_LIB=/opt/isl-0.22/install/lib"; \
	    fi; \
	    rm -f /tmp/.vega-tc-check.log; \
	    exit 1; }
	@rm -f /tmp/.vega-tc-check.log

build: check-toolchain $(BIN) $(HEX)
	@echo ""
	@echo "Built:"
	@echo "  ELF : $(ELF)"
	@echo "  BIN : $(BIN)"
	@echo "  HEX : $(HEX)"
	@echo "  MAP : $(MAP)"
	@echo ""

$(BUILD_DIR)/common/src/%.o: ../common/src/%.c
	@mkdir -p $(@D)
	@echo "  CC      $<"
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/driver/src/%.o: ../driver/src/%.c
	@mkdir -p $(@D)
	@echo "  CC      $<"
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(@D)
	@echo "  CC      $<"
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(@D)
	@echo "  AS      $<"
	@$(CC) $(ASMFLAGS) -c $< -o $@

$(ELF): $(OBJECTS) $(LDSCRIPT)
	@mkdir -p $(@D)
	@echo "  LD      $@"
	@$(CC) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@
	@$(SIZE) $@
	@$(OBJDUMP) -d $@ > $(DMP)
	@$(OBJDUMP) --source --all-headers --demangle --line-numbers --wide $@ > $(LST)

$(BIN): $(ELF)
	@echo "  OBJCOPY $@"
	@$(OBJCOPY) -I elf32-littleriscv -O binary $< $@

$(HEX): $(ELF)
	@echo "  OBJCOPY $@"
	@$(OBJCOPY) -I elf32-littleriscv -O ihex $< $@

size: $(ELF)
	@$(SIZE) -A $(ELF)

disasm: $(ELF)
	@$(OBJDUMP) -d $(ELF) > $(DMP)
	@$(OBJDUMP) --source --all-headers --demangle --line-numbers --wide $(ELF) > $(LST)
	@echo "  DMP : $(DMP)"
	@echo "  LST : $(LST)"

# The ARIES board enumerates as a USB-UART and is normally /dev/ttyUSB0.
check-serial:
	@test -e "$(SERIAL_PORT)" || { \
	    echo "error: serial port $(SERIAL_PORT) not found."; \
	    echo "       plug in the ARIES board and set SERIAL_PORT=..."; \
	    ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sed 's/^/       found: /'; \
	    exit 1; }
	@test -r "$(SERIAL_PORT)" && test -w "$(SERIAL_PORT)" || { \
	    echo "error: no read/write access to $(SERIAL_PORT)"; \
	    ls -l "$(SERIAL_PORT)" | sed 's/^/       /'; \
	    echo "       your groups: $$(id -nG)"; \
	    if ! id -nG | tr ' ' '\n' | grep -qx dialout; then \
	        echo "       fix: sudo usermod -aG dialout $$(whoami)"; \
	        echo "            then log out and back in."; \
	    fi; \
	    exit 1; }

flash: $(BIN) check-serial
	@echo "  FLASH  $(BIN) -> $(SERIAL_PORT)"
	@if [ ! -x "$(FLASH_TOOL)" ]; then \
	    echo "error: flash tool not found/executable: $(FLASH_TOOL)"; \
	    echo "       set FLASH_TOOL=/path/to/xmodemflasher"; \
	    exit 1; \
	fi
	@echo "  Press RESET on the ARIES board if it does not auto-enter the loader."
	@echo "  Waiting for the board loader (no output until the board is reset)..."
	@$(FLASH_TOOL) $(SERIAL_PORT) $(BIN)

run: flash

# Flash first, then hand the serial port over to picocom.
monitor: flash
	@command -v $(PICOCOM) >/dev/null 2>&1 || { \
	    echo "error: $(PICOCOM) not found in PATH."; \
	    echo "       install it or set PICOCOM=..."; \
	    exit 1; }
	@echo "  MONITOR $(SERIAL_PORT) @ $(MONITOR_BAUD) baud (Ctrl-A Ctrl-X to quit)"
	@$(PICOCOM) --baud $(MONITOR_BAUD) --flow n --echo $(SERIAL_PORT)

serial:
	@command -v $(PICOCOM) >/dev/null 2>&1 || { \
	    echo "error: $(PICOCOM) not found in PATH."; \
	    exit 1; }
	@$(PICOCOM) --baud $(MONITOR_BAUD) --flow n --echo $(SERIAL_PORT)

info:
	@echo "Target        : $(TARGET)"
	@echo "Prefix        : $(CROSS_COMPILE)"
	@echo "Toolchain bin : $(TOOLCHAIN_BIN)"
	@echo "CC            : $(CC)"
	@echo "Arch / ABI    : $(ARCH) $(ABI) $(CMODEL)"
	@echo "Linker script : $(LDSCRIPT)"
	@echo "Serial port   : $(SERIAL_PORT)"
	@echo "Flash tool    : $(FLASH_TOOL)"
	@echo "Monitor       : $(PICOCOM) --baud $(MONITOR_BAUD) $(SERIAL_PORT)"
	@echo "ACCESS        : $$([ -r "$(SERIAL_PORT)" ] && [ -w "$(SERIAL_PORT)" ] \
	    && echo ok || echo 'DENIED / not present')"

help:
	@echo "make            build the firmware"
	@echo "make build      same as make"
	@echo "make flash      upload the .bin over XMODEM"
	@echo "make run        build + flash"
	@echo "make monitor    build + flash, then open picocom"
	@echo "make serial     open picocom without flashing"
	@echo "make size       show section sizes"
	@echo "make disasm     regenerate listings"
	@echo "make info       show resolved configuration"
	@echo "make clean      remove build artefacts"

clean:
	@echo "  CLEAN  $(BUILD_DIR)"
	@rm -rf $(BUILD_DIR)

distclean: clean

-include $(DEPS)
