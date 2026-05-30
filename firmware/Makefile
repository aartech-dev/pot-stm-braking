# ============================================================
#  AART Brake Module Firmware — Makefile
#  Library : libopencm3
#  Target  : STM32G041J6M6 (Cortex-M0+, 32K flash, 8K RAM)
#
#  Requires: LIBOPENCM3_DIR to be set (done by Docker environment,
#            or override: make LIBOPENCM3_DIR=../libopencm3)
# ============================================================

TARGET  = aart_brake
BUILD   = /workspace/build
SRCDIR  = Core/Src
INCDIR  = Core/Inc

# ── libopencm3 location ──────────────────────────────────────
LIBOPENCM3_DIR ?= /opt/libopencm3
LIBOPENCM3_INC  = $(LIBOPENCM3_DIR)/include
LIBOPENCM3_LIB  = $(LIBOPENCM3_DIR)/lib

# ── Toolchain ────────────────────────────────────────────────
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size
GDB     = $(PREFIX)gdb

# ── MCU — Cortex-M0+, no FPU ─────────────────────────────────
ARCH_FLAGS = -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft

# ── Sources ──────────────────────────────────────────────────
SRCS = \
    $(SRCDIR)/main.c \
    $(SRCDIR)/brake_module.c

# ── Includes ─────────────────────────────────────────────────
INCS = \
    -I$(INCDIR) \
    -I$(LIBOPENCM3_INC)

# ── Defines ──────────────────────────────────────────────────
# STM32G041 — libopencm3 uses this to select the right headers
DEFS = -DSTM32G0

# ── Linker script ─────────────────────────────────────────────
# libopencm3 ships linker scripts for each device.
# STM32G041J6: 32K flash, 8K RAM
LDSCRIPT = $(LIBOPENCM3_DIR)/lib/stm32/g0/stm32g041j6.ld

# ── Compiler flags ───────────────────────────────────────────
CFLAGS  = $(ARCH_FLAGS) \
          $(DEFS) \
          $(INCS) \
          -std=c11 \
          -Wall -Wextra -Wpedantic \
          -Wno-unused-parameter \
          -fdata-sections \
          -ffunction-sections \
          -fstack-usage \
          -MMD -MP

# Default: debug
CFLAGS += -Og -g3 -DDEBUG

ifeq ($(BUILD_TYPE),release)
    CFLAGS  := $(filter-out -Og -g3 -DDEBUG,$(CFLAGS))
    CFLAGS  += -O2
endif

# ── Linker flags ─────────────────────────────────────────────
LDFLAGS = $(ARCH_FLAGS) \
          -T$(LDSCRIPT) \
          -L$(LIBOPENCM3_LIB) \
          -nostartfiles \
          -Wl,--start-group \
          -lopencm3_stm32g0 \
          -lc \
          -lgcc \
          -lnosys \
          -Wl,--end-group \
          -Wl,--gc-sections \
          -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
          -Wl,--print-memory-usage \
          --specs=nano.specs

# ── Objects ──────────────────────────────────────────────────
OBJS = $(addprefix $(BUILD)/,$(addsuffix .o,$(basename $(notdir $(SRCS)))))
DEPS = $(OBJS:.o=.d)

vpath %.c $(sort $(dir $(SRCS)))

# ── Targets ──────────────────────────────────────────────────
.PHONY: all clean flash flash-stlink flash-jlink debug-server size dump help

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin

$(BUILD):
	mkdir -p $@

$(BUILD)/%.o: %.c | $(BUILD)
	@echo "  CC  $<"
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	@echo "  LD  $@"
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

# ── Flash ────────────────────────────────────────────────────
flash: flash-stlink

flash-stlink: $(BUILD)/$(TARGET).elf
	openocd \
	    -f interface/stlink.cfg \
	    -f target/stm32g0x.cfg \
	    -c "program $(BUILD)/$(TARGET).elf verify reset exit"

flash-jlink: $(BUILD)/$(TARGET).elf
	openocd \
	    -f interface/jlink.cfg \
	    -c "transport select swd" \
	    -f target/stm32g0x.cfg \
	    -c "program $(BUILD)/$(TARGET).elf verify reset exit"

# ── Debug server (connect with arm-none-eabi-gdb on :3333) ───
debug-server:
	openocd \
	    -f interface/stlink.cfg \
	    -f target/stm32g0x.cfg \
	    -c "init" -c "reset halt"

# ── Utilities ────────────────────────────────────────────────
size: $(BUILD)/$(TARGET).elf
	$(SIZE) --format=berkeley $<

dump: $(BUILD)/$(TARGET).elf
	$(OBJDUMP) -D $< > $(BUILD)/$(TARGET).dump
	@echo "Dump written to $(BUILD)/$(TARGET).dump"

clean:
	rm -rf $(BUILD)

help:
	@echo ""
	@echo "  AART Brake — Makefile (libopencm3)"
	@echo ""
	@echo "  make                   Debug build"
	@echo "  make BUILD_TYPE=release  Optimised build"
	@echo "  make flash             Flash via ST-Link"
	@echo "  make flash-stlink      Flash via ST-Link"
	@echo "  make flash-jlink       Flash via J-Link"
	@echo "  make debug-server      OpenOCD GDB server on :3333"
	@echo "  make size              Section sizes"
	@echo "  make dump              Disassembly"
	@echo "  make clean             Remove build/"
	@echo ""
	@echo "  LIBOPENCM3_DIR=$(LIBOPENCM3_DIR)"
	@echo ""

-include $(DEPS)
