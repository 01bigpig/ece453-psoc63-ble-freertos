CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS_COMMON = -Wall -Os -ffunction-sections -fdata-sections -nostdlib -ffreestanding
LDFLAGS_COMMON = -Wl,--gc-sections -nostdlib

# CM0+ (Cortex-M0+)
CM0P_CFLAGS = $(CFLAGS_COMMON) -mcpu=cortex-m0plus -mthumb
CM0P_LDFLAGS = $(LDFLAGS_COMMON) -T cm0p/linker.ld

# CM4 (Cortex-M4)
CM4_CFLAGS = $(CFLAGS_COMMON) -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
CM4_LDFLAGS = $(LDFLAGS_COMMON) -T cm4/linker.ld

BUILD = build

.PHONY: all clean flash

all: $(BUILD)/cm0p.hex $(BUILD)/cm4.hex $(BUILD)/combined.hex
	@echo "Build complete!"
	@$(SIZE) $(BUILD)/cm0p.elf $(BUILD)/cm4.elf

$(BUILD)/cm0p.elf: cm0p/startup.c cm0p/linker.ld
	@mkdir -p $(BUILD)
	$(CC) $(CM0P_CFLAGS) $(CM0P_LDFLAGS) -o $@ cm0p/startup.c

$(BUILD)/cm4.elf: cm4/main.c cm4/linker.ld
	@mkdir -p $(BUILD)
	$(CC) $(CM4_CFLAGS) $(CM4_LDFLAGS) -o $@ cm4/main.c

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCOPY) -O ihex $< $@

# Combine both hex files into one for flashing
$(BUILD)/combined.hex: $(BUILD)/cm0p.hex $(BUILD)/cm4.hex
	cat $(BUILD)/cm0p.hex $(BUILD)/cm4.hex | grep -v ':00000001FF' > $@
	echo ':00000001FF' >> $@

flash: $(BUILD)/combined.hex
	openocd -f interface/cmsis-dap.cfg -f target/psoc6.cfg \
		-c "init" \
		-c "reset halt" \
		-c "flash write_image erase $(BUILD)/combined.hex" \
		-c "reset run" \
		-c "shutdown"

clean:
	rm -rf $(BUILD)
