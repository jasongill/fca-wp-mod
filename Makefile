PROJ_NAME = panda

CC = $(COMPILER_PATH)arm-none-eabi-gcc
OBJCOPY = $(COMPILER_PATH)arm-none-eabi-objcopy
DFU_UTIL = dfu-util

CFLAGS = -g -Wall -Wextra -Wstrict-prototypes -Werror
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4
CFLAGS += -mhard-float -DSTM32F4 -DSTM32F413xx -mfpu=fpv4-sp-d16 -fsingle-precision-constant
CFLAGS += -I inc -nostdlib -fno-builtin -std=gnu11 -Os
CFLAGS += -Tstm32_flash.ld
CFLAGS += -DALLOW_DEBUG

CERT = ./certs/debug
STARTUP = startup_stm32f413xx

# Create output directories
OBJDIR = obj
$(shell mkdir -p $(OBJDIR) >/dev/null)

DEPDIR = generated_dependencies
$(shell mkdir -p $(DEPDIR) >/dev/null)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td
POSTCOMPILE = @mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d && touch $@

# Default target
all: $(OBJDIR)/bootstub.$(PROJ_NAME).bin $(OBJDIR)/$(PROJ_NAME).bin

# Flash firmware via DFU
flash: $(OBJDIR)/bootstub.$(PROJ_NAME).bin $(OBJDIR)/$(PROJ_NAME).bin
	@echo "Press and hold the button on your WP Mod while connecting to USB"
	@sleep 1.0
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000 -D $(OBJDIR)/$(PROJ_NAME).bin
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08000000:leave -D $(OBJDIR)/bootstub.$(PROJ_NAME).bin

# Compile C files
$(OBJDIR)/%.$(PROJ_NAME).o: %.c $(DEPDIR)/%.d
	$(CC) $(DEPFLAGS) $(CFLAGS) -o $@ -c $<
	$(POSTCOMPILE)

# Compile startup assembly
$(OBJDIR)/$(STARTUP).o: $(STARTUP).s
	$(CC) $(CFLAGS) -o $@ -c $<

# Main firmware binary
$(OBJDIR)/$(PROJ_NAME).bin: $(OBJDIR)/$(STARTUP).o $(OBJDIR)/main.$(PROJ_NAME).o
	$(CC) -Wl,--section-start,.isr_vector=0x8004000 $(CFLAGS) -o $(OBJDIR)/$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary $(OBJDIR)/$(PROJ_NAME).elf $(OBJDIR)/code.bin
	SETLEN=1 crypto/sign.py $(OBJDIR)/code.bin $@ $(CERT)

# Bootstub binary
$(OBJDIR)/bootstub.$(PROJ_NAME).bin: $(OBJDIR)/$(STARTUP).o $(OBJDIR)/bootstub.$(PROJ_NAME).o $(OBJDIR)/sha.$(PROJ_NAME).o $(OBJDIR)/rsa.$(PROJ_NAME).o
	$(CC) $(CFLAGS) -o $(OBJDIR)/bootstub.$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary $(OBJDIR)/bootstub.$(PROJ_NAME).elf $@

# Dependency tracking
$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d
include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(wildcard *.c))))

# Clean build artifacts
clean:
	@rm -rf $(DEPDIR) $(OBJDIR)

.PHONY: all recover clean
