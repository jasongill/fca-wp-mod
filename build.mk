CFLAGS += -I inc -nostdlib -fno-builtin -std=gnu11 -Os

CFLAGS += -Tstm32_flash.ld

DFU_UTIL = "dfu-util"

BUILDER = DEV

CC = $(COMPILER_PATH)arm-none-eabi-gcc
OBJCOPY = $(COMPILER_PATH)arm-none-eabi-objcopy
OBJDUMP = $(COMPILER_PATH)arm-none-eabi-objdump

CERT = ./certs/debug
CFLAGS += "-DALLOW_DEBUG"

$(shell mkdir -p obj >/dev/null)

DEPDIR = generated_dependencies
$(shell mkdir -p -m 777 $(DEPDIR) >/dev/null)
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td
POSTCOMPILE = @mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d && touch $@

recover: obj/bootstub.$(PROJ_NAME).bin obj/$(PROJ_NAME).bin
	echo "Press and hold the button on your WP Mod while connecting to USB"
	sleep 1.0
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08004000 -D obj/$(PROJ_NAME).bin
	$(DFU_UTIL) -d 0483:df11 -a 0 -s 0x08000000:leave -D obj/bootstub.$(PROJ_NAME).bin

obj/%.$(PROJ_NAME).o: %.c $(DEPDIR)/%.d
	$(CC) $(DEPFLAGS) $(CFLAGS) -o $@ -c $<
	$(POSTCOMPILE)

obj/%.$(PROJ_NAME).o: ../crypto/%.c
	$(CC) $(CFLAGS) -o $@ -c $<

obj/$(STARTUP_FILE).o: $(STARTUP_FILE).s
	$(CC) $(CFLAGS) -o $@ -c $<

obj/$(PROJ_NAME).bin: obj/$(STARTUP_FILE).o obj/main.$(PROJ_NAME).o
  # hack
	$(CC) -Wl,--section-start,.isr_vector=0x8004000 $(CFLAGS) -o obj/$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary obj/$(PROJ_NAME).elf obj/code.bin
	SETLEN=1 crypto/sign.py obj/code.bin $@ $(CERT)

obj/bootstub.$(PROJ_NAME).bin: obj/$(STARTUP_FILE).o obj/bootstub.$(PROJ_NAME).o obj/sha.$(PROJ_NAME).o obj/rsa.$(PROJ_NAME).o
	$(CC) $(CFLAGS) -o obj/bootstub.$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary obj/bootstub.$(PROJ_NAME).elf $@

$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d

include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(wildcard *.c))))

clean:
	@$(RM) obj/*
	@rm -rf $(DEPDIR)
