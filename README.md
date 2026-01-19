# fca-wp-mod

Custom firmware for the Comma "White Panda" (STM32F413-based) that improves the functionality of Comma for FCA (Chrysler/Dodge/Jeep/Ram) vehicles.

## Prerequisites

### Linux (Debian/Ubuntu)
```bash
sudo apt-get install gcc-arm-none-eabi dfu-util python3
```

### macOS
```bash
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc dfu-util python3
```

## Building

```bash
make
```

## Flashing

Hold the button on the White Panda while connecting to USB, then run:

```bash
make flash
```

Or, install manually with:
```bash
dfu-util -d 0483:df11 -a 0 -s 0x08004000 -D obj/panda.bin
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D obj/bootstub.panda.bin
```