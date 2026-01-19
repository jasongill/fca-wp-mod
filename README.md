# fca-wp-mod

Custom firmware for the Comma "White Panda" (STM32F413-based) that enables low-speed steering on FCA (Chrysler/Dodge/Jeep/Ram) vehicles using Comma.ai "self-driving" units.

Since the White Panda is no longer commercially available, this mod targets the custom-made version created by VanillaGorillaa, which is plug-and-play with the Chrysler Pacifica (and likely many other FCA vehicles). You can [purchase the VanillaGorilla WP Mod here](https://shop.retropilot.org/product/chrysler-wp-mod-board/), and [find installation instructions for the module here](https://docs.google.com/document/d/1hfttNMyqunWpZZNb7Pa9Lmi9tGDNRmtYrjkyO2-mUjs/edit?usp=sharing).

## Installation

### Option 1: Pre-built Binary (Recommended)

1. Download the latest `fca-wp-mod-vX.X.X.zip` from [Releases](../../releases)
2. Extract `fca-wp-mod.bin` from the zip
3. Install `dfu-util`:
   - **Linux:** `sudo apt-get install dfu-util`
   - **macOS:** `brew install dfu-util`
   - **Windows:** Download from [dfu-util releases](http://dfu-util.sourceforge.net/releases/)
4. Hold the button on the White Panda while connecting to USB
5. Flash the firmware:
   ```bash
   dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D fca-wp-mod.bin
   ```

### Option 2: Build from Source

#### Prerequisites

**Linux (Debian/Ubuntu)**
```bash
sudo apt-get install gcc-arm-none-eabi dfu-util
```

**macOS**
```bash
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc dfu-util
```

#### Building

```bash
make
```

#### Flashing

Hold the button on the White Panda while connecting to USB, then run:

```bash
make flash
```
