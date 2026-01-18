#!/bin/bash
# Need formula for gcc
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
brew tap ArmMbed/homebrew-formulae
brew install dfu-util arm-none-eabi-gcc