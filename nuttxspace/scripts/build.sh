#!/bin/bash
set -e
cd ~/Documents/Projects/FC-soft-toolkit/nuttxspace

# Clean and configure
make distclean
nuttx/tools/configure.sh -l ../boards/arm/stm32h7/shirley-fc-dev-board/configs/nsh

# Interactive config — fc-stack apps visible under Application Configuration
make menuconfig

# Build
make -j
