#!/bin/bash
cd ~/Documents/Projects/FC-soft-toolkit/nuttxspace/nuttx

#clean the build
make distclean

# Configure with your board
./tools/configure.sh -l ../boards/arm/stm32h7/shirley-fc-dev-board/configs/nsh

# Build with your external app directory
# make EXTRA_APPS_DIR=$(pwd)/../fc-stack

# Build NuttX
make -j