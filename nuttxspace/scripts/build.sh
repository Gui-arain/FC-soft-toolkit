#!/bin/bash
cd nuttx

# Configure with your board
./tools/configure.sh fc-dev:nsh

# Build with your external app directory
make EXTRA_APPS_DIR=$(pwd)/../fc-stack