# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FC Soft Toolkit is flight controller firmware for the Shirley FC, a custom flight controller based on STM32H743ZIT6 (ARM Cortex-M7). The project supports both baremetal development via STM32CubeIDE and Apache NuttX RTOS.

## Build Commands

### STM32CubeIDE Projects (Baremetal)

Projects are built through STM32CubeIDE. Current projects:
- `STM32CubeIDE/LedBlinkTest/` - RGB LED PWM demo (STM32H743ZIT6)
- `STM32CubeIDE/NucleoH7TestCode/` - Nucleo board test code (STM32H753ZIT6)
- `STM32CubeIDE/SensorsReadTest/` - IMU (ICM-40609-D) and magnetometer (MMC5983MA) read test over SPI

### NuttX RTOS

```bash
# Configure board (run from nuttxspace/nuttx/)
./tools/configure.sh -l nucleo-h743zi:nsh

# Interactive configuration
make menuconfig

# Build
make -j

# Clean
make distclean
```

### Flashing (OpenOCD)

```bash
# Flash binary to STM32H743
openocd -f board/st_nucleo_h743zi.cfg \
  -c "init; reset halt" \
  -c "program nuttx.bin 0x08000000 verify reset exit"
```

### Serial Terminal

```bash
picocom -b 115200 /dev/cu.usbmodem*
```

## Architecture

### Directory Structure

- `nuttxspace/` - Apache NuttX RTOS workspace (git submodules for nuttx kernel and apps)
- `STM32CubeIDE/` - Baremetal STM32CubeIDE projects
- `Doc/` - Hardware bringup guides, NuttX setup, toolchain installation, datasheets, cheat sheets
- `config/` - Hardware configuration (pinout.yaml, power.yaml)
- `resources/` - Datasheets and reference documents

### Hardware

- **MCU**: STM32H743ZIT6 @ 480MHz max
- **Sensors**: ICM-40609-D (IMU/SPI5), MMC5983MA (Magnetometer/SPI4), BMP390 (Barometer/I2C2)
- **Interfaces**: 4x UART, 2x USART, 2x SPI, 3x I2C, FDCAN1, USB-C, SD Card
- **Debug**: SWD via J-Link or ST-LINK (PA13/PA14)
- **Power**: Dual 3.3V rails (digital + analog for sensor isolation)

### Dual Development Paths

1. **Baremetal (STM32CubeIDE)**: For hardware validation and simple demos. Edit `.ioc` files in STM32CubeMX for peripheral configuration, code in `/Core/Src/main.c`. Sensor drivers live in `/Core/Inc/` and `/Core/Src/` alongside generated HAL code.

2. **NuttX RTOS**: For complex applications. Configure via menuconfig, add custom apps to `nuttxspace/apps/`.

## Key Peripheral Pins

- **RGB LED**: PD12, PD13, PD14 (TIM4 PWM)
- **System LED**: PD6
- **Motor ESCs**: TIM1 PWM outputs
- **IMU ICM-40609-D (SPI5)**: PF6-PF9, CS=PF10 (driver: `icm-40609-d.h/.c`)
- **Magnetometer MMC5983MA (SPI4)**: PE2, PE5, PE6, PE11-PE14 (driver: `mmc5983ma.h/.c`)
- **Barometer BMP390 (I2C2)**: PF0, PF1

## Toolchain Requirements

See `Doc/Setup MacOS.md` or `Doc/Setup Linux.md` for full installation:
- ARM GNU Embedded Toolchain (arm-none-eabi-gcc/gdb)
- OpenOCD
- kconfig-mconf (for NuttX menuconfig)
- picocom (serial terminal)
