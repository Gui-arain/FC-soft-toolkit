# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FC Soft Toolkit is flight controller firmware for the **Shirley FC**, a custom flight controller based on STM32H743ZIT6 (ARM Cortex-M7). The project supports both baremetal development via STM32CubeIDE and Apache NuttX RTOS.

## Build Commands

### STM32CubeIDE Projects (Baremetal)

Built through STM32CubeIDE. Current projects:
- `STM32CubeIDE/LedBlinkTest/` — RGB LED PWM demo (STM32H743ZIT6)
- `STM32CubeIDE/NucleoH7TestCode/` — Nucleo board test code (STM32H753ZIT6)
- `STM32CubeIDE/SensorsReadTest/` — IMU (ICM-40609-D) and magnetometer (MMC5983MA) read test over SPI

### NuttX RTOS

Run all commands from `nuttxspace/nuttx/`.

```bash
# Configure board
./tools/configure.sh shirley-fc-dev-board:nsh

# Interactive configuration
make menuconfig

# Build (with out-of-tree fc-stack apps)
make -j EXTRA_APPS_DIR=$(pwd)/../fc-stack

# Or use the build script from nuttxspace/
bash scripts/build.sh

# Clean
make distclean
```

### Flashing (OpenOCD)

```bash
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

```
FC-soft-toolkit/
├── nuttxspace/
│   ├── nuttx/              ← submodule (apache/nuttx) — DO NOT MODIFY
│   ├── apps/               ← submodule (apache/nuttx-apps) — DO NOT MODIFY
│   ├── boards/             ← custom board port (out-of-tree)
│   │   └── arm/stm32h7/shirley-fc-dev-board/
│   ├── drivers/            ← custom out-of-tree drivers
│   │   └── sensors/icm40609d.c/.h
│   ├── fc-stack/           ← flight controller application modules
│   │   ├── fc_core/        ← main FC loop (C++)
│   │   └── estimator/      ← attitude estimator (C++)
│   └── scripts/
│       └── build.sh        ← configure + build helper
├── STM32CubeIDE/           ← baremetal projects
├── Doc/                    ← guides, cheat sheets, datasheets
├── config/
│   ├── pinout.yaml         ← authoritative pin mapping for the board
│   └── power.yaml
└── resources/datasheets/   ← ICM-40609-D, MMC5983MA, BMP388
```

### Submodule Policy

`nuttxspace/nuttx/` and `nuttxspace/apps/` are upstream Apache NuttX submodules. **Do not modify them directly** — changes will be lost on update. All custom code lives in the out-of-tree directories (`boards/`, `drivers/`, `fc-stack/`).

### Dual Development Paths

1. **Baremetal (STM32CubeIDE)**: Hardware validation and simple demos. Edit `.ioc` files in STM32CubeMX for peripheral config, application code in `Core/Src/main.c`. Sensor drivers live in `Core/Inc/` and `Core/Src/`.

2. **NuttX RTOS**: Complex applications. Custom board port in `nuttxspace/boards/`, out-of-tree drivers in `nuttxspace/drivers/`, flight stack apps in `nuttxspace/fc-stack/`. Configure via menuconfig, build with `EXTRA_APPS_DIR`.

### Hardware

- **MCU**: STM32H743ZIT6 @ 480 MHz max
- **IMU**: ICM-40609-D (SPI5)
- **Magnetometer**: MMC5983MA (SPI4)
- **Barometer**: BMP388 (I2C2)
- **Interfaces**: 4× UART, 2× USART, 2× SPI, 3× I2C, FDCAN1, USB-C, SD Card, 3× ADC
- **Debug**: SWD via J-Link or ST-LINK (PA13/PA14/PB3)
- **Power**: Dual 3.3 V rails (digital + analog for sensor isolation)

## Key Peripheral Pins

Source of truth: `config/pinout.yaml`

| Peripheral | Function | Pins |
|---|---|---|
| SPI5 | IMU (ICM-40609-D) | SCK=PF7, MISO=PF8, MOSI=PF9, CS=PF10 |
| SPI4 | Magnetometer (MMC5983MA) | SCK=PE2, MISO=PE5, MOSI=PE6, CS=PE4 |
| I2C2 | Barometer (BMP390) | SDA=PF0, SCL=PF1 |
| TIM1 | Motor ESCs (PWM/DShot) | CH1=PE9, CH2=PE11, CH3=PE13, CH4=PE14 |
| TIM4 | RGB LED (PWM) | R=PD12, G=PD13, B=PD14 |
| UART4 | Telemetry | TX=PA0, RX=PA1 |
| UART8 | GPS | TX=PE1, RX=PE0 |
| USART6 | RC Receiver | TX=PC6, RX=PC7 |
| SDMMC1 | SD Card | D0-D3=PC8-PC11, CK=PC12, CMD=PD2 |
| USB OTG FS | USB-C | DM=PA11, DP=PA12 |
| ADC1 | Battery voltage/current | V=PF11, I=PA6 |
| — | System LED | PD6 |
| — | SWD debug | SWDIO=PA13, SWCLK=PA14, SWO=PB3 |

## NuttX Out-of-Tree Structure

### Board Port (`nuttxspace/boards/arm/stm32h7/shirley-fc-dev-board/`)

- `include/board.h` — clock, pin, and peripheral definitions
- `src/stm32_boot.c` — early boot
- `src/stm32_bringup.c` — peripheral and driver registration
- `src/stm32_appinit.c` — application init (NSH)
- `src/stm32_spi.c` — SPI bus/CS routing
- `src/fc-dev.h` — shared board-level pin/device definitions
- `configs/nsh/defconfig` — board defconfig

### Custom Drivers (`nuttxspace/drivers/`)

- `sensors/icm40609d.c` — ICM-40609-D NuttX character driver
- `include/nuttx/sensors/icm40609d.h` — driver public header
- `Kconfig` / `sensors/Kconfig` — driver Kconfig entries

### FC Stack (`nuttxspace/fc-stack/`)

Out-of-tree NuttX apps, built via `EXTRA_APPS_DIR`. Each module has its own `Kconfig`, `Make.defs`, and `Makefile`.

- `fc_core/` — main flight controller loop (`fc_core_main.cpp`)
- `estimator/` — attitude estimator (`estimator_main.cpp`)

## Toolchain Requirements

See `Doc/Setup MacOS.md` or `Doc/Setup Linux.md` for full installation:
- ARM GNU Embedded Toolchain (`arm-none-eabi-gcc`/`gdb`)
- OpenOCD
- kconfig-mconf (for NuttX menuconfig)
- picocom (serial terminal)
