# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FC Soft Toolkit is flight controller firmware for STM32H743ZIT6-based (ARM Cortex-M7) flight controllers. The project supports both baremetal development via STM32CubeIDE and Apache NuttX RTOS, and targets two board ports:

- **Shirley FC** — custom flight controller, single IMU (ICM-40609-D). Board port: `shirley-fc-dev-board`.
- **DAKEFPV_H743** — dual-IMU (2x ICM-42688-P) flight controller. Board port: `dakefpv-h743`. Board reference files (Betaflight config, ChibiOS hw def) live in `resources/FC-boards/DAKEFPV_H743/`.

## Build Commands

### STM32CubeIDE Projects (Baremetal)

Built through STM32CubeIDE. Current projects:
- `STM32CubeIDE/LedBlinkTest/` — RGB LED PWM demo (STM32H743ZIT6)
- `STM32CubeIDE/NucleoH7TestCode/` — Nucleo board test code (STM32H753ZIT6)
- `STM32CubeIDE/SensorsReadTest/` — IMU (ICM-40609-D) and magnetometer (MMC5983MA) read test over SPI

### NuttX RTOS

Run all commands from `nuttxspace/nuttx/`.

```bash
# Configure board (pick one)
./tools/configure.sh shirley-fc-dev-board:nsh
./tools/configure.sh dakefpv-h743:nsh      # also has :usbnsh and :st7735 configs

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
│   ├── boards/             ← custom board ports (out-of-tree)
│   │   └── arm/stm32h7/
│   │       ├── shirley-fc-dev-board/  ← Shirley FC (single ICM-40609-D)
│   │       └── dakefpv-h743/          ← DAKEFPV_H743 (dual ICM-42688-P)
│   ├── drivers/            ← custom out-of-tree drivers
│   │   └── sensors/icm40609d.c, icm40609d-fifo.c, icm42688p-fifo.c (+ headers)
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

**Shirley FC** (board port: `shirley-fc-dev-board`)

- **MCU**: STM32H743ZIT6 @ 480 MHz max
- **IMU**: ICM-40609-D (SPI5)
- **Magnetometer**: MMC5983MA (SPI4)
- **Barometer**: BMP388 (I2C2)
- **Interfaces**: 4× UART, 2× USART, 2× SPI, 3× I2C, FDCAN1, USB-C, SD Card, 3× ADC
- **Debug**: SWD via J-Link or ST-LINK (PA13/PA14/PB3)
- **Power**: Dual 3.3 V rails (digital + analog for sensor isolation)

**DAKEFPV_H743** (board port: `dakefpv-h743`)

- **MCU**: STM32H743ZIT6
- **IMU**: 2x ICM-42688-P
  - IMU1: SPI1, SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4, INT1(EXTI)=PC4 → `/dev/imu0`
  - IMU2: SPI4, SCK=PE12, MISO=PE13, MOSI=PE14, CS=PB1, INT1(EXTI)=PB2 → `/dev/imu1`
- **User LED**: PD10
- No SD card slot on this board (unlike Shirley FC) — the board port previously carried leftover SDIO/card-detect code inherited from a WeAct-H743VIT reference port; it has been removed.
- Pin definitions live in `nuttxspace/boards/arm/stm32h7/dakefpv-h743/src/dakefpv-h743.h` (there is no `config/pinout.yaml` entry for this board yet); board reference material (Betaflight config, ChibiOS hw def) is under `resources/FC-boards/DAKEFPV_H743/`.

## Key Peripheral Pins (Shirley FC)

Source of truth: `config/pinout.yaml`. This table applies to **Shirley FC** only — see the Hardware section above for DAKEFPV_H743 pins.

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

### Board Ports (`nuttxspace/boards/arm/stm32h7/`)

`shirley-fc-dev-board/`:

- `include/board.h` — clock, pin, and peripheral definitions
- `src/stm32_boot.c` — early boot
- `src/stm32_bringup.c` — peripheral and driver registration
- `src/stm32_appinit.c` — application init (NSH)
- `src/stm32_spi.c` — SPI bus/CS routing
- `src/fc-dev.h` — shared board-level pin/device definitions
- `configs/nsh/defconfig` — board defconfig

`dakefpv-h743/`:

- `include/board.h` — clock, pin, and peripheral definitions
- `src/stm32_boot.c` — early boot
- `src/stm32_bringup.c` — peripheral and driver registration, registers both ICM-42688-P IMUs via `fc_imu_register()` (`/dev/imu0`, `/dev/imu1`)
- `src/stm32_spi.c` — SPI bus/CS routing, EXTI IRQ ack callbacks (`stm32_imu1_irq_ack`/`stm32_imu2_irq_ack`)
- `src/dakefpv-h743.h` — board-level pin/device definitions (GPIO_IMU1/2_CS, GPIO_IMU1/2_INT, etc.)
- `configs/nsh/`, `configs/usbnsh/`, `configs/st7735/` — board defconfigs

### Custom Drivers (`nuttxspace/drivers/`)

- `sensors/icm40609d.c` — ICM-40609-D NuttX character driver
- `sensors/icm40609d-fifo.c` — ICM-40609-D FIFO-streaming driver variant
- `sensors/icm42688p-fifo.c` — ICM-42688-P FIFO-streaming driver (used by DAKEFPV_H743, one instance per IMU); registration takes an `irq_ack` callback (arch/board-specific EXTI pending-bit clear) since the driver attaches directly to the raw IRQ vector
- `include/nuttx/sensors/icm40609d.h`, `icm40609d-fifo.h`, `icm42688p-fifo.h` — driver public headers
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
