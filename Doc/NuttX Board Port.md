# NuttX Board Port Guide

How to create a new NuttX board port from scratch using an existing board as a template. This guide is based on porting the `nucleo-h743zi` to the `shirley-fc-dev-board` (STM32H743ZIT6).

---

## Overview

A NuttX board port lives under `nuttxspace/nuttx/boards/<arch>/<chip-family>/<board-name>/` and consists of:

- Hardware pin/clock definitions (`include/board.h`)
- Board-specific source files (`src/`)
- Build configurations (`configs/`)
- Kconfig options (`Kconfig`)
- Linker scripts (`scripts/`)

The fastest way to create a new port is to copy an existing board that uses the same MCU and adapt it.

---

## `boards/` Directory Structure

The `boards/` directory contains board specific configuration logic. Each board must provide a subdirectory `<board>` under `boards/` with the following characteristics:

```
<board>
|-- include/
|   `-- (board-specific header files)
|-- src/
|   |-- Makefile
|   `-- (board-specific source files)
|-- <config1-dir>
|   |-- Make.defs
|   `-- defconfig
|-- <config2-dir>
|   |-- Make.defs
|   `-- defconfig
...
```

## Booting sequence of the board

```
Power on / Reset
│
├─► stm32_board_initialize()       ← your stm32_boot.c
│     • stm32_clockconfig()          switch HSI → HSE + PLL → 80 MHz
│     • stm32_fpuconfig()            enable Cortex-M7 FPU
│     • arm_earlyserialinit()        UART console ready
│
├─► board_early_initialize()       ← your stm32_boot.c
│     • (minimal, pre-scheduler)
│
├─► NuttX kernel boots
│     • memory regions initialized
│     • scheduler started
│     • idle task created
│
├─► board_app_initialize()         ← your stm32_appinit.c
│     │
│     ├─► stm32_bringup()          ← your stm32_bringup.c
│     │     • up_cxxinitialize()     C++ static constructors
│     │     • SPI4, SPI5 buses up
│     │     • I2C2 bus up
│     │     • SD card mounted
│     │
│     ├─► fc_imu_register()         /dev/imu0
│     ├─► fc_mag_register()         /dev/mag0
│     └─► fc_baro_register()        /dev/baro0
│
└─► task_create() × 4             ← TODO in stm32_appinit.c
      │
      ├─► fc_core_main()    ┐
      ├─► estimator_main()  │  scheduled by priority
      ├─► mixer_main()      │  preemptive round-robin
      └─► telemetry_main()  ┘
```

One thing worth adding to your mental model: **`board_app_initialize()` itself runs in a task context** (the NuttX init task), so by the time it's called the scheduler is already running. That's why it's safe to call `stm32_i2cbus_initialize()` and mount filesystems there — those operations can block, which is only legal once the scheduler is up. That's also why `up_cxxinitialize()` must not be called before that point.

## References

https://nuttx.apache.org/docs/latest/components/boards.html?utm_source=chatgpt.com