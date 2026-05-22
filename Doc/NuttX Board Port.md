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

## References

https://nuttx.apache.org/docs/latest/components/boards.html?utm_source=chatgpt.com