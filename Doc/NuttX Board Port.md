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

## Step 1 — Copy an Existing Board

Find a board in `boards/arm/stm32h7/` that uses the same MCU (e.g. `nucleo-h743zi` for STM32H743ZI):

```bash
cd nuttxspace/nuttx/boards/arm/stm32h7/
cp -r nucleo-h743zi my-board-name
```

Use lowercase with hyphens for the folder name — it becomes the board identifier used in `./tools/configure.sh`.

---

## Step 2 — Register the Board in the Global Kconfig

This is the most critical step. Without it, `./tools/configure.sh` cannot find your board.

Edit `nuttxspace/nuttx/boards/Kconfig` in **three** places:

### 2a — Add a config entry (near other boards for the same chip family)

```kconfig
config ARCH_BOARD_MY_BOARD_NAME
    bool "My Board Name"
    depends on ARCH_CHIP_STM32H743ZI
    select ARCH_HAVE_LEDS
    ---help---
        My custom board based on STM32H743ZIT6.
```

Omit `select ARCH_HAVE_BUTTONS` if your board has no user button.

### 2b — Add the string default (in the `config ARCH_BOARD string` block)

```kconfig
default "my-board-name"    if ARCH_BOARD_MY_BOARD_NAME
```

### 2c — Source the board Kconfig (near other boards of the same chip family)

```kconfig
if ARCH_BOARD_MY_BOARD_NAME
source "boards/arm/stm32h7/my-board-name/Kconfig"
endif
```

---

## Step 3 — Update the Board-Level Kconfig

Edit `my-board-name/Kconfig` and replace the copied board's symbol throughout:

```kconfig
# Before
if ARCH_BOARD_NUCLEO_H743ZI
...
endif # ARCH_BOARD_NUCLEO_H743ZI

# After
if ARCH_BOARD_MY_BOARD_NAME
...
endif # ARCH_BOARD_MY_BOARD_NAME
```

---

## Step 4 — Update All defconfig Files

Each `configs/*/defconfig` has two lines that need updating:

```
CONFIG_ARCH_BOARD="nucleo-h743zi"       →  CONFIG_ARCH_BOARD="my-board-name"
CONFIG_ARCH_BOARD_NUCLEO_H743ZI=y       →  CONFIG_ARCH_BOARD_MY_BOARD_NAME=y
```

Quick way with sed:

```bash
find configs/ -name "defconfig" | while read f; do
  sed -i 's/CONFIG_ARCH_BOARD="nucleo-h743zi"/CONFIG_ARCH_BOARD="my-board-name"/' "$f"
  sed -i 's/CONFIG_ARCH_BOARD_NUCLEO_H743ZI=y/CONFIG_ARCH_BOARD_MY_BOARD_NAME=y/' "$f"
done
```

---

## Step 5 — Create the Board Header

Rename `src/nucleo-h743zi.h` to `src/my-board-name.h` and update:

- **Header guard** — replace `NUCLEO_H743ZI` with your board's symbol
- **LED GPIO definitions** — match your board's LED pins
- **SPI CS pins** — one `#define` per sensor chip select
- **PWM timer** — rename the `*_PWMTIMER` macro
- **Remove** peripherals not present on your board (shields, wireless modules, etc.)
- **Declare** only the `stm32_*` functions that your `stm32_bringup.c` calls

### Example: LED pins

```c
/* LEDs on PD12 (Red), PD13 (Green), PD14 (Blue) */

#define GPIO_LED_RED   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN12)
#define GPIO_LED_GREEN (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN13)
#define GPIO_LED_BLUE  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN14)
```

### Example: SPI CS pins

```c
/* ICM-40609-D IMU on SPI5, CS = PF10 */

#define GPIO_ICM40609_CS  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN10)
```

---

## Step 6 — Update `include/board.h`

This file defines clocking, LED indices, alternate function pin mappings, and DMA channels.

### Header guards

Replace `NUCLEO_H743ZI` with your board symbol in the `#ifndef`/`#define`/`#endif` guards.

### HSE clock

If your board has a **real crystal oscillator** (not the ST-LINK MCO bypass), remove:

```c
#define STM32_HSEBYP_ENABLE   // delete this line
```

If your board uses the ST-LINK MCO signal as clock (like the Nucleo), keep it.

Update `STM32_BOARD_XTAL` to your crystal frequency:

```c
#define STM32_BOARD_XTAL   8000000ul   /* 8 MHz HSE crystal */
```

### CPU clock

The STM32H743 can run at **480 MHz** (the Nucleo config caps it at 400 MHz). To unlock 480 MHz, adjust the PLL1 dividers — for example with an 8 MHz HSE:

```
PLL1_VCO = (8 MHz / 2) * 240 = 960 MHz
PLL1P    = 960 / 2 = 480 MHz  → SYSCLK
```

### LED index order

`BOARD_LED1/2/3` must match the order in your board header's `g_ledmap[]` array:

```c
#define BOARD_LED_RED     BOARD_LED1   // index 0 → GPIO_LED_RED
#define BOARD_LED_GREEN   BOARD_LED2   // index 1 → GPIO_LED_GREEN
#define BOARD_LED_BLUE    BOARD_LED3   // index 2 → GPIO_LED_BLUE
```

### SPI alternate function pins

Add entries for each SPI bus you use. Look up the correct `_N` suffix in `arch/arm/src/stm32h7/hardware/stm32h7x3xx_pinmap.h`:

```c
/* SPI5 — ICM-40609-D IMU */
#define GPIO_SPI5_MISO    (GPIO_SPI5_MISO_1 | GPIO_SPEED_50MHz) /* PF8 */
#define GPIO_SPI5_MOSI    (GPIO_SPI5_MOSI_1 | GPIO_SPEED_50MHz) /* PF9 */
#define GPIO_SPI5_SCK     (GPIO_SPI5_SCK_1  | GPIO_SPEED_50MHz) /* PF7 */
```

### Buttons

If your board has no user button:

```c
#define NUM_BUTTONS   0
```

Remove `BUTTON_USER` and its `_BIT` macro entirely.

---

## Step 7 — Update Source Files

### Replace all includes

Every `.c` file that pulled in `nucleo-h743zi.h` needs to use your new header:

```bash
find src/ -name "*.c" | xargs sed -i 's/"nucleo-h743zi.h"/"my-board-name.h"/g'
```

### `stm32_spi.c` — CS initialization and select callbacks

`stm32_spidev_initialize()` should configure your CS GPIOs:

```c
void stm32_spidev_initialize(void)
{
#ifdef CONFIG_STM32H7_SPI5
  stm32_configgpio(GPIO_ICM40609_CS);
  stm32_gpiowrite(GPIO_ICM40609_CS, true);   /* deassert */
#endif
}
```

The `stm32_spiNselect()` callbacks drive the CS line during transactions:

```c
#ifdef CONFIG_STM32H7_SPI5
void stm32_spi5select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  if (devid == SPIDEV_IMU(0))
    {
      stm32_gpiowrite(GPIO_ICM40609_CS, !selected);
    }
}
#endif
```

Use the appropriate `SPIDEV_*` type constant (`SPIDEV_IMU`, `SPIDEV_SENSOR`, `SPIDEV_MMCSD`, etc.) — these are defined in `include/nuttx/spi/spi.h`.

### `stm32_bringup.c` — Remove irrelevant sensor init calls

Delete `#ifdef` blocks for sensors not on your board (LSM6DSL, LSM9DS1, NRF24L01, PCA9635, etc.).

### `stm32_pwm.c` — Rename timer macro

```c
pwm = stm32_pwminitialize(MY_BOARD_PWMTIMER);   /* was NUCLEOH743ZI_PWMTIMER */
```

### `stm32_buttons.c` — Stub out if no button

```c
uint32_t board_button_initialize(void) { return NUM_BUTTONS; }
uint32_t board_buttons(void)           { return 0; }
```

---

## Step 8 — Verify No Stale References

```bash
grep -r "nucleo\|NUCLEO" boards/arm/stm32h7/my-board-name/ \
  --include="*.c" --include="*.h" --include="Kconfig" \
  --include="defconfig" --include="CMakeLists.txt" --include="Makefile"
```

The only hit should be the old copied header file (`src/nucleo-h743zi.h`) if you chose to keep it — delete it once you're confident nothing includes it.

---

## Step 9 — Configure and Build

```bash
cd nuttxspace/nuttx

# Configure (first time)
./tools/configure.sh -l my-board-name:nsh

# Verify the board was found — you should see no error above
make menuconfig    # optional: check CONFIG_ARCH_BOARD is correct

# Build
make -j
```

If `configure.sh` says the board is unknown, re-check Step 2 (global Kconfig registration).

---

## File Checklist

| File | What to change |
|---|---|
| `boards/Kconfig` (global) | Add config entry, default string, source directive |
| `Kconfig` | Replace board symbol in `if`/`endif` guards |
| `configs/*/defconfig` | `CONFIG_ARCH_BOARD` string and symbol |
| `include/board.h` | Header guards, clock config, LED indices, SPI pins, button count |
| `src/my-board-name.h` | New board header with GPIO macros and function declarations |
| `src/stm32_spi.c` | CS init in `stm32_spidev_initialize()`, CS toggling in `stm32_spiNselect()` |
| `src/stm32_pwm.c` | Timer macro name |
| `src/stm32_usb.c` | USB host priority/stack size macros |
| `src/stm32_bringup.c` | Remove sensor inits not present on your board |
| `src/stm32_autoleds.c` | LED array order matches new `BOARD_LED_*` indices |
| `src/stm32_userleds.c` | Same as above |
| `src/stm32_buttons.c` | Stub if no user button |
| All other `src/*.c` | Update `#include` to new header name |

---

## Common Mistakes

**Board not found by configure.sh** — You missed one of the three insertions in the global `boards/Kconfig` (config entry, default string, or source directive).

**Build error: `GPIO_BTN_USER` undeclared** — You removed `BUTTON_USER` from `board.h` but `stm32_buttons.c` still references it. Stub out the button functions.

**SPI transactions never assert CS** — The `stm32_spiNselect()` callback has no case for your `SPIDEV_*` type. Add the correct `case SPIDEV_IMU(0):` block.

**Wrong CPU speed** — If you kept `STM32_HSEBYP_ENABLE` with a real crystal, the clock will be misconfigured. Remove it for crystal oscillators.

**`#include "nucleo-h743zi.h"` compile error** — A `.c` file still includes the old header. Run the grep from Step 8 to find it.
