# NuttX Build Configuration

How the out-of-tree board port, drivers, and fc-stack apps are wired into the NuttX build system.

---

## Directory layout

```
nuttxspace/
├── nuttx/              ← upstream submodule (DO NOT MODIFY)
├── apps/               ← upstream submodule (DO NOT MODIFY)
│   └── fc-stack → ../fc-stack   ← symlink (see below)
├── boards/             ← out-of-tree board port
│   └── arm/stm32h7/shirley-fc-dev-board/
│       ├── include/board.h       clock, pin, LED definitions
│       ├── src/
│       │   ├── Makefile          lists all board C sources
│       │   ├── stm32_boot.c      early clock + FPU init
│       │   ├── stm32_bringup.c   peripheral init (SPI, I2C, SD)
│       │   ├── stm32_appinit.c   sensor driver registration
│       │   ├── stm32_spi.c       SPI CS routing
│       │   ├── stm32_autoleds.c  board_autoled_on/off (PD6)
│       │   └── fc-dev.h          shared pin / device macros
│       └── configs/nsh/defconfig board Kconfig defaults
├── drivers/            ← out-of-tree sensor drivers
│   ├── Kconfig
│   ├── sensors/
│   │   ├── Kconfig
│   │   └── icm40609d.c           ICM-40609-D NuttX char driver
│   └── include/nuttx/sensors/
│       └── icm40609d.h
└── fc-stack/           ← flight controller apps
    ├── Kconfig                   FC_CORE / FC_ESTIMATOR options
    ├── Make.defs                 CONFIGURED_APPS entries
    ├── estimator/
    │   ├── Makefile
    │   ├── estimator_main.cpp
    │   └── Kconfig
    └── fc_core/
        ├── Makefile
        ├── fc_core_main.cpp
        └── Kconfig
```

---

## How each piece plugs in

### Board port (`boards/`)

The board port is out-of-tree. NuttX locates it through two configure-time symlinks that `tools/configure.sh` creates automatically:

| Symlink | Points to |
|---|---|
| `nuttx/Make.defs` | `boards/arm/stm32h7/shirley-fc-dev-board/scripts/Make.defs` |
| `nuttx/include/arch` | arch headers resolved by chip selection |

Configure command (run from `nuttxspace/`):
```bash
nuttx/tools/configure.sh -l ../boards/arm/stm32h7/shirley-fc-dev-board/configs/nsh
```

The `-l` flag creates symlinks rather than copying files, which keeps the source tree clean for out-of-tree boards.

### Out-of-tree drivers (`drivers/`)

The ICM-40609-D driver is compiled into the board port library, not the NuttX kernel. The board's `src/Makefile` handles this:

```makefile
ifeq ($(CONFIG_SENSORS_ICM40609D),y)
CSRCS += icm40609d.c
VPATH += :$(TOPDIR)/../drivers/sensors
DEPPATH += --dep-path $(TOPDIR)/../drivers/sensors
endif
```

The driver header is in `drivers/include/nuttx/sensors/icm40609d.h`. The board Makefile's `VPATH`/`DEPPATH` only cover the driver source itself — app consumers must explicitly add `drivers/include/` to their compiler search path. Each fc-stack app that uses the driver does this in its own Makefile:

```makefile
CXXFLAGS += -I$(TOPDIR)/../drivers/include
```

With that line in place, consumers can use the canonical include form:

```c
#include <nuttx/sensors/icm40609d.h>
```

### fc-stack apps (`fc-stack/`)

The apps are made visible to the NuttX apps build system via a **permanent symlink** in the apps submodule:

```
apps/fc-stack  →  ../fc-stack
```

Create it once (if not already present):
```bash
cd nuttxspace/apps
ln -s ../fc-stack fc-stack
```

With the symlink in place, the apps `Makefile` sources `apps/fc-stack/Make.defs` as part of its normal app discovery. That file adds entries to `CONFIGURED_APPS` using `$(APPDIR)/fc-stack/<app>`, where `APPDIR` = `apps/`:

```makefile
# fc-stack/Make.defs
ifneq ($(CONFIG_FC_ESTIMATOR),)
CONFIGURED_APPS += $(APPDIR)/fc-stack/estimator
endif
```

The fc-stack `Kconfig` is likewise sourced automatically, so `FC_CORE` and `FC_ESTIMATOR` appear under **Application Configuration → fc-stack** in `make menuconfig`.

### C++ apps: CXXEXT override

NuttX defaults to `.cxx` for C++ source files (`CXXEXT ?= .cxx` in `apps/Make.defs`). The fc-stack apps use `.cpp`. Each app Makefile overrides this **before** including `Make.defs`:

```makefile
CXXEXT = .cpp          # must be before include $(APPDIR)/Make.defs
include $(APPDIR)/Make.defs
...
MAINSRC = estimator_main.cpp
include $(APPDIR)/Application.mk
```

Without this override, `Application.mk` silently skips `.cpp` MAINSRC files and produces an empty object list, causing a linker `undefined reference to 'estimator_main'` error.

### Auto-LED (`CONFIG_ARCH_LEDS`)

NuttX expects `board_autoled_on()` and `board_autoled_off()` whenever `CONFIG_ARCH_LEDS=y`. The board provides these in `src/stm32_autoleds.c`, driving the system status LED on **PD6** (active-high GPIO). The board `src/Makefile` includes this file conditionally:

```makefile
ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
endif
```

---

## Build commands

```bash
cd nuttxspace/

# One-time: configure from the board defconfig
nuttx/tools/configure.sh -l ../boards/arm/stm32h7/shirley-fc-dev-board/configs/nsh

# Enable/disable fc-stack apps (Application Configuration → fc-stack)
make menuconfig

# Incremental build
make -j

# Full clean + reconfigure
make distclean
```

> `make` from `nuttxspace/` delegates to `nuttx/Makefile` via the thin wrapper `nuttxspace/Makefile`. No `EXTRA_APPS_DIR` is needed because the symlink makes `fc-stack` visible as a regular apps subdirectory.
