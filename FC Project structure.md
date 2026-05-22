# FC Stack Project Structure

Here is an example structure:

```
your-project/
├── nuttx/                              ← submodule (apache/nuttx, unmodified)
├── apps/                               ← submodule (apache/apps, unmodified)
│
├── boards/
│   └── arm/stm32h7/fc-dev/
│       ├── Kconfig
│       ├── Make.defs
│       ├── include/
│       │   └── board.h
│       ├── src/
│       │   ├── Makefile
│       │   ├── stm32_boot.c
│       │   ├── stm32_bringup.c        ← call up_cxxinitialize() here
│       │   └── stm32_appinit.c
│       └── configs/
│           └── nsh/
│               ├── defconfig          ← CONFIG_HAVE_CXX=y etc.
│               └── Make.defs
│
├── fc-stack/
│   ├── Kconfig                        ← sources all module Kconfigs
│   ├── Make.defs                      ← registers enabled modules
│   │
│   ├── fc_core/
│   │   ├── Kconfig
│   │   ├── Make.defs
│   │   ├── Makefile                   ← CXXSRCS, MAINSRC = .cpp
│   │   ├── fc_core_main.cpp           ← extern "C" entry point
│   │   ├── FcCore.hpp
│   │   └── FcCore.cpp
│   │
│   ├── estimator/
│   │   ├── Kconfig
│   │   ├── Make.defs
│   │   ├── Makefile
│   │   ├── estimator_main.cpp         ← extern "C" entry point
│   │   ├── Estimator.hpp
│   │   └── Estimator.cpp
│   │
│   ├── mixer/
│   │   ├── Kconfig
│   │   ├── Make.defs
│   │   ├── Makefile
│   │   ├── mixer_main.cpp
│   │   ├── Mixer.hpp
│   │   └── Mixer.cpp
│   │
│   └── telemetry/
│       ├── Kconfig
│       ├── Make.defs
│       ├── Makefile
│       ├── telemetry_main.cpp
│       ├── Telemetry.hpp
│       └── Telemetry.cpp
│
├── .gitmodules
└── scripts/
    ├── setup.sh                       ← symlink boards/ into nuttx/boards/
    └── build.sh                       ← make with EXTRA_APPS_DIR
```