# ICM-40609-D NuttX Driver

**Files:** `nuttxspace/drivers/sensors/icm40609d.c` / `icm40609d.h`
**Interface:** SPI only (Mode 0, 2 MHz)
**WHO_AM_I:** `0x3B`

---

## Architecture

```
Board init code
    └── icm40609d_register("/dev/imu0", &cfg)   ← public API
            └── mpu_reset()                      ← soft-reset + default config
                    └── __icm_write/read_reg()   ← SPI helpers (lock required)

User app
    └── open/read/ioctl("/dev/imu0")
            └── mpu_open / mpu_read / mpu_ioctl  ← file ops
```

The driver is **legacy-style** (not uORB). It exposes a character device that returns raw 14-byte snapshots.

---

## Registration

```c
#include <nuttx/sensors/icm40609d.h>

struct icm_config_s cfg;
memset(&cfg, 0, sizeof(cfg));
cfg.spi      = spi_bus;          /* FAR struct spi_dev_s * */
cfg.spi_devid = SPIDEV_IMU(0);  /* chip-select id */

int ret = icm40609d_register("/dev/imu0", &cfg);
```

Call once from board bringup (e.g. `stm32_bringup.c`). The function allocates the device state, runs the reset/init sequence, and registers the character device node.

---

## Default configuration after reset

| Parameter | Default |
|-----------|---------|
| Gyro full-scale | ±2000 dps |
| Accel full-scale | ±16 g |
| ODR (both) | 1 kHz |
| Power mode | Low-noise |
| FIFO | Disabled |
| INT1 | Active-high, push-pull, latched |

---

## Reading sensor data

```c
int fd = open("/dev/imu0", O_RDONLY);

struct sensor_data_s sample;
read(fd, &sample, sizeof(sample));
```

`read()` returns exactly 14 bytes — a single atomic snapshot of all sensors. The struct layout matches the hardware register burst starting at `TEMP_DATA1 (0x1D)`:

```c
struct sensor_data_s {   /* all big-endian signed 16-bit from hardware */
    int16_t temp;        /* raw — convert: T(°C) = (raw / 132.48) + 25 */
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
};
```

Converting raw values (default full-scale):

| Sensor | Scale factor |
|--------|-------------|
| Accel ±16 g | raw / 2048.0 → g |
| Gyro ±2000 dps | raw / 16.4 → dps |
| Temp | raw / 132.48 + 25 → °C |

Partial reads are supported — the driver maintains a byte cursor so you can read fewer than 14 bytes per call. The buffer is refreshed on the next read once the previous snapshot is fully consumed.

---

## IOCTL commands

Defined in `<nuttx/sensors/ioctl.h>`.

| Command | Arg type | Effect |
|---------|----------|--------|
| `SNIOC_SET_AFS_SEL` | `uint8_t` (0–4) | Set accel full-scale: 0=±16 g, 1=±8 g, 2=±4 g, 3=±2 g, 4=±32 g |
| `SNIOC_READ_SAMPLE_RATE` | `uint32_t *` | Read gyro ODR into pointed value (Hz) |
| `SNIOC_READ_FIFO_COUNT` | `uint16_t *` | Read number of bytes currently in hardware FIFO |
| `SNIOC_ENABLE_FIFO` | `uint8_t` (0/1) | Enable or disable FIFO streaming mode |

```c
/* Example: set accel to ±8 g */
uint8_t afs = 1;
ioctl(fd, SNIOC_SET_AFS_SEL, (unsigned long)afs);

/* Example: read sample rate */
uint32_t rate;
ioctl(fd, SNIOC_READ_SAMPLE_RATE, (unsigned long)&rate);
```

---

## FIFO mode

When enabled via `SNIOC_ENABLE_FIFO`, the driver:
1. Puts the FIFO in **stream (continuous) mode** — oldest packets are overwritten when full.
2. Routes temp + accel + gyro into the FIFO (`FIFO_CONFIG1`).

`read()` then pulls data from `FIFO_DATA` instead of the direct output registers. The 2 kB hardware FIFO holds ~146 packets (14 bytes each).

Disable with `ioctl(fd, SNIOC_ENABLE_FIFO, 0)`.

---

## Locking

Internal helpers prefixed `__icm_` require the `mpu_dev_s` mutex to be held by the caller. The public file-ops (`mpu_read`, `mpu_ioctl`, etc.) handle locking themselves. Do not call `__icm_*` helpers from outside the driver.

---

## Register bank note

All registers used by this driver live in **Bank 0** (active after reset). If you extend the driver to access Bank 1–4 registers (e.g. ±4000 dps gyro range via `GYRO_CONFIG1` in Bank 1), write `REG_BANK_SEL` before and after the access.
