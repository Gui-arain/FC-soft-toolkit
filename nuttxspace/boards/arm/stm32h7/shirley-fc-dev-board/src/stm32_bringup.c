/****************************************************************************
 * boards/arm/stm32h7/shirley-fc-dev-board/src/stm32_bringup.c
 *
 * Board-level driver initialization for the Shirley FC Dev Board (STM32H743)
 *
 * Peripheral map (from pinout.yaml):
 *   SPI4  → Magnetometer  (SCK: PE2, MISO: PE5, MOSI: PE6, CS: PE4)
 *   SPI5  → IMU           (SCK: PF7, MISO: PF8, MOSI: PF9, CS: PF10)
 *   I2C2  → Barometer     (SDA: PF0, SCL: PF1)
 *   SDMMC1→ SD Card       (PC8-PC12, PD2, CDS: PD3)
 *
 * Sensor map:
 *   ICM-40609-D  → SPI5  (CS: PF10)  — IMU (accel + gyro)
 *   MMC5983MA    → SPI4  (CS: PE4)   — Magnetometer
 *   BMP388       → I2C2  (addr: 0x76 or 0x77) — Barometer
 *
 * NOTE ON DRIVER AVAILABILITY: MMC5983MA and BMP388 have no upstream or
 * out-of-tree NuttX drivers yet -- their register functions below are
 * TODO stubs that bring up the bus and log a warning.
 *
 * Called from board_late_initialize() (stm32_boot.c), which is the only
 * board-init hook this NuttX version actually calls after the scheduler
 * starts -- board_app_initialize() is dead/unreachable API in this NuttX
 * version (BOARDIOC_INIT is a deprecated no-op in boards/boardctl.c), so
 * all bus bring-up AND sensor registration live here now, mirroring
 * dakefpv-h743's stm32_bringup.c.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>
#include <sched.h>
#include <unistd.h>
#include <stdbool.h>

#include <arch/board/board.h>
#include <nuttx/board.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kthread.h>

#include "shirley-fc-dev.h"
#include "stm32_gpio.h"

#ifdef CONFIG_MMCSD
#  include <nuttx/mmcsd.h>
#endif

#include <nuttx/i2c/i2c_master.h>
#include "stm32_i2c.h"

#ifdef CONFIG_SENSORS_ICM40609D_UORB
  #include <string.h>
  #include <arch/irq.h>
  #include <nuttx/spi/spi.h>
  #include <nuttx/sensors/icm40609d_uorb.h>
  #include "stm32_spi.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI bus numbers (matches spi4/spi5 in pinout) */

#define FC_MAG_SPI_BUS       4
#define FC_IMU_SPI_BUS       5

/* I2C bus number (matches i2c2 in pinout) */

#define FC_BARO_I2C_BUS      2

/* BMP388 I2C address: 0x76 when SDO=GND, 0x77 when SDO=VDD
 * Check your board schematic for the SDO pin connection.
 */

#define BMP388_I2C_ADDR      0x76

/* Device minor numbers -- sets the /dev/xxx index */

#define IMU_DEVNO            0     /* -> /dev/imu0  (chip-specific path) */
#define MAG_DEVNO            0     /* -> /dev/mag0  */
#define BARO_DEVNO           0     /* -> /dev/baro0 */

/* GPIO_MAG_CS and GPIO_IMU_CS are defined in shirley-fc-dev.h */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fc_i2c_initialize
 *
 * Description:
 *   Bring up I2C2 for the barometer (SDA: PF0, SCL: PF1).
 *   The bus handle is stored for use by the sensor driver registered
 *   in fc_baro_register() below.
 *
 ****************************************************************************/

static int fc_i2c_initialize(void)
{
#ifdef CONFIG_STM32H7_I2C2
  FAR struct i2c_master_s *i2c2 = stm32_i2cbus_initialize(FC_BARO_I2C_BUS);
  if (i2c2 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C%d (BARO)\n",
             FC_BARO_I2C_BUS);
      return -ENODEV;
    }

  syslog(LOG_INFO, "I2C%d (BARO) initialized\n", FC_BARO_I2C_BUS);
#endif
  return OK;
}

/****************************************************************************
 * Name: fc_sdcard_initialize
 *
 * Description:
 *   Initialize SDMMC1 and mount the SD card as FAT filesystem at /mnt/sd.
 *   Card detect is on PD3 (active low).
 *   This is best-effort at bringup — no SD card is not a fatal error.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD
static int fc_sdcard_initialize(void)
{
  int ret;

  /* Initialize the SDMMC1 driver slot 0 */

  ret = stm32_sdmmc_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_sdmmc_initialize(0) failed: %d\n", ret);
      return ret;
    }

  /* Mount FAT filesystem — fail gracefully if no card is inserted */

  ret = mount("/dev/mmcsd0", "/mnt/sd", "vfat", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_WARNING,
             "WARNING: Could not mount SD card at /mnt/sd (no card?): %d\n",
             ret);
      /* Not fatal — continue bringup */
    }
  else
    {
      syslog(LOG_INFO, "SD card mounted at /mnt/sd\n");
    }

  return OK;
}
#endif /* CONFIG_MMCSD */

/****************************************************************************
 * Name: fc_imu_register
 *
 * Description:
 *   Register the ICM-40609-D IMU driver on SPI5.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ICM40609D_UORB

static int fc_imu_register(int bus, uint32_t spi_devid, int irq,
                           CODE void (*irq_ack)(void), int devno)
{
  FAR struct spi_dev_s *spi;
  struct icm_config_s cfg;
  int ret;

  spi = stm32_spibus_initialize(bus);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get SPI%d for IMU\n", bus);
      return -ENODEV;
    }

  memset(&cfg, 0, sizeof(cfg));
  cfg.spi       = spi;
  cfg.spi_devid = spi_devid;
  cfg.irq       = irq;
  cfg.irq_ack   = irq_ack;

  ret = icm40609d_uorb_register(IMU_DEVNO, &cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: icm42688p_register(devno=%d) failed: %d\n", devno, ret);
      return ret;
    }

  syslog(LOG_INFO,
         "IMU (ICM-40609-D) registered: sensor_accel%d/sensor_gyro%d\n",
         IMU_DEVNO, IMU_DEVNO);

  return OK;
}

#endif

/****************************************************************************
 * Name: fc_mag_register
 *
 * Description:
 *   Register the MMC5983MA magnetometer driver on SPI4.
 *
 * TODO: MMC5983MA has no upstream NuttX driver.
 *   Porting path:
 *     1. Write drivers/sensors/mmc5983ma.c from scratch using the
 *        MMC5983MA datasheet (MEMSIC). The device uses a standard
 *        SPI Mode 0/3 interface with 16-bit magnetic field output.
 *     2. Key registers: Product ID (0x2F = 0x30), Control0/1/2,
 *        X/Y/Z output (18-bit). Implement SET/RESET degaussing sequence
 *        on init — this is mandatory for accurate readings.
 *     3. Expose as NuttX uORB sensor type SENSOR_TYPE_MAGNETIC_FIELD
 *     4. Add Kconfig entry CONFIG_SENSORS_MMC5983MA
 *     5. Replace the stub below with:
 *          ret = mmc5983ma_register("/dev/mag0", spi, MAG_DEVNO);
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_MMC5983MA

static int fc_mag_register(void)
{
  FAR struct spi_dev_s *spi;
  int ret = OK;

  spi = stm32_spibus_initialize(FC_MAG_SPI_BUS);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get SPI%d for MAG\n",
             FC_MAG_SPI_BUS);
      return -ENODEV;
    }

  /* TODO: Replace with mmc5983ma_register() once driver is written.
   *
   * CS (PE4) was configured by stm32_spidev_initialize() in stm32_spi.c.
   */

  syslog(LOG_WARNING,
         "MAG (MMC5983MA) on SPI%d: driver not yet registered (TODO)\n",
         FC_MAG_SPI_BUS);

  return ret;
}

#endif

/****************************************************************************
 * Name: fc_baro_register
 *
 * Description:
 *   Register the BMP388 barometer driver on I2C2.
 *
 * TODO: BMP388 has no upstream NuttX driver. The upstream bmp280 driver
 *   (drivers/sensors/bmp280.c) is NOT compatible — the register map
 *   is completely different.
 *   Porting path:
 *     1. Write drivers/sensors/bmp388.c using the BMP388 datasheet (Bosch).
 *        The BMP388 uses a FIFO-based output, OSR/ODR/IIR filter config,
 *        and a 21-coefficient calibration table for compensation.
 *     2. Bosch provides an open-source compensation library (bmp3_api on
 *        GitHub) which can be adapted for NuttX.
 *     3. Expose as SENSOR_TYPE_BAROMETER (pressure + temperature)
 *     4. Add Kconfig entry CONFIG_SENSORS_BMP388
 *     5. Replace the stub below with:
 *          ret = bmp388_register("/dev/baro0", i2c, BMP388_I2C_ADDR,
 *                                BARO_DEVNO);
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP388

static int fc_baro_register(void)
{
#ifdef CONFIG_STM32H7_I2C2
  FAR struct i2c_master_s *i2c;

  i2c = stm32_i2cbus_initialize(FC_BARO_I2C_BUS);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d for BARO\n",
             FC_BARO_I2C_BUS);
      return -ENODEV;
    }

  /* TODO: Replace with bmp388_register() once driver is written.
   *
   * I2C address is 0x%02x — verify SDO pin on schematic.
   */

  syslog(LOG_WARNING,
         "BARO (BMP388) on I2C%d addr=0x%02x: driver not yet registered"
         " (TODO)\n",
         FC_BARO_I2C_BUS, BMP388_I2C_ADDR);
#else
  syslog(LOG_WARNING,
         "BARO (BMP388): I2C%d not enabled, skipping (TODO)\n",
         FC_BARO_I2C_BUS);
#endif

  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Called from board_late_initialize() (stm32_boot.c) after the OS
 *   scheduler is running. Initializes all board peripherals and sensor
 *   drivers in dependency order:
 *
 *     1. SPI buses  (IMU, MAG)
 *     2. I2C buses  (BARO)
 *     3. SD card
 *     4. Sensor driver registration (IMU, MAG, BARO)
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

  UNUSED(ret);

  /* 3. SD card (SDMMC1) */

#ifdef CONFIG_MMCSD
  ret = fc_sdcard_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_sdcard_initialize failed: %d\n", ret);
    }
#endif

  /* 4. Register sensor drivers.
   *    Each call is non-fatal — a missing sensor logs a warning but
   *    does not prevent the rest of the system from starting.
   */

  #ifdef CONFIG_SENSORS_ICM40609D_UORB

  ret = fc_imu_register(1, FC_IMU_SPIDEV, STM32_IRQ_EXTI95,
                        stm32_imu_irq_ack, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_imu_register(devno=0) failed: %d\n", ret);
    }

  #endif

  #ifdef CONFIG_SENSORS_MMC5983MA

  ret = fc_mag_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_mag_register() failed: %d\n", ret);
    }

  #endif

  #ifdef CONFIG_SENSORS_BMP388

  ret = fc_baro_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_baro_register() failed: %d\n", ret);
    }

  #endif

  syslog(LOG_INFO, "FC Dev board initialized\n");

  return OK;
}