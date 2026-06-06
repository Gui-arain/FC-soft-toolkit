/****************************************************************************
 * boards/arm/stm32h7/fc-dev/src/stm32_bringup.c
 *
 * Board-level driver initialization for the Shirley FC Dev Board (STM32H743)
 *
 * Peripheral map (from pinout.yaml):
 *   SPI4  → Magnetometer  (SCK: PE2, MISO: PE5, MOSI: PE6, CS: PE4)
 *   SPI5  → IMU           (SCK: PF7, MISO: PF8, MOSI: PF9, CS: PF10)
 *   I2C2  → Barometer     (SDA: PF0, SCL: PF1)
 *   SDMMC1→ SD Card       (PC8-PC12, PD2, CDS: PD3)
 *
 * Called from stm32_appinit.c after the OS scheduler is running.
 * up_cxxinitialize() is called here before any fc-stack modules start.
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

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_MMCSD
#  include <nuttx/mmcsd.h>
#endif

#include <arch/board/board.h>
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_i2c.h"
#include "fc-dev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI bus numbers (matches spi4/spi5 in pinout) */

#define FC_MAG_SPI_BUS       4
#define FC_IMU_SPI_BUS       5

/* I2C bus number (matches i2c2 in pinout) */

#define FC_BARO_I2C_BUS      2

/* GPIO_MAG_CS and GPIO_IMU_CS are defined in fc-dev.h */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fc_spi_initialize
 *
 * Description:
 *   Configure the CS GPIO pins for SPI devices and verify the buses
 *   come up. Actual sensor driver registration happens in stm32_appinit.c
 *   once we know which sensor ICs are fitted.
 *
 ****************************************************************************/

static int fc_spi_initialize(void)
{
  int ret = OK;

  /* Configure CS pins as GPIO outputs, deasserted (high) at boot */

  stm32_configgpio(GPIO_MAG_CS);   /* PE4 — Magnetometer CS */
  stm32_configgpio(GPIO_IMU_CS);   /* PF10 — IMU CS */

  /* Bring up SPI4 (Magnetometer) */

  FAR struct spi_dev_s *spi4 = stm32_spibus_initialize(FC_MAG_SPI_BUS);
  if (spi4 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI%d (MAG)\n",
             FC_MAG_SPI_BUS);
      ret = -ENODEV;
    }
  else
    {
      syslog(LOG_INFO, "SPI%d (MAG) initialized\n", FC_MAG_SPI_BUS);
    }

  /* Bring up SPI5 (IMU) */

  FAR struct spi_dev_s *spi5 = stm32_spibus_initialize(FC_IMU_SPI_BUS);
  if (spi5 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI%d (IMU)\n",
             FC_IMU_SPI_BUS);
      ret = -ENODEV;
    }
  else
    {
      syslog(LOG_INFO, "SPI%d (IMU) initialized\n", FC_IMU_SPI_BUS);
    }

  return ret;
}

/****************************************************************************
 * Name: fc_i2c_initialize
 *
 * Description:
 *   Bring up I2C2 for the barometer (SDA: PF0, SCL: PF1).
 *   The bus handle is stored for use by the sensor driver registered
 *   in stm32_appinit.c.
 *
 ****************************************************************************/

static int fc_i2c_initialize(void)
{
  FAR struct i2c_master_s *i2c2 = stm32_i2cbus_initialize(FC_BARO_I2C_BUS);
  if (i2c2 == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C%d (BARO)\n",
             FC_BARO_I2C_BUS);
      return -ENODEV;
    }

  syslog(LOG_INFO, "I2C%d (BARO) initialized\n", FC_BARO_I2C_BUS);
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Called from stm32_appinit.c after the OS is fully up.
 *   Initializes all board peripherals in dependency order:
 *
 *     1. C++ static constructors  ← MUST be first, before any fc-stack code
 *     2. SPI buses  (IMU, MAG)
 *     3. I2C buses  (BARO)
 *     4. SD card
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

  /* 1. Run C++ static constructors.
   *
   *    This MUST happen before any fc-stack module (FcCore, Estimator, etc.)
   *    is started. Without this call, all C++ objects with static storage
   *    duration (global/static class instances) will be uninitialized and
   *    will silently malfunction or hard-fault.
   */

#ifdef CONFIG_HAVE_CXXINITIALIZE
  up_cxxinitialize();
#endif

  /* 2. SPI buses — IMU (SPI5) and Magnetometer (SPI4) */

  ret = fc_spi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_spi_initialize failed: %d\n", ret);
      /* Non-fatal at this stage — sensor drivers will report missing bus */
    }

  /* 3. I2C bus — Barometer (I2C2) */

  ret = fc_i2c_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_i2c_initialize failed: %d\n", ret);
    }

  /* 4. SD card (SDMMC1) */

#ifdef CONFIG_MMCSD
  ret = fc_sdcard_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_sdcard_initialize failed: %d\n", ret);
    }
#endif

  return OK;
}