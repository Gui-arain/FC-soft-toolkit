/****************************************************************************
 * boards/arm/stm32h7/shirley-fc-dev-board/src/stm32_appinit.c
 *
 * Application-level initialization for the Shirley FC Dev Board (STM32H743)
 *
 * Sensor map (from pinout.yaml):
 *   ICM-40609-D  → SPI5  (CS: PF10)  — IMU (accel + gyro)
 *   MMC5983MA    → SPI4  (CS: PE4)   — Magnetometer
 *   BMP388       → I2C2  (addr: 0x76 or 0x77) — Barometer
 *
 * NOTE ON DRIVER AVAILABILITY:
 *   None of these three sensors have upstream NuttX drivers as of 2025.
 *   The closest upstream equivalents are:
 *     - icm42688  (SPI, accel+gyro)  → basis for ICM-40609-D port
 *     - (none)                        → MMC5983MA needs a new driver
 *     - bmp280    (I2C, baro)         → register map differs from BMP388
 *
 *   Each sensor section below is marked TODO with the porting guidance.
 *   The bus handles and CS GPIO setup are done here so driver registration
 *   can be dropped in with minimal changes once drivers are ready.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#ifdef CONFIG_SENSORS_ICM40609D_UORB
#include <nuttx/sensors/icm40609d_uorb.h>
#else
#include <nuttx/sensors/icm40609d.h>
#endif

#include <arch/board/board.h>
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_i2c.h"
#include "fc-dev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bus assignments — must match pinout.yaml */

#define FC_IMU_SPI_BUS        5     /* SPI5: ICM-40609-D */
#define FC_MAG_SPI_BUS        4     /* SPI4: MMC5983MA   */
#define FC_BARO_I2C_BUS       2     /* I2C2: BMP388      */

/* BMP388 I2C address: 0x76 when SDO=GND, 0x77 when SDO=VDD
 * Check your board schematic for the SDO pin connection.
 */

#define BMP388_I2C_ADDR       0x76

/* Device minor numbers — sets the /dev/xxx index */

#define IMU_DEVNO             0     /* → /dev/imu0   (chip-specific path) */
#define MAG_DEVNO             0     /* → /dev/mag0   */
#define BARO_DEVNO            0     /* → /dev/baro0  */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fc_imu_register
 *
 * Description:
 *   Register the ICM-40609-D IMU driver on SPI5.
 *
 *
 ****************************************************************************/

static int fc_imu_register(void)
{
  FAR struct spi_dev_s *spi;
  struct icm_config_s cfg;
  int ret;

  spi = stm32_spibus_initialize(FC_IMU_SPI_BUS);
  if (spi == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get SPI%d for IMU\n",
             FC_IMU_SPI_BUS);
      return -ENODEV;
    }

  memset(&cfg, 0, sizeof(cfg));
  cfg.spi      = spi;
  cfg.spi_devid = FC_IMU_SPIDEV;

#ifdef CONFIG_SENSORS_ICM40609D_UORB
  cfg.irq     = STM32_IRQ_EXTI95;
  cfg.irq_ack = stm32_imu_irq_ack;

  ret = icm40609d_uorb_register(IMU_DEVNO, &cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: icm40609d_uorb_register(devno=%d) failed: %d\n",
             IMU_DEVNO, ret);
      return ret;
    }

  syslog(LOG_INFO,
         "IMU (ICM-40609-D) registered: sensor_accel%d/sensor_gyro%d\n",
         IMU_DEVNO, IMU_DEVNO);
#else
  ret = icm40609d_register("/dev/imu0", &cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: icm40609d_register(/dev/imu0) failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "IMU (ICM-40609-D) registered at /dev/imu0\n");
#endif

  return OK;
}

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
   * CS (PE4) was configured in stm32_bringup.c.
   */

  syslog(LOG_WARNING,
         "MAG (MMC5983MA) on SPI%d: driver not yet registered (TODO)\n",
         FC_MAG_SPI_BUS);

  return ret;
}

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

static int fc_baro_register(void)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

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

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   NuttX calls this function after the OS is fully running.
 *   This is the top-level application init entry point for the board.
 *
 *   Initialization order:
 *     1. stm32_bringup()   — buses, filesystems, up_cxxinitialize()
 *     2. Sensor registration (IMU, MAG, BARO)
 *     3. fc-stack task launch (fc_core, estimator, mixer, telemetry)
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  int ret;

  // Calls stm32_bringup to initialise buses, cxx...
  stm32_bringup();

  /* Register sensor drivers.
   *    Each call is non-fatal — a missing sensor logs a warning but
   *    does not prevent the rest of the system from starting.
   */

  ret = fc_imu_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_imu_register() failed: %d\n", ret);
    }

  ret = fc_mag_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_mag_register() failed: %d\n", ret);
    }

  ret = fc_baro_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_baro_register() failed: %d\n", ret);
    }

  /* 3. TODO: Launch fc-stack tasks.
   *
   *    Once sensor drivers are registered, start the fc-stack modules.
   *    Each module has an extern "C" entry point defined in its _main.cpp.
   *    Example:
   *
   *      task_create("fc_core",   CONFIG_FC_CORE_PRIORITY,
   *                  CONFIG_FC_CORE_STACKSIZE,   fc_core_main,   NULL);
   *      task_create("estimator", CONFIG_ESTIMATOR_PRIORITY,
   *                  CONFIG_ESTIMATOR_STACKSIZE, estimator_main, NULL);
   *      task_create("mixer",     CONFIG_MIXER_PRIORITY,
   *                  CONFIG_MIXER_STACKSIZE,     mixer_main,     NULL);
   *      task_create("telemetry", CONFIG_TELEMETRY_PRIORITY,
   *                  CONFIG_TELEMETRY_STACKSIZE, telemetry_main, NULL);
   *
   *    Priorities and stack sizes should be defined in fc-stack/Kconfig.
   */

  syslog(LOG_INFO, "FC Dev board initialized\n");
  return OK;
}