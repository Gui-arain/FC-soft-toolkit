
#ifndef __INCLUDE_NUTTX_SENSORS_ICM40609D_H
#define __INCLUDE_NUTTX_SENSORS_ICM40609D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct spi_dev_s;

/* Specifies the initial chip configuration and bus wiring.
 *
 * This driver supports SPI only.  Zero-initialise the struct and then
 * set the spi/spi_devid fields:
 *
 *    struct icm_config_s cfg;
 *    memset(&cfg, 0, sizeof(cfg));
 *    cfg.spi      = spi_bus;
 *    cfg.spi_devid = FC_IMU_SPIDEV;
 */

struct icm_config_s
{
  /* For users on SPI.
   *
   *  spi_devid : the SPI master's slave-select number
   *              for the chip, as used in SPI_SELECT(..., dev_id, ...)
   *  spi       : the SPI master device, as used in SPI_SELECT(spi, ..., ...)
   */

  FAR struct spi_dev_s *spi;
  int spi_devid;
};

/* Describes the ICM-40609-D sensor output registers. This structure reflects
 * the underlying hardware, so don't change it!
 */

begin_packed_struct struct icm40609d_data_s
{
  int16_t temp;
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
} end_packed_struct;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: icm40609d_register
 *
 * Description:
 *   Registers the ICM-40609-D at devpath.
 *
 * Returns 0 on success, or negative errno.
 *
 ****************************************************************************/

int icm40609d_register(FAR const char *path, FAR struct icm_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_ICM40609D_H */
