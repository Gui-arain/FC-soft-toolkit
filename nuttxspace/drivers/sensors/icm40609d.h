
#ifndef __INCLUDE_NUTTX_SENSORS_ICM40609D_H
#define __INCLUDE_NUTTX_SENSORS_ICM40609D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These structures are defined elsewhere, and we don't need their
 * definitions here.
 */

struct spi_dev_s;

/* Specifies the initial chip configuration and bus wiring.
 *
 * This driver supports SPI only.  Zero-initialise the struct and then
 * set the spi/spi_devid fields:
 *
 *    struct icm_config_s cfg;
 *    memset(&cfg, 0, sizeof(cfg));
 *    cfg.spi      = spi_bus;
 *    cfg.spi_devid = SPIDEV_IMU(0);
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
