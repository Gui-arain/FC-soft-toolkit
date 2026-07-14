/****************************************************************************
 * boards/arm/stm32h7/dakefpv-h743/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>

#include "dakefpv-h743.h"

#include "stm32_gpio.h"

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_SENSORS_ICM42688P
#  include <string.h>
#  include <arch/irq.h>
#  include <nuttx/spi/spi.h>
#  include <nuttx/sensors/icm42688p-fifo.h>
#  include "stm32_spi.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ICM42688P

/****************************************************************************
 * Name: fc_imu_register
 *
 * Description:
 *   Register the two ICM-42688-P IMU drivers, per
 *   resources/FC-boards/DAKEFPV_H743:
 *     IMU1: SPI1 (CS: PA4, EXTI: PC4) -> /dev/imu0
 *     IMU2: SPI4 (CS: PB1, EXTI: PB2) -> /dev/imu1
 *
 ****************************************************************************/

static int fc_imu_register(int bus, uint32_t spi_devid, int irq,
                           FAR const char *path)
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

  ret = icm42688p_register(path, &cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: icm42688p_register(%s) failed: %d\n", path, ret);
      return ret;
    }

  syslog(LOG_INFO, "IMU (ICM-42688-P) registered at %s\n", path);
  return OK;
}

#endif /* CONFIG_SENSORS_ICM42688P */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

  UNUSED(ret);

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

#if defined(CONFIG_FAT_DMAMEMORY)
  if (stm32_dma_alloc_init() < 0)
    {
      syslog(LOG_ERR, "DMA alloc FAILED");
    }
#endif

#ifdef HAVE_SDIO
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_ICM42688P
  ret = fc_imu_register(1, FC_IMU1_SPIDEV, STM32_IRQ_EXTI4, "/dev/imu0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_imu_register(/dev/imu0) failed: %d\n", ret);
    }

  ret = fc_imu_register(4, FC_IMU2_SPIDEV, STM32_IRQ_EXTI2, "/dev/imu1");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_imu_register(/dev/imu1) failed: %d\n", ret);
    }
#endif

  return OK;
}
