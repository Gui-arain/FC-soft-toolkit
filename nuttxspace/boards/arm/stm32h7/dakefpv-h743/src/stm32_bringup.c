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
#include <sched.h>
#include <unistd.h>
#include <stdbool.h>

#include <arch/board/board.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kthread.h>

#include "dakefpv-h743.h"

#include "stm32_gpio.h"

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_SENSORS_ICM42688P
#  include <string.h>
#  include <arch/irq.h>
#  include <nuttx/spi/spi.h>
#  include <nuttx/sensors/icm42688p_uorb.h>
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
 *     IMU1: SPI1 (CS: PA4, EXTI: PC4) -> devno 0 -> sensor_accel0/gyro0
 *     IMU2: SPI4 (CS: PB1, EXTI: PB2) -> devno 1 -> sensor_accel1/gyro1
 *
 ****************************************************************************/

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

  ret = icm42688p_register(devno, &cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: icm42688p_register(devno=%d) failed: %d\n", devno, ret);
      return ret;
    }

  syslog(LOG_INFO,
         "IMU (ICM-42688-P) registered: sensor_accel%d/sensor_gyro%d\n",
         devno, devno);
  return OK;
}

#endif /* CONFIG_SENSORS_ICM42688P */

/****************************************************************************
 * Name: heartbeat_main
 *
 * Description:
 *   Toggles GPIO_LD1 on its own kernel thread, independent of HPWORK, the
 *   console, and USB. Purely a diagnostic: no debugger is currently
 *   attached (SWD isn't wired up right now), so this is the only way to
 *   tell "whole system locked up" apart from "one task/driver call is
 *   stuck" by eye — if this stops blinking, the lockup is global; if it
 *   keeps blinking, whatever's stuck is isolated to that one task/call.
 *   Remove once the estimator hang is root-caused.
 *
 ****************************************************************************/

static int heartbeat_main(int argc, FAR char *argv[])
{
  bool state = false;

  stm32_configgpio(GPIO_LD1);

  for (; ; )
    {
      state = !state;
      stm32_gpiowrite(GPIO_LD1, state);
      usleep(300 * 1000);
    }

  return 0;
}

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

  /* Priority deliberately near-max (well above the estimator task's
   * SCHED_PRIORITY_DEFAULT and close to HPWORK's 224): if this still
   * blinks through a freeze that a lower-priority heartbeat missed, the
   * freeze is CPU starvation from a spin-loop in some other task/thread's
   * context, not a true global-interrupt lockup.
   */

  // kthread_create("heartbeat", 230, 1024, heartbeat_main, NULL);

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

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_ICM42688P
  /* GPIO_LD1 used to be configured as an output by heartbeat_main()
   * above (now disabled). The ISR storm-probe in stm32_spi.c drives
   * this same pin directly on every watermark IRQ entry, so it still
   * needs to be in push-pull output mode -- neither CONFIG_ARCH_LEDS
   * nor CONFIG_USERLED is enabled in this defconfig to do it for us.
   */

  stm32_configgpio(GPIO_LD1);

  ret = fc_imu_register(1, FC_IMU1_SPIDEV, STM32_IRQ_EXTI4,
                        stm32_imu1_irq_ack, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_imu_register(devno=0) failed: %d\n", ret);
    }

  ret = fc_imu_register(4, FC_IMU2_SPIDEV, STM32_IRQ_EXTI2,
                        stm32_imu2_irq_ack, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fc_imu_register(devno=1) failed: %d\n", ret);
    }
#endif

  return OK;
}
