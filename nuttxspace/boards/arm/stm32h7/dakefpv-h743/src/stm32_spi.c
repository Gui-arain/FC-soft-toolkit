/****************************************************************************
 * boards/arm/stm32h7/dakefpv-h743/src/stm32_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "hardware/stm32_exti.h"

#include "dakefpv-h743.h"
#include <arch/board/board.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ICM42688P
/****************************************************************************
 * Name: stm32_imu_int_placeholder
 *
 * Description:
 *   stm32_gpiosetevent() only sets the EXTI peripheral's own interrupt
 *   mask register (separate from, and in addition to, the NVIC enable
 *   toggled by up_enable_irq()/up_disable_irq()) when passed a non-NULL
 *   handler. A NULL handler here would leave the EXTI line masked at the
 *   source forever, regardless of anything the driver does at the NVIC
 *   level — so this placeholder exists purely to get that bit set once
 *   at boot. The driver's own irq_attach() in icm_fifo_start() overwrites
 *   the actual vector once streaming begins; this is never called for
 *   real.
 ****************************************************************************/

static int stm32_imu_int_placeholder(int irq, FAR void *context,
                                     FAR void *arg)
{
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ICM42688P
/****************************************************************************
 * Name: stm32_imu1_irq_ack / stm32_imu2_irq_ack
 *
 * Description:
 *   Clears the EXTI pending bit for IMU1 (PC4, line 4) / IMU2 (PB2, line
 *   2). Passed to the driver as icm_config_s.irq_ack — see that field's
 *   doc comment for why this is required.
 ****************************************************************************/

void stm32_imu1_irq_ack(void)
{
  putreg32(STM32_EXTI_MASK(4), STM32_EXTI_CPUPR1);
}

void stm32_imu2_irq_ack(void)
{
  putreg32(STM32_EXTI_MASK(2), STM32_EXTI_CPUPR1);
}
#endif

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the dakefpv-h743
 *   board.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
#ifdef CONFIG_SENSORS_ICM42688P
  stm32_configgpio(GPIO_IMU1_CS);   /* ICM-42688-P #1 chip select (SPI1) */
  stm32_configgpio(GPIO_IMU2_CS);   /* ICM-42688-P #2 chip select (SPI4) */

  /* Arm the data-ready EXTI lines (rising edge). A non-NULL handler is
   * required here so stm32_gpiosetevent() unmasks the EXTI line itself —
   * see stm32_imu_int_placeholder() above. The driver takes over the
   * actual vector via irq_attach()/up_enable_irq() when it starts
   * streaming (icm_fifo_start() in icm42688p-fifo.c).
   */

  stm32_gpiosetevent(GPIO_IMU1_INT, true, false, false,
                     stm32_imu_int_placeholder, NULL);
  stm32_gpiosetevent(GPIO_IMU2_INT, true, false, false,
                     stm32_imu_int_placeholder, NULL);
#endif
}

/****************************************************************************
 * Name:  stm32_spi1/2/3select and stm32_spi1/2/3status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *  (including stm32_spibus_initialize()) are provided by common STM32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI4
void stm32_spi4select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

#ifdef CONFIG_LCD_ST7735
  if (devid == SPIDEV_DISPLAY(0))
    {
      stm32_gpiowrite(GPIO_LCD_CS, !selected);
    }
#endif

#ifdef CONFIG_SENSORS_ICM42688P
  if (devid == FC_IMU2_SPIDEV)
    {
      stm32_gpiowrite(GPIO_IMU2_CS, !selected);   /* active low */
    }
#endif
}

uint8_t stm32_spi4status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1select and stm32_spi1status
 *
 * Description:
 *   SPI1 chip-select — IMU #1 (ICM-42688-P) on PA4.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI1
void stm32_spi1select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

#ifdef CONFIG_SENSORS_ICM42688P
  if (devid == FC_IMU1_SPIDEV)
    {
      stm32_gpiowrite(GPIO_IMU1_CS, !selected);   /* active low */
    }
#endif
}

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi4cmddata
 *
 * Description:
 *   This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32H7_SPI4
int stm32_spi4cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
#ifdef CONFIG_LCD_ST7735
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */

      stm32_gpiowrite(GPIO_LCD_DC, !cmd);

      return OK;
    }
#endif

  return -ENODEV;
}
#endif
#endif /* CONFIG_SPI_CMDDATA */
