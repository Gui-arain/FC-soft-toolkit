/****************************************************************************
 * boards/arm/stm32h7/weact-stm32h743/src/weact-stm32h743.h
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

#ifndef __BOARDS_ARM_STM32H7_DAKEFPV_H743_SRC_DAKEFPV_H743_H
#define __BOARDS_ARM_STM32H7_DAKEFPV_H743_SRC_DAKEFPV_H743_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_USBDEV          1
#define HAVE_USBHOST         1

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32H7_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#endif

/* Can't support USB device if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

#if !defined(CONFIG_STM32H7_PROGMEM) || !defined(CONFIG_MTD_PROGMEM)
#  undef HAVE_PROGMEM_CHARDEV
#endif

/* Check if we can support the RTC driver */

#define HAVE_RTC_DRIVER 1
#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* LED
 *
 * The dakefpv-h743 board has one user LED controlled by GPIO.
 * LED: connected to PD10
 */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN10)


/* IMU (2x ICM-42688-P), per resources/FC-boards/DAKEFPV_H743
 *
 * IMU1: SPI1  SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4,  EXTI=PC4
 * IMU2: SPI4  SCK=PE12, MISO=PE13, MOSI=PE14, CS=PB1, EXTI=PB2
 */

#define GPIO_IMU1_CS    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)   /* PA4  */

#define GPIO_IMU2_CS    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN1)   /* PB1  */

#define FC_IMU1_SPIDEV  SPIDEV_IMU(0)
#define FC_IMU2_SPIDEV  SPIDEV_IMU(1)

/* IMU data-ready interrupts (INT1, push-pull active-high on the
 * ICM-42688-P). GPIO_EXTI is baked into these definitions so
 * stm32_gpiosetevent() both configures the pin and routes the SYSCFG
 * EXTI mux. It's called with a placeholder handler (see
 * stm32_imu_int_placeholder() in stm32_spi.c) purely so it unmasks the
 * EXTI line's own interrupt mask register — the driver then takes over
 * the actual vector via irq_attach()/up_enable_irq() when it starts
 * streaming.
 */

#define GPIO_IMU1_INT   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                         GPIO_PORTC | GPIO_PIN4)   /* PC4 -> STM32_IRQ_EXTI4 */

#define GPIO_IMU2_INT   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                         GPIO_PORTB | GPIO_PIN2)   /* PB2 -> STM32_IRQ_EXTI2 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Mikroe Clicker2
 *   STM32 board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_imu1_irq_ack / stm32_imu2_irq_ack
 *
 * Description:
 *   Clear the EXTI pending bit for the ICM-42688-P IMU1/IMU2 data-ready
 *   lines. Passed to icm42688p_register() via icm_config_s.irq_ack.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ICM42688P
void stm32_imu1_irq_ack(void);
void stm32_imu2_irq_ack(void);
#endif

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

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_dma_alloc_init
 *
 * Description:
 *   Called to create a FAT DMA allocator.
 *
 * Returned Value:
 *   0 on success or -ENOMEM
 *
 ****************************************************************************/

#if defined (CONFIG_FAT_DMAMEMORY)
int stm32_dma_alloc_init(void);
#endif

#endif /* __BOARDS_ARM_STM32H7_DAKEFPV_H743_SRC_DAKEFPV_H743_H */
