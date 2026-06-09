/****************************************************************************
 * boards/arm/stm32h7/shirley-fc-dev-board/src/fc-dev.h
 *
 * Board-level shared definitions for the Shirley FC Dev Board (STM32H743).
 * Included by stm32_bringup.c, stm32_appinit.c, and stm32_spi.c.
 *
 * Peripheral map (pinout.yaml):
 *   SPI4  → Magnetometer  (CS: PE4  / GPIO_MAG_CS)
 *   SPI5  → IMU           (CS: PF10 / GPIO_IMU_CS)
 *   I2C2  → Barometer     (SDA: PF0, SCL: PF1)
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32H7_SHIRLEY_FC_DEV_BOARD_SRC_FC_DEV_H
#define __BOARDS_ARM_STM32H7_SHIRLEY_FC_DEV_BOARD_SRC_FC_DEV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI chip-select GPIO pins — active-low, push-pull, deasserted at boot */

#define GPIO_MAG_CS   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                       GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN4)   /* PE4  */

#define GPIO_IMU_CS   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                       GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN10)  /* PF10 */

/* SPI device IDs — passed to stm32_spiNselect() via SPI_SELECT().
 * Each bus has a single sensor so we use SPIDEV_SENSOR(0) on both.
 */

#define FC_IMU_SPIDEV   SPIDEV_IMU(0)
#define FC_MAG_SPIDEV   SPIDEV_USER(0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __BOARDS_ARM_STM32H7_SHIRLEY_FC_DEV_BOARD_SRC_FC_DEV_H */
