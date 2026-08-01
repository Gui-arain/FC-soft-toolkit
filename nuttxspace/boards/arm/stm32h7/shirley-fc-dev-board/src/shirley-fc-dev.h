/****************************************************************************
 * boards/arm/stm32h7/shirley-fc-dev-board/src/shirley-fc-dev.h
 *
 * Board-level shared definitions for the Shirley FC Dev Board (STM32H743).
 * Included by stm32_boot.c, stm32_bringup.c, and stm32_spi.c.
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
#include <nuttx/irq.h>
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI chip-select GPIO pins — active-low, push-pull, deasserted at boot */

#define GPIO_MAG_CS   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                       GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN4)   /* PE4  */

#define GPIO_IMU_CS   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                       GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN10)  /* PF10 */

/* IMU data-ready interrupt (INT1), per config/shirley-fc-config/pinout.yaml
 * (imu_int1: PF5) -> EXTI line 5 -> shared STM32_IRQ_EXTI95 vector.
 * GPIO_EXTI baked in so stm32_gpiosetevent() configures the pin AND routes
 * the SYSCFG EXTI mux in one call.
 */

#define GPIO_IMU_INT  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                       GPIO_PORTF | GPIO_PIN5)   /* PF5 -> STM32_IRQ_EXTI95 */

/* SPI device IDs — passed to stm32_spiNselect() via SPI_SELECT().
 * Each bus has a single sensor so we use SPIDEV_SENSOR(0) on both.
 */

#define FC_IMU_SPIDEV   SPIDEV_IMU(0)
#define FC_MAG_SPIDEV   SPIDEV_USER(0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int stm32_bringup(void);

/* stm32_spidev_initialize : configures SPI chip-select GPIOs (and, when
 * the uORB IMU driver is enabled, the IMU's data-ready EXTI line). See
 * stm32_spi.c.
 */

void stm32_spidev_initialize(void);

#ifdef CONFIG_SENSORS_ICM40609D_UORB
/* stm32_imu_int_placeholder : no-op handler passed to stm32_gpiosetevent()
 * so it unmasks the EXTI line at boot; the driver's own irq_attach() takes
 * over the real vector once streaming starts (see stm32_spi.c).
 */

int stm32_imu_int_placeholder(int irq, FAR void *context, FAR void *arg);

/* stm32_imu_irq_ack : clears the EXTI5 pending bit. Passed to the driver
 * as icm_config_s.irq_ack (see stm32_spi.c).
 */

void stm32_imu_irq_ack(void);
#endif

#endif /* __BOARDS_ARM_STM32H7_SHIRLEY_FC_DEV_BOARD_SRC_FC_DEV_H */
