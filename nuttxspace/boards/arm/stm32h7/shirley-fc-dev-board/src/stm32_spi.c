/****************************************************************************
 * boards/arm/stm32h7/shirley-fc-dev-board/src/stm32_spi.c
 *
 * Board-level SPI chip-select callbacks for the Shirley FC Dev Board.
 *
 * NuttX SPI framework calls stm32_spiNselect() whenever SPI_SELECT() is
 * invoked by a driver.  This file provides the implementations that drive
 * the board CS GPIO pins.
 *
 *   SPI4 → Magnetometer (MMC5983MA), CS: PE4
 *   SPI5 → IMU (ICM-40609-D),       CS: PF10
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <nuttx/spi/spi.h>
#include <nuttx/debug.h>

#include "stm32_gpio.h"
#include "fc-dev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spi4select
 *
 * Description:
 *   SPI4 chip-select — Magnetometer (MMC5983MA) on PE4.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI4
void stm32_spi4select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");

  if (devid == FC_MAG_SPIDEV)
    {
      stm32_gpiowrite(GPIO_MAG_CS, !selected); /* active low */
    }
}

uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif /* CONFIG_STM32H7_SPI4 */

/****************************************************************************
 * Name: stm32_spi5select
 *
 * Description:
 *   SPI5 chip-select — IMU (ICM-40609-D) on PF10.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI5
void stm32_spi5select(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");

  if (devid == FC_IMU_SPIDEV)
    {
      stm32_gpiowrite(GPIO_IMU_CS, !selected); /* active low */
    }
}

uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif /* CONFIG_STM32H7_SPI5 */
