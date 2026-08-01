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

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "hardware/stm32_exti.h"
#include "shirley-fc-dev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_ICM40609D_UORB
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

int stm32_imu_int_placeholder(int irq, FAR void *context, FAR void *arg)
{
  return OK;
}

/****************************************************************************
 * Name: stm32_imu_irq_ack
 *
 * Description:
 *   Clears the EXTI pending bit for the IMU's INT1 line (PF5, EXTI line
 *   5). Passed to the driver as icm_config_s.irq_ack — see that field's
 *   doc comment for why this is required.
 ****************************************************************************/

void stm32_imu_irq_ack(void)
{
  putreg32(STM32_EXTI_MASK(5), STM32_EXTI_CPUPR1);
}
#endif /* CONFIG_SENSORS_ICM40609D_UORB */

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the shirley-fc-dev
 *   board.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
  /* Configure CS pins as GPIO outputs, deasserted (high) at boot */

  stm32_configgpio(GPIO_MAG_CS);   /* PE4 — Magnetometer CS */
  stm32_configgpio(GPIO_IMU_CS);   /* PF10 — IMU CS */

#ifdef CONFIG_SENSORS_ICM40609D_UORB
  /* Arm the IMU's data-ready EXTI line (rising edge). A non-NULL handler
   * is required here so stm32_gpiosetevent() unmasks the EXTI line itself
   * — see stm32_imu_int_placeholder()'s doc comment in stm32_spi.c. The
   * driver takes over the actual vector via irq_attach()/up_enable_irq()
   * when it starts streaming (icm_fifo_start() in icm40609d_uorb.c).
   */

  stm32_configgpio(GPIO_IMU_INT);   /* PF5 — IMU INT1 */
  stm32_gpiosetevent(GPIO_IMU_INT, true, false, false,
                     stm32_imu_int_placeholder, NULL);
#endif
}

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

  #ifdef CONFIG_SENSORS_ICM40609D_UORB
    if (devid == FC_IMU_SPIDEV)
      {
        stm32_gpiowrite(GPIO_IMU_CS, !selected); /* active low */
      }
  #endif
  }

uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif /* CONFIG_STM32H7_SPI5 */
