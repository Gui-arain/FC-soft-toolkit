
#ifndef __INCLUDE_NUTTX_SENSORS_ICM40609D_UORB_H
#define __INCLUDE_NUTTX_SENSORS_ICM40609D_UORB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * IOCTL Commands
 *
 * Device-specific commands live in the 0x00B0+ range (upstream NuttX
 * sensor ioctls top out at 0x00A9 as of this writing). Standard uORB
 * ioctls (SNIOC_SET_INTERVAL, SNIOC_BATCH, ...) cover ODR and FIFO
 * watermark/latency -- see <nuttx/sensors/ioctl.h>. Consumers only need
 * to include this header for the device-specific commands below.
 ****************************************************************************/

/* Set gyroscope full-scale range.  Arg: uint16_t dps (125/250/500/1000/2000) */

#define ICM40609D_IOC_SET_GYRO_FSR        _SNIOC(0x00B0)

/* Set accel full-scale range: ±4, ±8, ±16, ±32 g (no ±2g on this chip) */

#define ICM40609D_IOC_SET_ACCEL_FSR        _SNIOC(0x00B1)

/* 0x00B2 (formerly SET_ODR) retired -- use SNIOC_SET_INTERVAL instead. */

/* Set gyro digital notch filter center frequency.  Arg: uint32_t Hz */

#define ICM40609D_IOC_SET_GYRO_DNF_FREQ   _SNIOC(0x00B3)

/* Set gyro digital notch filter bandwidth.
 * Arg: uint32_t Hz (10/20/40/80/162/329/680/1449) */

#define ICM40609D_IOC_SET_GYRO_DNF_BW     _SNIOC(0x00B4)

/* 0x00B5 (formerly SET_WATERMARK) retired -- use SNIOC_BATCH instead. */

/* Read-and-clear the hardware lost-packet counter.  Arg: FAR uint32_t * */

#define ICM40609D_IOC_GET_LOST_PKTS       _SNIOC(0x00B6)

/* 0x00B7 (formerly RESET_FIFO) retired -- there is no software ring
 * buffer left to flush; the uORB upper half owns the circular buffer.
 */

/* Set gyro/accel offset registers.  Arg: FAR const struct icm40609d_offset_s * */

#define ICM40609D_IOC_SET_GYRO_OFFSET     _SNIOC(0x00B8)
#define ICM40609D_IOC_SET_ACCEL_OFFSET    _SNIOC(0x00B9)

/* Enable/disable the gyro digital notch filter (DNF).  Arg: bool (0=off, 1=on) */

#define ICM40609D_IOC_SET_GYRO_DNF_EN     _SNIOC(0x00BA)

/* Enable/disable the accel anti-aliasing filter (AAF).  Arg: bool (0=off, 1=on) */

#define ICM40609D_IOC_SET_ACCEL_AAF_EN    _SNIOC(0x00BB)

/* Set per-axis gyro notch filter center frequency.  Arg: uint32_t Hz */

#define ICM40609D_IOC_SET_GYRO_DNF_FREQ_X _SNIOC(0x00BC)
#define ICM40609D_IOC_SET_GYRO_DNF_FREQ_Y _SNIOC(0x00BD)
#define ICM40609D_IOC_SET_GYRO_DNF_FREQ_Z _SNIOC(0x00BE)

/* Set gyro UI filter order.  Arg: uint8_t (1/2/3) */

#define ICM40609D_IOC_SET_GYRO_UI_FILT_ORD  _SNIOC(0x00BF)

/* Set accel UI filter order.  Arg: uint8_t (1/2/3) */

#define ICM40609D_IOC_SET_ACCEL_UI_FILT_ORD _SNIOC(0x00C0)

/* Set gyro UI filter bandwidth selector.  Arg: uint8_t (register GYRO_ACCEL_CONFIG0 value) */

#define ICM40609D_IOC_SET_GYRO_UI_FILT_BW   _SNIOC(0x00C1)

/* Set accel UI filter bandwidth selector.  Arg: uint8_t (register GYRO_ACCEL_CONFIG0 value) */

#define ICM40609D_IOC_SET_ACCEL_UI_FILT_BW  _SNIOC(0x00C2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct spi_dev_s;

/* Gyro/accel offset for all three axes (host byte order, raw register units). */

struct icm40609d_offset_s
{
  int16_t x;
  int16_t y;
  int16_t z;
};

/* Specifies the initial chip configuration and bus wiring.
 *
 * This driver supports SPI only.  Zero-initialise the struct and then
 * set the spi/spi_devid/irq fields:
 *
 *    struct icm_config_s cfg;
 *    memset(&cfg, 0, sizeof(cfg));
 *    cfg.spi       = spi_bus;
 *    cfg.spi_devid = FC_IMU_SPIDEV;
 *    cfg.irq       = STM32_IRQ_EXTI95;  // whatever line INT1 is wired to
 *    cfg.irq_ack   = my_board_exti95_ack; // clears the arch's EXTI/GPIO
 *                                         // pending bit for that line
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

  /* irq : the arch-specific IRQ number wired to the chip's INT1 pin
   *       (e.g. STM32_IRQ_EXTI95). The board is responsible for configuring
   *       that GPIO as an EXTI input; this driver only attaches to and
   *       enables/disables the already-numbered IRQ.
   */

  int irq;

  /* irq_ack : clears the arch/board-specific pending bit for `irq`.
   *
   * This driver attaches directly to the raw IRQ vector (irq_attach()),
   * bypassing whatever shared per-line dispatcher the arch's GPIO/EXTI
   * framework would otherwise use — and on STM32 (and most EXTI-style
   * peripherals), clearing that dispatcher's pending bit is exactly what
   * makes the interrupt edge-triggered instead of level-triggered. Skip
   * this and the IRQ re-fires the instant the ISR returns, forever.
   * Required — must not be NULL.
   */

  CODE void (*irq_ack)(void);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: icm40609d_uorb_register
 *
 * Description:
 *   Registers an ICM-40609-D as a pair of uORB topics:
 *   /dev/uorb/sensor_accel<devno> and /dev/uorb/sensor_gyro<devno>.
 *
 * Input Parameters:
 *   devno  - Device instance number (0, 1, ...) shared by both topics.
 *   config - Configuration information (SPI bus + chip-select + IRQ).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int icm40609d_uorb_register(int devno, FAR struct icm_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_ICM40609D_UORB_H */
