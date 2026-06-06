
/* WARNING for developers:
 *
 * This driver uses the legacy style of writing sensor drivers for NuttX. The
 * project has since decided to adopt a new sensor framework in order to
 * have a consistent API and feature-set.
 *
 * Sensors which use the uORB framework are typically suffixed "_uorb". You
 * can also visit the documentation about the new sensor framework to learn
 * more.
 */

/****************************************************************************
 * TODO: Theory of Operation
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/debug.h>
#include <string.h>
#include <limits.h>
#include <nuttx/bits.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/icm40609d.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Creates a mask of @m bits, i.e. MASK(2) -> 00000011 */

#define MASK(m) (BIT(m) - 1)

/* Masks and shifts @v into bit field @m */

#define TO_BITFIELD(m,v) (((v) & MASK(m ##__WIDTH)) << (m ##__SHIFT))

/* Un-masks and un-shifts bit field @m from @v */

#define FROM_BITFIELD(m,v) (((v) >> (m ##__SHIFT)) & MASK(m ##__WIDTH))

/* SPI read/write codes */

#define ICM_REG_READ 0x80
#define ICM_REG_WRITE 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ICM-40609-D register map (ICM-4x6xx family, Bank 0).
 *
 * __SHIFT : number of empty bits to the right of the field
 * __WIDTH : width of the field, in bits
 *
 * Single-bit fields use BIT(n) directly and have no __SHIFT/__WIDTH pair.
 *
 * NOTE: The ICM-4x6xx uses a banked register scheme selected via
 * REG_BANK_SEL (0x76).  All registers below live in Bank 0, which is
 * active after reset.  Configuration registers used during init that
 * live in other banks are listed at the bottom with their bank noted.
 */

enum icm_regaddr_e
{
  /* Device configuration (Bank 0) */

  DEVICE_CONFIG = 0x11,
  DEVICE_CONFIG__SPI_MODE = BIT(4),           /* 0: Mode 0&3 (default), 1: Mode 1&2 */
  DEVICE_CONFIG__SOFT_RESET_CONFIG = BIT(0),  /* write 1; wait 1 ms before next access */

  /* Drive slew-rate configuration (Bank 0) */

  DRIVE_CONFIG = 0x13,
  DRIVE_CONFIG__I2C_SLEW_RATE__SHIFT = 3,
  DRIVE_CONFIG__I2C_SLEW_RATE__WIDTH = 3,
  DRIVE_CONFIG__SPI_SLEW_RATE__SHIFT = 0,
  DRIVE_CONFIG__SPI_SLEW_RATE__WIDTH = 3,

  /* Interrupt pin configuration (Bank 0) */

  INT_CONFIG = 0x14,
  INT_CONFIG__INT2_MODE = BIT(5),
  INT_CONFIG__INT2_DRIVE_CIRCUIT = BIT(4),
  INT_CONFIG__INT2_POLARITY = BIT(3),
  INT_CONFIG__INT1_MODE = BIT(2),
  INT_CONFIG__INT1_DRIVE_CIRCUIT = BIT(1),
  INT_CONFIG__INT1_POLARITY = BIT(0),

  /* FIFO mode (Bank 0) */

  FIFO_CONFIG = 0x16,
  FIFO_CONFIG__FIFO_MODE__SHIFT = 6,
  FIFO_CONFIG__FIFO_MODE__WIDTH = 2,

  /* Sensor output registers — read sequentially from TEMP_DATA1 for
   * an atomic snapshot (14 bytes: temp, accel XYZ, gyro XYZ).
   * All values are big-endian signed 16-bit. (Bank 0)
   */

  TEMP_DATA1 = 0x1d,     /* RO – temperature high byte  */
  TEMP_DATA0 = 0x1e,     /* RO – temperature low byte   */
  ACCEL_DATA_X1 = 0x1f,  /* RO – accel X high byte      */
  ACCEL_DATA_X0 = 0x20,  /* RO – accel X low byte       */
  ACCEL_DATA_Y1 = 0x21,  /* RO – accel Y high byte      */
  ACCEL_DATA_Y0 = 0x22,  /* RO – accel Y low byte       */
  ACCEL_DATA_Z1 = 0x23,  /* RO – accel Z high byte      */
  ACCEL_DATA_Z0 = 0x24,  /* RO – accel Z low byte       */
  GYRO_DATA_X1 = 0x25,   /* RO – gyro X high byte       */
  GYRO_DATA_X0 = 0x26,   /* RO – gyro X low byte        */
  GYRO_DATA_Y1 = 0x27,   /* RO – gyro Y high byte       */
  GYRO_DATA_Y0 = 0x28,   /* RO – gyro Y low byte        */
  GYRO_DATA_Z1 = 0x29,   /* RO – gyro Z high byte       */
  GYRO_DATA_Z0 = 0x2a,   /* RO – gyro Z low byte        */

  /* FSYNC timestamp (Bank 0) */

  TMST_FSYNCH = 0x2b,    /* RO */
  TMST_FSYNCL = 0x2c,    /* RO */

  /* Interrupt status — clears on read (Bank 0) */

  INT_STATUS = 0x2d,
  INT_STATUS__UI_FSYNC_INT = BIT(6),
  INT_STATUS__PLL_RDY_INT = BIT(5),
  INT_STATUS__RESET_DONE_INT = BIT(4),
  INT_STATUS__DATA_RDY_INT = BIT(3),
  INT_STATUS__FIFO_THS_INT = BIT(2),
  INT_STATUS__FIFO_FULL_INT = BIT(1),
  INT_STATUS__AGC_RDY_INT = BIT(0),

  /* FIFO (Bank 0) */

  FIFO_COUNTH = 0x2e,    /* RO – reading this latches both bytes */
  FIFO_COUNTL = 0x2f,    /* RO */
  FIFO_DATA = 0x30,      /* RO – burst-read FIFO port */

  /* APEX pedometer / tap-detection output (Bank 0) */

  APEX_DATA0 = 0x31,     /* RO – step count [7:0]   */
  APEX_DATA1 = 0x32,     /* RO – step count [15:8]  */
  APEX_DATA2 = 0x33,     /* RO – walk/run cadence   */
  APEX_DATA3 = 0x34,     /* RO – DMP idle, activity */
  APEX_DATA4 = 0x35,     /* RO – tap num/axis/dir   */
  APEX_DATA5 = 0x36,     /* RO – double-tap timing  */

  /* Additional interrupt status (Bank 0) */

  INT_STATUS2 = 0x37,    /* RO/C – WOM / SMD */
  INT_STATUS3 = 0x38,    /* RO/C – step / tilt / tap */

  /* Signal path reset (Bank 0) */

  SIGNAL_PATH_RESET = 0x4b,
  SIGNAL_PATH_RESET__DMP_INIT_EN = BIT(6),
  SIGNAL_PATH_RESET__DMP_MEM_RESET_EN = BIT(5),
  SIGNAL_PATH_RESET__ABORT_AND_RESET = BIT(3),
  SIGNAL_PATH_RESET__TMST_STROBE = BIT(2),
  SIGNAL_PATH_RESET__FIFO_FLUSH = BIT(1),

  /* Interface configuration (Bank 0) */

  INTF_CONFIG0 = 0x4c,
  INTF_CONFIG0__FIFO_HOLD_LAST_DATA_EN = BIT(7),
  INTF_CONFIG0__FIFO_COUNT_REC = BIT(6),    /* 0: bytes, 1: records */
  INTF_CONFIG0__FIFO_COUNT_ENDIAN = BIT(5), /* 0: little, 1: big (default) */
  INTF_CONFIG0__SENSOR_DATA_ENDIAN = BIT(4),/* 0: little, 1: big (default) */
  INTF_CONFIG0__UI_SIFS_CFG__SHIFT = 0,
  INTF_CONFIG0__UI_SIFS_CFG__WIDTH = 2,

  INTF_CONFIG1 = 0x4d,
  INTF_CONFIG1__ACCEL_LP_CLK_SEL = BIT(3),
  INTF_CONFIG1__RTC_MODE = BIT(2),
  INTF_CONFIG1__CLKSEL__SHIFT = 0,
  INTF_CONFIG1__CLKSEL__WIDTH = 2,

  /* Power management (Bank 0) */

  PWR_MGMT0 = 0x4e,                        /* Reset: 0x00 */
  PWR_MGMT0__TEMP_DIS = BIT(5),
  PWR_MGMT0__IDLE = BIT(4),
  PWR_MGMT0__GYRO_MODE__SHIFT = 2,
  PWR_MGMT0__GYRO_MODE__WIDTH = 2,
  PWR_MGMT0__ACCEL_MODE__SHIFT = 0,
  PWR_MGMT0__ACCEL_MODE__WIDTH = 2,

  /* Gyroscope configuration (Bank 0)
   *
   * GYRO_FS_SEL (bits 7:5):
   *   000 = ±2000 dps  (default)
   *   001 = ±1000 dps
   *   010 = ±500  dps
   *   011 = ±250  dps
   *   100 = ±125  dps
   *   101 = ±62.5 dps
   *   110 = ±31.25 dps
   *   111 = ±15.625 dps
   *
   * ICM-40609-D also supports ±4000 dps via GYRO_CONFIG1 in Bank 1.
   *
   * GYRO_ODR (bits 3:0):
   *   0001 = 32 kHz  … 0110 = 1 kHz (default) … 1111 = 500 Hz
   */

  GYRO_CONFIG0 = 0x4f,                     /* Reset: 0x06 */
  GYRO_CONFIG0__GYRO_FS_SEL__SHIFT = 5,
  GYRO_CONFIG0__GYRO_FS_SEL__WIDTH = 3,
  GYRO_CONFIG0__GYRO_ODR__SHIFT = 0,
  GYRO_CONFIG0__GYRO_ODR__WIDTH = 4,

  /* Accelerometer configuration (Bank 0)
   *
   * ACCEL_FS_SEL (bits 7:5):
   *   000 = ±16 g  (default)
   *   001 = ±8  g
   *   010 = ±4  g
   *   011 = ±2  g
   *   100 = ±32 g  (ICM-40609-D only)
   *
   * ACCEL_ODR (bits 3:0):
   *   0001 = 32 kHz … 0110 = 1 kHz (default) … 1111 = 500 Hz
   */

  ACCEL_CONFIG0 = 0x50,                    /* Reset: 0x06 */
  ACCEL_CONFIG0__ACCEL_FS_SEL__SHIFT = 5,
  ACCEL_CONFIG0__ACCEL_FS_SEL__WIDTH = 3,
  ACCEL_CONFIG0__ACCEL_ODR__SHIFT = 0,
  ACCEL_CONFIG0__ACCEL_ODR__WIDTH = 4,

  /* Gyroscope filter configuration (Bank 0) */

  GYRO_CONFIG1 = 0x51,                     /* Reset: 0x16 */
  GYRO_CONFIG1__TEMP_FILT_BW__SHIFT = 5,
  GYRO_CONFIG1__TEMP_FILT_BW__WIDTH = 3,
  GYRO_CONFIG1__GYRO_UI_FILT_ORD__SHIFT = 2,
  GYRO_CONFIG1__GYRO_UI_FILT_ORD__WIDTH = 2,
  GYRO_CONFIG1__GYRO_DEC2_M2_ORD__SHIFT = 0,
  GYRO_CONFIG1__GYRO_DEC2_M2_ORD__WIDTH = 2,

  /* Gyro/accel low-pass filter bandwidth (Bank 0) */

  GYRO_ACCEL_CONFIG0 = 0x52,               /* Reset: 0x11 */
  GYRO_ACCEL_CONFIG0__ACCEL_UI_FILT_BW__SHIFT = 4,
  GYRO_ACCEL_CONFIG0__ACCEL_UI_FILT_BW__WIDTH = 4,
  GYRO_ACCEL_CONFIG0__GYRO_UI_FILT_BW__SHIFT = 0,
  GYRO_ACCEL_CONFIG0__GYRO_UI_FILT_BW__WIDTH = 4,

  /* Accelerometer filter configuration (Bank 0) */

  ACCEL_CONFIG1 = 0x53,                    /* Reset: 0x0D */
  ACCEL_CONFIG1__ACCEL_UI_FILT_ORD__SHIFT = 3,
  ACCEL_CONFIG1__ACCEL_UI_FILT_ORD__WIDTH = 2,
  ACCEL_CONFIG1__ACCEL_DEC2_M2_ORD__SHIFT = 1,
  ACCEL_CONFIG1__ACCEL_DEC2_M2_ORD__WIDTH = 2,

  /* Timestamp configuration (Bank 0) */

  TMST_CONFIG = 0x54,                      /* Reset: 0x23 */
  TMST_CONFIG__TMST_TO_REGS_EN = BIT(4),
  TMST_CONFIG__TMST_RES = BIT(3),
  TMST_CONFIG__TMST_DELTA_EN = BIT(2),
  TMST_CONFIG__TMST_FSYNC_EN = BIT(1),
  TMST_CONFIG__TMST_EN = BIT(0),

  /* APEX feature configuration (Bank 0) */

  APEX_CONFIG0 = 0x56,                     /* Reset: 0x82 */
  APEX_CONFIG0__DMP_POWER_SAVE = BIT(7),
  APEX_CONFIG0__TAP_ENABLE = BIT(6),
  APEX_CONFIG0__PED_ENABLE = BIT(5),
  APEX_CONFIG0__TILT_ENABLE = BIT(4),
  APEX_CONFIG0__R2W_EN = BIT(3),
  APEX_CONFIG0__DMP_ODR__SHIFT = 0,
  APEX_CONFIG0__DMP_ODR__WIDTH = 2,

  /* SMD / Wake-on-Motion configuration (Bank 0) */

  SMD_CONFIG = 0x57,                       /* Reset: 0x00 */
  SMD_CONFIG__WOM_INT_MODE = BIT(3),
  SMD_CONFIG__WOM_MODE = BIT(2),
  SMD_CONFIG__SMD_MODE__SHIFT = 0,
  SMD_CONFIG__SMD_MODE__WIDTH = 2,

  /* FIFO sensor enable + features (Bank 0) */

  FIFO_CONFIG1 = 0x5f,                     /* Reset: 0x00 */
  FIFO_CONFIG1__FIFO_RESUME_PARTIAL_RD = BIT(6),
  FIFO_CONFIG1__FIFO_WM_GT_TH = BIT(5),
  FIFO_CONFIG1__FIFO_HIRES_EN = BIT(4),
  FIFO_CONFIG1__FIFO_TMST_FSYNC_EN = BIT(3),
  FIFO_CONFIG1__FIFO_TEMP_EN = BIT(2),
  FIFO_CONFIG1__FIFO_GYRO_EN = BIT(1),
  FIFO_CONFIG1__FIFO_ACCEL_EN = BIT(0),

  FIFO_CONFIG2 = 0x60,  /* RW – watermark[7:0]  */
  FIFO_CONFIG3 = 0x61,  /* RW – watermark[11:8] */

  /* FSYNC configuration (Bank 0) */

  FSYNC_CONFIG = 0x62,                     /* Reset: 0x10 */
  FSYNC_CONFIG__FSYNC_UI_SEL__SHIFT = 4,
  FSYNC_CONFIG__FSYNC_UI_SEL__WIDTH = 3,
  FSYNC_CONFIG__FSYNC_UI_FLAG_CLEAR_SEL = BIT(1),
  FSYNC_CONFIG__FSYNC_POLARITY = BIT(0),

  /* Interrupt clear options (Bank 0) */

  INT_CONFIG0 = 0x63,
  INT_CONFIG0__UI_DRDY_INT_CLEAR__SHIFT = 4,
  INT_CONFIG0__UI_DRDY_INT_CLEAR__WIDTH = 2,
  INT_CONFIG0__FIFO_THS_INT_CLEAR__SHIFT = 2,
  INT_CONFIG0__FIFO_THS_INT_CLEAR__WIDTH = 2,
  INT_CONFIG0__FIFO_FULL_INT_CLEAR__SHIFT = 0,
  INT_CONFIG0__FIFO_FULL_INT_CLEAR__WIDTH = 2,

  /* Interrupt pulse configuration (Bank 0) */

  INT_CONFIG1 = 0x64,                      /* Reset: 0x10 */
  INT_CONFIG1__INT_TPULSE_DURATION = BIT(6),
  INT_CONFIG1__INT_TDEASSERT_DISABLE = BIT(5),
  INT_CONFIG1__INT_ASYNC_RESET = BIT(4),   /* must clear to 0 for normal INT operation */

  /* INT1 source routing (Bank 0) */

  INT_SOURCE0 = 0x65,                      /* Reset: 0x10 */
  INT_SOURCE0__UI_FSYNC_INT1_EN = BIT(6),
  INT_SOURCE0__PLL_RDY_INT1_EN = BIT(5),
  INT_SOURCE0__RESET_DONE_INT1_EN = BIT(4),
  INT_SOURCE0__UI_DRDY_INT1_EN = BIT(3),
  INT_SOURCE0__FIFO_THS_INT1_EN = BIT(2),
  INT_SOURCE0__FIFO_FULL_INT1_EN = BIT(1),
  INT_SOURCE0__UI_AGC_RDY_INT1_EN = BIT(0),

  INT_SOURCE1 = 0x66,
  INT_SOURCE1__I3C_PROTOCOL_ERROR_INT1_EN = BIT(6),
  INT_SOURCE1__SMD_INT1_EN = BIT(3),
  INT_SOURCE1__WOM_Z_INT1_EN = BIT(2),
  INT_SOURCE1__WOM_Y_INT1_EN = BIT(1),
  INT_SOURCE1__WOM_X_INT1_EN = BIT(0),

  /* INT2 source routing (Bank 0) */

  INT_SOURCE3 = 0x68,
  INT_SOURCE3__UI_FSYNC_INT2_EN = BIT(6),
  INT_SOURCE3__PLL_RDY_INT2_EN = BIT(5),
  INT_SOURCE3__RESET_DONE_INT2_EN = BIT(4),
  INT_SOURCE3__UI_DRDY_INT2_EN = BIT(3),
  INT_SOURCE3__FIFO_THS_INT2_EN = BIT(2),
  INT_SOURCE3__FIFO_FULL_INT2_EN = BIT(1),
  INT_SOURCE3__UI_AGC_RDY_INT2_EN = BIT(0),

  INT_SOURCE4 = 0x69,
  INT_SOURCE4__I3C_PROTOCOL_ERROR_INT2_EN = BIT(6),
  INT_SOURCE4__SMD_INT2_EN = BIT(3),
  INT_SOURCE4__WOM_Z_INT2_EN = BIT(2),
  INT_SOURCE4__WOM_Y_INT2_EN = BIT(1),
  INT_SOURCE4__WOM_X_INT2_EN = BIT(0),

  /* FIFO lost packet counter (Bank 0) */

  FIFO_LOST_PKT0 = 0x6c,   /* RO – count[7:0]  */
  FIFO_LOST_PKT1 = 0x6d,   /* RO – count[15:8] */

  /* Self-test configuration (Bank 0) */

  SELF_TEST_CONFIG = 0x70,
  SELF_TEST_CONFIG__ACCEL_ST_POWER = BIT(6),
  SELF_TEST_CONFIG__EN_AZ_ST = BIT(5),
  SELF_TEST_CONFIG__EN_AY_ST = BIT(4),
  SELF_TEST_CONFIG__EN_AX_ST = BIT(3),
  SELF_TEST_CONFIG__EN_GZ_ST = BIT(2),
  SELF_TEST_CONFIG__EN_GY_ST = BIT(1),
  SELF_TEST_CONFIG__EN_GX_ST = BIT(0),

  /* Device identity (Bank 0) */

  WHO_AM_I = 0x75,   /* RO – 0x3B for ICM-40609-D */

  /* Register bank selection — accessible from all banks */

  REG_BANK_SEL = 0x76,
  REG_BANK_SEL__BANK_SEL__SHIFT = 0,
  REG_BANK_SEL__BANK_SEL__WIDTH = 3,
};

/* Describes the ICM-40609-D sensor output registers. This structure reflects
 * the underlying hardware, so don't change it!
 */

begin_packed_struct struct sensor_data_s
{
  int16_t temp;
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
} end_packed_struct;

/* Used by the driver to manage the device */

struct mpu_dev_s
{
  mutex_t lock;               /* mutex for this structure */
  struct icm_config_s config; /* board-specific information */

  struct sensor_data_s buf;   /* temporary buffer (for read(), etc.) */
  size_t bufpos;              /* cursor into @buf, in bytes (!) */

  uint8_t gyro_odr;           /* gyro output data rate selector */
  uint8_t accel_odr;          /* accel output data rate selector */
  uint8_t afs_sel;            /* full scale range of the accelerometer */
  uint8_t dnf_config;         /* digital notch filter configuration */
  uint8_t daaf_config;        /* digital anti aliasing filter configuration */
  bool fifo_enabled;          /* current enable state of FIFO buffer */
  float sample_rate;          /* current sample rate */
};

/****************************************************************************
 * Private Function Function Prototypes
 ****************************************************************************/

static int mpu_open(FAR struct file *filep);
static int mpu_close(FAR struct file *filep);
static ssize_t mpu_read(FAR struct file *filep, FAR char *buf, size_t len);
static ssize_t mpu_write(FAR struct file *filep, FAR const char *buf,
                         size_t len);
static off_t mpu_seek(FAR struct file *filep, off_t offset, int whence);
static int mpu_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mpu_fops =
{
  mpu_open,        /* open */
  mpu_close,       /* close */
  mpu_read,        /* read */
  mpu_write,       /* write */
  mpu_seek,        /* seek */
  mpu_ioctl,       /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NOTE :
 *
 * In all of the following code, functions named with a double leading
 * underscore '__' must be invoked ONLY if the mpu_dev_s lock is
 * already held. Failure to do this might cause the transaction to get
 * interrupted, which will likely confuse the data you get back.
 *
 * The mpu_dev_s lock is NOT the same thing as, i.e. the SPI master
 * interface lock: the latter protects the bus interface hardware
 * (which may have other SPI devices attached), the former protects
 * the chip and its associated data.
 */


/* __icm_read_reg(), but for SPI-connected devices. See that function
 * for documentation.
 */

static int __icm_read_reg_spi(FAR struct mpu_dev_s *dev,
                              enum icm_regaddr_e reg_addr,
                              FAR uint8_t *buf, uint8_t len)
{
  int ret;
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* We'll probably return the number of bytes asked for. */

  ret = len;

  /* Grab and configure the SPI master device: always mode 0, 20MHz if it's a
   * data register, 1MHz otherwise (per datasheet).
   */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0); //CPOL = 0 (Low), CPHA = 0 (1 Edge)
  SPI_SETFREQUENCY(spi, 2000000); // SPI frequency set to 2MHz (based on clock config)


  /* Select the chip. */

  SPI_SELECT(spi, id, true); //Select the slave by pulling the CS line low

  /* Send the read request. */

  SPI_SEND(spi, reg_addr | ICM_REG_READ);

  /* Clock in the data. */

  while (0 != len--)
    {
      *buf++ = (uint8_t) (SPI_SEND(spi, 0xff));
    }

  /* Deselect the chip, release the SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}

/* __icm_write_reg(), but for SPI connections. */

static int __icm_write_reg_spi(FAR struct mpu_dev_s *dev,
                               enum icm_regaddr_e reg_addr,
                               FAR const uint8_t * buf, uint8_t len)
{
  int ret;
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* Hopefully, we'll return all the bytes they're asking for. */

  ret = len;

  /* Grab and configure the SPI master device. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0); //CPOL = 0 (Low), CPHA = 0 (1 Edge)
  SPI_SETFREQUENCY(spi, 2000000); // SPI frequency set to 2MHz (based on clock config)

  /* Select the chip. */

  SPI_SELECT(spi, id, true); //Select the slave by pulling the CS line low

  /* Send the write request. */

  SPI_SEND(spi, reg_addr | ICM_REG_WRITE); //bit7 = 0 for writing in a register

  /* Send the data. */

  while (0 != len--)
    {
      SPI_SEND(spi, *buf++);
    }

  /* Release the chip and SPI master. */

  SPI_SELECT(spi, id, false);
  SPI_LOCK(spi, false);

  return ret;
}


/* __icm_read_reg()
 *
 * Reads a block of @len byte-wide registers, starting at @reg_addr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

static inline int __icm_read_reg(FAR struct mpu_dev_s *dev,
                                 enum icm_regaddr_e reg_addr,
                                 FAR uint8_t *buf, uint8_t len)
{
  /* If we're wired to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return __icm_read_reg_spi(dev, reg_addr, buf, len);
    }

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* __icm_write_reg()
 *
 * Writes a block of @len byte-wide registers, starting at @reg_addr,
 * using the values in @buf to the device connected to @dev. Register
 * values are taken in numerical order from @buf, i.e.:
 *
 *   buf[0] -> register[@reg_addr]
 *   buf[1] -> register[@reg_addr + 1]
 *   ...
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes written, or a negative errno.
 */

static inline int __icm_write_reg(FAR struct mpu_dev_s *dev,
                                  enum icm_regaddr_e reg_addr,
                                  FAR const uint8_t *buf, uint8_t len)
{
  /* If we're connected to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return __icm_write_reg_spi(dev, reg_addr, buf, len);
    }

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* __icm_read_imu()
 *
 * Reads the whole IMU data file from @dev in one uninterrupted pass,
 * placing the sampled values into @buf. This function is the only way
 * to guarantee that the measured values are sampled as closely-spaced
 * in time as the hardware permits, which is almost always what you
 * want.
 */

static inline int __icm_read_imu(FAR struct mpu_dev_s *dev,
                                 FAR struct sensor_data_s *buf)
{
  if (dev->fifo_enabled)
    {
      return __icm_read_reg(dev, FIFO_DATA, (FAR uint8_t *)buf, sizeof(*buf));
    }

  return __icm_read_reg(dev, TEMP_DATA1, (FAR uint8_t *)buf, sizeof(*buf));
}

static inline int __icm_write_signal_path_reset(FAR struct mpu_dev_s *dev,
                                                uint8_t val)
{
  return __icm_write_reg(dev, SIGNAL_PATH_RESET, &val, sizeof(val));
}

static inline int __icm_write_int_config(FAR struct mpu_dev_s *dev,
                                         uint8_t val)
{
  return __icm_write_reg(dev, INT_CONFIG, &val, sizeof(val));
}

static inline int __icm_write_pwr_mgmt0(FAR struct mpu_dev_s *dev,
                                        uint8_t val)
{
  return __icm_write_reg(dev, PWR_MGMT0, &val, sizeof(val));
}

/* __icm_write_gyro_config() :
 *
 * Sets GYRO_FS_SEL in GYRO_CONFIG0.
 *
 * GYRO_FS_SEL (bits 7:5):
 *   000 = ±2000 dps (default)   001 = ±1000 dps   010 = ±500 dps
 *   011 = ±250 dps               100 = ±125 dps    101 = ±62.5 dps
 *   110 = ±31.25 dps             111 = ±15.625 dps
 */

static inline int __icm_write_gyro_config(FAR struct mpu_dev_s *dev,
                                          uint8_t fs_sel)
{
  uint8_t val = TO_BITFIELD(GYRO_CONFIG0__GYRO_FS_SEL, fs_sel);
  return __icm_write_reg(dev, GYRO_CONFIG0, &val, sizeof(val));
}

/* __icm_write_accel_config() :
 *
 * Sets ACCEL_FS_SEL in ACCEL_CONFIG0.
 *
 * ACCEL_FS_SEL (bits 7:5):
 *   000 = ±16 g (default)   001 = ±8 g   010 = ±4 g
 *   011 = ±2 g               100 = ±32 g  (ICM-40609-D only)
 */

static inline int __icm_write_accel_config(FAR struct mpu_dev_s *dev,
                                           uint8_t afs_sel)
{
  uint8_t val;
  if (afs_sel > 4)
    {
      snerr("ERROR: Invalid ACCEL_FS_SEL value\n");
      return -EINVAL;
    }

  val = TO_BITFIELD(ACCEL_CONFIG0__ACCEL_FS_SEL, afs_sel);
  return __icm_write_reg(dev, ACCEL_CONFIG0, &val, sizeof(val));
}

/* Reads current sample rate from GYRO_CONFIG0 ODR field.
 * Value is stored in dev->sample_rate (Hz).
 *
 * GYRO_ODR encoding (Table 16 of the datasheet):
 *   0x01 = 32 kHz,  0x02 = 16 kHz, 0x03 = 8 kHz,  0x04 = 4 kHz,
 *   0x05 = 2 kHz,   0x06 = 1 kHz,  0x0f = 500 Hz,  0x07 = 200 Hz,
 *   0x08 = 100 Hz,  0x09 = 50 Hz,  0x0a = 25 Hz,   0x0b = 12.5 Hz
 */

static inline int __icm_read_sample_rate(FAR struct mpu_dev_s *dev)
{
  static const float odr_table[16] =
  {
    0.0f,      /* 0x00 – reserved */
    32000.0f,  /* 0x01 */
    16000.0f,  /* 0x02 */
    8000.0f,   /* 0x03 */
    4000.0f,   /* 0x04 */
    2000.0f,   /* 0x05 */
    1000.0f,   /* 0x06 – reset default */
    200.0f,    /* 0x07 */
    100.0f,    /* 0x08 */
    50.0f,     /* 0x09 */
    25.0f,     /* 0x0a */
    12.5f,     /* 0x0b */
    0.0f,      /* 0x0c – reserved */
    0.0f,      /* 0x0d – reserved */
    0.0f,      /* 0x0e – reserved */
    500.0f,    /* 0x0f */
  };

  int ret;
  uint8_t reg;

  ret = __icm_read_reg(dev, GYRO_CONFIG0, &reg, sizeof(reg));
  if (ret < 0)
    {
      return ret;
    }

  dev->gyro_odr = FROM_BITFIELD(GYRO_CONFIG0__GYRO_ODR, reg);
  dev->sample_rate = odr_table[dev->gyro_odr & 0x0f];

  return OK;
}

/* Read the number of bytes currently in FIFO buffer. */

static inline int __icm_read_fifo_count(FAR struct mpu_dev_s *dev,
                                        uint16_t *buf)
{
  int ret;
  uint8_t fifo_counter[2];
  ret = __icm_read_reg(dev, FIFO_COUNTH, fifo_counter, sizeof(fifo_counter));
  if (ret < 0)
    {
      snerr("ERROR: Failed to read FIFO counter\n");
      *buf = 0;
    }
  else
    {
      *buf = (fifo_counter[0] << 8) | fifo_counter[1];
    }

  return ret;
}

/* Enables or disables sensor data in the FIFO via FIFO_CONFIG1.
 * Pass the OR of FIFO_CONFIG1__FIFO_*_EN bits you want active,
 * or 0 to disable all sensors from the FIFO.
 */

static inline int __icm_set_fifo(FAR struct mpu_dev_s *dev,
                                 uint8_t val)
{
  return __icm_write_reg(dev, FIFO_CONFIG1, &val, sizeof(val));
}

/* Resets the ICM-40609-D and applies a default configuration. */

static int mpu_reset(FAR struct mpu_dev_s *dev)
{
  int ret;
  uint8_t status;

  if (dev->config.spi == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  /* Issue soft reset via DEVICE_CONFIG. */

  uint8_t rst = DEVICE_CONFIG__SOFT_RESET_CONFIG;
  ret = __icm_write_reg(dev, DEVICE_CONFIG, &rst, sizeof(rst));
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      snerr("Could not reach ICM-40609-D!\n");
      return ret;
    }

  /* Wait for RESET_DONE_INT (datasheet: ~1 ms typical). */

  do
    {
      nxsched_usleep(2000);
      __icm_read_reg(dev, INT_STATUS, &status, sizeof(status));
    }
  while (!(status & INT_STATUS__RESET_DONE_INT));

  /* Enable gyro and accel in low-noise mode, temperature on.
   *
   * PWR_MGMT0 GYRO_MODE bits 3:2 = 0b11 (low-noise)
   * PWR_MGMT0 ACCEL_MODE bits 1:0 = 0b11 (low-noise)
   */

  ret = __icm_write_pwr_mgmt0(dev,
          TO_BITFIELD(PWR_MGMT0__GYRO_MODE, 3) |
          TO_BITFIELD(PWR_MGMT0__ACCEL_MODE, 3));
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      snerr("ERROR: Failed to enable gyro/accel: %d\n", ret);
      return ret;
    }

  /* Per datasheet: wait 200 µs after enabling sensors before first read. */

  nxsched_usleep(200);

  /* Default gyro: ±2000 dps, 1 kHz ODR (FS_SEL=0, ODR=0x06). */

  __icm_write_gyro_config(dev, 0);
  dev->gyro_odr = 0x06;

  /* Default accel: ±16 g, 1 kHz ODR (FS_SEL=0, ODR=0x06). */

  __icm_write_accel_config(dev, 0);
  dev->afs_sel = 0;
  dev->accel_odr = 0x06;

  /* Configure INT1 pin: active-high push-pull, latched.
   * Clear INT_CONFIG1 async-reset bit so the pin works normally.
   */

  __icm_write_int_config(dev, INT_CONFIG__INT1_POLARITY |
                              INT_CONFIG__INT1_DRIVE_CIRCUIT);

  uint8_t int_cfg1 = 0;  /* clear INT_ASYNC_RESET (bit 4) */
  __icm_write_reg(dev, INT_CONFIG1, &int_cfg1, sizeof(int_cfg1));

  /* Disable FIFO sensor feeds. */

  __icm_set_fifo(dev, 0);
  dev->fifo_enabled = false;

  nxmutex_unlock(&dev->lock);
  return 0;
}

/****************************************************************************
 * Name: mpu_open
 *
 * Note: we don't deal with multiple users trying to access this interface at
 * the same time. Until further notice, don't do that.
 *
 * And no, it's not as simple as just prohibiting concurrent opens or
 * reads with a mutex: there are legit reasons for truy concurrent
 * access, but they must be treated carefully in this interface lest a
 * partial reader end up with a mixture of old and new samples. This
 * will make some users unhappy.
 *
 ****************************************************************************/

static int mpu_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  /* Reset the register cache */

  nxmutex_lock(&dev->lock);
  dev->bufpos = 0;
  nxmutex_unlock(&dev->lock);

  return 0;
}

/****************************************************************************
 * Name: mpu_close
 ****************************************************************************/

static int mpu_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  /* Reset (clear) the register cache. */

  nxmutex_lock(&dev->lock);
  dev->bufpos = 0;
  nxmutex_unlock(&dev->lock);

  return 0;
}

/****************************************************************************
 * Name: mpu_read
 *
 * Returns a snapshot of the accelerometer, temperature, and gyro registers.
 *
 * Note: the chip uses traditional, twos-complement notation, i.e. "0"
 * is encoded as 0, and full-scale-negative is 0x8000, and
 * full-scale-positive is 0x7fff. If we read the registers
 * sequentially and directly into memory (as we do), the measurements
 * from each sensor are captured as big endian words.
 *
 * In contrast, ASN.1 maps "0" to 0x8000, full-scale-negative to 0,
 * and full-scale-positive to 0xffff. So if we want to send in a
 * format that an ASN.1 PER-decoder would recognize, must:
 *
 *   1. Treat the register data/measurements as unsigned,
 *   2. Add 0x8000 to each measurement, and then,
 *   3. Send each word in big-endian order.
 *
 * The result of the above will be something you could neatly describe
 * like this (confirmed with asn1scc):
 *
 *    Sint16  ::= INTEGER(-32768..32767)
 *
 *    Mpu60x0Sample ::= SEQUENCE
 *    {
 *      accel-X  Sint16,
 *      accel-Y  Sint16,
 *      accel-Z  Sint16,
 *      temp     Sint16,
 *      gyro-X   Sint16,
 *      gyro-Y   Sint16,
 *      gyro-Z   Sint16
 *    }
 *
 ****************************************************************************/

static ssize_t mpu_read(FAR struct file *filep, FAR char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;
  size_t send_len = 0;

  nxmutex_lock(&dev->lock);

  /* Populate the register cache if it seems empty. */

  if (!dev->bufpos)
    {
      __icm_read_imu(dev, &dev->buf);
    }

  /* Send the lesser of: available bytes, or amount requested. */

  send_len = sizeof(dev->buf) - dev->bufpos;
  if (send_len > len)
    {
      send_len = len;
    }

  if (send_len)
    {
      memcpy(buf, ((FAR uint8_t *)&dev->buf) + dev->bufpos, send_len);
    }

  /* Move the cursor, to mark them as sent. */

  dev->bufpos += send_len;

  /* If we've sent the last byte, reset the buffer. */

  if (dev->bufpos >= sizeof(dev->buf))
    {
      dev->bufpos = 0;
    }

  nxmutex_unlock(&dev->lock);
  return send_len;
}

/****************************************************************************
 * Name: mpu_write
 ****************************************************************************/

static ssize_t mpu_write(FAR struct file *filep, FAR const char *buf,
                         size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);
  snerr("ERROR: %p %p %zu\n", inode, dev, len);

  return len;
}

/****************************************************************************
 * Name: icm40609d_seek
 ****************************************************************************/

static off_t mpu_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);

  snerr("ERROR: %p %p\n", inode, dev);

  return 0;
}

/****************************************************************************
 * Name: icm40609d_ioctl
 ****************************************************************************/

static int mpu_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mpu_dev_s *priv = inode->i_private;
  uint8_t write_data = (uint8_t)arg;
  int ret = OK;

  switch (cmd)
    {
      /* Sets the accelerometer full scale range. Arg: uint8_t value */

      case SNIOC_SET_AFS_SEL:
        ret = __icm_write_accel_config(priv, write_data);
        if (ret < 0)
          {
            snerr("ERROR: SNIOC_SET_AFS_SEL fails. Returns: %d\n", ret);
          }
        else
          {
            priv->afs_sel = write_data;
            sninfo("SNIOC_SET_AFS_SEL: %d Returns: %d\n", priv->afs_sel,
                   ret);
          }
        break;

      /* Read current sample rate. Arg: uint32_t* pointer */

      case SNIOC_READ_SAMPLE_RATE:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          ret = __icm_read_sample_rate(priv);
          sninfo("SNIOC_READ_SAMPLE_RATE: Returns: %d. Read: %f\n",
                 ret, priv->sample_rate);
          *ptr = (uint32_t)priv->sample_rate;
          break;
        }

      /* Read current number of bytes in FIFO buffer. Arg: uint16_t* */

      case SNIOC_READ_FIFO_COUNT:
       {
          FAR uint16_t *ptr = (FAR uint16_t *)((uintptr_t)arg);
          uint16_t fifo_count = 0;
          ret = __icm_read_fifo_count(priv, &fifo_count);
          *ptr = fifo_count;
          sninfo("SNIOC_READ_FIFO_COUNT: Returns: %d. Read: 0x%x\n",
                 ret, fifo_count);
          break;
       }

      /* Enable or disable the use of FIFO buffer. Arg: bool value */

      case SNIOC_ENABLE_FIFO:
        if (!write_data)
          {
            /* Disable stream mode and clear all sensor feeds. */

            uint8_t fifo_mode = 0;
            ret = __icm_write_reg(priv, FIFO_CONFIG, &fifo_mode,
                                  sizeof(fifo_mode));
            if (ret < 0)
              {
                sninfo("SNIOC_ENABLE_FIFO failed. Returns: %d\n", ret);
              }

            ret = __icm_set_fifo(priv, 0);
            priv->fifo_enabled = false;
          }
        else
          {
            /* Put FIFO into stream (continuous) mode: bits 7:6 = 0b01. */

            uint8_t fifo_mode =
              TO_BITFIELD(FIFO_CONFIG__FIFO_MODE, 1);
            ret = __icm_write_reg(priv, FIFO_CONFIG, &fifo_mode,
                                  sizeof(fifo_mode));
            if (ret < 0)
              {
                sninfo("SNIOC_ENABLE_FIFO failed. Returns: %d\n", ret);
              }

            /* Enable temperature, accelerometer, and gyro in the FIFO.
             * Each packet = 14 bytes; 2 kB FIFO holds ~146 packets.
             */

            ret = __icm_set_fifo(priv,
                    FIFO_CONFIG1__FIFO_TEMP_EN |
                    FIFO_CONFIG1__FIFO_GYRO_EN |
                    FIFO_CONFIG1__FIFO_ACCEL_EN);
            priv->fifo_enabled = true;
          }

        sninfo("SNIOC_ENABLE_FIFO: %d Returns: %d\n", write_data, ret);
        break;

      default:
        sninfo("Unrecognized IOCTL command: 0x%04x\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm40609d_register
 *
 * Description:
 *   Registers the ICM-40609-D interface as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the interface to register. E.g., "/dev/imu0"
 *   config   - Configuration information (SPI bus + chip-select)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int icm40609d_register(FAR const char *path, FAR struct icm_config_s *config)
{
  FAR struct mpu_dev_s *priv;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  priv = kmm_malloc(sizeof(struct mpu_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate ICM-40609-D device instance\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(*priv));
  nxmutex_init(&priv->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  priv->config = *config;

  /* Reset the chip, to give it an initial configuration. */

  ret = mpu_reset(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure ICM-40609-D: %d\n", ret);

      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return ret;
    }

  /* Register the device node. */

  ret = register_driver(path, &g_mpu_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register ICM-40609-D interface: %d\n", ret);

      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return ret;
    }

  return OK;
}
