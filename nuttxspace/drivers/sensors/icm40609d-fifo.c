/*
  Driver for the ICM-40609-D IMU designed for 32kHz FIFO SPI burst reading
  ! This driver uses the legacy style of writing sensor drivers for NuttX
*/

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

//#include <nuttx/compiler.h> -> moved to <nuttx/sensors/icm40609d.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/icm40609d.h>
#include <nuttx/sensors/ioctl.h>

#include <nuttx/wqueue.h>     /* work_queue() bottom half */
#include <nuttx/circbuf.h>    /* ring buffer */
#include <nuttx/semaphore.h>
#include <poll.h>

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

/* FIFO readings */

#define ICM_FIFO_PACKET_SIZE   16    /* Packet 3: accel+gyro+temp+tmst   */
#define ICM_FIFO_MAX_BYTES     2080  /* datasheet-mandated allocation, §6.3 */
#define ICM_FIFO_HEADER_MSG    BIT(7)  /* 1 = FIFO empty / padding entry */
#define ICM_FIFO_HEADER_ACCEL  BIT(6)
#define ICM_FIFO_HEADER_GYRO   BIT(5)

#define ICM_NPOLLWAITERS       4

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


/****************************************************************************
 * Parsed sample — what actually goes in the ring buffer. Keep it
 * pre-converted (host byte order) rather than storing raw register bytes,
 * since the consumer shouldn't have to know the wire format.
 ****************************************************************************/

struct icm_fifo_sample_s
{
  int16_t  accel_x, accel_y, accel_z;
  int16_t  gyro_x, gyro_y, gyro_z;
  int8_t   temp;          /* FIFO temp is 8-bit; see datasheet conversion */
  uint16_t tmst;           /* raw ODR timestamp counter from FIFO */
};

/* Used by the driver to manage the device */

struct icm_dev_s
{
  mutex_t lock;               /* mutex for this structure */
  struct icm_config_s config; /* board-specific information */

  struct icm40609d_data_s buf;   /* temporary buffer (for read(), etc.) */
  size_t bufpos;              /* cursor into @buf, in bytes (!) */

  uint8_t gyro_odr;           /* gyro output data rate selector */
  uint8_t accel_odr;          /* accel output data rate selector */
  uint8_t afs_sel;            /* full scale range of the accelerometer */
  uint8_t dnf_config;         /* digital notch filter configuration */
  uint8_t daaf_config;        /* digital anti aliasing filter configuration */
  float sample_rate;          /* current sample rate */
  bool fifo_enabled;          /* current enable state of FIFO buffer */
  
  /* --- IRQ-driven FIFO streaming --- */
  struct circbuf_s    fifo_rb;        /* ring buffer of icm_fifo_sample_s */
  sem_t                rb_sem;         /* protects fifo_rb (NOT dev->lock) */
  struct work_s        fifo_work;      /* work_queue() job token */
  FAR struct pollfd   *fds[ICM_NPOLLWAITERS];
  uint16_t              watermark_bytes;
  bool                  streaming;     /* true once FIFO+IRQ both armed */

};

/****************************************************************************
 * icm_fifo_isr()
 *
 * Runs in actual interrupt context (or close to it, depending on board
 * GPIO IRQ implementation). MUST NOT perform SPI transactions — most
 * NuttX SPI drivers are not safe to call from hard-IRQ context, and even
 * if they were, you don't want SPI bus arbitration latency inside an ISR.
 *
 * Just schedule the bottom half and get out.
 ****************************************************************************/
 
static int icm_fifo_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct icm_dev_s *dev = (FAR struct icm_dev_s *)arg;
 
  /* If work is already queued, this is a no-op — that's fine, it means
   * the previous interrupt hasn't been serviced yet and the next service
   * pass will drain everything that's accumulated since.
   */
 
  work_queue(HPWORK, &dev->fifo_work, icm_fifo_worker, dev, 0);
 
  return OK;
}
 
/****************************************************************************
 * icm_fifo_worker()
 *
 * Bottom half. Runs in work-queue thread context — safe to call SPI,
 * take dev->lock, sleep, etc. This is where the actual acquisition
 * pipeline lives:
 *
 *   1. Read FIFO_COUNTH/L to find out how many bytes are waiting.
 *   2. Single burst SPI read of that many bytes from FIFO_DATA.
 *   3. Walk the burst buffer in ICM_FIFO_PACKET_SIZE strides, parsing
 *      each packet's header + payload.
 *   4. Push parsed samples into the ring buffer.
 *   5. Wake any blocked reader / poll() waiters.
 ****************************************************************************/
 
static void icm_fifo_worker(FAR void *arg)
{
  FAR struct icm_dev_s *dev = (FAR struct icm_dev_s *)arg;
  uint8_t raw[ICM_FIFO_MAX_BYTES];
  uint16_t fifo_bytes;
  uint16_t n_packets;
  uint16_t i;
  int ret;
 
  nxmutex_lock(&dev->lock);
 
  /* Step 1: how much is actually in the FIFO right now? Burst-reading a
   * fixed/maximal size every time wastes SPI bandwidth and time you don't
   * have at 32kHz — read the count first, then read exactly that much.
   */
 
  ret = __icm_read_fifo_count(dev, &fifo_bytes);
  if (ret < 0 || fifo_bytes == 0)
    {
      nxmutex_unlock(&dev->lock);
      return;
    }
 
  if (fifo_bytes > ICM_FIFO_MAX_BYTES)
    {
      /* Shouldn't happen given the datasheet's storage ceiling — if it
       * does, something upstream (servicing latency, SPI clock, watermark
       * sizing) is wrong. Clamp and flag rather than overrun the stack
       * buffer.
       */
 
      snerr("ERROR: FIFO count %u exceeds expected max %u — "
            "check watermark/latency budget\n",
            fifo_bytes, ICM_FIFO_MAX_BYTES);
      fifo_bytes = ICM_FIFO_MAX_BYTES;
    }
 
  /* Round down to a whole number of packets — a partial trailing packet
   * means we read mid-write; leave those bytes for next time rather than
   * mis-parsing them. (FIFO_RESUME_PARTIAL_RD in FIFO_CONFIG1 affects
   * this behavior — check it matches what you assume here.)
   */
 
  n_packets = fifo_bytes / ICM_FIFO_PACKET_SIZE;
  fifo_bytes = n_packets * ICM_FIFO_PACKET_SIZE;
 
  if (n_packets == 0)
    {
      nxmutex_unlock(&dev->lock);
      return;
    }
 
  /* Step 2: ONE burst SPI transaction for everything waiting. This is the
   * part that actually gives you the throughput to keep up with 32kHz —
   * not the interrupt model alone.
   */
 
  ret = __icm_read_reg(dev, FIFO_DATA, raw, fifo_bytes);
  if (ret < 0)
    {
      snerr("ERROR: FIFO burst read failed: %d\n", ret);
      nxmutex_unlock(&dev->lock);
      return;
    }
 
  nxmutex_unlock(&dev->lock);
 
  /* Step 3+4: parse and push. Done outside dev->lock since this only
   * touches the ring buffer (guarded by its own rb_sem), not the chip.
   */
 
  nxsem_wait_uninterruptible(&dev->rb_sem);
 
  for (i = 0; i < n_packets; i++)
    {
      FAR uint8_t *pkt = &raw[i * ICM_FIFO_PACKET_SIZE];
      uint8_t header = pkt[0];
      struct icm_fifo_sample_s sample;
 
      if (header & ICM_FIFO_HEADER_MSG)
        {
          /* Empty-FIFO padding marker — datasheet §6.2. Stop here; no
           * more real packets follow even if n_packets implied there
           * should be (count and content can race against the live
           * FIFO write pointer).
           */
 
          break;
        }
 
      if (!(header & ICM_FIFO_HEADER_ACCEL) ||
          !(header & ICM_FIFO_HEADER_GYRO))
        {
          /* We configured the FIFO for combined accel+gyro packets only.
           * Anything else means either a config mismatch or a corrupted
           * SPI transaction — don't silently misinterpret the remaining
           * bytes as if alignment is still good.
           */
 
          snerr("ERROR: Unexpected FIFO header 0x%02x at packet %u/%u — "
                "aborting parse, dropping rest of this burst\n",
                header, i, n_packets);
          break;
        }
 
      sample.accel_x = (int16_t)((pkt[1] << 8) | pkt[2]);
      sample.accel_y = (int16_t)((pkt[3] << 8) | pkt[4]);
      sample.accel_z = (int16_t)((pkt[5] << 8) | pkt[6]);
      sample.gyro_x  = (int16_t)((pkt[7] << 8) | pkt[8]);
      sample.gyro_y  = (int16_t)((pkt[9] << 8) | pkt[10]);
      sample.gyro_z  = (int16_t)((pkt[11] << 8) | pkt[12]);
      sample.temp    = (int8_t)pkt[13];
      sample.tmst    = (uint16_t)((pkt[14] << 8) | pkt[15]);
 
      ret = circbuf_write(&dev->fifo_rb, &sample, sizeof(sample));
      if (ret != sizeof(sample))
        {
          /* Ring buffer full — consumer isn't draining fast enough.
           * Decide deliberately whether you want overwrite-oldest or
           * drop-newest semantics here; circbuf's default behavior on a
           * full, non-overwriting buffer is to reject the write, which
           * is what's shown. Silently losing samples either way means
           * your consumer-side budget is the next thing to fix.
           */
 
          snwarn("WARNING: ring buffer full, dropping FIFO sample "
                 "(tmst=%u)\n", sample.tmst);
        }
    }
 
  nxsem_post(&dev->rb_sem);
 
  /* Step 5: wake anyone blocked in read() or poll(). */
 
  poll_notify(dev->fds, ICM_NPOLLWAITERS, POLLIN);
}
 
/****************************************************************************
 * icm_read() — replaces your current synchronous-SPI version.
 *
 * Now just drains the ring buffer. No SPI calls happen here at all — by
 * the time read() is called, the worker has already done the work.
 ****************************************************************************/
 
static ssize_t icm_read(FAR struct file *filep, FAR char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct icm_dev_s *dev = inode->i_private;
  ssize_t nread;
 
  /* Round len down to a whole number of samples — partial-sample reads
   * don't make sense for this interface and would desync the caller's
   * own framing if allowed.
   */
 
  len -= len % sizeof(struct icm_fifo_sample_s);
  if (len == 0)
    {
      return -EINVAL;
    }
 
  nxsem_wait_uninterruptible(&dev->rb_sem);
 
  if (circbuf_is_empty(&dev->fifo_rb))
    {
      nxsem_post(&dev->rb_sem);
 
      if (filep->f_oflags & O_NONBLOCK)
        {
          return -EAGAIN;
        }
 
      /* Blocking path: wait on a data-available semaphore posted by the
       * worker. (Omitted here for brevity — standard NuttX pattern:
       * nxsem_wait() on a dedicated dev->data_sem that icm_fifo_worker()
       * posts after circbuf_write(), mirroring poll_notify() above.)
       */
    }
 
  nread = circbuf_read(&dev->fifo_rb, buf, len);
  nxsem_post(&dev->rb_sem);
 
  return nread;
}
 
/****************************************************************************
 * icm_poll() — new file_operations entry, lets userspace select()/poll()
 * on the fd instead of busy-calling read(). Add this to g_icm_fops.
 ****************************************************************************/
 
static int icm_poll(FAR struct file *filep, FAR struct pollfd *fds,
                     bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct icm_dev_s *dev = inode->i_private;
  int ret = OK;
  int i;
 
  nxsem_wait_uninterruptible(&dev->rb_sem);
 
  if (setup)
    {
      for (i = 0; i < ICM_NPOLLWAITERS; i++)
        {
          if (dev->fds[i] == NULL)
            {
              dev->fds[i] = fds;
              fds->priv = &dev->fds[i];
              break;
            }
        }
 
      if (i == ICM_NPOLLWAITERS)
        {
          ret = -ENOSPC;
        }
      else if (!circbuf_is_empty(&dev->fifo_rb))
        {
          poll_notify(&fds, 1, POLLIN);
        }
    }
  else if (fds->priv != NULL)
    {
      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      *slot = NULL;
      fds->priv = NULL;
    }
 
  nxsem_post(&dev->rb_sem);
  return ret;
}
