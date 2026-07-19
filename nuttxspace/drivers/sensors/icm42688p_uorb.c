/*
  uORB driver for the ICM-42688-P IMU designed for 32kHz FIFO SPI burst
  reading. Registers two independent uORB topics (accel, gyro) off one
  physical chip -- see icm42688p_uorb.h for the registration entry point.
*/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <nuttx/debug.h>
#include <string.h>
#include <stdint.h>
#include <nuttx/bits.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <syslog.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/nuttx.h>       /* container_of() */

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/icm42688p_uorb.h>

#include <nuttx/wqueue.h>     /* work_queue() bottom half */
#include <math.h>

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

/* SPI bus frequency */

#define ICM_SPI_FREQ 10000000 // SPI frequency set to 10MHz (based on clock config /2)

/* FIFO readings */

#define ICM_FIFO_PACKET_SIZE   16    /* Packet 3: accel+gyro+temp+tmst   */
#define ICM_FIFO_MAX_BYTES     2080  /* datasheet-mandated allocation, §6.3 */
#define ICM_FIFO_MAX_RECS      ICM_FIFO_MAX_BYTES / ICM_FIFO_PACKET_SIZE  // Size of the FIFO buffer in number of samples
#define ICM_FIFO_HEADER_MSG    BIT(7)  /* 1 = FIFO empty / padding entry */
#define ICM_FIFO_HEADER_ACCEL  BIT(6)
#define ICM_FIFO_HEADER_GYRO   BIT(5)

#define FIFO_WM_REC_TH 50 // 50 : 32kHz -> 641Hz Default watermark threshold in number of samples

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ICM-42688-P register map (ICM-4x6xx family, Bank 0).
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
  FIFO_CONFIG__FIFO_MODE_STREAM = BIT(7),
  FIFO_CONFIG__FIFO_MODE_STOP = BIT(7) | BIT(6),

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
  PWR_MGMT0__GYRO_MODE_STANDBY = BIT(2),
  PWR_MGMT0__GYRO_MODE_LN = BIT(3) | BIT(2),  // Low noise mode
  PWR_MGMT0__ACCEL_MODE_LP = BIT(1),          // Low power mode
  PWR_MGMT0__ACCEL_MODE_LN = BIT(1) | 0x01,               // Low noise mode

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
   * ICM-42688-P also supports ±2000 dps via GYRO_CONFIG1 in Bank 1.
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
   *   100 = ±32 g  (ICM-42688-P only)
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

  WHO_AM_I = 0x75,   /* RO – 0x47 for ICM-42688-P */

  /* Register bank selection — accessible from all banks */

  REG_BANK_SEL = 0x76,
  REG_BANK_SEL__BANK_SEL__SHIFT = 0,
  REG_BANK_SEL__BANK_SEL__WIDTH = 3,

  /* ======================= Bank 1 ============================= *
   * Switch via REG_BANK_SEL before access; always restore Bank 0. */

  /* Gyro AAF enable/disable + notch-filter global disable share
   * STATIC2. Note both are active-low (1 = disabled).            */
  GYRO_CONFIG_STATIC2 = 0x0B,                          /* RW Bank 1, reset 0xA0 */
  GYRO_CONFIG_STATIC2__GYRO_AAF_DIS = BIT(1),          /* 1 = gyro AAF off  */
  GYRO_CONFIG_STATIC2__GYRO_NF_DIS  = BIT(0),          /* 1 = notch off     */

  /* Gyro AAF coefficients (unused by the enable/disable path,
   * listed so the map is complete for AAF-bandwidth work).       */
  GYRO_CONFIG_STATIC3 = 0x0C,                          /* RW Bank 1 – GYRO_AAF_DELT[5:0]     */
  GYRO_CONFIG_STATIC3__GYRO_AAF_DELT__SHIFT = 0,
  GYRO_CONFIG_STATIC3__GYRO_AAF_DELT__WIDTH = 6,
  GYRO_CONFIG_STATIC4 = 0x0D,                          /* RW Bank 1 – GYRO_AAF_DELTSQR[7:0]  */
  GYRO_CONFIG_STATIC5 = 0x0E,                          /* RW Bank 1                          */
  GYRO_CONFIG_STATIC5__GYRO_AAF_BITSHIFT__SHIFT = 4,
  GYRO_CONFIG_STATIC5__GYRO_AAF_BITSHIFT__WIDTH = 4,
  GYRO_CONFIG_STATIC5__GYRO_AAF_DELTSQR_HI__SHIFT = 0, /* DELTSQR[11:8] */
  GYRO_CONFIG_STATIC5__GYRO_AAF_DELTSQR_HI__WIDTH = 4,

  /* Gyro notch-filter coefficients. NF_COSWZ is 9 bits:
   *   [7:0] in STATIC6/7/8, [8] in STATIC9 (bits 0..2).          */
  GYRO_CONFIG_STATIC6 = 0x0F,   /* RW Bank 1 – GYRO_X_NF_COSWZ[7:0] */
  GYRO_CONFIG_STATIC7 = 0x10,   /* RW Bank 1 – GYRO_Y_NF_COSWZ[7:0] */
  GYRO_CONFIG_STATIC8 = 0x11,   /* RW Bank 1 – GYRO_Z_NF_COSWZ[7:0] */

  /* STATIC9 bit layout is identical to what your packing code
   * already assumes — only the address moved (0x0E -> 0x12):
   *   bit[0]=X_NF_COSWZ[8]  bit[1]=Y_NF_COSWZ[8]  bit[2]=Z_NF_COSWZ[8]
   *   bit[3]=X_NF_COSWZ_SEL bit[4]=Y_NF_COSWZ_SEL bit[5]=Z_NF_COSWZ_SEL */
  GYRO_CONFIG_STATIC9 = 0x12,   /* RW Bank 1 (factory-trimmed reset) */

  /* Gyro notch-filter bandwidth. On the 42688 this is its own
   * register (STATIC10), and the field is bits[6:4], NOT [5:3].
   *   0=1449 1=680 2=329 3=162 4=80 5=40 6=20 7=10 Hz            */
  GYRO_CONFIG_STATIC10 = 0x13,                         /* RW Bank 1, reset 0x11 */
  GYRO_CONFIG_STATIC10__GYRO_NF_BW_SEL__SHIFT = 4,
  GYRO_CONFIG_STATIC10__GYRO_NF_BW_SEL__WIDTH = 3,

  /* ======================= Bank 2 ============================= *
   * The accel static config lives in Bank 2 on the 42688 — NOT
   * Bank 1 as on your previous part. Switch to bank 2 for these. */

  ACCEL_CONFIG_STATIC2 = 0x03,                         /* RW Bank 2, reset 0x30 */
  ACCEL_CONFIG_STATIC2__ACCEL_AAF_DELT__SHIFT = 1,
  ACCEL_CONFIG_STATIC2__ACCEL_AAF_DELT__WIDTH = 6,
  ACCEL_CONFIG_STATIC2__ACCEL_AAF_DIS = BIT(0),        /* 1 = accel AAF off */
  ACCEL_CONFIG_STATIC3 = 0x04,                         /* RW Bank 2 – ACCEL_AAF_DELTSQR[7:0] */
  ACCEL_CONFIG_STATIC4 = 0x05,                         /* RW Bank 2                          */
  ACCEL_CONFIG_STATIC4__ACCEL_AAF_BITSHIFT__SHIFT = 4,
  ACCEL_CONFIG_STATIC4__ACCEL_AAF_BITSHIFT__WIDTH = 4,
  ACCEL_CONFIG_STATIC4__ACCEL_AAF_DELTSQR_HI__SHIFT = 0,
  ACCEL_CONFIG_STATIC4__ACCEL_AAF_DELTSQR_HI__WIDTH = 4,

  /* User offset trim registers (Bank 4).
   * Offsets are 12-bit two's complement, split across adjacent bytes:
   *   OFFSET_USER0  GX[7:0]
   *   OFFSET_USER1  GY[11:8] | GX[11:8]
   *   OFFSET_USER2  GY[7:0]
   *   OFFSET_USER3  GZ[7:0]
   *   OFFSET_USER4  AX[11:8] | GZ[11:8]   ← shared, requires RMW
   *   OFFSET_USER5  AX[7:0]
   *   OFFSET_USER6  AY[7:0]
   *   OFFSET_USER7  AZ[11:8] | AY[11:8]
   *   OFFSET_USER8  AZ[7:0]
   */

  OFFSET_USER0 = 0x77,  /* RW Bank 4 – GX_OFF_USR[7:0]                      */
  OFFSET_USER1 = 0x78,  /* RW Bank 4 – GY_OFF_USR[11:8] | GX_OFF_USR[11:8]  */
  OFFSET_USER2 = 0x79,  /* RW Bank 4 – GY_OFF_USR[7:0]                      */
  OFFSET_USER3 = 0x7A,  /* RW Bank 4 – GZ_OFF_USR[7:0]                      */
  OFFSET_USER4 = 0x7B,  /* RW Bank 4 – AX_OFF_USR[11:8] | GZ_OFF_USR[11:8]  */
  OFFSET_USER5 = 0x7C,  /* RW Bank 4 – AX_OFF_USR[7:0]                      */
  OFFSET_USER6 = 0x7D,  /* RW Bank 4 – AY_OFF_USR[7:0]                      */
  OFFSET_USER7 = 0x7E,  /* RW Bank 4 – AZ_OFF_USR[11:8] | AY_OFF_USR[11:8]  */
  OFFSET_USER8 = 0x7F,  /* RW Bank 4 – AZ_OFF_USR[7:0]                      */
};

/* One of these per uORB topic (accel, gyro) -- shares the physical chip
 * state in icm_dev_s via the dev back-pointer.
 */

struct icm_sens_s
{
  struct sensor_lowerhalf_s lower;
  FAR struct icm_dev_s *dev;
};

/* Used by the driver to manage the shared physical chip. One physical
 * FIFO/IRQ/ODR-register-pair is shared by both the accel and gyro
 * uORB topics -- see icm_activate()/icm_set_interval()/icm_batch() for
 * how the two independent topics' requests get reconciled onto this
 * single piece of hardware.
 */

struct icm_dev_s
{
  mutex_t lock;               /* mutex for this structure */
  struct icm_config_s config; /* board-specific information */

  struct icm_sens_s accel;
  struct icm_sens_s gyro;

  /* --- IMU configurations --- */
  uint8_t gyro_odr;           /* gyro output data rate selector */
  uint8_t accel_odr;          /* accel output data rate selector */
  uint8_t acc_fs_sel;         /* full scale range of the accelerometer */
  uint8_t gyro_fs_sel;        /* full scale range of the gyro */
  uint8_t dnf_config;         /* digital notch filter configuration */
  uint8_t daaf_config;        /* digital anti aliasing filter configuration */
  bool dnf_active;
  bool aaf_active;
  float accel_scale;          /* m/s^2 per LSB, current acc_fs_sel */
  float gyro_scale;           /* rad/s per LSB, current gyro_fs_sel */
  float sample_rate;          /* current shared ODR, Hz */
  uint32_t sample_period_us;  /* current shared ODR, us (1e6 / sample_rate) */

  /* --- shared activation across the two topics --- */
  int refcount;                /* accel active? + gyro active?, 0..2 */
  bool streaming;               /* true once FIFO+IRQ both armed */

  /* --- per-topic requests, reconciled onto the one physical FIFO --- */
  uint32_t accel_period_us;    /* last period requested via accel set_interval() */
  uint32_t gyro_period_us;     /* last period requested via gyro set_interval()  */
  uint32_t accel_latency_us;   /* last latency requested via accel batch(), 0=unset */
  uint32_t gyro_latency_us;    /* last latency requested via gyro batch(), 0=unset  */
  uint16_t watermark_samples;  /* currently programmed shared FIFO watermark */

  /* --- IRQ-driven FIFO streaming --- */
  struct work_s        fifo_work;      /* work_queue() job token */
  uint64_t              isr_timestamp_us; /* wall-clock anchor, captured in ISR */

  /* Burst-read/parse scratch. ICM_FIFO_MAX_BYTES (2080) alone exceeds
   * the default HPWORK stack (CONFIG_SCHED_HPWORKSTACKSIZE, commonly
   * 2048) — these must live off the worker's stack, not as locals.
   * Only ever touched from icm_fifo_worker(), which runs one job at a
   * time on HPWORK.
   */

  uint8_t              fifo_raw[ICM_FIFO_MAX_BYTES];
  struct sensor_accel  accel_burst[ICM_FIFO_MAX_RECS];
  struct sensor_gyro   gyro_burst[ICM_FIFO_MAX_RECS];
};

/* Maps a requested ODR period (us) to the nearest supported hardware
 * rate. Ordered fastest (smallest period) to slowest (largest period).
 */

struct icm_odr_entry_s
{
  uint32_t period_us;
  uint32_t hz;
  uint8_t  odr_sel;
};

static const struct icm_odr_entry_s g_icm_odr_map[] =
{
  { 31,   32000, 0x01 },
  { 63,   16000, 0x02 },
  { 125,  8000,  0x03 },
  { 250,  4000,  0x04 },
  { 500,  2000,  0x05 },
  { 1000, 1000,  0x06 },
  { 2000, 500,   0x0f },
};

#define ICM_NUM_ODR (sizeof(g_icm_odr_map) / sizeof(g_icm_odr_map[0]))

/* Accel/gyro raw-LSB-to-physical-unit scale factors, indexed by the
 * chip's ACCEL_FS_SEL / GYRO_FS_SEL selector values (see
 * icm_set_accel_fsr()/icm_set_gyro_fsr()).
 */

static const float g_icm_accel_scale[] =   /* m/s^2 per LSB */
{
  16.0f * 9.80665f / 32768.0f,  /* sel 0: +-16g */
  8.0f  * 9.80665f / 32768.0f,  /* sel 1: +-8g  */
  4.0f  * 9.80665f / 32768.0f,  /* sel 2: +-4g  */
  2.0f  * 9.80665f / 32768.0f,  /* sel 3: +-2g  */
};

static const float g_icm_gyro_scale[] =    /* rad/s per LSB */
{
  2000.0f * (float)M_PI / 180.0f / 32768.0f,  /* sel 0: +-2000dps */
  1000.0f * (float)M_PI / 180.0f / 32768.0f,  /* sel 1: +-1000dps */
  500.0f  * (float)M_PI / 180.0f / 32768.0f,  /* sel 2: +-500dps  */
  250.0f  * (float)M_PI / 180.0f / 32768.0f,  /* sel 3: +-250dps  */
  125.0f  * (float)M_PI / 180.0f / 32768.0f,  /* sel 4: +-125dps  */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int icm_activate(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, bool enable);
static int icm_set_interval(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR uint32_t *period_us);
static int icm_batch(FAR struct sensor_lowerhalf_s *lower,
                     FAR struct file *filep,
                     FAR uint32_t *latency_us);
static int icm_control(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, int cmd, unsigned long arg);

static int icm_fifo_isr(int irq, FAR void *context, FAR void *arg);
static void icm_fifo_worker(FAR void *arg);
static int icm_fifo_start(FAR struct icm_dev_s *dev);
static int icm_fifo_stop(FAR struct icm_dev_s *dev);

static int icm_set_gyro_fsr(FAR struct icm_dev_s *dev, uint16_t dps);
static int icm_set_accel_fsr(FAR struct icm_dev_s *dev, uint16_t g);
static int icm_program_odr(FAR struct icm_dev_s *dev, uint32_t period_us);
static int icm_program_watermark(FAR struct icm_dev_s *dev, uint16_t samples);
static int icm_get_lost_packets(FAR struct icm_dev_s *dev, FAR uint32_t *count);
static int icm_set_gyro_dnf_freq(FAR struct icm_dev_s *dev, uint32_t hz);
static int icm_set_gyro_dnf_bandwidth(FAR struct icm_dev_s *dev, uint32_t hz);
static int icm_set_gyro_offset(FAR struct icm_dev_s *dev, FAR const struct icm42688p_offset_s *off);
static int icm_set_accel_offset(FAR struct icm_dev_s *dev, FAR const struct icm42688p_offset_s *off);
static int icm_set_gyro_dnf_en(FAR struct icm_dev_s *dev, bool en);
static int icm_set_accel_aaf_en(FAR struct icm_dev_s *dev, bool en);
static int icm_set_gyro_dnf_freq_x(FAR struct icm_dev_s *dev, uint32_t hz);
static int icm_set_gyro_dnf_freq_y(FAR struct icm_dev_s *dev, uint32_t hz);
static int icm_set_gyro_dnf_freq_z(FAR struct icm_dev_s *dev, uint32_t hz);
static int icm_set_gyro_ui_filt_ord(FAR struct icm_dev_s *dev, uint8_t order);
static int icm_set_accel_ui_filt_ord(FAR struct icm_dev_s *dev, uint8_t order);
static int icm_set_gyro_ui_filt_bw(FAR struct icm_dev_s *dev, uint8_t bw);
static int icm_set_accel_ui_filt_bw(FAR struct icm_dev_s *dev, uint8_t bw);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_icm_ops =
{
  .activate     = icm_activate,
  .set_interval = icm_set_interval,
  .batch        = icm_batch,
  .control      = icm_control,
};

/* __icm_read_reg(), but for SPI-connected devices. See that function
 * for documentation.
 */

static int __icm_read_reg_spi(FAR struct icm_dev_s *dev, enum icm_regaddr_e reg_addr, FAR uint8_t *buf, uint8_t len)
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
  SPI_SETMODE(spi, SPIDEV_MODE3); //CPOL = 1 (High), CPHA = 1 (2 Edges)
  SPI_SETFREQUENCY(spi, ICM_SPI_FREQ);


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

static int __icm_write_reg_spi(FAR struct icm_dev_s *dev, enum icm_regaddr_e reg_addr, FAR const uint8_t * buf, uint8_t len)
{
  int ret;
  FAR struct spi_dev_s *spi = dev->config.spi;
  int id = dev->config.spi_devid;

  /* Hopefully, we'll return all the bytes they're asking for. */

  ret = len;

  /* Grab and configure the SPI master device. */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE3); //CPOL = 1 (High), CPHA = 1 (2 Edges)
  SPI_SETFREQUENCY(spi, ICM_SPI_FREQ); // SPI frequency set to 2MHz (based on clock config)

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

static inline int __icm_read_reg(FAR struct icm_dev_s *dev, enum icm_regaddr_e reg_addr, FAR uint8_t *buf, uint8_t len)
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

static inline int __icm_write_reg(FAR struct icm_dev_s *dev,  enum icm_regaddr_e reg_addr, FAR const uint8_t *buf, uint8_t len)
{
  /* If we're connected to SPI, use that function. */

  if (dev->config.spi != NULL)
    {
      return __icm_write_reg_spi(dev, reg_addr, buf, len);
    }

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* __icm_set_bank()
 *
 * Switches the active register bank via REG_BANK_SEL.
 * Caller must hold dev->lock.
 * Always restore Bank 0 after accessing Bank 1/2/4 registers.
 */

static inline int __icm_set_bank(FAR struct icm_dev_s *dev, uint8_t bank)
{
  return __icm_write_reg(dev, REG_BANK_SEL, &bank, 1);
}

/* __icm_read_fifo_count()
 *
 * Reads how many bytes are available in the IMU FIFO
 *
 * Note: This prevents reading too many bytes from the register than necessary
 *
 * Returns 0 uppon success or a negative errno.
 * Updates the fifo_bytes variable with the actual count
 */

static int __icm_read_fifo_count(FAR struct icm_dev_s *dev, u_int16_t *fifo_bytes)
{
  int ret;
  uint8_t fifo_counter[2];
  ret = __icm_read_reg(dev, FIFO_COUNTH, fifo_counter, sizeof(fifo_counter));
  if (ret < 0)
    {
      snerr("ERROR: Failed to read FIFO counter\n");
      *fifo_bytes = 0;
    }
  else
    {
      *fifo_bytes = (fifo_counter[0] << 8) | fifo_counter[1];
    }

  return ret;
}

/****************************************************************************
 * icm_reset()
 *
 * Resets the IMU signal path / FIFO buffer
 * Configures the little-endian convention
 * Turn Gyro and Accel back on (in Low Noise mode)
 *
 ****************************************************************************/

static int icm_reset(FAR struct icm_dev_s *dev)
{
  int ret = OK;

  /* Signal reset handling:
   * SIGNAL_PATH_RESET__ABORT_AND_RESET = 1     => Reset the entire signal path
   * SIGNAL_PATH_RESET__TMST_STROBE = 0
   * SIGNAL_PATH_RESET__FIFO_FLUSH = 1          => Flush FIFO buffer
   */
  const uint8_t signal_path_reset = SIGNAL_PATH_RESET__ABORT_AND_RESET |SIGNAL_PATH_RESET__FIFO_FLUSH ;

  /* Endian connfig and records count
   * INTF_CONFIG0__FIFO_HOLD_LAST_DATA_EN = 1
   * INTF_CONFIG0__FIFO_COUNT_ENDIAN = 0        => STM32s are strictly little-endian so we reconfigure the IMU like so
   * INTF_CONFIG0__SENSOR_DATA_ENDIAN = 0
   * INTF_CONFIG0__FIFO_COUNT_REC = 1           => We also configure FIFO_COUNT in samples instead of bytes
   */
  const uint8_t intf_config0 = INTF_CONFIG0__FIFO_HOLD_LAST_DATA_EN | INTF_CONFIG0__FIFO_COUNT_REC;

  /* 6-axis low-noise mode: gyro LN + accel LN */
  const uint8_t pwr_mgmt0 = PWR_MGMT0__GYRO_MODE_LN | PWR_MGMT0__ACCEL_MODE_LN;

  /* INT_CONFIG: reset default is open-drain + active-low (0x00). Our board
   * wiring (GPIO_IMU1_INT/GPIO_IMU2_INT: floating input, rising-edge EXTI
   * trigger) assumes push-pull + active-high, so reconfigure INT1 to match.
   */
  const uint8_t int_config = INT_CONFIG__INT1_DRIVE_CIRCUIT |
                             INT_CONFIG__INT1_POLARITY;

  /* INT_CONFIG1: reset default (0x10) leaves INT_ASYNC_RESET set, which
   * the datasheet says must be cleared "for proper INT1 and INT2 pin
   * operation". The other two bits already default to 0.
   */
  const uint8_t int_config1 = 0x00;

  nxmutex_lock(&dev->lock);

  /* Sanity-check the SPI link before touching anything else: if this
   * comes back wrong (0x00, 0xff, or anything but 0x47), the chip isn't
   * responding at all — wrong CS/mode/clock/wiring — and every write
   * below is going nowhere.
   */

  uint8_t whoami = 0;
  ret = __icm_read_reg(dev, WHO_AM_I, &whoami, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  if (whoami != 0x47)
    {
      snerr("ERROR: ICM-42688-P WHO_AM_I mismatch: got 0x%02x, expected 0x47 "
            "— chip not responding on SPI\n", whoami);
      nxmutex_unlock(&dev->lock);
      return -ENODEV;
    }

  ret = __icm_write_reg(dev, SIGNAL_PATH_RESET, &signal_path_reset, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  up_mdelay(1);   /* wait for reset to complete */

  ret = __icm_write_reg(dev, INTF_CONFIG0, &intf_config0, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  ret = __icm_write_reg(dev, PWR_MGMT0, &pwr_mgmt0, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  ret = __icm_write_reg(dev, INT_CONFIG, &int_config, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  ret = __icm_write_reg(dev, INT_CONFIG1, &int_config1, 1);

  nxmutex_unlock(&dev->lock);

  up_udelay(200);   /* datasheet: ≥200µs before issuing any register writes */

  return ret;
}

/****************************************************************************
 * icm_program_watermark()
 *
 * Writes the FIFO watermark threshold registers and updates
 * dev->watermark_samples. Caller must hold dev->lock. This backs both
 * icm_fifo_start() (initial/current value) and icm_batch() (live
 * reprogramming while streaming, no restart needed).
 ****************************************************************************/

static int icm_program_watermark(FAR struct icm_dev_s *dev, uint16_t samples)
{
  uint8_t wm[2];

  wm[0] = (uint8_t)(samples & 0xff);
  wm[1] = (uint8_t)((samples >> 8) & 0x0f);

  dev->watermark_samples = samples;
  return __icm_write_reg(dev, FIFO_CONFIG2, wm, 2);
}

/****************************************************************************
 * icm_program_odr()
 *
 * Snaps @period_us to the nearest supported hardware rate (never
 * slower than requested) and writes it to both GYRO_CONFIG0 and
 * ACCEL_CONFIG0 (one shared ODR field pair drives both sensor paths).
 * Updates dev->{gyro,accel}_odr and dev->sample_rate/sample_period_us.
 * Caller must hold dev->lock.
 ****************************************************************************/

static int icm_program_odr(FAR struct icm_dev_s *dev, uint32_t period_us)
{
  uint8_t greg, areg;
  int ret;
  size_t i;

  for (i = ICM_NUM_ODR - 1; i > 0; i--)
    {
      if (period_us >= g_icm_odr_map[i].period_us)
        {
          break;
        }
    }

  ret = __icm_read_reg(dev, GYRO_CONFIG0, &greg, 1);
  if (ret < 0)
    {
      return ret;
    }

  greg = (uint8_t)((greg & ~(0x0fu << GYRO_CONFIG0__GYRO_ODR__SHIFT)) |
                   ((g_icm_odr_map[i].odr_sel & 0x0fu) << GYRO_CONFIG0__GYRO_ODR__SHIFT));
  ret = __icm_write_reg(dev, GYRO_CONFIG0, &greg, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = __icm_read_reg(dev, ACCEL_CONFIG0, &areg, 1);
  if (ret < 0)
    {
      return ret;
    }

  areg = (uint8_t)((areg & ~(0x0fu << ACCEL_CONFIG0__ACCEL_ODR__SHIFT)) |
                   ((g_icm_odr_map[i].odr_sel & 0x0fu) << ACCEL_CONFIG0__ACCEL_ODR__SHIFT));
  ret = __icm_write_reg(dev, ACCEL_CONFIG0, &areg, 1);
  if (ret >= 0)
    {
      dev->gyro_odr         = g_icm_odr_map[i].odr_sel;
      dev->accel_odr        = g_icm_odr_map[i].odr_sel;
      dev->sample_rate      = (float)g_icm_odr_map[i].hz;
      dev->sample_period_us = g_icm_odr_map[i].period_us;
    }

  return ret;
}

/****************************************************************************
 * icm_fifo_start()
 *
 * Configure the required FIFO registers
 * Attach and enable the watermark interrupt and
 * ties it to our interrupt handling function
 *
 ****************************************************************************/

static int icm_fifo_start(FAR struct icm_dev_s *dev)
{
  int ret = OK;
  // Enable FIFO stream mode (no stop when full)
  const uint8_t fifo_config = FIFO_CONFIG__FIFO_MODE_STREAM;

  /* Timestamp configuration for the FIFO output
   * TMST_TO_REGS_EN = 0              => we don't manually read the tmst in TMST_VALUE
   * TMST_CONFIG__TMST_RES = 1        => resolution = 1us
   * TMST_CONFIG__TMST_DELTA_EN = 0
   * TMST_CONFIG__TMST_FSYNC = 0
   * TMST_CONFIG__TMST_EN = 1
   */
  const uint8_t tmst_config = TMST_CONFIG__TMST_RES | TMST_CONFIG__TMST_EN;

  /* FIFO configuration:
   * FIFO_CONFIG1__FIFO_RESUME_PARTIAL_RD = 1   => enables to read only a part of the buffer
   * FIFO_CONFIG1__FIFO_WM_GT_TH = 1            => enables the trigger when reaching the watermark
   * FIFO_CONFIG1__FIFO_TMST_FSYNC_EN = 0       => we don't use the FSYNC feature (no external sync trigger)
   * FIFO_CONFIG1__FIFO_TEMP_EN = 1             => should be enabled by default when using gyro + acc
   * FIFO_CONFIG1__FIFO_GYRO_EN = 1
   * FIFO_CONFIG1__FIFO_ACCEL_EN = 1
   */
  const uint8_t fifo_config1 = FIFO_CONFIG1__FIFO_RESUME_PARTIAL_RD | FIFO_CONFIG1__FIFO_WM_GT_TH |
                               FIFO_CONFIG1__FIFO_TEMP_EN |FIFO_CONFIG1__FIFO_GYRO_EN | FIFO_CONFIG1__FIFO_ACCEL_EN;

  /* Interrupt output 1 configuration
   * INT_SOURCE0__FIFO_THS_INT1_EN = 1        => configure INT1 on the FIFO watermark threshold
   * INT_SOURCE0__FIFO_FULL_INT1_EN = 0       => we don't trigger on full FIFO
   */
  const uint8_t int_source0 = INT_SOURCE0__FIFO_THS_INT1_EN;

  nxmutex_lock(&dev->lock);

  ret = __icm_write_reg(dev, FIFO_CONFIG, &fifo_config, 1);
  if (ret < 0)  // if an error occured, directly abort and return it
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  ret = __icm_write_reg(dev, TMST_CONFIG, &tmst_config, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  ret = __icm_write_reg(dev, FIFO_CONFIG1, &fifo_config1, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  /* Program whatever watermark is currently in effect -- either the
   * driver-default set at registration, or a value already requested
   * live via icm_batch() before this (re)activation. Don't reset it
   * back to a hardcoded default here: the other topic may depend on
   * a watermark it already negotiated.
   */

  ret = icm_program_watermark(dev, dev->watermark_samples);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  ret = __icm_write_reg(dev, INT_SOURCE0, &int_source0, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  // Attach and enable the interrupt on the board-configured INT1 line
  irq_attach(dev->config.irq, icm_fifo_isr, dev);
  syslog(LOG_INFO, "icm_fifo_start: irq attached\n");
  up_enable_irq(dev->config.irq);
  syslog(LOG_INFO, "icm_fifo_start: irq enabled\n");

  dev->streaming = true; // Signals the FIFO+IRQ are armed and streaming

  nxmutex_unlock(&dev->lock);

  return ret;
}

/****************************************************************************
 * icm_fifo_stop()
 *
 * Disarm the FIFO and IRQ
 *
 *
 ****************************************************************************/

static int icm_fifo_stop(FAR struct icm_dev_s *dev)
{
  int ret;
  const uint8_t zeroes = 0x00;

  /* Disarm first — prevents a race where the ISR fires between disabling
   * the FIFO and detaching the handler.
   */

  up_disable_irq(dev->config.irq);
  irq_detach(dev->config.irq);

  nxmutex_lock(&dev->lock);
  /* Clear INT1 FIFO threshold routing. */

  ret = __icm_write_reg(dev, INT_SOURCE0, &zeroes, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  /* Disable sensor enables in FIFO. */

  ret = __icm_write_reg(dev, FIFO_CONFIG1, &zeroes, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  /* Put FIFO back to bypass mode (clears stream bit). */

  ret = __icm_write_reg(dev, FIFO_CONFIG, &zeroes, 1);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->lock);
      return ret;
    }

  dev->streaming = false;

  nxmutex_unlock(&dev->lock);

  return OK;
}

/****************************************************************************
 * icm_fifo_isr()
 *
 * Runs in actual interrupt context (or close to it, depending on board
 * GPIO IRQ implementation). MUST NOT perform SPI transactions — most
 * NuttX SPI drivers are not safe to call from hard-IRQ context, and even
 * if they were, you don't want SPI bus arbitration latency inside an ISR.
 *
 * Note: This interrupt will be triggered byt the FIFO watermark interrupt,
 * once the bytes in FIFO > Water mark count
 *
 * Captures the wall-clock anchor used to reconstruct per-sample
 * timestamps (see icm_fifo_worker()), then schedules the bottom half
 * and gets out.
 ****************************************************************************/

static int icm_fifo_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct icm_dev_s *dev = (FAR struct icm_dev_s *)arg;

  /* Must happen before returning: this driver attaches directly to the
   * raw IRQ vector, bypassing the arch's own dispatcher that would
   * normally clear this — skip it and the IRQ re-fires the instant this
   * ISR returns, forever. See icm_config_s.irq_ack's doc comment.
   */

  dev->config.irq_ack();

  /* Anchor for the per-sample timestamp reconstruction done in
   * icm_fifo_worker() -- captured here, as close as possible to the
   * moment the watermark condition actually latched in hardware, to
   * minimize the jitter that HPWORK scheduling latency would add.
   */

  dev->isr_timestamp_us = sensor_get_timestamp();

  /* If work is already queued, this is a no-op — that's fine, it means
   * the previous interrupt hasn't been serviced yet and the next service
   * pass will drain everything that's accumulated since.
   */

  work_queue(HPWORK, &dev->fifo_work, icm_fifo_worker, dev, 0);

  return 0;
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
 *      each packet's header + payload into sensor_accel/sensor_gyro
 *      arrays (physical units, FSR-scaled).
 *   4. Reconstruct each sample's timestamp from the ISR wall-clock
 *      anchor and the FIFO's own tmst counter deltas.
 *   5. Push the whole burst to each uORB topic in one push_event()
 *      call -- the upper half fans it out to every subscriber at
 *      their own decimated rate.
 ****************************************************************************/

static void icm_fifo_worker(FAR void *arg)
{
  FAR struct icm_dev_s *dev = (FAR struct icm_dev_s *)arg;
  uint16_t fifo_packets;
  uint16_t n_bytes;
  uint16_t n_valid;
  uint16_t i;
  int ret;
  float accel_scale;
  float gyro_scale;
  uint64_t base_ts;
  static bool s_first_isr_logged = false;

  if (!s_first_isr_logged)
    {
      /* Confirms the EXTI/NVIC chain actually delivered an interrupt and
       * icm_fifo_isr() scheduled this worker — logged once so we know the
       * interrupt path itself is alive, independent of whether the FIFO
       * happens to have data by the time we get here.
       */

      syslog(LOG_INFO, "icm_fifo_worker: first watermark interrupt serviced\n");
      s_first_isr_logged = true;
    }

  nxmutex_lock(&dev->lock);

  /* Step 1: how much is actually in the FIFO right now? Burst-reading a
   * fixed/maximal size every time wastes SPI bandwidth and time you don't
   * have at 32kHz — read the count first, then read exactly that much.
   */

  ret = __icm_read_fifo_count(dev, &fifo_packets);
  if (ret < 0 || fifo_packets == 0)
    {
      nxmutex_unlock(&dev->lock);   // If no packets available in FIFO or an error occured we end the worker
      return;
    }

  if (fifo_packets > ICM_FIFO_MAX_RECS)
    {
      /* Shouldn't happen given the datasheet's storage ceiling — if it
       * does, something upstream (servicing latency, SPI clock, watermark
       * sizing) is wrong. Clamp and flag rather than overrun the stack
       * buffer.
       */

      snerr("ERROR: FIFO count %u exceeds expected max %u — check watermark/latency budget\n",
            fifo_packets, ICM_FIFO_MAX_RECS);
      fifo_packets = ICM_FIFO_MAX_RECS;
    }

  /* Step 2: ONE burst SPI transaction for everything waiting. This is the
   * part that actually gives you the throughput to keep up with 32kHz —
   * not the interrupt model alone.
   */
  n_bytes = fifo_packets*ICM_FIFO_PACKET_SIZE;

  ret = __icm_read_reg(dev, FIFO_DATA, dev->fifo_raw, n_bytes);
  if (ret < 0)
    {
      snerr("ERROR: FIFO burst read failed: %d\n", ret);
      nxmutex_unlock(&dev->lock);
      return;
    }

  accel_scale = dev->accel_scale;
  gyro_scale  = dev->gyro_scale;
  base_ts     = dev->isr_timestamp_us;

  nxmutex_unlock(&dev->lock);

  /* Step 3: parse every packet (accel+gyro always paired per record)
   * into physical-unit samples. Done outside dev->lock since this only
   * touches the per-worker scratch arrays, not the chip.
   */

  n_valid = 0;

  for (i = 0; i < fifo_packets; i++)
    {
      FAR uint8_t *pkt = &dev->fifo_raw[i * ICM_FIFO_PACKET_SIZE];
      uint8_t header = pkt[0];
      int16_t raw;
      float temp_c;

      if (header & ICM_FIFO_HEADER_MSG)
        {
          /* Empty-FIFO padding marker — datasheet §6.2. Stop here; no
           * more real packets follow even if fifo_packets implied there
           * should be (count and content can race against the live
           * FIFO write pointer).
           */

          break;
        }

      if (!(header & ICM_FIFO_HEADER_ACCEL) || !(header & ICM_FIFO_HEADER_GYRO))
        {
          /* We configured the FIFO for combined accel+gyro packets only.
           * Anything else means either a config mismatch or a corrupted
           * SPI transaction — don't silently misinterpret the remaining
           * bytes as if alignment is still good.
           */

          snerr("ERROR: Unexpected FIFO header 0x%02x at packet %u/%u — aborting parse, dropping rest of this burst\n",
                header, i, fifo_packets);
          break;
        }

      raw = (int16_t)((pkt[1] << 8) | pkt[2]);
      dev->accel_burst[n_valid].x = (float)raw * accel_scale;
      raw = (int16_t)((pkt[3] << 8) | pkt[4]);
      dev->accel_burst[n_valid].y = (float)raw * accel_scale;
      raw = (int16_t)((pkt[5] << 8) | pkt[6]);
      dev->accel_burst[n_valid].z = (float)raw * accel_scale;

      raw = (int16_t)((pkt[7] << 8) | pkt[8]);
      dev->gyro_burst[n_valid].x = (float)raw * gyro_scale;
      raw = (int16_t)((pkt[9] << 8) | pkt[10]);
      dev->gyro_burst[n_valid].y = (float)raw * gyro_scale;
      raw = (int16_t)((pkt[11] << 8) | pkt[12]);
      dev->gyro_burst[n_valid].z = (float)raw * gyro_scale;

      temp_c = (float)(int8_t)pkt[13] / 2.07f + 25.0f;
      dev->accel_burst[n_valid].temperature = temp_c;
      dev->gyro_burst[n_valid].temperature  = temp_c;

      n_valid++;
    }

  if (n_valid == 0)
    {
      return;
    }

  /* Step 4: per-sample timestamp reconstruction. Anchor the LAST valid
   * sample to the wall-clock time captured in the ISR, then walk
   * backward using the FIFO's own tmst counter for inter-sample
   * spacing. tmst is a 16-bit, 1us-resolution, free-running counter
   * (TMST_RES set, TMST_DELTA_EN clear in icm_fifo_start()) -- it has
   * no relationship to the OS clock domain, only to itself, so it
   * can't populate `timestamp` directly. It wraps every 65536us; at the
   * slowest supported ODR (500Hz = 2ms/sample) a full watermark burst
   * can span more than one wrap, so deltas MUST be computed between
   * ADJACENT samples only (each gap is at most one ODR period, always
   * far inside the wrap window) and accumulated -- never as a single
   * first-vs-last subtraction across the whole burst, which would
   * alias across a multi-wrap span.
   */

  {
    uint64_t ts = base_ts;
    int j;

    dev->accel_burst[n_valid - 1].timestamp = ts;
    dev->gyro_burst[n_valid - 1].timestamp  = ts;

    for (j = (int)n_valid - 2; j >= 0; j--)
      {
        FAR uint8_t *pkt_next = &dev->fifo_raw[(j + 1) * ICM_FIFO_PACKET_SIZE];
        FAR uint8_t *pkt_this = &dev->fifo_raw[j * ICM_FIFO_PACKET_SIZE];
        uint16_t tmst_next = (uint16_t)((pkt_next[14] << 8) | pkt_next[15]);
        uint16_t tmst_this = (uint16_t)((pkt_this[14] << 8) | pkt_this[15]);
        uint16_t delta_ticks = (uint16_t)(tmst_next - tmst_this);

        ts -= delta_ticks;   /* TMST_RES=1 => 1 tick = 1 us */

        dev->accel_burst[j].timestamp = ts;
        dev->gyro_burst[j].timestamp  = ts;
      }
  }

  /* Step 5: push the whole burst to each topic in one call -- the
   * upper half fans it out to every subscriber at their own decimated
   * rate (see icm42688p_uorb.c design notes / sensor_push_event()).
   */

  dev->accel.lower.push_event(dev->accel.lower.priv, dev->accel_burst,
                              n_valid * sizeof(struct sensor_accel));

  dev->gyro.lower.push_event(dev->gyro.lower.priv, dev->gyro_burst,
                             n_valid * sizeof(struct sensor_gyro));
}

/****************************************************************************
 * Configuration helper functions (called from icm_control)
 ****************************************************************************/

static int icm_set_gyro_fsr(FAR struct icm_dev_s *dev, uint16_t dps)
{
  uint8_t fsr_sel;
  uint8_t reg;
  int ret;

  switch (dps)
    {
      case 2000: fsr_sel = 0; break;
      case 1000: fsr_sel = 1; break;
      case 500:  fsr_sel = 2; break;
      case 250:  fsr_sel = 3; break;
      case 125:  fsr_sel = 4; break;
      default:   return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = __icm_read_reg(dev, GYRO_CONFIG0, &reg, 1);
  if (ret < 0) goto out;

  reg = (uint8_t)((reg & ~(0x07u << GYRO_CONFIG0__GYRO_FS_SEL__SHIFT)) |
                  ((fsr_sel & 0x07u) << GYRO_CONFIG0__GYRO_FS_SEL__SHIFT));
  ret = __icm_write_reg(dev, GYRO_CONFIG0, &reg, 1);
  if (ret >= 0)
    {
      dev->gyro_fs_sel = fsr_sel;
      dev->gyro_scale  = g_icm_gyro_scale[fsr_sel];
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_accel_fsr(FAR struct icm_dev_s *dev, uint16_t g)
{
  uint8_t fsr_sel;
  uint8_t reg;
  int ret;

  switch (g)
    {
      case 16: fsr_sel = 0; break;
      case 8:  fsr_sel = 1; break;
      case 4:  fsr_sel = 2; break;
      case 2:  fsr_sel = 3; break;
      default: return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = __icm_read_reg(dev, ACCEL_CONFIG0, &reg, 1);
  if (ret < 0) goto out;

  reg = (uint8_t)((reg & ~(0x07u << ACCEL_CONFIG0__ACCEL_FS_SEL__SHIFT)) |
                  ((fsr_sel & 0x07u) << ACCEL_CONFIG0__ACCEL_FS_SEL__SHIFT));
  ret = __icm_write_reg(dev, ACCEL_CONFIG0, &reg, 1);
  if (ret >= 0)
    {
      dev->acc_fs_sel  = fsr_sel;
      dev->accel_scale = g_icm_accel_scale[fsr_sel];
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_get_lost_packets(FAR struct icm_dev_s *dev, FAR uint32_t *count)
{
  int ret;
  uint8_t buf[2];

  nxmutex_lock(&dev->lock);
  ret = __icm_read_reg(dev, FIFO_LOST_PKT0, buf, 2);
  nxmutex_unlock(&dev->lock);

  if (ret >= 0)
    {
      *count = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8);
    }

  return ret;
}

/* icm_dnf_compute_coswz()
 *
 * Pure-math helper: converts a desired notch frequency (Hz) into the
 * 9-bit NF_COSWZ register value and its SEL encoding bit.
 *
 * Algorithm from the ICM-42688-P datasheet:
 *
 *   coswz = cos(2π × f_hz / f_odr)
 *
 *   if |coswz| < 0.875:                          SEL = 0
 *     NF_COSWZ = round(coswz × 256)
 *
 *   if  coswz ≥  0.875:                          SEL = 1
 *     NF_COSWZ = round((1 − coswz) × 8 × 256)
 *
 *   if  coswz ≤ −0.875:                          SEL = 1
 *     NF_COSWZ = round((1 + coswz) × 8 × 256)
 *
 * Outputs:
 *   *lo  — bits[7:0] of the 9-bit result  → GYRO_CONFIG_STATIC{6,7,8}
 *   *hi  — bit[8] of the 9-bit result     → GYRO_CONFIG_STATIC9 bits[2:0]
 *   *sel — NF_COSWZ_SEL                   → GYRO_CONFIG_STATIC9 bits[5:3]
 */

static void icm_dnf_compute_coswz(uint32_t f_hz, float f_odr,
                                  uint8_t *lo, uint8_t *hi, uint8_t *sel)
{
  float coswz = cosf(2.0f * (float)M_PI * (float)f_hz / f_odr);
  uint16_t nf_coswz;

  if (fabsf(coswz) < 0.875f)
    {
      nf_coswz = (uint16_t)lroundf(coswz * 256.0f);
      *sel = 0;
    }
  else
    {
      *sel = 1;
      if (coswz >= 0.875f)
        {
          nf_coswz = (uint16_t)lroundf((1.0f - coswz) * 8.0f * 256.0f);
        }
      else
        {
          nf_coswz = (uint16_t)lroundf((1.0f + coswz) * 8.0f * 256.0f);
        }
    }

  *lo = (uint8_t)(nf_coswz & 0xff);
  *hi = (uint8_t)((nf_coswz >> 8) & 0x01);
}

/* icm_set_gyro_dnf_freq()
 *
 * Convenience wrapper: applies the same notch frequency to all three
 * gyro axes in a single burst (4 Bank-1 register writes).
 */

static int icm_set_gyro_dnf_freq(FAR struct icm_dev_s *dev, uint32_t hz)
{
  uint8_t lo[3], hi[3], sel[3];
  uint8_t static9;
  int ret;
  int i;

  for (i = 0; i < 3; i++)
    {
      icm_dnf_compute_coswz(hz, dev->sample_rate, &lo[i], &hi[i], &sel[i]);
    }

  /* Pack GYRO_CONFIG_STATIC9:
   *   bits[2:0] = {Z,Y,X}_NF_COSWZ[8]
   *   bits[5:3] = {Z,Y,X}_NF_COSWZ_SEL
   */

  static9 = (uint8_t)(hi[0]        | (hi[1]  << 1) | (hi[2]  << 2) |
                      (sel[0] << 3) | (sel[1] << 4) | (sel[2] << 5));

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 1);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC6, &lo[0], 1);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC7, &lo[1], 1);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC8, &lo[2], 1);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC9, &static9, 1);

out:
  __icm_set_bank(dev, 0);   /* always restore Bank 0 */
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_gyro_dnf_freq_x(FAR struct icm_dev_s *dev, uint32_t hz)
{
  uint8_t lo, hi, sel, static9;
  int ret;

  icm_dnf_compute_coswz(hz, dev->sample_rate, &lo, &hi, &sel);

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 1);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC6, &lo, 1);
  if (ret < 0) goto out;

  /* RMW STATIC9: X occupies bit[0] (hi) and bit[3] (sel) */
  ret = __icm_read_reg(dev, GYRO_CONFIG_STATIC9, &static9, 1);
  if (ret < 0) goto out;

  static9 = (uint8_t)((static9 & ~(BIT(0) | BIT(3))) | (hi << 0) | (sel << 3));
  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC9, &static9, 1);

out:
  __icm_set_bank(dev, 0);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_gyro_dnf_freq_y(FAR struct icm_dev_s *dev, uint32_t hz)
{
  uint8_t lo, hi, sel, static9;
  int ret;

  icm_dnf_compute_coswz(hz, dev->sample_rate, &lo, &hi, &sel);

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 1);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC7, &lo, 1);
  if (ret < 0) goto out;

  /* RMW STATIC9: Y occupies bit[1] (hi) and bit[4] (sel) */
  ret = __icm_read_reg(dev, GYRO_CONFIG_STATIC9, &static9, 1);
  if (ret < 0) goto out;

  static9 = (uint8_t)((static9 & ~(BIT(1) | BIT(4))) | (hi << 1) | (sel << 4));
  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC9, &static9, 1);

out:
  __icm_set_bank(dev, 0);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_gyro_dnf_freq_z(FAR struct icm_dev_s *dev, uint32_t hz)
{
  uint8_t lo, hi, sel, static9;
  int ret;

  icm_dnf_compute_coswz(hz, dev->sample_rate, &lo, &hi, &sel);

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 1);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC8, &lo, 1);
  if (ret < 0) goto out;

  /* RMW STATIC9: Z occupies bit[2] (hi) and bit[5] (sel) */
  ret = __icm_read_reg(dev, GYRO_CONFIG_STATIC9, &static9, 1);
  if (ret < 0) goto out;

  static9 = (uint8_t)((static9 & ~(BIT(2) | BIT(5))) | (hi << 2) | (sel << 5));
  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC9, &static9, 1);

out:
  __icm_set_bank(dev, 0);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_gyro_dnf_bandwidth(FAR struct icm_dev_s *dev, uint32_t hz)
{
  uint8_t bw_sel;
  uint8_t reg;
  int ret;

  switch (hz)
    {
      case 1449: bw_sel = 0; break;
      case 680:  bw_sel = 1; break;
      case 329:  bw_sel = 2; break;
      case 162:  bw_sel = 3; break;
      case 80:   bw_sel = 4; break;
      case 40:   bw_sel = 5; break;
      case 20:   bw_sel = 6; break;
      case 10:   bw_sel = 7; break;
      default:   return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 1);
  if (ret < 0) goto out;

  ret = __icm_read_reg(dev, GYRO_CONFIG_STATIC10, &reg, 1);
  if (ret < 0) goto out;

  reg = (uint8_t)((reg & ~(0x07u << GYRO_CONFIG_STATIC10__GYRO_NF_BW_SEL__SHIFT)) |
                  ((bw_sel & 0x07u) << GYRO_CONFIG_STATIC10__GYRO_NF_BW_SEL__SHIFT));
  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC10, &reg, 1);

out:
  __icm_set_bank(dev, 0);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_gyro_offset(FAR struct icm_dev_s *dev,
                               FAR const struct icm42688p_offset_s *off)
{
  uint8_t buf[4];
  uint8_t user4;
  int ret;
  uint16_t gx = (uint16_t)(off->x & 0x0FFF);
  uint16_t gy = (uint16_t)(off->y & 0x0FFF);
  uint16_t gz = (uint16_t)(off->z & 0x0FFF);

  buf[0] = (uint8_t)(gx & 0xFF);                              /* OFFSET_USER0: GX[7:0]              */
  buf[1] = (uint8_t)(((gy >> 8) << 4) | ((gx >> 8) & 0x0F)); /* OFFSET_USER1: GY[11:8] | GX[11:8]  */
  buf[2] = (uint8_t)(gy & 0xFF);                              /* OFFSET_USER2: GY[7:0]              */
  buf[3] = (uint8_t)(gz & 0xFF);                              /* OFFSET_USER3: GZ[7:0]              */

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 4);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, OFFSET_USER0, buf, 4);
  if (ret < 0) goto out;

  /* OFFSET_USER4 upper nibble = AX[11:8], lower nibble = GZ[11:8] — RMW to preserve accel X. */
  ret = __icm_read_reg(dev, OFFSET_USER4, &user4, 1);
  if (ret < 0) goto out;

  user4 = (uint8_t)((user4 & 0xF0) | ((gz >> 8) & 0x0F));
  ret = __icm_write_reg(dev, OFFSET_USER4, &user4, 1);

out:
  __icm_set_bank(dev, 0);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_accel_offset(FAR struct icm_dev_s *dev,
                                FAR const struct icm42688p_offset_s *off)
{
  uint8_t buf[4];
  uint8_t user4;
  int ret;
  uint16_t ax = (uint16_t)(off->x & 0x0FFF);
  uint16_t ay = (uint16_t)(off->y & 0x0FFF);
  uint16_t az = (uint16_t)(off->z & 0x0FFF);

  buf[0] = (uint8_t)(ax & 0xFF);                              /* OFFSET_USER5: AX[7:0]              */
  buf[1] = (uint8_t)(ay & 0xFF);                              /* OFFSET_USER6: AY[7:0]              */
  buf[2] = (uint8_t)(((az >> 8) << 4) | ((ay >> 8) & 0x0F)); /* OFFSET_USER7: AZ[11:8] | AY[11:8]  */
  buf[3] = (uint8_t)(az & 0xFF);                              /* OFFSET_USER8: AZ[7:0]              */

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 4);
  if (ret < 0) goto out;

  /* OFFSET_USER4 upper nibble = AX[11:8], lower nibble = GZ[11:8] — RMW to preserve gyro Z. */
  ret = __icm_read_reg(dev, OFFSET_USER4, &user4, 1);
  if (ret < 0) goto out;

  user4 = (uint8_t)((user4 & 0x0F) | (((ax >> 8) & 0x0F) << 4));
  ret = __icm_write_reg(dev, OFFSET_USER4, &user4, 1);
  if (ret < 0) goto out;

  ret = __icm_write_reg(dev, OFFSET_USER5, buf, 4);

out:
  __icm_set_bank(dev, 0);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_gyro_dnf_en(FAR struct icm_dev_s *dev, bool en)
{
  uint8_t reg;
  int ret;

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 1);
  if (ret < 0) goto out;

  ret = __icm_read_reg(dev, GYRO_CONFIG_STATIC2, &reg, 1);
  if (ret < 0) goto out;

  if (en)
    reg &= ~GYRO_CONFIG_STATIC2__GYRO_NF_DIS;  /* clear DIS bit → filter on  */
  else
    reg |= GYRO_CONFIG_STATIC2__GYRO_NF_DIS;   /* set   DIS bit → filter off */

  ret = __icm_write_reg(dev, GYRO_CONFIG_STATIC2, &reg, 1);
  if (ret >= 0)
    {
      dev->dnf_active = en;
    }

out:
  __icm_set_bank(dev, 0);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_accel_aaf_en(FAR struct icm_dev_s *dev, bool en)
{
  uint8_t reg;
  int ret;

  nxmutex_lock(&dev->lock);

  ret = __icm_set_bank(dev, 2);
  if (ret < 0) goto out;

  ret = __icm_read_reg(dev, ACCEL_CONFIG_STATIC2, &reg, 1);
  if (ret < 0) goto out;

  if (en)
    reg &= ~ACCEL_CONFIG_STATIC2__ACCEL_AAF_DIS;  /* clear DIS bit → filter on  */
  else
    reg |= ACCEL_CONFIG_STATIC2__ACCEL_AAF_DIS;   /* set   DIS bit → filter off */

  ret = __icm_write_reg(dev, ACCEL_CONFIG_STATIC2, &reg, 1);
  if (ret >= 0)
    {
      dev->aaf_active = en;
    }

out:
  __icm_set_bank(dev, 0);
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_gyro_ui_filt_ord(FAR struct icm_dev_s *dev, uint8_t order)
{
  uint8_t ord_sel;
  uint8_t reg;
  int ret;

  switch (order)
    {
      case 1: ord_sel = 0; break;
      case 2: ord_sel = 1; break;
      case 3: ord_sel = 2; break;
      default: return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = __icm_read_reg(dev, GYRO_CONFIG1, &reg, 1);
  if (ret < 0) goto out;

  reg = (uint8_t)((reg & ~(0x03u << GYRO_CONFIG1__GYRO_UI_FILT_ORD__SHIFT)) |
                  ((ord_sel & 0x03u) << GYRO_CONFIG1__GYRO_UI_FILT_ORD__SHIFT));
  ret = __icm_write_reg(dev, GYRO_CONFIG1, &reg, 1);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_accel_ui_filt_ord(FAR struct icm_dev_s *dev, uint8_t order)
{
  uint8_t ord_sel;
  uint8_t reg;
  int ret;

  switch (order)
    {
      case 1: ord_sel = 0; break;
      case 2: ord_sel = 1; break;
      case 3: ord_sel = 2; break;
      default: return -EINVAL;
    }

  nxmutex_lock(&dev->lock);

  ret = __icm_read_reg(dev, ACCEL_CONFIG1, &reg, 1);
  if (ret < 0) goto out;

  reg = (uint8_t)((reg & ~(0x03u << ACCEL_CONFIG1__ACCEL_UI_FILT_ORD__SHIFT)) |
                  ((ord_sel & 0x03u) << ACCEL_CONFIG1__ACCEL_UI_FILT_ORD__SHIFT));
  ret = __icm_write_reg(dev, ACCEL_CONFIG1, &reg, 1);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_gyro_ui_filt_bw(FAR struct icm_dev_s *dev, uint8_t bw)
{
  uint8_t reg;
  int ret;

  nxmutex_lock(&dev->lock);

  ret = __icm_read_reg(dev, GYRO_ACCEL_CONFIG0, &reg, 1);
  if (ret < 0) goto out;

  reg = (uint8_t)((reg & ~(0x0fu << GYRO_ACCEL_CONFIG0__GYRO_UI_FILT_BW__SHIFT)) |
                  ((bw & 0x0fu) << GYRO_ACCEL_CONFIG0__GYRO_UI_FILT_BW__SHIFT));
  ret = __icm_write_reg(dev, GYRO_ACCEL_CONFIG0, &reg, 1);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int icm_set_accel_ui_filt_bw(FAR struct icm_dev_s *dev, uint8_t bw)
{
  uint8_t reg;
  int ret;

  nxmutex_lock(&dev->lock);

  ret = __icm_read_reg(dev, GYRO_ACCEL_CONFIG0, &reg, 1);
  if (ret < 0) goto out;

  reg = (uint8_t)((reg & ~(0x0fu << GYRO_ACCEL_CONFIG0__ACCEL_UI_FILT_BW__SHIFT)) |
                  ((bw & 0x0fu) << GYRO_ACCEL_CONFIG0__ACCEL_UI_FILT_BW__SHIFT));
  ret = __icm_write_reg(dev, GYRO_ACCEL_CONFIG0, &reg, 1);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * //////////////////// uORB sensor_ops_s callbacks ////////////////////
 ****************************************************************************/

/****************************************************************************
 * Name: icm_activate
 *
 * Description:
 *   Called by the uORB upper half on each topic's own 0->1 (enable) and
 *   1->0 (disable) subscriber-count transition. Since accel and gyro
 *   share one physical FIFO/IRQ, dev->refcount tracks how many of the
 *   two topics are currently active: the hardware is armed on the
 *   combined count's 0->1 transition and disarmed on its 1->0
 *   transition, so streaming stays alive as long as EITHER topic has a
 *   subscriber.
 *
 ****************************************************************************/

static int icm_activate(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep, bool enable)
{
  FAR struct icm_sens_s *sens = container_of(lower, struct icm_sens_s, lower);
  FAR struct icm_dev_s *dev = sens->dev;
  bool start = false;
  bool stop = false;
  int ret = OK;

  /* icm_fifo_start()/icm_fifo_stop() take dev->lock themselves -- this
   * lock only protects the refcount bookkeeping and must be released
   * before calling them, or the 0->1 (or 1->0) transition self-deadlocks.
   */

  nxmutex_lock(&dev->lock);

  if (enable)
    {
      start = (dev->refcount == 0);
      dev->refcount++;
    }
  else
    {
      dev->refcount--;
      stop = (dev->refcount == 0);
    }

  nxmutex_unlock(&dev->lock);

  if (start)
    {
      ret = icm_fifo_start(dev);
      if (ret < 0)
        {
          /* Don't leave refcount stuck -- the next activate() must retry
           * icm_fifo_start(), not silently skip it.
           */

          nxmutex_lock(&dev->lock);
          dev->refcount--;
          nxmutex_unlock(&dev->lock);
        }
    }
  else if (stop)
    {
      ret = icm_fifo_stop(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: icm_set_interval
 *
 * Description:
 *   accel and gyro share one physical ODR register pair, but each
 *   topic negotiates its own rate independently through the uORB
 *   upper half. Program the shared hardware to whichever of the two
 *   topics' last-requested periods is faster (smaller), while still
 *   reporting back to THIS topic the rate it will actually see (which
 *   is always at least as fast as it asked for).
 *
 ****************************************************************************/

static int icm_set_interval(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR uint32_t *period_us)
{
  FAR struct icm_sens_s *sens = container_of(lower, struct icm_sens_s, lower);
  FAR struct icm_dev_s *dev = sens->dev;
  uint32_t reconciled;
  uint32_t own_period_us;
  int ret;
  size_t i;

  for (i = ICM_NUM_ODR - 1; i > 0; i--)
    {
      if (*period_us >= g_icm_odr_map[i].period_us)
        {
          break;
        }
    }

  own_period_us = g_icm_odr_map[i].period_us;

  nxmutex_lock(&dev->lock);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      dev->accel_period_us = own_period_us;
    }
  else
    {
      dev->gyro_period_us = own_period_us;
    }

  reconciled = dev->accel_period_us < dev->gyro_period_us ?
               dev->accel_period_us : dev->gyro_period_us;

  ret = icm_program_odr(dev, reconciled);
  if (ret >= 0)
    {
      *period_us = own_period_us;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: icm_batch
 *
 * Description:
 *   Same shared-hardware reconciliation as icm_set_interval(), but for
 *   the FIFO watermark: accel and gyro each request a max latency
 *   independently, and both get serviced by whichever is the tighter
 *   (smaller) of the two, since one physical FIFO/watermark produces
 *   samples for both topics simultaneously. A latency_us of 0 means
 *   "no explicit request from this topic" and must not override a
 *   real request already made by the other topic; if neither topic has
 *   ever requested batching, fall back to the driver's original
 *   default watermark.
 *
 ****************************************************************************/

static int icm_batch(FAR struct sensor_lowerhalf_s *lower,
                     FAR struct file *filep,
                     FAR uint32_t *latency_us)
{
  FAR struct icm_sens_s *sens = container_of(lower, struct icm_sens_s, lower);
  FAR struct icm_dev_s *dev = sens->dev;
  uint32_t effective_latency_us;
  uint32_t samples;
  uint16_t wm;
  int ret;

  nxmutex_lock(&dev->lock);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      dev->accel_latency_us = *latency_us;
    }
  else
    {
      dev->gyro_latency_us = *latency_us;
    }

  if (dev->accel_latency_us == 0 && dev->gyro_latency_us == 0)
    {
      wm = FIFO_WM_REC_TH;
      effective_latency_us = (uint32_t)((uint64_t)wm * 1000000ull /
                                        (uint32_t)dev->sample_rate);
    }
  else
    {
      if (dev->accel_latency_us == 0)
        {
          effective_latency_us = dev->gyro_latency_us;
        }
      else if (dev->gyro_latency_us == 0)
        {
          effective_latency_us = dev->accel_latency_us;
        }
      else
        {
          effective_latency_us = dev->accel_latency_us < dev->gyro_latency_us ?
                                 dev->accel_latency_us : dev->gyro_latency_us;
        }

      samples = (uint32_t)((uint64_t)effective_latency_us *
                           (uint32_t)dev->sample_rate / 1000000ull);
      if (samples < 1)
        {
          samples = 1;
        }
      else if (samples > ICM_FIFO_MAX_RECS)
        {
          samples = ICM_FIFO_MAX_RECS;
        }

      wm = (uint16_t)samples;
      effective_latency_us = (uint32_t)((uint64_t)wm * 1000000ull /
                                        (uint32_t)dev->sample_rate);
    }

  ret = icm_program_watermark(dev, wm);
  if (ret >= 0)
    {
      *latency_us = effective_latency_us;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: icm_control
 *
 * Description:
 *   Dispatches the device-specific ICM42688P_IOC_* commands (FSR,
 *   notch/AAF filter config, offsets, lost-packet count). ODR and FIFO
 *   watermark/latency are handled by the standard set_interval()/
 *   batch() ops instead -- see icm42688p_uorb.h.
 *
 ****************************************************************************/

static int icm_control(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct icm_sens_s *sens = container_of(lower, struct icm_sens_s, lower);
  FAR struct icm_dev_s *dev = sens->dev;
  int ret = OK;

  switch (cmd)
    {
      /* Set gyro full-scale range: ±125, ±250, ±500, ±1000, ±2000 dps */
      case ICM42688P_IOC_SET_GYRO_FSR:
        ret = icm_set_gyro_fsr(dev, (uint16_t)arg);
        break;

      /* Set accel full-scale range: ±2, ±4, ±8, ±16 g */
      case ICM42688P_IOC_SET_ACCEL_FSR:
        ret = icm_set_accel_fsr(dev, (uint16_t)arg);
        break;

      /* Set the notch filter frequency for each Gyro axis */
      case ICM42688P_IOC_SET_GYRO_DNF_FREQ:
        ret = icm_set_gyro_dnf_freq(dev, (uint32_t)arg);
        break;

      /* Set the notch filter bandwidth for all Gyro axis
         GYRO_NF_BW_SEL: 1449, 680, 329, 162, 80, 40, 20, 10 Hz*/
      case ICM42688P_IOC_SET_GYRO_DNF_BW:
        ret = icm_set_gyro_dnf_bandwidth(dev, (uint32_t)arg);
        break;

      /* Read and clear the FIFO overflow counter from FIFO_LOST_PKT0/1 */
      case ICM42688P_IOC_GET_LOST_PKTS:
        ret = icm_get_lost_packets(dev, (FAR uint32_t *)arg);
        break;

      /* Set gyro/accel offset (all three axes at once) */
      case ICM42688P_IOC_SET_GYRO_OFFSET:
        ret = icm_set_gyro_offset(dev, (FAR const struct icm42688p_offset_s *)arg);
        break;

      case ICM42688P_IOC_SET_ACCEL_OFFSET:
        ret = icm_set_accel_offset(dev, (FAR const struct icm42688p_offset_s *)arg);
        break;

      /* Enable/disable gyro digital notch filter */
      case ICM42688P_IOC_SET_GYRO_DNF_EN:
        ret = icm_set_gyro_dnf_en(dev, (bool)arg);
        break;

      /* Enable/disable accel anti-aliasing filter */
      case ICM42688P_IOC_SET_ACCEL_AAF_EN:
        ret = icm_set_accel_aaf_en(dev, (bool)arg);
        break;

      /* Per-axis gyro notch filter center frequency */
      case ICM42688P_IOC_SET_GYRO_DNF_FREQ_X:
        ret = icm_set_gyro_dnf_freq_x(dev, (uint32_t)arg);
        break;

      case ICM42688P_IOC_SET_GYRO_DNF_FREQ_Y:
        ret = icm_set_gyro_dnf_freq_y(dev, (uint32_t)arg);
        break;

      case ICM42688P_IOC_SET_GYRO_DNF_FREQ_Z:
        ret = icm_set_gyro_dnf_freq_z(dev, (uint32_t)arg);
        break;

      /* Gyro/accel UI filter order */
      case ICM42688P_IOC_SET_GYRO_UI_FILT_ORD:
        ret = icm_set_gyro_ui_filt_ord(dev, (uint8_t)arg);
        break;

      case ICM42688P_IOC_SET_ACCEL_UI_FILT_ORD:
        ret = icm_set_accel_ui_filt_ord(dev, (uint8_t)arg);
        break;

      /* Gyro/accel UI filter bandwidth */
      case ICM42688P_IOC_SET_GYRO_UI_FILT_BW:
        ret = icm_set_gyro_ui_filt_bw(dev, (uint8_t)arg);
        break;

      case ICM42688P_IOC_SET_ACCEL_UI_FILT_BW:
        ret = icm_set_accel_ui_filt_bw(dev, (uint8_t)arg);
        break;

      default:
        sninfo("Unrecognized IOCTL command: 0x%04x\n", cmd);
        ret = -ENOTTY;   /* standard POSIX response for unknown ioctl */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icm42688p_register
 *
 * Description:
 *   Registers the ICM-42688-P as a pair of uORB topics.
 *
 * Input Parameters:
 *   devno  - Device instance number shared by both topics, e.g. devno=0
 *            -> /dev/uorb/sensor_accel0, /dev/uorb/sensor_gyro0
 *   config - Configuration information (SPI bus + chip-select + IRQ)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int icm42688p_register(int devno, FAR struct icm_config_s *config)
{
  FAR struct icm_dev_s *priv;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct icm_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate ICM-42688-P device instance\n");
      return -ENOMEM;
    }

  nxmutex_init(&priv->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  priv->config = *config;

  priv->accel.dev = priv;
  priv->gyro.dev  = priv;

  /* No preference from either topic yet -- default to the slowest
   * supported rate / the driver's original default watermark until a
   * subscriber explicitly asks for something tighter via
   * set_interval()/batch() (see their reconciliation logic above).
   */

  priv->accel_period_us  = UINT32_MAX;
  priv->gyro_period_us   = UINT32_MAX;
  priv->accel_latency_us = 0;
  priv->gyro_latency_us  = 0;
  priv->watermark_samples = FIFO_WM_REC_TH;

  /* Chip reset default is +-16g / +-2000dps / 1kHz -- match the cached
   * scale factors and sample rate to that until SET_*_FSR or
   * set_interval() change them.
   */

  priv->accel_scale      = g_icm_accel_scale[0];
  priv->gyro_scale       = g_icm_gyro_scale[0];
  priv->sample_rate      = 1000.0f;
  priv->sample_period_us = 1000;

  /* Reset the chip, to give it an initial configuration. */

  ret = icm_reset(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure ICM-42688-P: %d\n", ret);
      goto err_free;
    }

  priv->accel.lower.type    = SENSOR_TYPE_ACCELEROMETER;
  priv->accel.lower.ops     = &g_icm_ops;
  priv->accel.lower.nbuffer = CONFIG_SENSORS_ICM42688P_ACCEL_ORB_BUFSIZE;

  ret = sensor_register(&priv->accel.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register ICM-42688-P accelerometer topic: %d\n", ret);
      goto err_free;
    }

  priv->gyro.lower.type    = SENSOR_TYPE_GYROSCOPE;
  priv->gyro.lower.ops     = &g_icm_ops;
  priv->gyro.lower.nbuffer = CONFIG_SENSORS_ICM42688P_GYRO_ORB_BUFSIZE;

  ret = sensor_register(&priv->gyro.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register ICM-42688-P gyroscope topic: %d\n", ret);
      goto err_unreg_accel;
    }

  return OK;

err_unreg_accel:
  sensor_unregister(&priv->accel.lower, devno);
err_free:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
