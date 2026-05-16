/**
 * bmp388.h — BMP388 barometric pressure sensor driver
 * Target: STM32H743, I2C2, HAL
 *
 * Wiring:
 *   SDO → GND   → I2C address 0x76
 *   SDO → VDDIO → I2C address 0x77
 */

#ifndef BMP388_H_
#define BMP388_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── I2C address ─────────────────────────────────────────────────────────── */
#define BMP388_I2C_ADDR_SDO_LOW   (0x76 << 1)   /* SDO → GND  */
#define BMP388_I2C_ADDR_SDO_HIGH  (0x77 << 1)   /* SDO → VDDIO */

/* ── Register map ────────────────────────────────────────────────────────── */
#define BMP388_REG_CHIP_ID        0x00
#define BMP388_REG_ERR_REG        0x02
#define BMP388_REG_STATUS         0x03
#define BMP388_REG_DATA_0         0x04   /* press_xlsb */
#define BMP388_REG_DATA_5         0x09   /* temp_msb   */
#define BMP388_REG_SENSORTIME_0   0x0C
#define BMP388_REG_EVENT          0x10
#define BMP388_REG_INT_STATUS     0x11
#define BMP388_REG_FIFO_LENGTH_0  0x12
#define BMP388_REG_FIFO_DATA      0x14
#define BMP388_REG_FIFO_WTM_0     0x15
#define BMP388_REG_FIFO_CONFIG_1  0x17
#define BMP388_REG_FIFO_CONFIG_2  0x18
#define BMP388_REG_INT_CTRL       0x19
#define BMP388_REG_IF_CONF        0x1A
#define BMP388_REG_PWR_CTRL       0x1B
#define BMP388_REG_OSR            0x1C
#define BMP388_REG_ODR            0x1D
#define BMP388_REG_CONFIG         0x1F
#define BMP388_REG_CALIB_START    0x31   /* T1 LSB */
#define BMP388_REG_CMD            0x7E

/* ── Commands (CMD register) ──────────────────────────────────────────────── */
#define BMP388_CMD_FIFO_FLUSH     0xB0
#define BMP388_CMD_SOFT_RESET     0xB6

/* ── Fixed chip-ID value ─────────────────────────────────────────────────── */
#define BMP388_CHIP_ID            0x50

/* ── Power modes (PWR_CTRL bits 5:4) ─────────────────────────────────────── */
#define BMP388_MODE_SLEEP         0x00
#define BMP388_MODE_FORCED        0x01   /* 01 or 10 */
#define BMP388_MODE_NORMAL        0x03

/* ── Oversampling (OSR register) ─────────────────────────────────────────── */
typedef enum {
    BMP388_OSR_X1  = 0,
    BMP388_OSR_X2  = 1,
    BMP388_OSR_X4  = 2,
    BMP388_OSR_X8  = 3,
    BMP388_OSR_X16 = 4,
    BMP388_OSR_X32 = 5,
} BMP388_OSR;

/* ── IIR filter coefficient (CONFIG register bits 3:1) ───────────────────── */
typedef enum {
    BMP388_IIR_OFF   = 0,
    BMP388_IIR_1     = 1,
    BMP388_IIR_3     = 2,
    BMP388_IIR_7     = 3,
    BMP388_IIR_15    = 4,
    BMP388_IIR_31    = 5,
    BMP388_IIR_63    = 6,
    BMP388_IIR_127   = 7,
} BMP388_IIR;

/* ── Output data rate (ODR register bits 4:0) ────────────────────────────── */
typedef enum {
    BMP388_ODR_200   = 0x00,   /*  200 Hz — 5 ms   */
    BMP388_ODR_100   = 0x01,   /*  100 Hz — 10 ms  */
    BMP388_ODR_50    = 0x02,   /*   50 Hz — 20 ms  */
    BMP388_ODR_25    = 0x03,   /*   25 Hz — 40 ms  */
    BMP388_ODR_12P5  = 0x04,   /* 12.5 Hz — 80 ms  */
    BMP388_ODR_6P25  = 0x05,   /*  6.25 Hz          */
    BMP388_ODR_3P1   = 0x06,
    BMP388_ODR_1P5   = 0x07,
    BMP388_ODR_0P78  = 0x08,
    BMP388_ODR_0P39  = 0x09,
    BMP388_ODR_0P2   = 0x0A,
    BMP388_ODR_0P1   = 0x0B,
} BMP388_ODR;

/* ── Calibration coefficients (float-converted, Appendix 9) ──────────────── */
typedef struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4;
    float par_p5, par_p6, par_p7, par_p8;
    float par_p9, par_p10, par_p11;
    float t_lin;   /* temperature intermediate used by pressure compensation */
} BMP388_Calib;

/* ── Configuration structure ─────────────────────────────────────────────── */
typedef struct {
    BMP388_OSR  osr_p;      /* pressure oversampling   */
    BMP388_OSR  osr_t;      /* temperature oversampling */
    BMP388_IIR  iir;        /* IIR filter coefficient   */
    BMP388_ODR  odr;        /* output data rate (normal mode) */
    uint8_t     mode;       /* BMP388_MODE_SLEEP/FORCED/NORMAL */
} BMP388_Config;

/* ── Driver handle ───────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            dev_addr;   /* 0x76<<1 or 0x77<<1 */
    BMP388_Config      cfg;
    BMP388_Calib       calib;
} BMP388_Handle;

/* ── Status codes ────────────────────────────────────────────────────────── */
typedef enum {
    BMP388_OK            =  0,
    BMP388_ERR_COMM      = -1,
    BMP388_ERR_CHIP_ID   = -2,
    BMP388_ERR_TIMEOUT   = -3,
} BMP388_Status;

/* ── API ─────────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the BMP388 driver.
 *
 * Call once after HAL and I2C are ready.  The function:
 *   1. Verifies the chip-ID (0x50).
 *   2. Issues a soft-reset.
 *   3. Reads calibration coefficients from NVM.
 *   4. Applies the configuration in h->cfg.
 *
 * Populate h->hi2c, h->dev_addr and h->cfg before calling.
 *
 * @param  h  Pointer to an initialised BMP388_Handle.
 * @return BMP388_OK on success, error code otherwise.
 */
BMP388_Status BMP388_Init(BMP388_Handle *h);

/**
 * @brief  Read compensated pressure and temperature.
 *
 * In NORMAL mode call freely; the sensor updates autonomously.
 * In FORCED mode this function triggers one measurement, waits for
 * completion, then returns the result.
 *
 * @param  h          Driver handle.
 * @param  pressure   Compensated pressure in Pa  (e.g. ~101325 at sea level).
 * @param  temperature Compensated temperature in °C.
 * @return BMP388_OK on success.
 */
BMP388_Status BMP388_Read(BMP388_Handle *h, float *pressure, float *temperature);

/**
 * @brief  Send a soft-reset command and wait for POR to complete.
 */
BMP388_Status BMP388_SoftReset(BMP388_Handle *h);

/**
 * @brief  Change power mode at runtime (SLEEP / FORCED / NORMAL).
 */
BMP388_Status BMP388_SetMode(BMP388_Handle *h, uint8_t mode);

#endif /* BMP388_H */
