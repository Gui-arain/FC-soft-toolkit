/**
 * bmp388.c — BMP388 driver implementation
 *
 * Compensation formulae from Bosch datasheet Rev 1.7 Appendix 9.
 * All register names match the datasheet section 4.
 */

#include "bmp388.h"
#include <string.h>

/* ── I2C timeout (ms) ────────────────────────────────────────────────────── */
#define I2C_TIMEOUT   100U

/* ── Low-level register helpers ──────────────────────────────────────────── */

static BMP388_Status reg_write(BMP388_Handle *h, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (HAL_I2C_Master_Transmit(h->hi2c, h->dev_addr, buf, 2, I2C_TIMEOUT) != HAL_OK)
        return BMP388_ERR_COMM;
    return BMP388_OK;
}

static BMP388_Status reg_read(BMP388_Handle *h, uint8_t reg, uint8_t *dst, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(h->hi2c, h->dev_addr, &reg, 1, I2C_TIMEOUT) != HAL_OK)
        return BMP388_ERR_COMM;
    if (HAL_I2C_Master_Receive(h->hi2c, h->dev_addr, dst, len, I2C_TIMEOUT) != HAL_OK)
        return BMP388_ERR_COMM;
    return BMP388_OK;
}

/* ── Calibration loading and conversion (datasheet section 9.1) ─────────── */

/*
 * The 21 NVM bytes starting at 0x31 hold raw trimming values.
 * Bytes are little-endian; 16-bit values are spread across two adjacent
 * registers.  After reading we apply the scale factors from section 9.1
 * to produce the float PAR_Tx / PAR_Px coefficients.
 */
static BMP388_Status load_calibration(BMP388_Handle *h)
{
    uint8_t raw[21];
    BMP388_Status st = reg_read(h, BMP388_REG_CALIB_START, raw, 21);
    if (st != BMP388_OK) return st;

    BMP388_Calib *c = &h->calib;

    /* --- temperature --- */
    uint16_t nvm_t1 = (uint16_t)raw[1] << 8 | raw[0];
    uint16_t nvm_t2 = (uint16_t)raw[3] << 8 | raw[2];
    int8_t   nvm_t3 = (int8_t)raw[4];

    c->par_t1 = (float)nvm_t1 / 0.00390625f;          /* / 2^-8  */
    c->par_t2 = (float)nvm_t2 / 1073741824.0f;         /* / 2^30  */
    c->par_t3 = (float)nvm_t3 / 281474976710656.0f;    /* / 2^48  */

    /* --- pressure --- */
    int16_t  nvm_p1  = (int16_t)((uint16_t)raw[6]  << 8 | raw[5]);
    int16_t  nvm_p2  = (int16_t)((uint16_t)raw[8]  << 8 | raw[7]);
    int8_t   nvm_p3  = (int8_t)raw[9];
    int8_t   nvm_p4  = (int8_t)raw[10];
    uint16_t nvm_p5  = (uint16_t)raw[12] << 8 | raw[11];
    uint16_t nvm_p6  = (uint16_t)raw[14] << 8 | raw[13];
    int8_t   nvm_p7  = (int8_t)raw[15];
    int8_t   nvm_p8  = (int8_t)raw[16];
    int16_t  nvm_p9  = (int16_t)((uint16_t)raw[18] << 8 | raw[17]);
    int8_t   nvm_p10 = (int8_t)raw[19];
    int8_t   nvm_p11 = (int8_t)raw[20];

    c->par_p1  = ((float)nvm_p1  - 16384.0f) / 1048576.0f;     /* (nvm-2^14)/2^20 */
    c->par_p2  = ((float)nvm_p2  - 16384.0f) / 536870912.0f;   /* (nvm-2^14)/2^29 */
    c->par_p3  = (float)nvm_p3  / 4294967296.0f;                /* /2^32 */
    c->par_p4  = (float)nvm_p4  / 137438953472.0f;              /* /2^37 */
    c->par_p5  = (float)nvm_p5  / 0.125f;                       /* /2^-3 */
    c->par_p6  = (float)nvm_p6  / 64.0f;                        /* /2^6  */
    c->par_p7  = (float)nvm_p7  / 256.0f;                       /* /2^8  */
    c->par_p8  = (float)nvm_p8  / 32768.0f;                     /* /2^15 */
    c->par_p9  = (float)nvm_p9  / 281474976710656.0f;           /* /2^48 */
    c->par_p10 = (float)nvm_p10 / 281474976710656.0f;           /* /2^48 */
    c->par_p11 = (float)nvm_p11 / 36893488147419103232.0f;      /* /2^65 */

    c->t_lin = 0.0f;

    return BMP388_OK;
}

/* ── Temperature compensation (datasheet section 9.2) ───────────────────── */

static float compensate_temperature(BMP388_Handle *h, uint32_t raw_t)
{
    BMP388_Calib *c = &h->calib;
    float pd1 = (float)raw_t - c->par_t1;
    float pd2 = pd1 * c->par_t2;
    c->t_lin  = pd2 + (pd1 * pd1) * c->par_t3;
    return c->t_lin;
}

/* ── Pressure compensation (datasheet section 9.3) ──────────────────────── */

static float compensate_pressure(BMP388_Handle *h, uint32_t raw_p)
{
    BMP388_Calib *c = &h->calib;
    float t = c->t_lin;

    float pd1 = c->par_p6 * t;
    float pd2 = c->par_p7 * (t * t);
    float pd3 = c->par_p8 * (t * t * t);
    float po1 = c->par_p5 + pd1 + pd2 + pd3;

    pd1 = c->par_p2 * t;
    pd2 = c->par_p3 * (t * t);
    pd3 = c->par_p4 * (t * t * t);
    float po2 = (float)raw_p * (c->par_p1 + pd1 + pd2 + pd3);

    pd1 = (float)raw_p * (float)raw_p;
    pd2 = c->par_p9 + c->par_p10 * t;
    pd3 = pd1 * pd2;
    float pd4 = pd3 + ((float)raw_p * (float)raw_p * (float)raw_p) * c->par_p11;

    return po1 + po2 + pd4;
}

/* ── Apply configuration registers ──────────────────────────────────────── */

static BMP388_Status apply_config(BMP388_Handle *h)
{
    BMP388_Status st;
    BMP388_Config *cfg = &h->cfg;

    /* OSR: bits [2:0] = osr_p, bits [5:3] = osr_t */
    uint8_t osr = (uint8_t)((cfg->osr_t & 0x07) << 3) | (cfg->osr_p & 0x07);
    st = reg_write(h, BMP388_REG_OSR, osr);
    if (st != BMP388_OK) return st;

    /* ODR: bits [4:0] */
    st = reg_write(h, BMP388_REG_ODR, cfg->odr & 0x1F);
    if (st != BMP388_OK) return st;

    /* CONFIG (IIR filter): bits [3:1] */
    st = reg_write(h, BMP388_REG_CONFIG, (uint8_t)((cfg->iir & 0x07) << 1));
    if (st != BMP388_OK) return st;

    /* PWR_CTRL: enable press + temp, set mode
     * bit 0 = press_en, bit 1 = temp_en, bits [5:4] = mode */
    uint8_t pwr = 0x03 | (uint8_t)((cfg->mode & 0x03) << 4);
    st = reg_write(h, BMP388_REG_PWR_CTRL, pwr);
    if (st != BMP388_OK) return st;

    return BMP388_OK;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

BMP388_Status BMP388_SoftReset(BMP388_Handle *h)
{
    BMP388_Status st = reg_write(h, BMP388_REG_CMD, BMP388_CMD_SOFT_RESET);
    if (st != BMP388_OK) return st;
    HAL_Delay(10);   /* datasheet: startup ≤ 2 ms after POR; 10 ms is safe */
    return BMP388_OK;
}

BMP388_Status BMP388_Init(BMP388_Handle *h)
{
    uint8_t chip_id = 0;
    BMP388_Status st;

    /* Verify communication and chip identity */
    st = reg_read(h, BMP388_REG_CHIP_ID, &chip_id, 1);
    if (st != BMP388_OK)          return BMP388_ERR_COMM;
    if (chip_id != BMP388_CHIP_ID) return BMP388_ERR_CHIP_ID;

    /* Soft-reset to clear any stale state */
    st = BMP388_SoftReset(h);
    if (st != BMP388_OK) return st;

    /* Load NVM calibration coefficients */
    st = load_calibration(h);
    if (st != BMP388_OK) return st;

    /* Apply user configuration */
    st = apply_config(h);
    return st;
}

BMP388_Status BMP388_SetMode(BMP388_Handle *h, uint8_t mode)
{
    h->cfg.mode = mode;

    uint8_t pwr;
    BMP388_Status st = reg_read(h, BMP388_REG_PWR_CTRL, &pwr, 1);
    if (st != BMP388_OK) return st;

    pwr = (pwr & ~(0x03 << 4)) | (uint8_t)((mode & 0x03) << 4);
    return reg_write(h, BMP388_REG_PWR_CTRL, pwr);
}

BMP388_Status BMP388_Read(BMP388_Handle *h, float *pressure, float *temperature)
{
    BMP388_Status st;

    /* In FORCED mode: write mode bits to trigger one measurement,
     * then poll STATUS until both drdy_press and drdy_temp are set. */
    if (h->cfg.mode == BMP388_MODE_FORCED) {
        st = BMP388_SetMode(h, BMP388_MODE_FORCED);
        if (st != BMP388_OK) return st;

        uint32_t t_start = HAL_GetTick();
        uint8_t  status  = 0;
        do {
            HAL_Delay(1);
            st = reg_read(h, BMP388_REG_STATUS, &status, 1);
            if (st != BMP388_OK) return st;
            if ((HAL_GetTick() - t_start) > 200U)   /* 200 ms absolute guard */
                return BMP388_ERR_TIMEOUT;
        } while ((status & 0x60) != 0x60);           /* bits 5 and 6 */
    }

    /* Burst read 6 bytes: press_xlsb..temp_msb (0x04–0x09) */
    uint8_t buf[6];
    st = reg_read(h, BMP388_REG_DATA_0, buf, 6);
    if (st != BMP388_OK) return st;

    /* Reassemble 24-bit ADC values (little-endian, unsigned) */
    uint32_t raw_p = (uint32_t)buf[0]
                   | ((uint32_t)buf[1] << 8)
                   | ((uint32_t)buf[2] << 16);

    uint32_t raw_t = (uint32_t)buf[3]
                   | ((uint32_t)buf[4] << 8)
                   | ((uint32_t)buf[5] << 16);

    /* Temperature MUST be compensated first — it updates calib.t_lin
     * which is then consumed by the pressure compensation. */
    *temperature = compensate_temperature(h, raw_t);
    *pressure    = compensate_pressure(h, raw_p);

    return BMP388_OK;
}


