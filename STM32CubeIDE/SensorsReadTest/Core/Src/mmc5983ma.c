#include "mmc5983ma.h"
#include <string.h>

/* ── Private helpers ──────────────────────────────────────────────────── */

static inline void cs_low(MMC5983_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(MMC5983_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/**
 * Write one byte to a register.
 *
 * SPI frame (16 clocks):
 *   bits[7:2] = address, bit[1] = don't-care, bit[0] = 0 (write)
 *   followed by 8 data bits
 */
static bool reg_write(MMC5983_Handle *dev, uint8_t addr, uint8_t data)
{
    uint8_t tx[2] = { MMC5983_SPI_WRITE(addr), data };
    cs_low(dev);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->hspi, tx, 2, MMC5983_TIMEOUT_MS);
    cs_high(dev);
    return (st == HAL_OK);
}

/**
 * Read one byte from a register.
 *
 * SPI frame (16 clocks):
 *   bits[7:2] = address, bit[1] = don't-care, bit[0] = 1 (read)
 *   chip drives SDO on the second byte
 */
static bool reg_read(MMC5983_Handle *dev, uint8_t addr, uint8_t *val)
{
    uint8_t tx[2] = { MMC5983_SPI_READ(addr), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    cs_low(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, MMC5983_TIMEOUT_MS);
    cs_high(dev);
    if (st != HAL_OK) return false;
    *val = rx[1];
    return true;
}

/**
 * Burst-read n consecutive registers starting at addr.
 * Each additional register costs one extra byte (8 clocks).
 */
static bool reg_read_burst(MMC5983_Handle *dev, uint8_t addr, uint8_t *buf, uint8_t len)
{
    /* First byte = command, then (len) data bytes */
    uint8_t tx[9] = { 0 };      /* max 1 cmd + 8 data bytes */
    uint8_t rx[9] = { 0 };
    if (len > 8) return false;

    tx[0] = MMC5983_SPI_READ(addr);
    cs_low(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len + 1, MMC5983_TIMEOUT_MS);
    cs_high(dev);
    if (st != HAL_OK) return false;
    memcpy(buf, &rx[1], len);
    return true;
}

/* ── Public API ───────────────────────────────────────────────────────── */

/**
 * Initialise the sensor:
 *   1. Software reset
 *   2. Verify product ID (0x30)
 *   3. Run SET to establish a clean magnetic state
 *   4. Set bandwidth to 100 Hz (BW=00, lowest noise)
 */
bool MMC5983_Init(MMC5983_Handle *dev)
{
    /* Ensure CS starts deasserted */
    cs_high(dev);
    HAL_Delay(10);

    /* Software reset – clears all registers, re-reads OTP */
    if (!reg_write(dev, MMC5983_REG_CTRL1, MMC5983_CTRL1_SW_RST)) return false;
    HAL_Delay(15);  /* datasheet: power-on time 10 ms */

    /* Verify product ID */
    uint8_t pid = MMC5983_ReadProductID(dev);
    if (pid != MMC5983_PRODUCT_ID) return false;

    /* Initial SET to establish magnetisation */
    if (!MMC5983_DoSet(dev)) return false;

    /* BW = 00 → 8 ms measurement, 100 Hz bandwidth, lowest RMS noise (0.4 mG) */
    if (!reg_write(dev, MMC5983_REG_CTRL1, MMC5983_CTRL1_BW_100HZ)) return false;

    return true;
}

/**
 * Read product ID register (0x2F).
 * Returns 0xFF on SPI error.
 */
uint8_t MMC5983_ReadProductID(MMC5983_Handle *dev)
{
    uint8_t val = 0xFF;
    reg_read(dev, MMC5983_REG_PRODUCT_ID, &val);
    return val;
}

/**
 * Perform a SET operation:
 * Sends a high-current pulse through the sensor coil for 500 ns to restore
 * the magnetisation. Needed after exposure to strong external fields and
 * before accurate measurements.
 */
bool MMC5983_DoSet(MMC5983_Handle *dev)
{
    if (!reg_write(dev, MMC5983_REG_CTRL0, MMC5983_CTRL0_SET)) return false;
    HAL_Delay(1);   /* bit self-clears after 500 ns, 1 ms margin */
    return true;
}

/**
 * Perform a RESET operation:
 * Sends the opposite pulse to SET. Used in the SET/RESET offset-cancellation
 * technique described in the datasheet.
 */
bool MMC5983_DoReset(MMC5983_Handle *dev)
{
    if (!reg_write(dev, MMC5983_REG_CTRL0, MMC5983_CTRL0_RESET)) return false;
    HAL_Delay(1);
    return true;
}

/**
 * Trigger one measurement, wait for completion, burst-read all axes and
 * temperature, then convert to physical units.
 *
 * 18-bit output:
 *   Xout[17:10] → Xout0 (reg 0x00)
 *   Xout[9:2]   → Xout1 (reg 0x01)
 *   Xout[1:0]   → XYZout2 bits [7:6] (reg 0x06)
 *
 * Zero-field code = 131072 (2^17), output is unsigned and centred there.
 * Sensitivity = 16384 counts/G in 18-bit mode.
 */
bool MMC5983_ReadData(MMC5983_Handle *dev, MMC5983_Data *out)
{
    /* ── 1. Trigger magnetic measurement ─────────────────────────────── */
    if (!reg_write(dev, MMC5983_REG_CTRL0, MMC5983_CTRL0_TM_M)) return false;

    /* ── 2. Poll Meas_M_Done (bit 0 of status register 0x08) ─────────── */
    uint8_t status = 0;
    uint32_t t0 = HAL_GetTick();
    do {
        if (!reg_read(dev, MMC5983_REG_STATUS, &status)) return false;
        if (HAL_GetTick() - t0 > MMC5983_TIMEOUT_MS) return false;
    } while (!(status & MMC5983_STATUS_MEAS_M_DONE));

    /* ── 3. Burst-read output registers 0x00 – 0x07 (8 bytes) ────────── */
    uint8_t raw[8] = { 0 };
    if (!reg_read_burst(dev, MMC5983_REG_XOUT0, raw, 8)) return false;

    /*
     * raw[0] = Xout[17:10]
     * raw[1] = Xout[9:2]
     * raw[2] = Yout[17:10]
     * raw[3] = Yout[9:2]
     * raw[4] = Zout[17:10]
     * raw[5] = Zout[9:2]
     * raw[6] = XYZout2  → bits[7:6]=Xout[1:0], [5:4]=Yout[1:0], [3:2]=Zout[1:0]
     * raw[7] = Tout[7:0]
     */

    uint32_t x = ((uint32_t)raw[0] << 10)
               | ((uint32_t)raw[1] << 2)
               | ((raw[6] >> 6) & 0x03);

    uint32_t y = ((uint32_t)raw[2] << 10)
               | ((uint32_t)raw[3] << 2)
               | ((raw[6] >> 4) & 0x03);

    uint32_t z = ((uint32_t)raw[4] << 10)
               | ((uint32_t)raw[5] << 2)
               | ((raw[6] >> 2) & 0x03);

    out->raw_x = x;
    out->raw_y = y;
    out->raw_z = z;

    /* Convert to Gauss: subtract null-field offset, divide by sensitivity */
    out->x_gauss = ((float)x - (float)MMC5983_NULL_FIELD) / MMC5983_COUNTS_PER_G;
    out->y_gauss = ((float)y - (float)MMC5983_NULL_FIELD) / MMC5983_COUNTS_PER_G;
    out->z_gauss = ((float)z - (float)MMC5983_NULL_FIELD) / MMC5983_COUNTS_PER_G;

    /* Temperature: 0x00 = -75 °C, 0.8 °C/LSB */
    out->temp_celsius = -75.0f + (float)raw[7] * 0.8f;

    return true;
}
