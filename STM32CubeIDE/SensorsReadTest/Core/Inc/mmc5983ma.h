#ifndef MMC5983MA_H_
#define MMC5983MA_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Register addresses ───────────────────────────────────────────────── */
#define MMC5983_REG_XOUT0       0x00
#define MMC5983_REG_XOUT1       0x01
#define MMC5983_REG_YOUT0       0x02
#define MMC5983_REG_YOUT1       0x03
#define MMC5983_REG_ZOUT0       0x04
#define MMC5983_REG_ZOUT1       0x05
#define MMC5983_REG_XYZOUT2     0x06  /* bits [1:0] of each axis */
#define MMC5983_REG_TOUT        0x07
#define MMC5983_REG_STATUS      0x08
#define MMC5983_REG_CTRL0       0x09
#define MMC5983_REG_CTRL1       0x0A
#define MMC5983_REG_CTRL2       0x0B
#define MMC5983_REG_CTRL3       0x0C
#define MMC5983_REG_PRODUCT_ID  0x2F  /* expected value: 0x30 */

/* ── Status register bits ─────────────────────────────────────────────── */
#define MMC5983_STATUS_MEAS_M_DONE  (1 << 0)
#define MMC5983_STATUS_MEAS_T_DONE  (1 << 1)
#define MMC5983_STATUS_OTP_RD_DONE  (1 << 4)

/* ── Control register 0 bits ──────────────────────────────────────────── */
#define MMC5983_CTRL0_TM_M          (1 << 0)  /* take magnetic measurement  */
#define MMC5983_CTRL0_TM_T          (1 << 1)  /* take temperature measurement */
#define MMC5983_CTRL0_INT_EN        (1 << 2)
#define MMC5983_CTRL0_SET           (1 << 3)
#define MMC5983_CTRL0_RESET         (1 << 4)
#define MMC5983_CTRL0_AUTO_SR_EN    (1 << 5)

/* ── Control register 1 bits ──────────────────────────────────────────── */
#define MMC5983_CTRL1_BW_100HZ      0x00
#define MMC5983_CTRL1_BW_200HZ      0x01
#define MMC5983_CTRL1_BW_400HZ      0x02
#define MMC5983_CTRL1_BW_800HZ      0x03
#define MMC5983_CTRL1_SW_RST        (1 << 7)

/* ── SPI framing ──────────────────────────────────────────────────────── */
/* bit[0]=0 → write, bit[0]=1 → read ; bit[1]=don't care ; bits[7:2]=addr */
#define MMC5983_SPI_READ(addr)   (0x80 | ((addr) << 2))
#define MMC5983_SPI_WRITE(addr)  (0x00 | ((addr) << 2))

/* ── Expected product ID ──────────────────────────────────────────────── */
#define MMC5983_PRODUCT_ID      0x30

/* ── Sensitivity (18-bit mode) ───────────────────────────────────────── */
#define MMC5983_COUNTS_PER_G    16384.0f  /* 18-bit: 16384 counts/G       */
#define MMC5983_NULL_FIELD      131072UL  /* 18-bit zero-field output      */

/* ── Timeout ──────────────────────────────────────────────────────────── */
#define MMC5983_TIMEOUT_MS      100

/* ── Public types ─────────────────────────────────────────────────────── */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} MMC5983_Handle;

typedef struct {
    float x_gauss;
    float y_gauss;
    float z_gauss;
    float temp_celsius;
    uint32_t raw_x;   /* 18-bit unsigned */
    uint32_t raw_y;
    uint32_t raw_z;
} MMC5983_Data;

/* ── Public API ───────────────────────────────────────────────────────── */
bool MMC5983_Init(MMC5983_Handle *dev);
bool MMC5983_ReadData(MMC5983_Handle *dev, MMC5983_Data *out);
bool MMC5983_DoSet(MMC5983_Handle *dev);
bool MMC5983_DoReset(MMC5983_Handle *dev);
uint8_t MMC5983_ReadProductID(MMC5983_Handle *dev);

#endif /* MMC5983MA_H */
