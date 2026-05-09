/*
 * icm-40609-d.c
 *
 *  Created on: 7 mai 2026
 *      Author: guilhem_andrieu
 */
#include "icm-40609-d.h"

#define ICM_READ        0x80  // bit7=1 for read

// Register addresses (private to this file)
#define REG_WHO_AM_I    0x75
#define REG_DEVICE_CFG  0x11
#define REG_PWR_MGMT0   0x4E
#define REG_GYRO_CFG0   0x4F
#define REG_ACCEL_CFG0  0x50
#define REG_BANK_SEL    0x76
#define REG_TEMP_DATA1  0x1D

#define WHO_AM_I_VAL    0x3B

static void icm_write(ICM_Handle *dev, uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg & 0x7F, data };  // bit7=0 for write
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->hspi, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

static uint8_t icm_read(ICM_Handle *dev, uint8_t reg) {
    uint8_t tx[2] = { reg | ICM_READ, 0x00 };
    uint8_t rx[2] = { 0 };
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    return rx[1];
}

uint8_t icm_read_who_am_i(ICM_Handle *dev) {
    return icm_read(dev, REG_WHO_AM_I);
}

ICM_Status icm_init(ICM_Handle *dev) {
    // 1. Select Bank 0
    icm_write(dev, REG_BANK_SEL, 0x00);

    // 2. Soft reset
    icm_write(dev, REG_DEVICE_CFG, 0x01);
    HAL_Delay(10);  // Wait for reset to complete

    // 3. Check WHO_AM_I
    if (icm_read(dev, REG_WHO_AM_I) != WHO_AM_I_VAL) {
        return ICM_ERR_WHO_AM_I;
    }

    // 4. Wake up: enable Accel (LN) + Gyro (LN)
    // PWR_MGMT0: bits[3:2]=11 (accel LN), bits[1:0]=11 (gyro LN)
    icm_write(dev, REG_PWR_MGMT0, 0x0F);
    HAL_Delay(1);

    // 5. Gyro: ±2000 dps, 1kHz ODR
    // GYRO_CONFIG0: FSR=0b000 (±2000), ODR=0b0110 (1kHz)
    icm_write(dev, REG_GYRO_CFG0, 0x06);

    // 6. Accel: ±16g, 1kHz ODR
    // ACCEL_CONFIG0: FSR=0b001 (±16g), ODR=0b0110 (1kHz)
    icm_write(dev, REG_ACCEL_CFG0, 0x26);

    return ICM_OK;
}

ICM_Status icm_read_data(ICM_Handle *dev, ICM_Data *d) {
    uint8_t tx[15] = { REG_TEMP_DATA1 | ICM_READ };
    uint8_t rx[15] = { 0 };

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 15, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    d->temp = (int16_t)(rx[1] << 8 | rx[2]);
    d->ax   = (int16_t)(rx[3] << 8 | rx[4]);
    d->ay   = (int16_t)(rx[5] << 8 | rx[6]);
    d->az   = (int16_t)(rx[7] << 8 | rx[8]);
    d->gx   = (int16_t)(rx[9] << 8 | rx[10]);
    d->gy   = (int16_t)(rx[11] << 8 | rx[12]);
    d->gz   = (int16_t)(rx[13] << 8 | rx[14]);

    return ICM_OK;
}
