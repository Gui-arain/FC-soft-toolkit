/*
 * icm-40609-d.h
 *
 *  Created on: 7 mai 2026
 *      Author: guilhem_andrieu
 */

#ifndef INC_ICM_40609_D_H_
#define INC_ICM_40609_D_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum {
    ICM_OK = 0,
    ICM_ERR_SPI,
    ICM_ERR_WHO_AM_I
} ICM_Status;

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} ICM_Data;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} ICM_Handle;

// Sensitivity constants (LSB/unit)
#define ICM_ACCEL_SENS_8G   4096    // LSB/g at ±8g
#define ICM_ACCEL_SENS_16G   2048   // LSB/g at ±16g
#define ICM_GYRO_SENS_2000  16.4f   // LSB/dps at ±2000dps

ICM_Status icm_init(ICM_Handle *dev);
ICM_Status icm_read_data(ICM_Handle *dev, ICM_Data *d);
uint8_t icm_read_who_am_i(ICM_Handle *dev);


#endif /* INC_ICM_40609_D_H_ */
