/*
 * icm_42686_p.h - TDK InvenSense ICM-42686-P 6-axis IMU over SPI.
 *
 * Talks to the part through drivers/spi_driver.h. The port must be configured
 * with pcs_continuous = true: every transaction here is an address byte
 * followed by one or more data bytes in a single CS assertion, and without
 * TCR[CONT] the LPSPI drops CS between each 8-bit frame.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _ICM_42686_P_
#define _ICM_42686_P_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fsl_common.h"
#include "icm_42686_p_reg.h"
#include "spi_driver.h"

#define SPI_TX_B_SIZE 2 /* SPI tx transaction size*/

/* Sensor dimention constants */
#define ICM42688_TEMP_SENSITIVITY    132.48f
#define ICM42688_TEMP_OFFSET         25.0f
#define ICM42688_ACCEL_SENSITIVITY_16G  2048.0f  // LSB/g for ±16g range
#define ICM42688_GYRO_SENSITIVITY_2000DPS  16.4f  // LSB/dps for ±2000dps range
#define ICM42688_RESET_DELAY_MS      2
#define ICM42688_POWERUP_DELAY_MS    1
/*
 * Task sample period. Must match the configured ODR or the two alias against
 * each other: at 5 ms this is 200 Hz, which is one FreeRTOS tick here
 * (configTICK_RATE_HZ is 200) and so the fastest the task can be scheduled
 * without using the IMU's data-ready interrupt.
 */
#define IMU_SAMPLE_RATE_MS           5

/*
 * Time from the PWR_MGMT0 write until the gyroscope produces valid data. Until
 * it elapses the part reports 0x8000 (full negative scale) on every gyro axis
 * and the accelerometer is still settling. ICM42688_POWERUP_DELAY_MS only
 * covers the 200 us register-access restriction, which is a different thing.
 * 50 ms is conservative - check your datasheet for the exact figure.
 */
#define ICM42686_SENSOR_STARTUP_MS   50

/*
 * Full-scale selections, ICM-42686-P encoding.
 *
 * NOTE: these differ from the ACCEL_UI_FS_* / GYRO_UI_FS_* macros in
 * icm_42686_p_reg.h, which follow the ICM-42688-P map where FS_SEL 0 is +/-16g
 * and +/-2000dps. The -42686 is the high-range part: FS_SEL 0 is +/-32g and
 * +/-4000dps, and every other code shifts down by one. Check this against your
 * datasheet before trusting the scaling - it is the one thing here I could not
 * verify from the SDK.
 */
typedef enum {
    kICM42686_AccelFs32g = 0,
    kICM42686_AccelFs16g = 1,
    kICM42686_AccelFs8g  = 2,
    kICM42686_AccelFs4g  = 3,
    kICM42686_AccelFs2g  = 4,
} icm42686_accel_fs_t;

typedef enum {
    kICM42686_GyroFs4000dps  = 0,
    kICM42686_GyroFs2000dps  = 1,
    kICM42686_GyroFs1000dps  = 2,
    kICM42686_GyroFs500dps   = 3,
    kICM42686_GyroFs250dps   = 4,
    kICM42686_GyroFs125dps   = 5,
    kICM42686_GyroFs62_5dps  = 6,
    kICM42686_GyroFs31_25dps = 7,
} icm42686_gyro_fs_t;

/* Longest transaction: 1 address byte + TEMP_DATA1..GYRO_DATA_Z0 (14 bytes). */
#define ICM42686_BURST_BYTES  14U
#define ICM42686_XFER_MAX     (1U + ICM42686_BURST_BYTES)

typedef struct {
    spi_ctrl_t         *spi;       /*!< already-initialised LPSPI port */
    icm42686_accel_fs_t accel_fs;
    icm42686_gyro_fs_t  gyro_fs;
    uint8_t             accel_odr; /*!< ACCEL_ODR_* from icm_42686_p_reg.h */
    uint8_t             gyro_odr;  /*!< GYRO_ODR_*  from icm_42686_p_reg.h */
    /* Derived by icm42686_init() from the two ranges above. */
    float               accel_sensitivity; /*!< LSB/g */
    float               gyro_sensitivity;  /*!< LSB/dps */
} icm42686_t;

typedef struct {
    int16_t accel_raw[3]; /*!< X, Y, Z */
    int16_t gyro_raw[3];
    int16_t temp_raw;

    float accel_g[3];
    float gyro_dps[3];
    float temp_c;

    /*
     * The same values as integers, because PRINTF_FLOAT_ENABLE is off in this
     * build and %f would print garbage. Milli-units: 1000 = 1 g / 1 dps / 1 degC.
     */
    int32_t accel_mg[3];
    int32_t gyro_mdps[3];
    int32_t temp_mc;
} icm42686_data_t;

/* Fill dev with sane defaults (+/-16g, +/-2000dps, 100 Hz) bound to spi. */
void icm42686_get_default_config(icm42686_t *dev, spi_ctrl_t *spi);

/*
 * Reset the part, check WHO_AM_I, apply the ranges and ODRs from dev and bring
 * the accelerometer and gyroscope into low-noise mode.
 * Returns kStatus_Fail if WHO_AM_I does not read back ICM42686_WHO_AM_I_VAL.
 */
status_t icm42686_init(icm42686_t *dev);

status_t icm42686_read_reg(icm42686_t *dev, uint8_t reg, uint8_t *value);
status_t icm42686_write_reg(icm42686_t *dev, uint8_t reg, uint8_t value);
status_t icm42686_read_regs(icm42686_t *dev, uint8_t reg, uint8_t *buf, size_t len);

status_t icm42686_who_am_i(icm42686_t *dev, uint8_t *id);

/* One burst read of temperature, accelerometer and gyroscope. */
status_t icm42686_read_data(icm42686_t *dev, icm42686_data_t *out);

#endif /*_ICM_42688_P_*/
