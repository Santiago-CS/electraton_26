/*
 * bmm350.h - Bosch BMM350 3-axis magnetometer over I2C.
 *
 * Talks to the part through drivers/i2c_driver.h, so it works on either the
 * blocking or the eDMA path depending on how the i2c_ctrl_t was configured.
 * On this board that is LPI2C4 on GPIO_IO02/IO03 - see i2c_driver.h, and note
 * the bus pull-ups it describes are not fitted from the factory.
 *
 * ---------------------------------------------------------------------------
 * SCOPE: this driver reports RAW counts.
 *
 * Turning BMM350 counts into microtesla needs Bosch's factory compensation -
 * reading the OTP words and applying dut_offset / dut_sensit / dut_tco /
 * dut_tcs / cross-axis correction. That is a substantial piece of Bosch's
 * reference API and is not reproduced here; inventing coefficients would give
 * numbers that look right and are not. Raw counts are enough to bring the part
 * up and to see the field move when you wave a magnet at it. If you need
 * calibrated microtesla, port Bosch's BMM350 API compensation on top of
 * bmm350_read_data().
 * ---------------------------------------------------------------------------
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BMM350_H_
#define _BMM350_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bmm350_reg.h"
#include "fsl_common.h"
#include "i2c_driver.h"

/* Datasheet timings, in milliseconds. Generous rather than marginal. */
#define BMM350_SOFTRESET_DELAY_MS 24U
#define BMM350_STARTUP_DELAY_MS   3U
#define BMM350_MODE_DELAY_MS      10U

#define BMM350_SAMPLE_RATE_MS 200U /*!< how often the task polls */

typedef struct {
    i2c_ctrl_t *i2c;       /*!< already-initialised LPI2C port */
    uint8_t     addr7;     /*!< 7-bit address; 0x14 on this board */
    uint8_t     odr;       /*!< BMM350_ODR_* */
    uint8_t     avg;       /*!< BMM350_AVG_* */
    uint8_t     axis_mask; /*!< BMM350_AXIS_EN_* */
} bmm350_t;

typedef struct {
    /* Signed 24-bit, sign-extended into int32_t. Uncompensated - see above. */
    int32_t mag_raw[3]; /*!< X, Y, Z */
    int32_t temp_raw;
} bmm350_data_t;

/*! @brief Fill dev with defaults (address 0x14, 25 Hz, 4x averaging, all axes). */
void bmm350_get_default_config(bmm350_t *dev, i2c_ctrl_t *i2c);

/*!
 * @brief Soft reset, verify CHIP_ID, apply ODR/averaging and enter normal mode.
 * @return kStatus_Fail if CHIP_ID does not read back BMM350_CHIP_ID_VAL.
 */
status_t bmm350_init(bmm350_t *dev);

status_t bmm350_read_regs(bmm350_t *dev, uint8_t reg, uint8_t *buf, size_t len);
status_t bmm350_write_reg(bmm350_t *dev, uint8_t reg, uint8_t value);
status_t bmm350_chip_id(bmm350_t *dev, uint8_t *id);

/*! @brief One burst read of the three magnetic axes and the temperature. */
status_t bmm350_read_data(bmm350_t *dev, bmm350_data_t *out);

#endif /* _BMM350_H_ */
