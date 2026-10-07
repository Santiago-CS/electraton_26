/*
 * bmp581.h - Bosch BMP581 barometric pressure and temperature sensor over I2C.
 *
 * Talks to the part through drivers/i2c_driver.h, so it works on either the
 * blocking or the eDMA path depending on how the i2c_ctrl_t was configured.
 * On this board that is LPI2C3 on GPIO_IO00/IO01 - the schematic's I2C5 - and
 * the bus pull-ups for it (R55, R57) are NOT fitted from the factory. See the
 * barometer section of i2c_driver.h before wondering why every transfer NAKs.
 *
 * ---------------------------------------------------------------------------
 * SCOPE: unlike sensors/bmm350.c, this driver reports REAL UNITS.
 *
 * That is not extra work on our part, it is the part being nicer: the BMP581
 * compensates internally and its data registers already hold scaled values -
 * temperature as degrees x 65536, pressure as pascals x 64. There are no
 * calibration coefficients to read and no polynomial to evaluate, which is the
 * big practical difference from the BMP280/BMP388/BMP390 family. So pascals
 * and millidegrees here are the sensor's own numbers, not a guess.
 *
 * Altitude is a different matter and is deliberately a separate function with
 * an explicit reference pressure - see bmp581_altitude_cm().
 * ---------------------------------------------------------------------------
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BMP581_H_
#define _BMP581_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bmp581_reg.h"
#include "fsl_common.h"
#include "i2c_driver.h"

/* Datasheet timings, milliseconds. Generous rather than marginal. */
#define BMP581_SOFTRESET_DELAY_MS 3U
#define BMP581_STARTUP_DELAY_MS   3U
/*
 * Time from entering normal mode until the first conversion has landed. One
 * ODR period plus the oversampling time, rounded well up: at the default 50 Hz
 * that is 20 ms, and reading before it elapses gives you the reset value
 * rather than a measurement.
 */
#define BMP581_FIRST_SAMPLE_MS 50U

#define BMP581_SAMPLE_RATE_MS 200U /*!< how often the task polls */

/*! @brief ISA standard sea-level pressure, the default altitude reference. */
#define BMP581_SEA_LEVEL_PA 101325U

typedef struct {
    i2c_ctrl_t *i2c;   /*!< already-initialised LPI2C port */
    uint8_t     addr7; /*!< 7-bit address; 0x46 on this board */
    uint8_t     odr;   /*!< BMP581_ODR_* */
    uint8_t     osr_p; /*!< BMP581_OSR_*, pressure oversampling */
    uint8_t     osr_t; /*!< BMP581_OSR_*, temperature oversampling */
    uint8_t     iir_p; /*!< BMP581_IIR_*, pressure filter */
    uint8_t     iir_t; /*!< BMP581_IIR_*, temperature filter */
    /* Filled in by bmp581_init() from OSR_EFF: the oversampling the part
     * actually applied, which is clamped if the ODR is too fast for what was
     * asked for. If these come back below osr_p/osr_t, slow the ODR down. */
    uint8_t osr_p_eff;
    uint8_t osr_t_eff;
    bool    odr_valid; /*!< OSR_EFF bit 7: the ODR/OSR combination is runnable */
} bmp581_t;

typedef struct {
    int32_t  temp_raw;  /*!< signed 24-bit, sign-extended */
    uint32_t press_raw; /*!< unsigned 24-bit */

    /* Scaled, as integers: PRINTF_FLOAT_ENABLE is off in this build. */
    int32_t  temp_m_c;   /*!< millidegrees Celsius */
    uint32_t press_pa;   /*!< whole pascals */
    uint32_t press_c_pa; /*!< centipascals, to keep the sub-pascal resolution */
} bmp581_data_t;

/*! @brief Fill dev with defaults (0x46, 50 Hz, 8x pressure / 2x temperature,
 *         IIR coefficient 3 on pressure). */
void bmp581_get_default_config(bmp581_t *dev, i2c_ctrl_t *i2c);

/*!
 * @brief Soft reset, verify CHIP_ID, check the NVM came up, apply the
 *        oversampling/filter/ODR settings and enter normal mode.
 *
 * @return kStatus_Fail if CHIP_ID is neither BMP581_CHIP_ID_VAL nor
 *         BMP581_CHIP_ID_VAL_ALT, or if the NVM reports an error.
 */
status_t bmp581_init(bmp581_t *dev);

status_t bmp581_read_regs(bmp581_t *dev, uint8_t reg, uint8_t *buf, size_t len);
status_t bmp581_write_reg(bmp581_t *dev, uint8_t reg, uint8_t value);
status_t bmp581_chip_id(bmp581_t *dev, uint8_t *id);

/*! @brief True when a new conversion is waiting. Optional - at a poll interval
 *         well above the ODR there is always one. */
status_t bmp581_data_ready(bmp581_t *dev, bool *ready);

/*!
 * @brief One burst read of temperature and pressure, scaled.
 *
 * @return kStatus_Fail if the result falls outside the part's operating range.
 *         That cannot come from the sensor, so it means the transfer was
 *         corrupted - common on a bus whose pull-ups are not fitted. The
 *         fields in out are still written, so a caller that wants to inspect
 *         the bad value can; it just should not treat it as a measurement.
 */
status_t bmp581_read_data(bmp581_t *dev, bmp581_data_t *out);

/*!
 * @brief Barometric altitude from a pressure reading, in centimetres.
 *
 * Separate from bmp581_read_data() and taking the reference explicitly,
 * because this is the one number here that is not the sensor's own:
 *
 *   ABSOLUTE altitude is only as good as sea_level_pa, which is a weather
 *   figure that moves by a few hPa day to day - roughly 10 m of apparent
 *   altitude. Passing BMP581_SEA_LEVEL_PA gives ISA-standard altitude, which
 *   is a defined quantity but not where you actually are.
 *
 *   RELATIVE altitude is accurate and is what a barometer is genuinely good
 *   for. Take a reading at a known height, keep that pressure, and the
 *   difference tracks height change to well under a metre.
 *
 * Uses the hypsometric formula against the ISA troposphere model, so it is
 * valid to roughly 11 km.
 */
int32_t bmp581_altitude_cm(uint32_t press_pa, uint32_t sea_level_pa);

#endif /* _BMP581_H_ */
