/*
 * bmp581.c - Bosch BMP581 barometric pressure and temperature over I2C.
 * See bmp581.h for the API and the note about altitude.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <math.h>
#include <string.h>

#include "bmp581.h"

/* TEMP_DATA_XLSB (0x1D) through PRESS_DATA_MSB (0x22), one burst. */
#define BMP581_DATA_BYTES 6U

/*******************************************************************************
 * Register access
 *
 * No dummy bytes. Worth saying explicitly because the BMM350 on the other bus
 * of this same board prepends two to every read, and carrying that habit over
 * here would shift every field by two and produce plausible nonsense.
 ******************************************************************************/

status_t bmp581_read_regs(bmp581_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    if ((dev == NULL) || (dev->i2c == NULL) || (buf == NULL) || (len == 0U)) {
        return kStatus_InvalidArgument;
    }

    return i2c_read_regs(dev->i2c, dev->addr7, reg, buf, len);
}

status_t bmp581_write_reg(bmp581_t *dev, uint8_t reg, uint8_t value)
{
    if ((dev == NULL) || (dev->i2c == NULL)) {
        return kStatus_InvalidArgument;
    }

    return i2c_write_regs(dev->i2c, dev->addr7, reg, &value, 1U);
}

status_t bmp581_chip_id(bmp581_t *dev, uint8_t *id)
{
    return bmp581_read_regs(dev, BMP581_REG_CHIP_ID, id, 1U);
}

status_t bmp581_data_ready(bmp581_t *dev, bool *ready)
{
    uint8_t  status_reg = 0U;
    status_t status;

    if (ready == NULL) {
        return kStatus_InvalidArgument;
    }

    status = bmp581_read_regs(dev, BMP581_REG_INT_STATUS, &status_reg, 1U);
    if (status != kStatus_Success) {
        return status;
    }

    *ready = (status_reg & BMP581_INT_STATUS_DRDY_MASK) != 0U;
    return kStatus_Success;
}

/*******************************************************************************
 * Bring-up
 ******************************************************************************/

void bmp581_get_default_config(bmp581_t *dev, i2c_ctrl_t *i2c)
{
    dev->i2c   = i2c;
    dev->addr7 = BMP581_I2C_ADDR_DEFAULT; /* 0x46 per the board schematic */
    dev->odr   = BMP581_ODR_50HZ;
    /*
     * Pressure is the reason the part is here, so it gets the oversampling;
     * temperature mostly exists to compensate the pressure and 2x is plenty.
     * IIR coefficient 3 on pressure takes the edge off without making the
     * reading lag noticeably - raise it if you care more about a quiet
     * altitude trace than about step response.
     */
    dev->osr_p = BMP581_OSR_8X;
    dev->osr_t = BMP581_OSR_2X;
    dev->iir_p = BMP581_IIR_COEF_3;
    dev->iir_t = BMP581_IIR_BYPASS;

    dev->osr_p_eff = 0U;
    dev->osr_t_eff = 0U;
    dev->odr_valid = false;
}

status_t bmp581_init(bmp581_t *dev)
{
    status_t status;
    uint8_t  id  = 0U;
    uint8_t  reg = 0U;

    if ((dev == NULL) || (dev->i2c == NULL)) {
        return kStatus_InvalidArgument;
    }

    SDK_DelayAtLeastUs(BMP581_STARTUP_DELAY_MS * 1000U, SystemCoreClock);

    status = bmp581_write_reg(dev, BMP581_REG_CMD, BMP581_CMD_SOFTRESET);
    if (status != kStatus_Success) {
        return status;
    }
    SDK_DelayAtLeastUs(BMP581_SOFTRESET_DELAY_MS * 1000U, SystemCoreClock);

    status = bmp581_chip_id(dev, &id);
    if (status != kStatus_Success) {
        return status;
    }
    if ((id != BMP581_CHIP_ID_VAL) && (id != BMP581_CHIP_ID_VAL_ALT)) {
        return kStatus_Fail;
    }

    /*
     * The part copies trim data out of NVM on reset. If that did not finish, or
     * reported an error, the compensation behind every value below is suspect -
     * so fail here rather than hand back numbers that look fine.
     */
    status = bmp581_read_regs(dev, BMP581_REG_STATUS, &reg, 1U);
    if (status != kStatus_Success) {
        return status;
    }
    if (((reg & BMP581_STATUS_NVM_RDY_MASK) == 0U) ||
        ((reg & BMP581_STATUS_NVM_ERR_MASK) != 0U)) {
        return kStatus_Fail;
    }

    /*
     * Pressure and temperature compensation on, and the IIR-filtered value is
     * what the data registers should show - without the shadow-select bits the
     * registers report the unfiltered sample and setting DSP_IIR achieves
     * nothing visible.
     */
    status = bmp581_write_reg(dev, BMP581_REG_DSP_CONFIG,
                              (uint8_t)(BMP581_DSP_COMP_PT_EN_MASK |
                                        BMP581_DSP_SHDW_SEL_IIR_T |
                                        BMP581_DSP_SHDW_SEL_IIR_P));
    if (status != kStatus_Success) {
        return status;
    }

    status = bmp581_write_reg(dev, BMP581_REG_DSP_IIR,
                              (uint8_t)(((uint32_t)dev->iir_t << BMP581_DSP_IIR_T_SHIFT) |
                                        ((uint32_t)dev->iir_p << BMP581_DSP_IIR_P_SHIFT)));
    if (status != kStatus_Success) {
        return status;
    }

    /* Oversampling, and press_en - without it the pressure path stays off and
     * only temperature updates. */
    status = bmp581_write_reg(dev, BMP581_REG_OSR_CONFIG,
                              (uint8_t)(BMP581_OSR_CONFIG_PRESS_EN_MASK |
                                        ((uint32_t)dev->osr_t << BMP581_OSR_CONFIG_OSR_T_SHIFT) |
                                        ((uint32_t)dev->osr_p << BMP581_OSR_CONFIG_OSR_P_SHIFT)));
    if (status != kStatus_Success) {
        return status;
    }

    /*
     * ODR and normal mode in one write. deep_dis is set: deep standby saves
     * power the board does not need to save and the wake-up latency would show
     * up as missed samples.
     */
    status = bmp581_write_reg(dev, BMP581_REG_ODR_CONFIG,
                              (uint8_t)(BMP581_ODR_CONFIG_DEEP_DIS_MASK |
                                        (((uint32_t)dev->odr << BMP581_ODR_CONFIG_ODR_SHIFT) &
                                         BMP581_ODR_CONFIG_ODR_MASK) |
                                        BMP581_PWR_NORMAL));
    if (status != kStatus_Success) {
        return status;
    }

    SDK_DelayAtLeastUs(BMP581_FIRST_SAMPLE_MS * 1000U, SystemCoreClock);

    /*
     * Read back what the part settled on. If the ODR is too fast for the
     * requested oversampling the BMP581 silently reduces the oversampling
     * rather than refusing, so asking is the only way to know what the
     * readings actually are.
     */
    status = bmp581_read_regs(dev, BMP581_REG_OSR_EFF, &reg, 1U);
    if (status == kStatus_Success) {
        dev->osr_t_eff = (uint8_t)(reg & 0x07U);
        dev->osr_p_eff = (uint8_t)((reg >> 3) & 0x07U);
        dev->odr_valid = (reg & 0x80U) != 0U;
    }

    return kStatus_Success;
}

/*******************************************************************************
 * Measurement
 ******************************************************************************/

status_t bmp581_read_data(bmp581_t *dev, bmp581_data_t *out)
{
    uint8_t  buf[BMP581_DATA_BYTES];
    status_t status;
    uint32_t traw;
    uint32_t praw;

    if ((dev == NULL) || (out == NULL)) {
        return kStatus_InvalidArgument;
    }

    status = bmp581_read_regs(dev, BMP581_REG_TEMP_DATA_XLSB, buf, BMP581_DATA_BYTES);
    if (status != kStatus_Success) {
        return status;
    }

    /* Both fields are 24-bit little-endian: XLSB, LSB, MSB. */
    traw = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
    praw = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);

    /* Temperature is signed; sign-extend from 24 bits. Pressure is not. */
    out->temp_raw = (traw & 0x800000U) != 0U ? ((int32_t)traw - 0x1000000) : (int32_t)traw;
    out->press_raw = praw;

    /*
     * temperature = raw / 65536 degrees Celsius, wanted as millidegrees.
     * raw * 1000 overflows int32 at about 2.1e6, which is only 32 degrees, so
     * the intermediate has to be 64-bit - an easy and silent thing to get
     * wrong.
     */
    out->temp_m_c = (int32_t)(((int64_t)out->temp_raw * 1000LL) / (int64_t)BMP581_TEMP_DIVISOR);

    /*
     * pressure = raw / 64 pascals. Whole pascals fit comfortably; centipascals
     * are raw * 100 / 64, which for a plausible 110 kPa is raw = 7.04e6 and
     * so needs 64-bit too.
     */
    out->press_pa   = praw / (uint32_t)BMP581_PRESS_DIVISOR;
    out->press_c_pa = (uint32_t)(((uint64_t)praw * 100ULL) / (uint64_t)BMP581_PRESS_DIVISOR);

    /*
     * Reject the physically impossible. The part cannot report outside its
     * operating range, so a reading that does came off the bus corrupted, not
     * out of the sensor - and a single bad sample is worth more as an error
     * than as a 262143 Pa / -8788 m outlier in the caller's data.
     *
     * This is validation, not a workaround: a driver should check what a bus
     * hands it. It does become load-bearing on a bus whose pull-ups are not
     * fitted, where an all-ones field turns up in roughly 1% of reads.
     */
    if ((out->press_pa < BMP581_PRESS_PA_MIN) || (out->press_pa > BMP581_PRESS_PA_MAX) ||
        (out->temp_m_c < BMP581_TEMP_MC_MIN) || (out->temp_m_c > BMP581_TEMP_MC_MAX)) {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

int32_t bmp581_altitude_cm(uint32_t press_pa, uint32_t sea_level_pa)
{
    float ratio;
    float metres;

    if ((press_pa == 0U) || (sea_level_pa == 0U)) {
        return 0;
    }

    /*
     * Hypsometric formula over the ISA troposphere:
     *
     *   h = (T0 / L) * (1 - (p/p0)^(L*R/(g*M)))
     *
     * with T0 = 288.15 K, L = 0.0065 K/m, giving the 44330 m scale factor and
     * the 0.1902632 exponent that every implementation of this quotes.
     *
     * Float, not fixed point, because the exponent is irrational and a
     * fixed-point version would be both slower and less clear. The M33 here
     * has a single-precision FPU (-mfpu=fpv5-sp-d16), so this is a handful of
     * instructions plus the powf call. The result is returned as an integer so
     * callers never need %f, which this build cannot print.
     */
    ratio  = (float)press_pa / (float)sea_level_pa;
    metres = 44330.0f * (1.0f - powf(ratio, 0.1902632f));

    return (int32_t)(metres * 100.0f);
}
