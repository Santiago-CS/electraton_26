/*
 * bmm350.c - Bosch BMM350 3-axis magnetometer over I2C.
 * See bmm350.h for the API and the scope note about compensation.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "bmm350.h"

/* MAG X/Y/Z then TEMP, 3 bytes each, read in one burst from MAG_X_XLSB. */
#define BMM350_DATA_BYTES    12U
#define BMM350_READ_MAX      (BMM350_DUMMY_BYTES + BMM350_DATA_BYTES)

/* Not re-entrant: one magnetometer, one task. */
static uint8_t s_buf[BMM350_READ_MAX];

/*******************************************************************************
 * Register access
 ******************************************************************************/

/*
 * The part prepends two dummy bytes to every read, so ask for len + 2 and hand
 * back what follows them.
 */
status_t bmm350_read_regs(bmm350_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    status_t status;

    if ((dev == NULL) || (dev->i2c == NULL) || (buf == NULL) || (len == 0U) ||
        (len > BMM350_DATA_BYTES)) {
        return kStatus_InvalidArgument;
    }

    status = i2c_read_regs(dev->i2c, dev->addr7, reg, s_buf, len + BMM350_DUMMY_BYTES);
    if (status != kStatus_Success) {
        return status;
    }

    (void)memcpy(buf, &s_buf[BMM350_DUMMY_BYTES], len);
    return kStatus_Success;
}

status_t bmm350_write_reg(bmm350_t *dev, uint8_t reg, uint8_t value)
{
    if ((dev == NULL) || (dev->i2c == NULL)) {
        return kStatus_InvalidArgument;
    }

    /* Writes have no dummy bytes - only reads do. */
    return i2c_write_regs(dev->i2c, dev->addr7, reg, &value, 1U);
}

status_t bmm350_chip_id(bmm350_t *dev, uint8_t *id)
{
    return bmm350_read_regs(dev, BMM350_REG_CHIP_ID, id, 1U);
}

status_t bmm350_pmu_status(bmm350_t *dev, uint8_t *status)
{
    return bmm350_read_regs(dev, BMM350_REG_PMU_CMD_STATUS_0, status, 1U);
}

/*******************************************************************************
 * Bring-up
 ******************************************************************************/

void bmm350_get_default_config(bmm350_t *dev, i2c_ctrl_t *i2c)
{
    dev->i2c       = i2c;
    dev->addr7     = BMM350_I2C_ADDR_DEFAULT; /* 0x14 per the board schematic */
    dev->odr       = BMM350_ODR_25HZ;
    dev->avg       = BMM350_AVG_4;
    dev->axis_mask = BMM350_AXIS_EN_XYZ;
}

status_t bmm350_init(bmm350_t *dev)
{
    status_t status;
    uint8_t  id = 0U;

    if ((dev == NULL) || (dev->i2c == NULL)) {
        return kStatus_InvalidArgument;
    }

    SDK_DelayAtLeastUs(BMM350_STARTUP_DELAY_MS * 1000U, SystemCoreClock);

    status = bmm350_write_reg(dev, BMM350_REG_CMD, BMM350_CMD_SOFTRESET);
    if (status != kStatus_Success) {
        return status;
    }
    SDK_DelayAtLeastUs(BMM350_SOFTRESET_DELAY_MS * 1000U, SystemCoreClock);

    status = bmm350_chip_id(dev, &id);
    if (status != kStatus_Success) {
        return status;
    }
    if (id != BMM350_CHIP_ID_VAL) {
        /* Wrong part, wrong address, or - most likely during bring-up - the
         * bus is not pulling up and every read comes back 0x00 or 0xFF. */
        return kStatus_Fail;
    }

    /*
     * Power the OTP off. Bosch's reference init does this straight after
     * reading the compensation words, and only then measures. Without it this
     * board read 0x7F in every data byte, with CHIP_ID correct and the bus
     * otherwise clean - see STATUS.md section 4.1.
     */
    status = bmm350_write_reg(dev, BMM350_REG_OTP_CMD, BMM350_OTP_CMD_PWR_OFF_OTP);
    if (status != kStatus_Success) {
        return status;
    }

    /*
     * Magnetic reset, as the reference init does it: bit reset, then
     * flux-guide reset, both issued from suspend (where the part sits after
     * the soft reset).
     */
    status = bmm350_write_reg(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_BR);
    if (status != kStatus_Success) {
        return status;
    }
    SDK_DelayAtLeastUs(BMM350_BR_DELAY_MS * 1000U, SystemCoreClock);

    status = bmm350_write_reg(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_FGR);
    if (status != kStatus_Success) {
        return status;
    }
    SDK_DelayAtLeastUs(BMM350_FGR_DELAY_MS * 1000U, SystemCoreClock);

    /* ODR and averaging, then UPD_OAE to latch them. */
    status = bmm350_write_reg(dev, BMM350_REG_PMU_CMD_AGGR_SET, (uint8_t)(dev->odr | dev->avg));
    if (status != kStatus_Success) {
        return status;
    }
    status = bmm350_write_reg(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_UPD_OAE);
    if (status != kStatus_Success) {
        return status;
    }
    SDK_DelayAtLeastUs(BMM350_MODE_DELAY_MS * 1000U, SystemCoreClock);

    status = bmm350_write_reg(dev, BMM350_REG_PMU_CMD_AXIS_EN, dev->axis_mask);
    if (status != kStatus_Success) {
        return status;
    }

    status = bmm350_write_reg(dev, BMM350_REG_PMU_CMD, BMM350_PMU_CMD_NORMAL);
    if (status != kStatus_Success) {
        return status;
    }
    SDK_DelayAtLeastUs(BMM350_NORMAL_DELAY_MS * 1000U, SystemCoreClock);

    return kStatus_Success;
}

/*******************************************************************************
 * Sampling
 ******************************************************************************/

/* 24-bit little-endian, two's complement, widened to int32_t. */
static int32_t le24(const uint8_t *p)
{
    uint32_t raw = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);

    if ((raw & 0x00800000U) != 0U) {
        raw |= 0xFF000000U; /* sign-extend */
    }

    return (int32_t)raw;
}

status_t bmm350_read_data(bmm350_t *dev, bmm350_data_t *out)
{
    uint8_t  data[BMM350_DATA_BYTES];
    status_t status;

    if ((dev == NULL) || (out == NULL)) {
        return kStatus_InvalidArgument;
    }

    /* MAG_X_XLSB through TEMP are contiguous, so one burst gets a coherent
     * sample of all four fields. */
    status = bmm350_read_regs(dev, BMM350_REG_MAG_X_XLSB, data, sizeof(data));
    if (status != kStatus_Success) {
        return status;
    }

    for (uint32_t i = 0U; i < 3U; i++) {
        out->mag_raw[i] = le24(&data[i * 3U]);
    }
    out->temp_raw = le24(&data[9]);

    return kStatus_Success;
}
