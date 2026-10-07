/*
 * icm_42686_p.c - TDK InvenSense ICM-42686-P 6-axis IMU over SPI.
 * See icm_42686_p.h for the API.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "icm_42686_p.h"

/*
 * Register access is always one address byte followed by the data, in a single
 * CS assertion. Bit 7 of the address selects direction: set to read, clear to
 * write. The SDK transfer is full duplex, so the RX byte paired with the
 * address is discarded and the TX bytes paired with read data are don't-care.
 */
#define ICM42686_ADDR_READ(reg)  ((uint8_t)((reg) | (uint8_t)REG_SPI_READ_BIT))
#define ICM42686_ADDR_WRITE(reg) ((uint8_t)((reg) & (uint8_t)REG_SPI_WRITE_BIT))

/* Offset of each field inside the TEMP_DATA1 burst. */
#define BURST_TEMP   0U
#define BURST_ACCEL  2U
#define BURST_GYRO   8U

/*
 * Transfer scratch. Static rather than on the stack so the buffers stay in
 * .bss - if the port is ever switched to enable_dma the eDMA has to be able to
 * address them, and a task stack is no worse but these are easier to reason
 * about. Not re-entrant: one IMU, one task.
 */
static uint8_t s_tx[ICM42686_XFER_MAX];
static uint8_t s_rx[ICM42686_XFER_MAX];

/* 32768 counts of full scale. Index by icm42686_accel_fs_t / icm42686_gyro_fs_t. */
static const float s_accel_sensitivity[] = {
    1024.0f,  /* +/-32 g */
    2048.0f,  /* +/-16 g */
    4096.0f,  /* +/-8 g  */
    8192.0f,  /* +/-4 g  */
    16384.0f, /* +/-2 g  */
};

static const float s_gyro_sensitivity[] = {
    8.192f,    /* +/-4000 dps  */
    16.384f,   /* +/-2000 dps  */
    32.768f,   /* +/-1000 dps  */
    65.536f,   /* +/-500 dps   */
    131.072f,  /* +/-250 dps   */
    262.144f,  /* +/-125 dps   */
    524.288f,  /* +/-62.5 dps  */
    1048.576f, /* +/-31.25 dps */
};

/*******************************************************************************
 * Register access
 ******************************************************************************/

status_t icm42686_read_regs(icm42686_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    status_t status;

    if ((dev == NULL) || (dev->spi == NULL) || (buf == NULL) || (len == 0U) ||
        (len > ICM42686_BURST_BYTES)) {
        return kStatus_InvalidArgument;
    }

    (void)memset(s_tx, 0, 1U + len);
    s_tx[0] = ICM42686_ADDR_READ(reg);

    status = spi_master_transfer(dev->spi, s_tx, s_rx, 1U + len);
    if (status != kStatus_Success) {
        return status;
    }

    /* s_rx[0] is whatever the part shifted out while it was still receiving
     * the address, so the payload starts one byte in. */
    (void)memcpy(buf, &s_rx[1], len);
    return kStatus_Success;
}

status_t icm42686_read_reg(icm42686_t *dev, uint8_t reg, uint8_t *value)
{
    return icm42686_read_regs(dev, reg, value, 1U);
}

status_t icm42686_write_reg(icm42686_t *dev, uint8_t reg, uint8_t value)
{
    if ((dev == NULL) || (dev->spi == NULL)) {
        return kStatus_InvalidArgument;
    }

    s_tx[0] = ICM42686_ADDR_WRITE(reg);
    s_tx[1] = value;

    return spi_master_transfer(dev->spi, s_tx, s_rx, SPI_TX_B_SIZE);
}

status_t icm42686_who_am_i(icm42686_t *dev, uint8_t *id)
{
    return icm42686_read_reg(dev, WHO_AM_I, id);
}

/*******************************************************************************
 * Bring-up
 ******************************************************************************/

void icm42686_get_default_config(icm42686_t *dev, spi_ctrl_t *spi)
{
    dev->spi       = spi;
    dev->accel_fs  = kICM42686_AccelFs16g;
    dev->gyro_fs   = kICM42686_GyroFs2000dps;
    dev->accel_odr = ACCEL_ODR_100Hz;
    dev->gyro_odr  = GYRO_ODR_100Hz;

    dev->accel_sensitivity = s_accel_sensitivity[kICM42686_AccelFs16g];
    dev->gyro_sensitivity  = s_gyro_sensitivity[kICM42686_GyroFs2000dps];
}

status_t icm42686_init(icm42686_t *dev)
{
    status_t status;
    uint8_t  id = 0U;

    if ((dev == NULL) || (dev->spi == NULL)) {
        return kStatus_InvalidArgument;
    }

    if (((size_t)dev->accel_fs >= ARRAY_SIZE(s_accel_sensitivity)) ||
        ((size_t)dev->gyro_fs >= ARRAY_SIZE(s_gyro_sensitivity))) {
        return kStatus_InvalidArgument;
    }

    /* Soft reset. The part does not answer while it is resetting, so the
     * WHO_AM_I check has to wait out the delay. */
    status = icm42686_write_reg(dev, DEVICE_CONFIG, (uint8_t)SOFT_RESET_CONFIG);
    if (status != kStatus_Success) {
        return status;
    }
    SDK_DelayAtLeastUs(ICM42688_RESET_DELAY_MS * 1000U, SystemCoreClock);

    /* Reset leaves the bank register at 0, but be explicit: every register
     * this driver touches is in bank 0 and a stale bank is a silent failure. */
    status = icm42686_write_reg(dev, BANK_SEL, (uint8_t)BANK_SEL_0);
    if (status != kStatus_Success) {
        return status;
    }

    status = icm42686_who_am_i(dev, &id);
    if (status != kStatus_Success) {
        return status;
    }
    if (id != ICM42686_WHO_AM_I_VAL) {
        /* Wrong part, or - far more likely during bring-up - no part answering
         * at all, which reads back as 0x00 or 0xFF. */
        return kStatus_Fail;
    }

    status = icm42686_write_reg(dev, GYRO_CONFIG0,
                                (uint8_t)(L_SHIFT((uint8_t)dev->gyro_fs, GYRO_FS_SEL_SHIFT) |
                                          dev->gyro_odr));
    if (status != kStatus_Success) {
        return status;
    }

    status = icm42686_write_reg(dev, ACCEL_CONFIG0,
                                (uint8_t)(L_SHIFT((uint8_t)dev->accel_fs, ACCEL_FS_SEL_SHIFT) |
                                          dev->accel_odr));
    if (status != kStatus_Success) {
        return status;
    }

    /* Both sensors to low-noise mode; TEMP_DIS left clear so the temperature
     * channel stays alive. */
    status = icm42686_write_reg(dev, PWR_MGMT0,
                                (uint8_t)(ACCEL_MODE_LNM | GYRO_MODE_LNM));
    if (status != kStatus_Success) {
        return status;
    }

    /* The datasheet forbids touching any other register for 200 us after
     * PWR_MGMT0 is written... */
    SDK_DelayAtLeastUs(ICM42688_POWERUP_DELAY_MS * 1000U, SystemCoreClock);

    /* ...and the gyro needs considerably longer than that before its output is
     * meaningful. Without this wait the first icm42686_read_data() returns
     * 0x8000 on all three gyro axes and a half-settled accelerometer. */
    SDK_DelayAtLeastUs(ICM42686_SENSOR_STARTUP_MS * 1000U, SystemCoreClock);

    dev->accel_sensitivity = s_accel_sensitivity[dev->accel_fs];
    dev->gyro_sensitivity  = s_gyro_sensitivity[dev->gyro_fs];

    return kStatus_Success;
}

/*******************************************************************************
 * Sampling
 ******************************************************************************/

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

status_t icm42686_read_data(icm42686_t *dev, icm42686_data_t *out)
{
    uint8_t  burst[ICM42686_BURST_BYTES];
    status_t status;

    if ((dev == NULL) || (out == NULL)) {
        return kStatus_InvalidArgument;
    }

    /* TEMP_DATA1 (0x1D) through GYRO_DATA_Z0 (0x2A) is contiguous, so one
     * transaction gets a coherent sample of all three. */
    status = icm42686_read_regs(dev, TEMP_DATA1, burst, sizeof(burst));
    if (status != kStatus_Success) {
        return status;
    }

    out->temp_raw = be16(&burst[BURST_TEMP]);
    for (uint32_t i = 0U; i < 3U; i++) {
        out->accel_raw[i] = be16(&burst[BURST_ACCEL + (i * 2U)]);
        out->gyro_raw[i]  = be16(&burst[BURST_GYRO + (i * 2U)]);
    }

    out->temp_c  = ((float)out->temp_raw / ICM42688_TEMP_SENSITIVITY) + ICM42688_TEMP_OFFSET;
    out->temp_mc = (int32_t)(out->temp_c * 1000.0f);

    for (uint32_t i = 0U; i < 3U; i++) {
        out->accel_g[i]   = (float)out->accel_raw[i] / dev->accel_sensitivity;
        out->gyro_dps[i]  = (float)out->gyro_raw[i] / dev->gyro_sensitivity;
        out->accel_mg[i]  = (int32_t)(out->accel_g[i] * 1000.0f);
        out->gyro_mdps[i] = (int32_t)(out->gyro_dps[i] * 1000.0f);
    }

    return kStatus_Success;
}
