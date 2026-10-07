/*
 * bmm350_reg.h - Bosch BMM350 magnetometer register map.
 *
 * Values here are transcribed from the BMM350 datasheet / Bosch's reference
 * API. Verify anything you depend on - CHIP_ID reading back as 0x33 is the
 * quickest check that the map and the bus are both right.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BMM350_REG_H_
#define _BMM350_REG_H_

#include <stdint.h>

/* ---- I2C addresses -------------------------------------------------------
 * The TMP_SENSOR_BOARD schematic notes, next to the BMM350 symbol:
 *   "I2C ADDR[6:0]: 0b0010100 (0x14) //0x15 if ADSEL is pulled-up"
 * so the board default is 0x14. 7-bit, unshifted - the LPI2C driver shifts it.
 */
#define BMM350_I2C_ADDR_DEFAULT 0x14U
#define BMM350_I2C_ADDR_ADSEL   0x15U

/* ---- Registers ---------------------------------------------------------- */
#define BMM350_REG_CHIP_ID           0x00U
#define BMM350_REG_ERR_REG           0x02U
#define BMM350_REG_PAD_CTRL          0x03U
#define BMM350_REG_PMU_CMD_AGGR_SET  0x04U
#define BMM350_REG_PMU_CMD_AXIS_EN   0x05U
#define BMM350_REG_PMU_CMD           0x06U
#define BMM350_REG_PMU_CMD_STATUS_0  0x07U
#define BMM350_REG_PMU_CMD_STATUS_1  0x08U
#define BMM350_REG_I3C_ERR           0x09U
#define BMM350_REG_I2C_WDT_SET       0x0AU
#define BMM350_REG_TRSDCR_REV_ID     0x0DU
#define BMM350_REG_TC_SYNC_TU        0x0EU
#define BMM350_REG_PROGRAM_CTRL      0x0FU
#define BMM350_REG_INT_CTRL          0x20U
#define BMM350_REG_INT_CTRL_IBI      0x21U
#define BMM350_REG_INT_STATUS        0x30U
/* Magnetic and temperature data: 3 bytes each, LSB first, 24-bit signed. */
#define BMM350_REG_MAG_X_XLSB        0x31U
#define BMM350_REG_MAG_Y_XLSB        0x34U
#define BMM350_REG_MAG_Z_XLSB        0x37U
#define BMM350_REG_TEMP_XLSB         0x3AU
#define BMM350_REG_SENSORTIME_XLSB   0x3DU
#define BMM350_REG_CMD               0x7EU

/* ---- CHIP_ID ------------------------------------------------------------ */
#define BMM350_CHIP_ID_VAL 0x33U

/* ---- CMD ---------------------------------------------------------------- */
#define BMM350_CMD_SOFTRESET 0xB6U

/* ---- PMU_CMD ------------------------------------------------------------ */
#define BMM350_PMU_CMD_SUSPEND  0x00U
#define BMM350_PMU_CMD_NORMAL   0x01U
#define BMM350_PMU_CMD_UPD_OAE  0x02U /*!< latch a new ODR/averaging setting */
#define BMM350_PMU_CMD_FM       0x03U /*!< forced mode */
#define BMM350_PMU_CMD_FM_FAST  0x04U
#define BMM350_PMU_CMD_FGR      0x05U /*!< flux guide reset */
#define BMM350_PMU_CMD_FGR_FAST 0x06U
#define BMM350_PMU_CMD_BR       0x07U /*!< bit reset */
#define BMM350_PMU_CMD_BR_FAST  0x08U

/* ---- PMU_CMD_AXIS_EN ---------------------------------------------------- */
#define BMM350_AXIS_EN_X   (1U << 0)
#define BMM350_AXIS_EN_Y   (1U << 1)
#define BMM350_AXIS_EN_Z   (1U << 2)
#define BMM350_AXIS_EN_XYZ (BMM350_AXIS_EN_X | BMM350_AXIS_EN_Y | BMM350_AXIS_EN_Z)

/* ---- PMU_CMD_AGGR_SET: ODR in [3:0], averaging in [5:4] ----------------- */
#define BMM350_ODR_SHIFT      0U
#define BMM350_ODR_400HZ      (0x02U << BMM350_ODR_SHIFT)
#define BMM350_ODR_200HZ      (0x03U << BMM350_ODR_SHIFT)
#define BMM350_ODR_100HZ      (0x04U << BMM350_ODR_SHIFT)
#define BMM350_ODR_50HZ       (0x05U << BMM350_ODR_SHIFT)
#define BMM350_ODR_25HZ       (0x06U << BMM350_ODR_SHIFT)
#define BMM350_ODR_12_5HZ     (0x07U << BMM350_ODR_SHIFT)
#define BMM350_ODR_6_25HZ     (0x08U << BMM350_ODR_SHIFT)
#define BMM350_ODR_3_125HZ    (0x09U << BMM350_ODR_SHIFT)
#define BMM350_ODR_1_5625HZ   (0x0AU << BMM350_ODR_SHIFT)

#define BMM350_AVG_SHIFT 4U
#define BMM350_AVG_NONE  (0x00U << BMM350_AVG_SHIFT)
#define BMM350_AVG_2     (0x01U << BMM350_AVG_SHIFT)
#define BMM350_AVG_4     (0x02U << BMM350_AVG_SHIFT)
#define BMM350_AVG_8     (0x03U << BMM350_AVG_SHIFT)

/* ---- Protocol ----------------------------------------------------------- */
/*
 * Every BMM350 read returns two dummy bytes before the first real one. This is
 * a BMM350 thing - the BMM150 had none - and forgetting it is the classic way
 * to get plausible-looking garbage out of this part.
 */
#define BMM350_DUMMY_BYTES 2U

#endif /* _BMM350_REG_H_ */
