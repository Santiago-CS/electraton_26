/*
 * bmp581_reg.h - Bosch BMP581 register map.
 *
 * Addresses and field positions are transcribed from the BMP581 datasheet.
 * Where a value is a guess rather than a transcription it says so; the one
 * checkpoint that makes the rest trustworthy is CHIP_ID, which must read back
 * BMP581_CHIP_ID_VAL before any of the configuration below means anything.
 *
 * Note the BMP581 is NOT register-compatible with the BMP280/BMP388/BMP390
 * line that most Bosch pressure sensor code targets: different register map,
 * different I2C addresses (0x46/0x47 rather than 0x76/0x77), and - the useful
 * part - no calibration-coefficient maths. The BMP581 compensates internally
 * and hands back ready-scaled values, which is why this driver can report real
 * pascals and degrees where sensors/bmm350.c can only report raw counts.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BMP581_REG_H_
#define _BMP581_REG_H_

/* 7-bit addresses. The sensor board strap gives 0x46:
 *   "I2C ADDR[6:0]: 0b1000110 (0x46) //0x47 if SDO is pulled-up" */
#define BMP581_I2C_ADDR_DEFAULT 0x46U
#define BMP581_I2C_ADDR_ALT     0x47U

/* Identification */
#define BMP581_REG_CHIP_ID     0x01U
#define BMP581_REG_REV_ID      0x02U
#define BMP581_REG_CHIP_STATUS 0x11U

/*
 * CHIP_ID. 0x50 is the BMP581; 0x51 is seen on the BMP585 and on some BMP581
 * revisions, so both are accepted rather than failing a part that works.
 */
#define BMP581_CHIP_ID_VAL     0x50U
#define BMP581_CHIP_ID_VAL_ALT 0x51U

/* Configuration */
#define BMP581_REG_DRIVE_CONFIG 0x13U
#define BMP581_REG_INT_CONFIG   0x14U
#define BMP581_REG_INT_SOURCE   0x15U
#define BMP581_REG_FIFO_CONFIG  0x16U
#define BMP581_REG_FIFO_COUNT   0x17U
#define BMP581_REG_FIFO_SEL     0x18U

/* Measurement data: 24-bit little-endian, XLSB first. */
#define BMP581_REG_TEMP_DATA_XLSB  0x1DU
#define BMP581_REG_TEMP_DATA_LSB   0x1EU
#define BMP581_REG_TEMP_DATA_MSB   0x1FU
#define BMP581_REG_PRESS_DATA_XLSB 0x20U
#define BMP581_REG_PRESS_DATA_LSB  0x21U
#define BMP581_REG_PRESS_DATA_MSB  0x22U

#define BMP581_REG_INT_STATUS 0x27U
#define BMP581_REG_STATUS     0x28U
#define BMP581_REG_FIFO_DATA  0x29U

#define BMP581_REG_NVM_ADDR     0x2BU
#define BMP581_REG_NVM_DATA_LSB 0x2CU
#define BMP581_REG_NVM_DATA_MSB 0x2DU

#define BMP581_REG_DSP_CONFIG 0x30U
#define BMP581_REG_DSP_IIR    0x31U

#define BMP581_REG_OSR_CONFIG 0x36U
#define BMP581_REG_ODR_CONFIG 0x37U
#define BMP581_REG_OSR_EFF    0x38U

#define BMP581_REG_CMD 0x7EU

/* CMD */
#define BMP581_CMD_SOFTRESET 0xB6U

/* STATUS (0x28) */
#define BMP581_STATUS_CORE_RDY_MASK 0x01U
#define BMP581_STATUS_NVM_RDY_MASK  0x02U
#define BMP581_STATUS_NVM_ERR_MASK  0x04U

/* INT_STATUS (0x27) */
#define BMP581_INT_STATUS_DRDY_MASK 0x01U

/* OSR_CONFIG (0x36): osr_t[2:0], osr_p[5:3], press_en[6] */
#define BMP581_OSR_CONFIG_OSR_T_SHIFT   0U
#define BMP581_OSR_CONFIG_OSR_P_SHIFT   3U
#define BMP581_OSR_CONFIG_PRESS_EN_MASK 0x40U

/* Oversampling codes, shared by osr_t and osr_p: 2^n samples. */
#define BMP581_OSR_1X   0U
#define BMP581_OSR_2X   1U
#define BMP581_OSR_4X   2U
#define BMP581_OSR_8X   3U
#define BMP581_OSR_16X  4U
#define BMP581_OSR_32X  5U
#define BMP581_OSR_64X  6U
#define BMP581_OSR_128X 7U

/* ODR_CONFIG (0x37): pwr_mode[1:0], odr[6:2], deep_dis[7] */
#define BMP581_ODR_CONFIG_PWR_MODE_MASK 0x03U
#define BMP581_ODR_CONFIG_ODR_SHIFT     2U
#define BMP581_ODR_CONFIG_ODR_MASK      0x7CU
#define BMP581_ODR_CONFIG_DEEP_DIS_MASK 0x80U

/* Power modes */
#define BMP581_PWR_STANDBY  0U
#define BMP581_PWR_NORMAL   1U
#define BMP581_PWR_FORCED   2U
#define BMP581_PWR_NON_STOP 3U

/*
 * Output data rates. The BMP581 ODR field counts *downwards* - code 0x00 is
 * the fastest rate and larger codes are slower - which is the opposite of most
 * Bosch parts and an easy thing to get backwards. Only the codes this driver
 * offers are named; the full table runs to 0x1F.
 */
#define BMP581_ODR_240HZ  0x00U
#define BMP581_ODR_50HZ   0x0AU /* verify against the datasheet ODR table */
#define BMP581_ODR_25HZ   0x0DU /* verify */
#define BMP581_ODR_10HZ   0x11U /* verify */
#define BMP581_ODR_1HZ    0x19U /* verify */

/* DSP_CONFIG (0x30) */
#define BMP581_DSP_COMP_PT_EN_MASK    0x03U /* pressure+temperature compensation */
#define BMP581_DSP_IIR_FLUSH_FORCED   0x04U
#define BMP581_DSP_SHDW_SEL_IIR_T     0x08U
#define BMP581_DSP_FIFO_SEL_IIR_T     0x10U
#define BMP581_DSP_SHDW_SEL_IIR_P     0x20U
#define BMP581_DSP_FIFO_SEL_IIR_P     0x40U
#define BMP581_DSP_OOR_SEL_IIR_P      0x80U

/* DSP_IIR (0x31): set_iir_t[2:0], set_iir_p[5:3] */
#define BMP581_DSP_IIR_T_SHIFT 0U
#define BMP581_DSP_IIR_P_SHIFT 3U

#define BMP581_IIR_BYPASS  0U
#define BMP581_IIR_COEF_1  1U
#define BMP581_IIR_COEF_3  2U
#define BMP581_IIR_COEF_7  3U
#define BMP581_IIR_COEF_15 4U
#define BMP581_IIR_COEF_31 5U
#define BMP581_IIR_COEF_63 6U
#define BMP581_IIR_COEF_127 7U

/*
 * Output scaling, from the datasheet and the reason this part is pleasant to
 * work with: the values arrive already compensated.
 *
 *   temperature = (int24)raw / 65536       degrees Celsius
 *   pressure    = (uint24)raw / 64         pascals
 */
/*
 * Operating range, from the datasheet. Used to sanity-check a reading: the
 * part cannot physically report outside this, so anything that does is a
 * corrupted transfer rather than data. On a bus running without its pull-ups
 * fitted that is a real and frequent event - an all-ones pressure field
 * (0xFFFFFF) decodes to 262143.98 Pa and drags the altitude to -8788 m.
 */
#define BMP581_PRESS_PA_MIN  30000U
#define BMP581_PRESS_PA_MAX  125000U
#define BMP581_TEMP_MC_MIN   (-45000)
#define BMP581_TEMP_MC_MAX   90000

#define BMP581_TEMP_DIVISOR  65536
#define BMP581_PRESS_DIVISOR 64

#endif /* _BMP581_REG_H_ */
