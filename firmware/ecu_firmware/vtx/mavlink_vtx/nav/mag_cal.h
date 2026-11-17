/*
 * mag_cal.h - BMM350 calibration: raw counts -> calibrated field.
 *
 *     m_cal = MAG_CAL_W * (raw - MAG_CAL_OFFSET)
 *
 * MAG_CAL_OFFSET is the hard-iron offset (fields that move with the board:
 * the chassis, the board's own parts). MAG_CAL_W corrects soft iron and the
 * per-axis sensitivity differences, and scales the field to about 1.
 *
 * Generate these with tools/mag_calibrate.py, ideally with the board mounted
 * in the car - a bench calibration does not know about the car's steel. The
 * values below are "no calibration": the magnetometer still runs, but its
 * heading error swings with heading, and the GPS check in nav/ahrs.c will drop
 * it much of the time.
 *
 * Raw sensor axes, before k_mag_to_body in main.c.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MAG_CAL_H_
#define _MAG_CAL_H_

#define MAG_CAL_OFFSET {0.0f, 0.0f, 0.0f}

#define MAG_CAL_W                \
    {                            \
        {1.0f, 0.0f, 0.0f},      \
        {0.0f, 1.0f, 0.0f},      \
        {0.0f, 0.0f, 1.0f},      \
    }

#endif /* _MAG_CAL_H_ */
