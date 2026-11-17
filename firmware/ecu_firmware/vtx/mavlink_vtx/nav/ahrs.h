/*
 * ahrs.h - roll, pitch and yaw from the IMU, with GPS course for heading.
 *
 * A Mahony complementary filter on a quaternion:
 *
 *   - the gyro is integrated at the IMU rate and carries the attitude between
 *     corrections;
 *   - roll and pitch are pulled toward the gravity direction the accelerometer
 *     measures, after removing the centripetal acceleration a turning car
 *     adds (GPS speed x yaw rate), and only when what is left is close to 1 g;
 *   - yaw is pulled toward the GPS course over ground, only while the car is
 *     moving fast enough and straight enough for the course to mean the
 *     direction the car is pointing;
 *   - while the GPS course is not usable - parked, slow, turning - yaw is
 *     pulled toward the tilt-compensated magnetometer heading instead, but
 *     only if the GPS has not caught the magnetometer disagreeing with it.
 *
 * The GPS is the heading authority. On a car with an electric motor and a
 * steel chassis the magnetometer is disturbed exactly when it would be
 * needed, so every usable GPS course is also a check of the magnetometer: if
 * the two disagree by more than AHRS_MAG_GPS_TOL the magnetometer is dropped,
 * and it is only taken back after AHRS_MAG_TRUST_N consecutive agreements.
 * What the magnetometer adds is heading while the GPS course is unusable,
 * and an absolute heading from power-up. The two never correct yaw at the
 * same time: with both integrating, any small disagreement between them would
 * settle as a fake gyro bias, which then drifts yaw as soon as one drops out.
 *
 * Each magnetometer sample must also pass on its own: field strength and dip
 * angle within tolerance of what was measured at start-up (motor current
 * changes both), and, once yaw has a reference, no more than
 * AHRS_MAG_INNOV_MAX from the current estimate.
 *
 * Until the first usable GPS course or magnetometer sample, yaw is relative -
 * it starts at 0 and follows the gyro.
 *
 * Frames: body is forward-right-down (FRD), earth is north-east-down (NED),
 * which is what MAVLink ATTITUDE expects. The caller maps the sensor axes into
 * body FRD before calling ahrs_update() - see the mounting matrix in main.c.
 *
 * Pure C with no SDK dependencies, so it runs unchanged on the host. Single
 * precision throughout: the M33 FPU (fpv5-sp-d16) has no double hardware.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _AHRS_H_
#define _AHRS_H_

#include <stdbool.h>
#include <stdint.h>

/* ---- Roll / pitch from gravity ----------------------------------------- */
#define AHRS_KP_ACCEL         1.0f   /*!< rad/s per unit of gravity-direction error */
#define AHRS_KI_ACCEL         0.02f  /*!< integral gain, learns gyro bias */
/* Skip the gravity correction when the compensated specific force is further
 * than this from 1 g: hard braking, acceleration, bumps. */
#define AHRS_ACCEL_GATE_G     0.15f
#define AHRS_I_TERM_MAX       0.1f   /*!< rad/s, anti-windup on the bias estimate */

/* ---- Yaw from GPS course ------------------------------------------------ */
#define AHRS_K_COURSE         0.1f   /*!< fraction of the course error removed per GPS sample */
#define AHRS_KI_COURSE        0.002f /*!< rad/s of yaw bias learned per rad of error per sample */
#define AHRS_COURSE_MIN_SPEED 3.0f   /*!< m/s; below this the course is mostly noise */
/* Course lags the car through a turn by the GPS latency, so only straight-ish
 * driving is trusted. */
#define AHRS_COURSE_MAX_RATE  0.26f  /*!< rad/s, about 15 deg/s of yaw rate */
#define AHRS_COURSE_MAX_ACC   0.17f  /*!< rad, about 10 deg of reported course accuracy */
/* Once aligned, an error larger than this is not drift - most likely the car
 * is reversing, where the course points backwards - so it is ignored... */
#define AHRS_COURSE_REJECT    1.5708f
/* ...unless it persists this many consecutive GPS samples (10 s at 10 Hz),
 * which no reverse manoeuvre does, and then yaw re-aligns. */
#define AHRS_COURSE_REALIGN_N 100U
#define AHRS_SPEED_MAX_AGE_S  0.5f   /*!< older GPS speed is treated as 0 */

/* ---- Magnetometer -------------------------------------------------------- */
#define AHRS_K_MAG            0.01f  /*!< fraction of yaw error removed per mag sample (25 Hz) */
#define AHRS_KI_MAG           0.001f /*!< rad/s of yaw bias learned per rad of error per sample */
/* The magnetometer only corrects yaw when no usable GPS course has arrived for
 * this long. */
#define AHRS_MAG_GPS_HOLDOFF_S 0.5f
#define AHRS_MAG_INNOV_MAX    0.35f  /*!< rad, ~20 deg: a sample further from yaw is ignored */
#define AHRS_MAG_GPS_TOL      0.26f  /*!< rad, ~15 deg: more than this from GPS drops the mag */
#define AHRS_MAG_TRUST_N      5U     /*!< consecutive GPS agreements to take a dropped mag back */
/*
 * The magnetometer heading carries a constant offset: declination plus however
 * the magnetometer is turned relative to the car. It is learned from the GPS,
 * slowly - a time constant of about 20 s at 10 Hz - so that what is left of
 * the hard-iron error, which swings with heading, averages out over a lap
 * instead of being learned.
 *
 * Until the magnetometer has agreed with the GPS once, any difference below
 * AHRS_MAG_LEARN_MAX teaches the offset - that is how a mounting offset of
 * tens of degrees gets learned. After that only differences within
 * AHRS_MAG_GPS_TOL do, so a disturbance that persists while the magnetometer
 * is dropped cannot teach itself into the offset and come back trusted.
 */
#define AHRS_MAG_OFFSET_K     0.005f
#define AHRS_MAG_LEARN_MAX    0.79f  /*!< rad, ~45 deg */
#define AHRS_MAG_DECLINATION  0.0f   /*!< rad, starting offset; learned from GPS after */
#define AHRS_MAG_NORM_TOL     0.15f  /*!< fraction of the start-up field strength */
#define AHRS_MAG_DIP_TOL      0.17f  /*!< rad, ~10 deg of the start-up dip angle */
#define AHRS_MAG_REF_N        25U    /*!< samples averaged for the reference, ~1 s */
#define AHRS_MAG_MAX_AGE_S    0.2f   /*!< a GPS check needs a mag sample this recent */

typedef enum {
    kAhrsMagUnverified, /*!< no GPS comparison yet: used, and sets yaw at power-up */
    kAhrsMagTrusted,    /*!< agreed with the GPS */
    kAhrsMagRejected,   /*!< disagreed with the GPS: ignored until it agrees again */
} ahrs_mag_state_t;

/* ---- Start-up gyro bias -------------------------------------------------- */
#define AHRS_BIAS_TIME_S      2.0f   /*!< stillness needed to measure the bias */
#define AHRS_BIAS_MAX_WAIT_S  10.0f  /*!< give up and start with bias 0 */
#define AHRS_STILL_RATE       0.05f  /*!< rad/s, about 3 deg/s */
#define AHRS_STILL_ACCEL_G    0.08f

typedef enum {
    kAhrsSettling, /*!< measuring gyro bias; attitude is accel-only levelling */
    kAhrsRunning,
} ahrs_state_t;

typedef struct {
    float        q[4];      /*!< body FRD -> earth NED, w x y z */
    float        bias[3];   /*!< start-up gyro bias, rad/s */
    float        i_term[3]; /*!< running bias correction, rad/s */
    float        rate[3];   /*!< bias-corrected body rates, rad/s */
    ahrs_state_t state;

    float    settle_t;      /*!< seconds of continuous stillness */
    float    wait_t;        /*!< seconds spent settling */
    float    bias_sum[3];
    uint32_t bias_n;

    float speed_m_s;        /*!< latest GPS ground speed */
    float speed_age_s;

    bool     yaw_aligned;   /*!< yaw is referenced to north, not relative */
    uint32_t course_rejects;

    /* Magnetometer. */
    ahrs_mag_state_t mag_state;
    float    mag_offset;    /*!< rad, learned: declination + mounting */
    float    mag_heading;   /*!< rad, last clean sample's heading, offset applied */
    float    mag_age_s;     /*!< since mag_heading was updated */
    float    mag_gps_err;   /*!< rad, mag - GPS at the last comparison; NAN if none */
    uint32_t mag_agree;
    bool     mag_offset_locked; /*!< has agreed with the GPS at least once */
    float    course_age_s;  /*!< since the last usable GPS course */
    bool     yaw_from_mag;  /*!< yaw was initialised from the magnetometer */
    bool     mag_ref_ok;
    float    mag_ref_norm, mag_ref_dip;
    float    mag_ref_sum_norm, mag_ref_sum_dip;
    uint32_t mag_ref_n;

    /* Diagnostics, for a debugger or a log. */
    bool     accel_used;    /*!< last update applied the gravity correction */
    uint32_t course_used;
    uint32_t mag_used;      /*!< samples that corrected yaw */
    uint32_t mag_disturbed; /*!< samples failing the strength, dip or innovation gate */
} ahrs_t;

void ahrs_init(ahrs_t *a);

/*!
 * @brief One IMU sample, already in body FRD.
 * @param gyro_rad_s  angular rate, rad/s
 * @param accel_g     specific force, g (reads (0, 0, -1) sitting level)
 * @param dt          seconds since the previous call
 */
void ahrs_update(ahrs_t *a, const float gyro_rad_s[3], const float accel_g[3], float dt);

/*!
 * @brief One GPS solution. Call once per new solution, not per IMU sample.
 * @param fix_ok        3-D fix
 * @param speed_m_s     2-D ground speed
 * @param course_rad    course over ground, 0 = north, clockwise; ignored when
 *                      negative (unknown)
 * @param course_acc_rad reported course accuracy; pass a large value when the
 *                      receiver does not report one
 */
void ahrs_update_course(ahrs_t *a, bool fix_ok, float speed_m_s, float course_rad,
                        float course_acc_rad);

/*!
 * @brief One magnetometer sample, calibrated and already in body FRD.
 *
 * Units do not matter - only the direction is used, and the strength is only
 * compared with itself. Hard-iron and soft-iron calibration must have been
 * applied by the caller (see nav/mag_cal.h); without it the heading error
 * swings with heading and the GPS check will mostly drop the magnetometer.
 */
void ahrs_update_mag(ahrs_t *a, const float mag[3]);

/*! @brief Euler angles, radians, aerospace ZYX order, MAVLink ATTITUDE ranges. */
void ahrs_euler(const ahrs_t *a, float *roll, float *pitch, float *yaw);

#endif /* _AHRS_H_ */
