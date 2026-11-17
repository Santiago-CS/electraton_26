/*
 * ahrs.c - Mahony attitude filter with GPS course for heading.
 * See ahrs.h for the design and the frame conventions.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <math.h>
#include <string.h>

#include "ahrs.h"

#define AHRS_PI       3.14159265f
#define AHRS_G_M_S2   9.80665f

/*******************************************************************************
 * Quaternion helpers. q = (w, x, y, z), rotating body FRD into earth NED.
 ******************************************************************************/

/* out = p (x) r. out may alias neither input. */
static void quat_mul(const float p[4], const float r[4], float out[4])
{
    out[0] = p[0] * r[0] - p[1] * r[1] - p[2] * r[2] - p[3] * r[3];
    out[1] = p[0] * r[1] + p[1] * r[0] + p[2] * r[3] - p[3] * r[2];
    out[2] = p[0] * r[2] - p[1] * r[3] + p[2] * r[0] + p[3] * r[1];
    out[3] = p[0] * r[3] + p[1] * r[2] - p[2] * r[1] + p[3] * r[0];
}

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    if (n > 0.0f) {
        for (uint32_t i = 0U; i < 4U; i++) {
            q[i] /= n;
        }
    } else {
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
    }
}

/* ZYX: yaw about down, then pitch about the new right, then roll. */
static void quat_from_euler(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

/*
 * The earth "down" axis expressed in body coordinates - the third row of the
 * body-to-earth rotation matrix. Both the gravity correction (where gravity
 * should point) and the yaw correction (which body axis is vertical) need it.
 */
static void down_in_body(const float q[4], float v[3])
{
    v[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    v[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    v[2] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
}

/* Rotate the attitude about the earth's down axis: yaw += angle. */
static void rotate_yaw(float q[4], float angle)
{
    float r[4] = {cosf(angle * 0.5f), 0.0f, 0.0f, sinf(angle * 0.5f)};
    float out[4];

    quat_mul(r, q, out);
    (void)memcpy(q, out, sizeof(out));
    quat_normalize(q);
}

static float wrap_pi(float a)
{
    while (a > AHRS_PI) {
        a -= 2.0f * AHRS_PI;
    }
    while (a < -AHRS_PI) {
        a += 2.0f * AHRS_PI;
    }
    return a;
}

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/*******************************************************************************
 * API
 ******************************************************************************/

void ahrs_init(ahrs_t *a)
{
    (void)memset(a, 0, sizeof(*a));
    a->q[0]        = 1.0f;
    a->state       = kAhrsSettling;
    a->mag_state   = kAhrsMagUnverified;
    a->mag_offset  = AHRS_MAG_DECLINATION;
    a->mag_age_s   = 1e9f;
    a->mag_gps_err  = NAN;
    a->course_age_s = 1e9f;
}

void ahrs_euler(const ahrs_t *a, float *roll, float *pitch, float *yaw)
{
    const float *q = a->q;

    *roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                    1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    *pitch = asinf(clampf(2.0f * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f));
    *yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

/*
 * While settling the car is assumed parked: roll and pitch come straight from
 * the accelerometer each sample, and the gyro is averaged for its bias. Any
 * movement restarts the average, because a bias measured while turning is the
 * turn rate, and integrating that for the rest of the run is far worse than
 * starting from zero and letting the integral term learn it.
 */
static void settle(ahrs_t *a, const float gyro[3], const float accel[3], float dt)
{
    float n    = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    float rate = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    float roll, pitch, yaw;

    a->wait_t += dt;

    if (n > 0.1f) {
        /* Measured down direction is -f. */
        float dx = -accel[0] / n, dy = -accel[1] / n, dz = -accel[2] / n;

        ahrs_euler(a, &roll, &pitch, &yaw);
        quat_from_euler(atan2f(dy, dz), atan2f(-dx, sqrtf(dy * dy + dz * dz)), yaw, a->q);
    }

    if ((rate < AHRS_STILL_RATE) && (fabsf(n - 1.0f) < AHRS_STILL_ACCEL_G)) {
        a->settle_t += dt;
        for (uint32_t i = 0U; i < 3U; i++) {
            a->bias_sum[i] += gyro[i];
        }
        a->bias_n++;
    } else {
        a->settle_t = 0.0f;
        a->bias_n   = 0U;
        (void)memset(a->bias_sum, 0, sizeof(a->bias_sum));
    }

    if ((a->settle_t >= AHRS_BIAS_TIME_S) || (a->wait_t >= AHRS_BIAS_MAX_WAIT_S)) {
        if ((a->settle_t >= AHRS_BIAS_TIME_S) && (a->bias_n != 0U)) {
            for (uint32_t i = 0U; i < 3U; i++) {
                a->bias[i] = a->bias_sum[i] / (float)a->bias_n;
            }
        }
        a->state = kAhrsRunning;
    }

    for (uint32_t i = 0U; i < 3U; i++) {
        a->rate[i] = gyro[i] - a->bias[i];
    }
}

void ahrs_update(ahrs_t *a, const float gyro_rad_s[3], const float accel_g[3], float dt)
{
    float w[3];
    float f[3];
    float n;

    if ((a == NULL) || !(dt > 0.0f)) {
        return;
    }

    a->speed_age_s += dt;
    a->mag_age_s   += dt;
    a->course_age_s += dt;

    if (a->state == kAhrsSettling) {
        settle(a, gyro_rad_s, accel_g, dt);
        return;
    }

    for (uint32_t i = 0U; i < 3U; i++) {
        a->rate[i] = gyro_rad_s[i] - a->bias[i] + a->i_term[i];
        w[i]       = a->rate[i];
        f[i]       = accel_g[i];
    }

    /*
     * Remove the centripetal acceleration of a car turning at speed v. With
     * the body velocity taken as (v, 0, 0) - no sideslip - the kinematic
     * acceleration is omega x v = (0, w_z v, -w_y v). What is left after
     * subtracting it is gravity plus longitudinal acceleration, and the gate
     * below throws out samples where the latter is large.
     */
    if (a->speed_age_s < AHRS_SPEED_MAX_AGE_S) {
        f[1] -= a->rate[2] * a->speed_m_s / AHRS_G_M_S2;
        f[2] += a->rate[1] * a->speed_m_s / AHRS_G_M_S2;
    }

    n = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    a->accel_used = (n > 0.1f) && (fabsf(n - 1.0f) < AHRS_ACCEL_GATE_G);

    if (a->accel_used) {
        float d[3] = {-f[0] / n, -f[1] / n, -f[2] / n}; /* measured down */
        float v[3];
        float e[3];

        down_in_body(a->q, v);                          /* estimated down */

        /* The error is the rotation that takes the estimate onto the
         * measurement; feeding it back as extra rate closes the loop. */
        e[0] = d[1] * v[2] - d[2] * v[1];
        e[1] = d[2] * v[0] - d[0] * v[2];
        e[2] = d[0] * v[1] - d[1] * v[0];

        for (uint32_t i = 0U; i < 3U; i++) {
            a->i_term[i] = clampf(a->i_term[i] + AHRS_KI_ACCEL * e[i] * dt,
                                  -AHRS_I_TERM_MAX, AHRS_I_TERM_MAX);
            w[i] += AHRS_KP_ACCEL * e[i];
        }
    }

    /* q += 0.5 q (x) (0, w) dt */
    {
        float wq[4] = {0.0f, w[0], w[1], w[2]};
        float qd[4];

        quat_mul(a->q, wq, qd);
        for (uint32_t i = 0U; i < 4U; i++) {
            a->q[i] += 0.5f * qd[i] * dt;
        }
        quat_normalize(a->q);
    }
}

/*
 * Compare the latest clean magnetometer heading with a usable GPS course. This
 * is the only thing that can drop the magnetometer or take it back, and the
 * only thing that teaches it its offset.
 */
static void mag_check(ahrs_t *a, float course_rad)
{
    float d;

    if (a->mag_age_s >= AHRS_MAG_MAX_AGE_S) {
        return;
    }

    d              = wrap_pi(a->mag_heading - course_rad);
    a->mag_gps_err = d;

    if (a->mag_offset_locked ? (fabsf(d) <= AHRS_MAG_GPS_TOL) : (fabsf(d) < AHRS_MAG_LEARN_MAX)) {
        a->mag_offset  = wrap_pi(a->mag_offset - AHRS_MAG_OFFSET_K * d);
        a->mag_heading = wrap_pi(a->mag_heading - AHRS_MAG_OFFSET_K * d);
    }

    if (fabsf(d) <= AHRS_MAG_GPS_TOL) {
        a->mag_agree++;
        if ((a->mag_state == kAhrsMagUnverified) || (a->mag_agree >= AHRS_MAG_TRUST_N)) {
            a->mag_state         = kAhrsMagTrusted;
            a->mag_offset_locked = true;
        }
    } else {
        a->mag_agree = 0U;
        a->mag_state = kAhrsMagRejected;
    }
}

void ahrs_update_mag(ahrs_t *a, const float mag[3])
{
    float n;
    float v[3];
    float dip;

    if (a == NULL) {
        return;
    }

    n = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    if (!(n > 0.0f)) {
        return;
    }

    /* Dip: the angle of the field below the horizontal, using the current
     * estimate of which body axis is vertical. */
    down_in_body(a->q, v);
    dip = asinf(clampf((mag[0] * v[0] + mag[1] * v[1] + mag[2] * v[2]) / n, -1.0f, 1.0f));

    /* The first second of samples sets what an undisturbed field looks like.
     * The car is normally parked with the motor idle at power-up. */
    if (!a->mag_ref_ok) {
        a->mag_ref_sum_norm += n;
        a->mag_ref_sum_dip  += dip;
        if (++a->mag_ref_n >= AHRS_MAG_REF_N) {
            a->mag_ref_norm = a->mag_ref_sum_norm / (float)a->mag_ref_n;
            a->mag_ref_dip  = a->mag_ref_sum_dip / (float)a->mag_ref_n;
            a->mag_ref_ok   = true;
        }
        return;
    }

    if (a->state != kAhrsRunning) {
        return;
    }

    /* Motor current and nearby steel change the strength and the dip as well
     * as the direction, so a sample that changed either is not used - not
     * even for the GPS comparison. */
    if ((fabsf(n / a->mag_ref_norm - 1.0f) > AHRS_MAG_NORM_TOL) ||
        (fabsf(dip - a->mag_ref_dip) > AHRS_MAG_DIP_TOL)) {
        a->mag_disturbed++;
        return;
    }

    /*
     * Tilt-compensated heading. Rotate the field into the estimated earth
     * frame; its horizontal direction there is how far the yaw estimate is
     * from magnetic north being straight ahead, whatever the roll and pitch.
     */
    {
        const float *q = a->q;
        float ex = (1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * mag[0] +
                   2.0f * (q[1] * q[2] - q[0] * q[3]) * mag[1] +
                   2.0f * (q[1] * q[3] + q[0] * q[2]) * mag[2];
        float ey = 2.0f * (q[1] * q[2] + q[0] * q[3]) * mag[0] +
                   (1.0f - 2.0f * (q[1] * q[1] + q[3] * q[3])) * mag[1] +
                   2.0f * (q[2] * q[3] - q[0] * q[1]) * mag[2];
        float roll, pitch, yaw;
        float err;

        ahrs_euler(a, &roll, &pitch, &yaw);
        a->mag_heading = wrap_pi(yaw - atan2f(ey, ex) + a->mag_offset);
        a->mag_age_s   = 0.0f;

        /* Recorded for the next GPS check even when dropped - that check is
         * how a dropped magnetometer earns its way back. */
        if (a->mag_state == kAhrsMagRejected) {
            return;
        }

        /* Nothing better yet: take the magnetometer's absolute heading
         * instead of starting from an arbitrary 0. */
        if (!a->yaw_aligned && !a->yaw_from_mag) {
            rotate_yaw(a->q, wrap_pi(a->mag_heading - yaw));
            a->yaw_from_mag = true;
            return;
        }

        /* The GPS course is steering yaw: the magnetometer only gets checked. */
        if (a->course_age_s < AHRS_MAG_GPS_HOLDOFF_S) {
            return;
        }

        err = wrap_pi(a->mag_heading - yaw);
        if (fabsf(err) > AHRS_MAG_INNOV_MAX) {
            a->mag_disturbed++;
            return;
        }

        rotate_yaw(a->q, AHRS_K_MAG * err);

        /* And learn the gyro's yaw bias from it, as the GPS path does - a
         * proportional pull alone leaves yaw trailing a drifting gyro by
         * bias / gain, several degrees for a bias of 1 deg/s. */
        down_in_body(a->q, v);
        for (uint32_t i = 0U; i < 3U; i++) {
            a->i_term[i] = clampf(a->i_term[i] + AHRS_KI_MAG * err * v[i],
                                  -AHRS_I_TERM_MAX, AHRS_I_TERM_MAX);
        }
        a->mag_used++;
    }
}

void ahrs_update_course(ahrs_t *a, bool fix_ok, float speed_m_s, float course_rad,
                        float course_acc_rad)
{
    float roll, pitch, yaw;
    float err;

    if (a == NULL) {
        return;
    }

    a->speed_m_s   = fix_ok ? speed_m_s : 0.0f;
    a->speed_age_s = 0.0f;

    /*
     * Course over ground is the direction of travel, which is the direction
     * the car points only when it is moving forward, fast, and not turning
     * hard: in a turn the course lags by the GPS latency.
     */
    if ((a->state != kAhrsRunning) || !fix_ok || (course_rad < 0.0f) ||
        (speed_m_s < AHRS_COURSE_MIN_SPEED) || (course_acc_rad > AHRS_COURSE_MAX_ACC) ||
        (fabsf(a->rate[2]) > AHRS_COURSE_MAX_RATE)) {
        return;
    }

    a->course_age_s = 0.0f;
    mag_check(a, course_rad);

    ahrs_euler(a, &roll, &pitch, &yaw);
    err = wrap_pi(course_rad - yaw);

    /* The first usable course replaces the relative yaw outright: the gyro has
     * had no reference until now, so the error can be anything. */
    if (!a->yaw_aligned) {
        rotate_yaw(a->q, err);
        a->yaw_aligned    = true;
        a->course_rejects = 0U;
        a->course_used++;
        return;
    }

    if (fabsf(err) > AHRS_COURSE_REJECT) {
        if (++a->course_rejects >= AHRS_COURSE_REALIGN_N) {
            rotate_yaw(a->q, err);
            a->course_rejects = 0U;
        }
        return;
    }
    a->course_rejects = 0U;

    rotate_yaw(a->q, AHRS_K_COURSE * err);

    /* Learn the gyro's yaw bias, along whichever body axis is currently
     * vertical. */
    {
        float v[3];

        down_in_body(a->q, v);
        for (uint32_t i = 0U; i < 3U; i++) {
            a->i_term[i] = clampf(a->i_term[i] + AHRS_KI_COURSE * err * v[i],
                                  -AHRS_I_TERM_MAX, AHRS_I_TERM_MAX);
        }
    }

    a->course_used++;
}
