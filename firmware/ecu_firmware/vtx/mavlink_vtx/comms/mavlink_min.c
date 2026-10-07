/*
 * mavlink_min.c - just enough MAVLink v2 to look like a flight controller.
 * See mavlink_min.h for why this is hand-rolled and how the checksum was
 * verified.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "mavlink_min.h"

/*******************************************************************************
 * Little-endian writers
 *
 * MAVLink payloads are little-endian and packed with no alignment guarantees,
 * so everything is written byte by byte rather than through a struct. That
 * also makes the wire layout of each message readable in one place.
 ******************************************************************************/

static uint32_t put_u8(uint8_t *p, uint32_t i, uint8_t v)
{
    p[i] = v;
    return i + 1U;
}

static uint32_t put_u16(uint8_t *p, uint32_t i, uint16_t v)
{
    p[i]      = (uint8_t)(v & 0xFFU);
    p[i + 1U] = (uint8_t)(v >> 8);
    return i + 2U;
}

static uint32_t put_u32(uint8_t *p, uint32_t i, uint32_t v)
{
    p[i]      = (uint8_t)(v & 0xFFU);
    p[i + 1U] = (uint8_t)((v >> 8) & 0xFFU);
    p[i + 2U] = (uint8_t)((v >> 16) & 0xFFU);
    p[i + 3U] = (uint8_t)((v >> 24) & 0xFFU);
    return i + 4U;
}

static uint32_t put_u64(uint8_t *p, uint32_t i, uint64_t v)
{
    i = put_u32(p, i, (uint32_t)(v & 0xFFFFFFFFU));
    return put_u32(p, i, (uint32_t)(v >> 32));
}

static uint32_t put_i16(uint8_t *p, uint32_t i, int16_t v)
{
    return put_u16(p, i, (uint16_t)v);
}

static uint32_t put_i32(uint8_t *p, uint32_t i, int32_t v)
{
    return put_u32(p, i, (uint32_t)v);
}

static uint32_t put_f32(uint8_t *p, uint32_t i, float v)
{
    uint32_t bits;

    (void)memcpy(&bits, &v, sizeof(bits)); /* not a cast: type-punning via
                                            * memcpy is the defined way */
    return put_u32(p, i, bits);
}

/* Clamp a wide value into an int16 field rather than letting it wrap - a
 * wrapped accelerometer reading looks like a real measurement. */
static int16_t sat_i16(int32_t v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return (int16_t)v;
}

/*******************************************************************************
 * Checksum
 ******************************************************************************/

uint16_t mavlink_crc(const uint8_t *data, uint32_t len, uint8_t crc_extra)
{
    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0U; i <= len; i++) {
        /* One extra round over crc_extra after the data - that byte is a hash
         * of the message's field layout, so two ends that disagree about a
         * message produce different checksums instead of silently
         * misinterpreting each other. */
        uint8_t byte = (i < len) ? data[i] : crc_extra;
        uint8_t tmp  = (uint8_t)(byte ^ (uint8_t)(crc & 0xFFU));

        tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
        crc = (uint16_t)((crc >> 8) ^ (uint16_t)((uint16_t)tmp << 8) ^
                         (uint16_t)((uint16_t)tmp << 3) ^ (uint16_t)(tmp >> 4));
    }

    return crc;
}

/*******************************************************************************
 * Framing
 ******************************************************************************/

uint16_t mavlink_build(uint8_t *out, size_t cap, mavlink_node_t *node,
                       uint32_t msgid, uint8_t crc_extra,
                       const uint8_t *payload, uint8_t len)
{
    uint16_t crc;
    uint32_t i = 0U;
    uint8_t  trimmed = len;

    if ((out == NULL) || (node == NULL) || ((payload == NULL) && (len != 0U))) {
        return 0U;
    }

    /* v2 truncation: trailing zero bytes are not transmitted and the receiver
     * re-inflates them. At least one byte is always sent. */
    while ((trimmed > 1U) && (payload[trimmed - 1U] == 0U)) {
        trimmed--;
    }
    if (len == 0U) {
        trimmed = 0U;
    }

    if (cap < ((size_t)MAVLINK_HEADER_BYTES + trimmed + MAVLINK_CKSUM_BYTES)) {
        return 0U;
    }

    i = put_u8(out, i, MAVLINK_STX_V2);
    i = put_u8(out, i, trimmed);
    i = put_u8(out, i, 0U);              /* incompat_flags: 0, so unsigned */
    i = put_u8(out, i, 0U);              /* compat_flags */
    i = put_u8(out, i, node->seq++);
    i = put_u8(out, i, node->sysid);
    i = put_u8(out, i, node->compid);
    i = put_u8(out, i, (uint8_t)(msgid & 0xFFU));
    i = put_u8(out, i, (uint8_t)((msgid >> 8) & 0xFFU));
    i = put_u8(out, i, (uint8_t)((msgid >> 16) & 0xFFU));

    if (trimmed != 0U) {
        (void)memcpy(&out[i], payload, trimmed);
        i += trimmed;
    }

    /* Over everything from len through the end of the payload - not the STX. */
    crc = mavlink_crc(&out[1], i - 1U, crc_extra);
    i   = put_u16(out, i, crc);

    return (uint16_t)i;
}

/*******************************************************************************
 * Messages
 ******************************************************************************/

uint16_t mavlink_heartbeat(uint8_t *out, size_t cap, mavlink_node_t *node,
                           uint8_t type, uint8_t system_status)
{
    uint8_t  pl[9];
    uint32_t i = 0U;

    i = put_u32(pl, i, 0U);                       /* custom_mode */
    i = put_u8(pl, i, type);                      /* type */
    i = put_u8(pl, i, MAV_AUTOPILOT_GENERIC);     /* autopilot */
    i = put_u8(pl, i, 0U);                        /* base_mode */
    i = put_u8(pl, i, system_status);             /* system_status */
    i = put_u8(pl, i, 3U);                        /* mavlink_version */

    return mavlink_build(out, cap, node, MAVLINK_MSG_HEARTBEAT,
                         MAVLINK_CRC_HEARTBEAT, pl, (uint8_t)i);
}

uint16_t mavlink_scaled_imu(uint8_t *out, size_t cap, mavlink_node_t *node,
                            uint32_t time_boot_ms, const int32_t acc_mg[3],
                            const int32_t gyro_mdps[3], const int16_t *mag_mgauss)
{
    uint8_t  pl[22];
    uint32_t i = 0U;

    i = put_u32(pl, i, time_boot_ms);

    for (uint32_t a = 0U; a < 3U; a++) {
        i = put_i16(pl, i, sat_i16(acc_mg[a]));
    }

    /*
     * SCALED_IMU carries angular rate in millirad/s, while the IMU driver
     * produces millidegrees/s. Nothing in the field names says so, and getting
     * it wrong gives a result that looks plausible and is out by 57x.
     * mrad/s = mdeg/s * pi/180, done in integer as * 31416 / 1800000.
     */
    for (uint32_t a = 0U; a < 3U; a++) {
        int32_t mrad = (int32_t)(((int64_t)gyro_mdps[a] * 31416LL) / 1800000LL);

        i = put_i16(pl, i, sat_i16(mrad));
    }

    for (uint32_t a = 0U; a < 3U; a++) {
        i = put_i16(pl, i, (mag_mgauss != NULL) ? mag_mgauss[a] : (int16_t)0);
    }

    return mavlink_build(out, cap, node, MAVLINK_MSG_SCALED_IMU,
                         MAVLINK_CRC_SCALED_IMU, pl, (uint8_t)i);
}

uint16_t mavlink_gps_raw_int(uint8_t *out, size_t cap, mavlink_node_t *node,
                             uint64_t time_usec, uint8_t fix_type, uint8_t sats,
                             int32_t lat_1e7, int32_t lon_1e7, int32_t alt_mm,
                             uint16_t eph, uint16_t epv,
                             uint16_t vel_cm_s, uint16_t cog_cdeg)
{
    uint8_t  pl[30];
    uint32_t i = 0U;

    i = put_u64(pl, i, time_usec);
    i = put_i32(pl, i, lat_1e7);
    i = put_i32(pl, i, lon_1e7);
    i = put_i32(pl, i, alt_mm);
    i = put_u16(pl, i, eph);
    i = put_u16(pl, i, epv);
    i = put_u16(pl, i, vel_cm_s);
    i = put_u16(pl, i, cog_cdeg);
    i = put_u8(pl, i, fix_type);
    i = put_u8(pl, i, sats);

    return mavlink_build(out, cap, node, MAVLINK_MSG_GPS_RAW_INT,
                         MAVLINK_CRC_GPS_RAW_INT, pl, (uint8_t)i);
}

uint16_t mavlink_attitude(uint8_t *out, size_t cap, mavlink_node_t *node,
                          uint32_t time_boot_ms, float roll, float pitch,
                          float yaw, float rollspeed, float pitchspeed,
                          float yawspeed)
{
    uint8_t  pl[28];
    uint32_t i = 0U;

    i = put_u32(pl, i, time_boot_ms);
    i = put_f32(pl, i, roll);
    i = put_f32(pl, i, pitch);
    i = put_f32(pl, i, yaw);
    i = put_f32(pl, i, rollspeed);
    i = put_f32(pl, i, pitchspeed);
    i = put_f32(pl, i, yawspeed);

    return mavlink_build(out, cap, node, MAVLINK_MSG_ATTITUDE,
                         MAVLINK_CRC_ATTITUDE, pl, (uint8_t)i);
}
