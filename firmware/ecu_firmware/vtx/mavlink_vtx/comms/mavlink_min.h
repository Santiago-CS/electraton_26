/*
 * mavlink_min.h - just enough MAVLink v2 to look like a flight controller.
 *
 * WHY THIS EXISTS, AND NOT THE GENERATED LIBRARY
 *
 * The VTX's datalink is built to carry a flight controller's MAVLink. It is
 * not a transparent serial pipe: it parses frames, and bytes that are not
 * valid MAVLink are discarded rather than forwarded. So a custom binary frame
 * - however well formed - never reaches the ground station. Speaking real
 * MAVLink is therefore not a nicety here, it is the requirement.
 *
 * The full generated C library is a large pile of headers for the four
 * messages this needs. Encoding MAVLink v2 by hand is about a hundred lines,
 * and the only part that is at all subtle is the checksum.
 *
 * FRAME
 *
 *   FD | len | incompat | compat | seq | sysid | compid | msgid[3] | payload | ck[2]
 *
 * The checksum is CRC-16/MCRF4XX (X.25) over everything from `len` through the
 * end of the payload, followed by one extra byte per message type - the
 * CRC_EXTRA, a hash of that message's field layout, which is what stops two
 * ends with different ideas of a message from silently agreeing.
 *
 * VERIFICATION
 *
 * The checksum routine below is confirmed against a frame captured from the
 * wfb-ng ground station - a third-party MAVLink sender - which also recovered
 * CRC_EXTRA 185 for RADIO_STATUS by search. So the algorithm is not in doubt.
 *
 * The CRC_EXTRA constants for the messages this file sends are written from
 * the common dialect and have NOT been confirmed against hardware. A wrong one
 * fails safely and loudly: the receiver's checksum will not match and the
 * message is dropped rather than misread. QGroundControl showing the vehicle
 * is the definitive check.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MAVLINK_MIN_H_
#define _MAVLINK_MIN_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAVLINK_STX_V2       0xFDU
#define MAVLINK_HEADER_BYTES 10U
#define MAVLINK_CKSUM_BYTES  2U
#define MAVLINK_MAX_PAYLOAD  255U

/* Message ids and their CRC_EXTRA - see the verification note above. */
#define MAVLINK_MSG_HEARTBEAT         0U
#define MAVLINK_CRC_HEARTBEAT         50U
#define MAVLINK_MSG_GPS_RAW_INT       24U
#define MAVLINK_CRC_GPS_RAW_INT       24U
#define MAVLINK_MSG_SCALED_IMU        26U
#define MAVLINK_CRC_SCALED_IMU        170U
#define MAVLINK_MSG_ATTITUDE          30U
#define MAVLINK_CRC_ATTITUDE          39U
#define MAVLINK_MSG_RADIO_STATUS      109U
#define MAVLINK_CRC_RADIO_STATUS      185U /* confirmed against wfb-ng */

/* MAV_TYPE / MAV_AUTOPILOT / MAV_STATE, enough to be recognised. */
#define MAV_TYPE_GENERIC              0U
#define MAV_TYPE_QUADROTOR            2U
#define MAV_AUTOPILOT_GENERIC         0U
#define MAV_AUTOPILOT_INVALID         8U
#define MAV_MODE_FLAG_SAFETY_ARMED    128U
#define MAV_STATE_ACTIVE              4U
#define MAV_STATE_STANDBY             3U

/* Longest frame this module builds: GPS_RAW_INT is 30 payload bytes. */
#define MAVLINK_FRAME_MAX (MAVLINK_HEADER_BYTES + 30U + MAVLINK_CKSUM_BYTES)

/*! @brief Identity this node presents. One per vehicle. */
typedef struct {
    uint8_t sysid;  /*!< 1 is the conventional first vehicle */
    uint8_t compid; /*!< 1 = MAV_COMP_ID_AUTOPILOT1 */
    uint8_t seq;    /*!< incremented per frame sent; gaps mean loss */
} mavlink_node_t;

/*! @brief X.25 / CRC-16-MCRF4XX, as MAVLink uses it. Exposed for testing. */
uint16_t mavlink_crc(const uint8_t *data, uint32_t len, uint8_t crc_extra);

/*!
 * @brief Wrap a payload into a MAVLink v2 frame.
 *
 * Applies the v2 trailing-zero truncation: bytes of zero at the end of the
 * payload are not sent and the receiver re-inflates them. Observed in the
 * wild - the wfb-ng RADIO_STATUS frame arrives 8 bytes rather than 9.
 *
 * @return frame length, or 0 if out is too small.
 */
uint16_t mavlink_build(uint8_t *out, size_t cap, mavlink_node_t *node,
                       uint32_t msgid, uint8_t crc_extra,
                       const uint8_t *payload, uint8_t len);

/*! @brief HEARTBEAT. A GCS will not show a vehicle without this, at ~1 Hz. */
uint16_t mavlink_heartbeat(uint8_t *out, size_t cap, mavlink_node_t *node,
                           uint8_t type, uint8_t system_status);

/*!
 * @brief SCALED_IMU.
 * @param acc_mg    milli-g, as sensors/icm_42686_p.c already produces
 * @param gyro_mdps millidegrees/s - converted to the millirad/s the message
 *                  wants, which is a standing trap: the field name says
 *                  nothing about the unit.
 * @param mag_mgauss may be NULL; sent as zero, which is what "no compass" looks
 *                  like to a GCS.
 */
uint16_t mavlink_scaled_imu(uint8_t *out, size_t cap, mavlink_node_t *node,
                            uint32_t time_boot_ms, const int32_t acc_mg[3],
                            const int32_t gyro_mdps[3], const int16_t *mag_mgauss);

/* GPS_FIX_TYPE */
#define GPS_FIX_TYPE_NO_GPS     0U
#define GPS_FIX_TYPE_NO_FIX     1U
#define GPS_FIX_TYPE_2D_FIX     2U
#define GPS_FIX_TYPE_3D_FIX     3U
#define GPS_FIX_TYPE_DGPS       4U

/* "Unknown" for the 16-bit fields of GPS_RAW_INT. */
#define MAVLINK_U16_UNKNOWN     65535U

/*!
 * @brief GPS_RAW_INT, base fields only.
 *
 * eph and epv are DILUTION OF PRECISION x 100, not distances - the names look
 * like "error, position, horizontal" and invite passing an accuracy in cm,
 * which a GCS would then display as a wildly wrong DOP. Pass
 * MAVLINK_U16_UNKNOWN where there is no figure. Likewise cog: centidegrees,
 * MAVLINK_U16_UNKNOWN when the vehicle is not moving fast enough for a course
 * over ground to mean anything.
 *
 * The MAVLink 2 extension fields (h_acc, v_acc and friends) are deliberately
 * not sent. They are excluded from CRC_EXTRA, so getting their order wrong
 * would not fail the checksum - the values would just land in the wrong
 * fields, silently. The base message is safe in a way they are not.
 */
uint16_t mavlink_gps_raw_int(uint8_t *out, size_t cap, mavlink_node_t *node,
                             uint64_t time_usec, uint8_t fix_type, uint8_t sats,
                             int32_t lat_1e7, int32_t lon_1e7, int32_t alt_mm,
                             uint16_t eph, uint16_t epv,
                             uint16_t vel_cm_s, uint16_t cog_cdeg);

/*! @brief ATTITUDE, radians. Unused until the AHRS lands; here so the Euler
 *         work has somewhere to go. */
uint16_t mavlink_attitude(uint8_t *out, size_t cap, mavlink_node_t *node,
                          uint32_t time_boot_ms, float roll, float pitch,
                          float yaw, float rollspeed, float pitchspeed,
                          float yawspeed);

#endif /* _MAVLINK_MIN_H_ */
