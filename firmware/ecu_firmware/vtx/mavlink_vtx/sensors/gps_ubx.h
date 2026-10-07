/*
 * gps_ubx.h - u-blox UBX binary protocol: framing, CFG-VALSET and NAV-PVT.
 *
 * Why UBX at all, when the receiver already streams NMEA:
 *
 *   - NMEA cannot configure anything. The navigation rate, which constellations
 *     are tracked and which messages are emitted are all UBX-only settings, so
 *     "run the GPS as fast as it will go" is necessarily a UBX job.
 *   - For position alone, one UBX-NAV-PVT frame (100 bytes on the wire) carries
 *     everything GGA and RMC carry plus accuracy estimates, already in the
 *     scaled-integer form this code wants. No ASCII, no per-sentence checksum
 *     walk, no field splitting, no waiting for a second sentence to pair up
 *     position with velocity.
 *
 * Scope: the frame codec, CFG-VALSET, ACK-ACK/ACK-NAK and NAV-PVT. That is the
 * minimum needed to configure the receiver and read a fix back. Everything is
 * transport-agnostic in the same way gps_m10.h is - this module never touches a
 * UART, it just builds byte arrays and consumes byte arrays.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _GPS_UBX_H_
#define _GPS_UBX_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*******************************************************************************
 * Frame layout
 *
 *   B5 62 | class | id | len_lo len_hi | payload[len] | ck_a ck_b
 *
 * The two checksum bytes are an 8-bit Fletcher over class, id, both length
 * bytes and the payload - NOT over the sync characters.
 ******************************************************************************/

#define UBX_SYNC1 0xB5U
#define UBX_SYNC2 0x62U

#define UBX_HEADER_BYTES   6U /*!< sync1, sync2, class, id, len_lo, len_hi */
#define UBX_CHECKSUM_BYTES 2U
#define UBX_OVERHEAD_BYTES (UBX_HEADER_BYTES + UBX_CHECKSUM_BYTES)

/* Longest payload this module needs to receive. NAV-PVT is 92 bytes. */
#define UBX_MAX_PAYLOAD 96U

/* Message classes and ids used here. */
#define UBX_CLASS_NAV 0x01U
#define UBX_ID_PVT    0x07U

#define UBX_CLASS_ACK 0x05U
#define UBX_ID_NAK    0x00U
#define UBX_ID_ACK    0x01U

#define UBX_CLASS_CFG 0x06U
#define UBX_ID_VALSET 0x8AU

#define UBX_NAV_PVT_PAYLOAD_LEN 92U

/*******************************************************************************
 * Configuration keys
 *
 * The framing and checksum in this file are from the protocol definition and
 * are not in doubt. The 32-bit key constants are written from recollection of
 * the u-blox M9/M10 configuration database, so their status differs by group:
 *
 *   VERIFIED ON HARDWARE - UBX_KEY_RATE_* and UBX_KEY_MSGOUT_*. All three
 *   VALSET steps built from them were ACK'd by the module on this board, and
 *   the resulting NAV-PVT stream decoded to a position that agrees with the
 *   NMEA it replaced to within 15 cm. These are good.
 *
 *   NOT YET VERIFIED - UBX_KEY_SIGNAL_*, used only by the 25 Hz single-GNSS
 *   path. Same provenance, different group, so the precedent above is
 *   encouraging but not evidence.
 *
 * A wrong key is accepted silently by the builder and then rejected by the
 * receiver. The code checks for that - every write is ACK'd or NAK'd and
 * gps_ubx_cfg_pending() reports which - so a bad key shows up as a NAK naming
 * the step rather than as mysteriously absent behaviour. Cross-check in
 * u-center: configure the receiver interactively, watch the UBX-CFG-VALSET it
 * emits, and compare bytes.
 *
 * Key encoding, which IS from the protocol definition:
 *   bits  0..11  item id
 *   bits 16..23  group id
 *   bits 28..30  storage size code (see ubx_key_value_size())
 * so the value width is derivable from the key itself, and this module does
 * derive it rather than trusting the caller to pass a matching size.
 *
 * The UART1 keys below are the I2C key + 1: the port order within a MSGOUT
 * group is I2C, UART1, UART2, USB, SPI.
 ******************************************************************************/

#define UBX_KEY_RATE_MEAS 0x30210001U /*!< U2, measurement period in ms */
#define UBX_KEY_RATE_NAV  0x30210002U /*!< U2, measurements per nav solution */

#define UBX_KEY_MSGOUT_UBX_NAV_PVT_UART1 0x20910007U /*!< U1, frames per nav solution */

#define UBX_KEY_MSGOUT_NMEA_GGA_UART1 0x209100BBU /*!< U1 */
#define UBX_KEY_MSGOUT_NMEA_RMC_UART1 0x209100ACU /*!< U1 */
#define UBX_KEY_MSGOUT_NMEA_GSV_UART1 0x209100C5U /*!< U1 */
#define UBX_KEY_MSGOUT_NMEA_GSA_UART1 0x209100C0U /*!< U1 */
#define UBX_KEY_MSGOUT_NMEA_GLL_UART1 0x209100CAU /*!< U1 */
#define UBX_KEY_MSGOUT_NMEA_VTG_UART1 0x209100B1U /*!< U1 */

/*
 * Constellation enables, for single-GNSS operation. Size code 0x1 - a single
 * bit, carried as one byte.
 *
 * GPS, SBAS and QZSS are not three GNSS as far as the receiver's rate limit is
 * concerned: SBAS and QZSS ride on the GPS L1 C/A signal and cannot be enabled
 * without GPS. So GPS + SBAS + QZSS still counts as "single GNSS" and is the
 * best satellite count available at 25 Hz. Galileo, BeiDou and GLONASS are the
 * ones that force the receiver down to 10 Hz.
 */
#define UBX_KEY_SIGNAL_GPS_ENA  0x1031001FU /*!< L (1 bit) */
#define UBX_KEY_SIGNAL_SBAS_ENA 0x10310020U /*!< L */
#define UBX_KEY_SIGNAL_GAL_ENA  0x10310021U /*!< L */
#define UBX_KEY_SIGNAL_BDS_ENA  0x10310022U /*!< L */
#define UBX_KEY_SIGNAL_QZSS_ENA 0x10310024U /*!< L */
#define UBX_KEY_SIGNAL_GLO_ENA  0x10310025U /*!< L */

/* Which non-volatile layers a VALSET writes. RAM alone is deliberate here:
 * it takes effect immediately and is forgotten on power cycle, so a bad
 * experiment cannot brick the module's configuration. */
#define UBX_LAYER_RAM   0x01U
#define UBX_LAYER_BBR   0x02U
#define UBX_LAYER_FLASH 0x04U

/*! @brief One key/value pair for gps_ubx_build_valset(). */
typedef struct {
    uint32_t key;
    uint64_t value; /*!< widened; only the low ubx_key_value_size(key) bytes are sent */
} gps_ubx_cfg_item_t;

/*******************************************************************************
 * Decoded NAV-PVT
 ******************************************************************************/

/*! @brief UBX-NAV-PVT fixType field. */
typedef enum {
    kUbxFixNone       = 0,
    kUbxFixDeadReck   = 1,
    kUbxFix2D         = 2,
    kUbxFix3D         = 3,
    kUbxFixGnssDeadRe = 4,
    kUbxFixTimeOnly   = 5,
} gps_ubx_fix_type_t;

typedef struct {
    uint32_t itow_ms;  /*!< GPS time of week */
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    uint8_t  valid;    /*!< bit0 validDate, bit1 validTime, bit2 fullyResolved */

    uint8_t fix_type;  /*!< gps_ubx_fix_type_t */
    bool    fix_ok;    /*!< flags bit0 gnssFixOK: within the configured limits */
    uint8_t num_sv;

    int32_t lon_1e7;   /*!< degrees * 1e7 */
    int32_t lat_1e7;   /*!< degrees * 1e7 */
    int32_t height_mm; /*!< above the ellipsoid */
    int32_t hmsl_mm;   /*!< above mean sea level - the GGA-comparable one */

    uint32_t h_acc_mm; /*!< horizontal accuracy estimate */
    uint32_t v_acc_mm; /*!< vertical accuracy estimate */

    int32_t  vel_n_mm_s, vel_e_mm_s, vel_d_mm_s;
    int32_t  gspeed_mm_s; /*!< 2-D ground speed */
    int32_t  head_mot_1e5; /*!< heading of motion, degrees * 1e5 */
    uint32_t s_acc_mm_s;
    uint32_t head_acc_1e5;
    uint16_t pdop_1e2;
} gps_ubx_nav_pvt_t;

/*******************************************************************************
 * Incremental receiver
 ******************************************************************************/

typedef enum {
    kUbxSync1 = 0,
    kUbxSync2,
    kUbxClass,
    kUbxId,
    kUbxLenLo,
    kUbxLenHi,
    kUbxPayload,
    kUbxCkA,
    kUbxCkB,
} gps_ubx_state_t;

typedef struct {
    gps_ubx_state_t state;
    uint8_t         msg_class, msg_id;
    uint16_t        length;
    uint16_t        index;
    uint8_t         ck_a, ck_b;      /*!< running, computed */
    uint8_t         rx_ck_a;
    uint8_t         payload[UBX_MAX_PAYLOAD];
    bool            oversize;        /*!< length > UBX_MAX_PAYLOAD: checksum it, drop it */

    /* Latest NAV-PVT, valid once pvt_count is non-zero. */
    gps_ubx_nav_pvt_t pvt;
    uint32_t          pvt_count;

    /* Outstanding CFG write, so an ACK can be matched to what asked for it. */
    uint8_t  ack_wait_class, ack_wait_id;
    bool     ack_pending;
    bool     ack_ok;       /*!< meaningful once ack_pending has gone false */
    uint32_t ack_count, nak_count;

    /* Link health. */
    uint32_t frames_ok;
    uint32_t checksum_errors;
    uint32_t dropped_oversize;
} gps_ubx_t;

/*! @brief Reset the frame parser. Does not clear the counters' meaning for the
 *         caller, it clears everything - call once at startup. */
void gps_ubx_init(gps_ubx_t *ubx);

/*!
 * @brief Offer one byte to the UBX parser.
 *
 * @return true if the byte belongs to a UBX frame and has been consumed. False
 *         means the parser is idle and the byte is not a sync character, so the
 *         caller should route it elsewhere (NMEA, in this project).
 */
bool gps_ubx_feed_byte(gps_ubx_t *ubx, uint8_t byte);

/*! @brief Value width in bytes implied by a configuration key, or 0 if the
 *         key's size field is not one the database defines. */
uint8_t ubx_key_value_size(uint32_t key);

/*!
 * @brief Build a complete UBX frame into out.
 * @return total frame length, or 0 if out is too small or len exceeds the cap.
 */
uint16_t gps_ubx_build(uint8_t *out, size_t out_cap, uint8_t msg_class, uint8_t msg_id,
                       const uint8_t *payload, uint16_t len);

/*!
 * @brief Build a UBX-CFG-VALSET frame setting every item in one transaction.
 *
 * @param layers  bitwise OR of UBX_LAYER_*.
 * @return total frame length, or 0 if an item's key has an unknown size code or
 *         out is too small.
 */
uint16_t gps_ubx_build_valset(uint8_t *out, size_t out_cap, uint8_t layers,
                              const gps_ubx_cfg_item_t *items, uint32_t count);

/*! @brief Note that a frame has been sent and its ACK is now expected. */
void gps_ubx_expect_ack(gps_ubx_t *ubx, uint8_t msg_class, uint8_t msg_id);

/*! @brief True while the ACK for the last gps_ubx_expect_ack() has not arrived. */
bool gps_ubx_cfg_pending(const gps_ubx_t *ubx);

#endif /* _GPS_UBX_H_ */
