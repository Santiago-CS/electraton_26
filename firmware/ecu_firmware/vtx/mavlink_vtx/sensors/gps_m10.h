/*
 * gps_m10.h - u-blox M10 GNSS receiver: NMEA and UBX over one byte stream.
 *
 * Deliberately decoupled from the UART: feed it whatever bytes arrive with
 * gps_m10_feed() and read the latest fix from gps_m10_data(). That keeps the
 * parser testable and means it does not care whether the port is running
 * polled, interrupt-driven or on eDMA.
 *
 * Both wire formats land in the same gps_m10_data_t, because in practice both
 * are on the wire at once: the receiver streams NMEA until it is told not to,
 * and the UBX-CFG-VALSET that tells it is itself acknowledged in UBX. So
 * gps_m10_feed() demultiplexes - a 0xB5 0x62 pair starts a UBX frame, a '$'
 * starts an NMEA sentence - and whichever arrives updates the same fields.
 *
 * NMEA decoding covers GGA and RMC. UBX decoding covers NAV-PVT, which carries
 * everything those two do plus accuracy estimates, and is what the receiver is
 * configured to emit by gps_m10_build_fast_config().
 *
 * Everything is fixed point: PRINTF_FLOAT_ENABLE is off in this build, and
 * scaled integers are how GNSS data is normally carried anyway.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _GPS_M10_H_
#define _GPS_M10_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gps_ubx.h"

/* An NMEA sentence is capped at 82 characters including "$" and CRLF. */
#define GPS_NMEA_MAX_LEN 84U

#define GPS_PRINT_RATE_MS 1000U /*!< how often the task reports */

/*
 * Single-GNSS 25 Hz mode. Off by default; set it to 1 here or pass
 * -DGPS_SINGLE_GNSS_25HZ=1 to the compiler.
 *
 * This is a real trade, not a free upgrade. The receiver's navigation rate
 * ceiling depends on how many GNSS it is tracking concurrently: 10 Hz for
 * several, 25 Hz for one. Getting to 25 Hz means switching Galileo, BeiDou and
 * GLONASS off, which on this board measurably costs satellite count and
 * geometry - the multi-GNSS fix runs 14 satellites at pDOP 2.13, and GPS alone
 * will see roughly half that with correspondingly worse dilution.
 *
 * Take it only if something downstream genuinely needs 40 ms position updates.
 * For logging, telemetry or anything a human reads, 10 Hz with a better fix is
 * the stronger choice.
 */
#ifndef GPS_SINGLE_GNSS_25HZ
#define GPS_SINGLE_GNSS_25HZ 0
#endif

/*
 * Navigation period, milliseconds - the one number that sets the output rate.
 * The receiver computes a solution every GPS_NAV_PERIOD_MS and emits one
 * NAV-PVT with it.
 *
 * Bandwidth is not the limit either way. One NAV-PVT is 100 bytes on the wire,
 * so 10 Hz is 1000 B/s and 25 Hz is 2500 B/s against 11520 B/s available at
 * 115200 baud - 8.7% and 21.7% of the link.
 */
#if GPS_SINGLE_GNSS_25HZ
#define GPS_NAV_PERIOD_MS 40U
#else
#define GPS_NAV_PERIOD_MS 100U
#endif

/* Room for the largest configuration frame built here. */
#define GPS_UBX_FRAME_MAX 64U

/* Four: the constellation step only exists in single-GNSS mode. */
#define GPS_CFG_STEP_MAX  4U

/*! @brief GGA fix quality field. */
typedef enum {
    kGpsFixNone      = 0,
    kGpsFixGnss      = 1,
    kGpsFixDgnss     = 2,
    kGpsFixRtkFixed  = 4,
    kGpsFixRtkFloat  = 5,
    kGpsFixDeadReck  = 6,
} gps_fix_quality_t;

typedef struct {
    bool    position_valid; /*!< RMC status field was 'A' */
    uint8_t fix_quality;    /*!< gps_fix_quality_t, from GGA */
    uint8_t satellites;     /*!< satellites used, from GGA */

    int32_t lat_1e7; /*!< degrees * 1e7, positive north */
    int32_t lon_1e7; /*!< degrees * 1e7, positive east */
    int32_t alt_mm;  /*!< altitude above mean sea level, millimetres */

    uint32_t speed_mm_s;  /*!< ground speed, millimetres per second */
    uint32_t course_1e3;  /*!< course over ground, degrees * 1000 */
    uint32_t hdop_1e2;    /*!< horizontal dilution of precision * 100, GGA only */

    /* UBX-only fields: NMEA carries no accuracy estimate at all, which is the
     * other reason to prefer NAV-PVT for anything that acts on the position. */
    uint32_t h_acc_mm;  /*!< horizontal accuracy estimate */
    uint32_t v_acc_mm;  /*!< vertical accuracy estimate */
    uint32_t pdop_1e2;  /*!< position dilution of precision * 100 */
    bool     from_ubx;  /*!< the most recent update came from NAV-PVT */

    uint8_t  hour, minute, second;
    uint8_t  day, month;
    uint16_t year; /*!< full year, e.g. 2026 */

    /* Link health - useful well before there is ever a fix. */
    uint32_t sentences_ok;    /*!< any valid sentence, including ones not decoded */
    uint32_t gga_count;       /*!< position/fix sentences actually decoded */
    uint32_t rmc_count;       /*!< velocity/date sentences actually decoded */
    uint32_t checksum_errors; /*!< NMEA checksum failures */
    uint32_t bytes_seen;

    uint32_t pvt_count;       /*!< NAV-PVT frames decoded */
    uint32_t ubx_frames_ok;   /*!< any valid UBX frame, NAV-PVT or not */
    uint32_t ubx_csum_errors;
} gps_m10_data_t;

typedef struct {
    char      line[GPS_NMEA_MAX_LEN];
    uint16_t  len;
    bool      in_sentence;
    gps_ubx_t ubx;          /*!< UBX frame parser, fed from the same stream */
    gps_m10_data_t data;
} gps_m10_t;

/*! @brief One built configuration frame, ready to put on the wire. */
typedef struct {
    const char *name;                     /*!< for the console: which step NAK'd */
    uint8_t     frame[GPS_UBX_FRAME_MAX];
    uint16_t    len;
    /*!
     * How long to wait after this step is ACK'd before sending the next one.
     * Nonzero only for the constellation change, which restarts the receiver's
     * GNSS subsystem: it has to re-acquire on the remaining signals, and
     * pushing the next frame into that window is asking for a NAK.
     */
    uint16_t settle_ms;
} gps_m10_cfg_step_t;

/*! @brief Reset the parser and clear all fields. */
void gps_m10_init(gps_m10_t *gps);

/*!
 * @brief Feed received bytes into the parser.
 * @return true if at least one complete, checksum-valid sentence was parsed
 *         during this call.
 */
bool gps_m10_feed(gps_m10_t *gps, const uint8_t *buf, size_t len);

/*! @brief Latest decoded state. Never NULL for a valid handle. */
const gps_m10_data_t *gps_m10_data(const gps_m10_t *gps);

/*!
 * @brief Build the frames that put the receiver into fast position-only output.
 *
 * Separate UBX-CFG-VALSET frames rather than one, deliberately. Each is
 * acknowledged on its own, so when a configuration key turns out to be wrong
 * for the installed firmware the NAK identifies which group was rejected
 * instead of condemning the whole set. See the key-status note in gps_ubx.h.
 *
 *   1. "gnss"     - single-GNSS only: GPS/SBAS/QZSS on, Galileo/BeiDou/GLONASS
 *                   off. Emitted first, because asking for a 40 ms period
 *                   while three constellations are still enabled is a request
 *                   the receiver is entitled to refuse.
 *   2. "rate"     - navigation period
 *   3. "nav-pvt"  - enable UBX-NAV-PVT on UART1
 *   4. "nmea-off" - silence GGA, RMC, GSV, GSA, GLL and VTG
 *
 * Step 4 is where most of the bandwidth saving is: GSV alone is the bulk of a
 * default NMEA burst and nothing decodes it.
 *
 * Writes RAM only, so a power cycle restores the factory configuration and a
 * wrong key cannot be made permanent.
 *
 * @param nav_period_ms measurement period; GPS_NAV_PERIOD_MS is the sane value.
 * @param single_gnss   true adds step 1 and is required for a period below
 *                      100 ms. Pass GPS_SINGLE_GNSS_25HZ.
 * @return number of steps written (3, or 4 with single_gnss), or 0 if a frame
 *         could not be built.
 */
uint32_t gps_m10_build_fast_config(gps_m10_cfg_step_t *steps, uint32_t max_steps,
                                   uint16_t nav_period_ms, bool single_gnss);

#endif /* _GPS_M10_H_ */
