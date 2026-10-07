/*
 * gps_m10.c - u-blox M10 GNSS receiver, NMEA and UBX over one byte stream.
 * See gps_m10.h for the API and what is in scope.
 *
 * All parsing is integer. Not because floats would be wrong, but because the
 * values are naturally fixed point, the results are then exactly printable
 * with %d, and nothing depends on how newlib-nano was configured.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "gps_m10.h"

#define NMEA_MAX_FIELDS 20U

/* ARRAY_SIZE comes from the SDK headers, which this file deliberately does not
 * pull in - it is built and unit-tested on the host as well. */
#define ARRAY_COUNT(a) (uint32_t)(sizeof(a) / sizeof((a)[0]))

/*******************************************************************************
 * Small parsing helpers
 ******************************************************************************/

static bool is_digit(char c)
{
    return (c >= '0') && (c <= '9');
}

static uint8_t hex_val(char c)
{
    if (is_digit(c)) {
        return (uint8_t)(c - '0');
    }
    if ((c >= 'A') && (c <= 'F')) {
        return (uint8_t)(c - 'A' + 10);
    }
    if ((c >= 'a') && (c <= 'f')) {
        return (uint8_t)(c - 'a' + 10);
    }
    return 0xFFU;
}

/* Plain unsigned decimal. Stops at the first non-digit. */
static uint32_t parse_uint(const char *s, size_t len)
{
    uint32_t v = 0U;

    for (size_t i = 0U; (i < len) && is_digit(s[i]); i++) {
        v = (v * 10U) + (uint32_t)(s[i] - '0');
    }
    return v;
}

/*
 * Parse a decimal like "123.456" into value * 10^scale, rounding nothing and
 * tolerating a missing or short fractional part. Returns false on an empty
 * field so callers can leave the previous value alone.
 */
static bool parse_scaled(const char *s, size_t len, uint32_t scale, uint32_t *out)
{
    uint32_t v     = 0U;
    size_t   i     = 0U;
    uint32_t decs  = 0U;
    bool     seen  = false;

    for (; (i < len) && is_digit(s[i]); i++) {
        v    = (v * 10U) + (uint32_t)(s[i] - '0');
        seen = true;
    }

    if (!seen) {
        return false;
    }

    if ((i < len) && (s[i] == '.')) {
        i++;
        for (; (i < len) && is_digit(s[i]) && (decs < scale); i++) {
            v = (v * 10U) + (uint32_t)(s[i] - '0');
            decs++;
        }
    }

    /* Pad out to the requested scale. */
    while (decs < scale) {
        v *= 10U;
        decs++;
    }

    *out = v;
    return true;
}

/*
 * NMEA latitude is ddmm.mmmm and longitude dddmm.mmmm - degrees and minutes
 * jammed together, which is the one genuinely awkward thing about the format.
 * deg_digits is 2 for latitude, 3 for longitude.
 */
static bool parse_lat_lon(const char *s, size_t len, uint32_t deg_digits, char hemi, int32_t *out_1e7)
{
    uint32_t degrees;
    uint32_t minutes_1e5;
    uint64_t result;

    if (len < (deg_digits + 1U)) {
        return false;
    }

    degrees = parse_uint(s, deg_digits);

    if (!parse_scaled(&s[deg_digits], len - deg_digits, 5U, &minutes_1e5)) {
        return false;
    }

    /* minutes/60 in degrees, carried at 1e7: (min * 1e5) * 100 / 60. The
     * intermediate reaches 6e8 for 59.99999 minutes, so 64-bit to be safe. */
    result = ((uint64_t)degrees * 10000000ULL) + (((uint64_t)minutes_1e5 * 100ULL) / 60ULL);

    *out_1e7 = (int32_t)result;
    if ((hemi == 'S') || (hemi == 'W')) {
        *out_1e7 = -*out_1e7;
    }

    return true;
}

/*******************************************************************************
 * Sentence handling
 ******************************************************************************/

/*
 * Split on commas in place. Returns the field count. field[0] is the talker +
 * type, e.g. "GNGGA". The caller owns the buffer, which is modified.
 */
static uint32_t nmea_split(char *line, char *field[], uint32_t max_fields)
{
    uint32_t n   = 0U;
    char    *p   = line;

    field[n++] = p;
    while ((*p != '\0') && (n < max_fields)) {
        if (*p == ',') {
            *p         = '\0';
            field[n++] = p + 1;
        }
        p++;
    }
    return n;
}

/* The two characters after '*' are the XOR of everything between '$' and '*'. */
static bool nmea_checksum_ok(const char *line, uint16_t len)
{
    uint8_t  sum = 0U;
    uint16_t i;

    for (i = 0U; i < len; i++) {
        if (line[i] == '*') {
            break;
        }
        sum ^= (uint8_t)line[i];
    }

    if ((i + 2U) >= len) {
        return false; /* no checksum present */
    }

    uint8_t hi = hex_val(line[i + 1U]);
    uint8_t lo = hex_val(line[i + 2U]);
    if ((hi == 0xFFU) || (lo == 0xFFU)) {
        return false;
    }

    return sum == (uint8_t)((hi << 4) | lo);
}

/* "hhmmss.ss" */
static void parse_time(const char *s, size_t len, gps_m10_data_t *d)
{
    if (len >= 6U) {
        d->hour   = (uint8_t)parse_uint(&s[0], 2U);
        d->minute = (uint8_t)parse_uint(&s[2], 2U);
        d->second = (uint8_t)parse_uint(&s[4], 2U);
    }
}

/* "ddmmyy" */
static void parse_date(const char *s, size_t len, gps_m10_data_t *d)
{
    if (len >= 6U) {
        d->day   = (uint8_t)parse_uint(&s[0], 2U);
        d->month = (uint8_t)parse_uint(&s[2], 2U);
        d->year  = (uint16_t)(2000U + parse_uint(&s[4], 2U));
    }
}

/*
 * GGA: time, lat, N/S, lon, E/W, fix quality, satellites, HDOP, altitude, ...
 * Any talker ID is accepted - the M10 emits GN when several constellations are
 * in use, GP when only GPS is.
 */
static void handle_gga(char *field[], uint32_t n, gps_m10_data_t *d)
{
    uint32_t v;

    if (n < 10U) {
        return;
    }

    parse_time(field[1], strlen(field[1]), d);

    (void)parse_lat_lon(field[2], strlen(field[2]), 2U, field[3][0], &d->lat_1e7);
    (void)parse_lat_lon(field[4], strlen(field[4]), 3U, field[5][0], &d->lon_1e7);

    d->fix_quality = (uint8_t)parse_uint(field[6], strlen(field[6]));
    d->satellites  = (uint8_t)parse_uint(field[7], strlen(field[7]));

    if (parse_scaled(field[8], strlen(field[8]), 2U, &v)) {
        d->hdop_1e2 = v;
    }

    /* Altitude may be negative; parse_scaled does not take a sign. */
    if (field[9][0] == '-') {
        if (parse_scaled(&field[9][1], strlen(field[9]) - 1U, 3U, &v)) {
            d->alt_mm = -(int32_t)v;
        }
    } else if (parse_scaled(field[9], strlen(field[9]), 3U, &v)) {
        d->alt_mm = (int32_t)v;
    } else {
        /* leave the previous altitude in place */
    }
}

/*
 * RMC: time, status, lat, N/S, lon, E/W, speed (knots), course, date, ...
 */
static void handle_rmc(char *field[], uint32_t n, gps_m10_data_t *d)
{
    uint32_t v;

    if (n < 10U) {
        return;
    }

    parse_time(field[1], strlen(field[1]), d);
    d->position_valid = (field[2][0] == 'A');

    (void)parse_lat_lon(field[3], strlen(field[3]), 2U, field[4][0], &d->lat_1e7);
    (void)parse_lat_lon(field[5], strlen(field[5]), 3U, field[6][0], &d->lon_1e7);

    /* Knots to mm/s: 1 knot = 514.444 mm/s, so speed_1e3 * 514444 / 1e6. */
    if (parse_scaled(field[7], strlen(field[7]), 3U, &v)) {
        d->speed_mm_s = (uint32_t)(((uint64_t)v * 514444ULL) / 1000000ULL);
    }

    if (parse_scaled(field[8], strlen(field[8]), 3U, &v)) {
        d->course_1e3 = v;
    }

    parse_date(field[9], strlen(field[9]), d);
}

static void nmea_dispatch(gps_m10_t *gps)
{
    char    *field[NMEA_MAX_FIELDS];
    uint32_t n;
    char    *type;

    /* Trim the checksum so the last field does not carry "*hh". */
    for (uint16_t i = 0U; i < gps->len; i++) {
        if (gps->line[i] == '*') {
            gps->line[i] = '\0';
            gps->len     = i;
            break;
        }
    }
    gps->line[gps->len] = '\0';

    n = nmea_split(gps->line, field, NMEA_MAX_FIELDS);
    if (n < 2U) {
        return;
    }

    /* field[0] is "GNGGA", "GPRMC" and so on - skip the two-letter talker. */
    type = field[0];
    if (strlen(type) < 5U) {
        return;
    }
    type += 2;

    if (strncmp(type, "GGA", 3) == 0) {
        handle_gga(field, n, &gps->data);
        gps->data.gga_count++;
        gps->data.from_ubx = false;
    } else if (strncmp(type, "RMC", 3) == 0) {
        handle_rmc(field, n, &gps->data);
        gps->data.rmc_count++;
        gps->data.from_ubx = false;
    } else {
        /* GSA, GSV, VTG, GLL and friends: counted as healthy traffic but not
         * decoded - GGA and RMC already carry everything in gps_m10_data_t. */
    }
}

/*******************************************************************************
 * UBX
 ******************************************************************************/

/*
 * Fold a decoded NAV-PVT into the same fields the NMEA path fills, so callers
 * do not care which format produced the fix.
 */
static void absorb_nav_pvt(const gps_ubx_nav_pvt_t *p, gps_m10_data_t *d)
{
    d->from_ubx = true;

    /*
     * "Valid" means the receiver says the solution is inside its configured
     * limits AND it is at least a 2-D fix. fix_ok alone is not enough: it is
     * also set for a time-only solution, which has no position.
     */
    d->position_valid = p->fix_ok && ((p->fix_type == (uint8_t)kUbxFix2D) ||
                                      (p->fix_type == (uint8_t)kUbxFix3D) ||
                                      (p->fix_type == (uint8_t)kUbxFixGnssDeadRe));

    /* Map onto the GGA quality codes the struct already uses, so the two paths
     * report the same thing. The M10 has no RTK, so 2-D, 3-D and GNSS+DR all
     * come out as a plain GNSS fix. */
    switch (p->fix_type) {
        case (uint8_t)kUbxFixDeadReck:
            d->fix_quality = (uint8_t)kGpsFixDeadReck;
            break;
        case (uint8_t)kUbxFix2D:
        case (uint8_t)kUbxFix3D:
        case (uint8_t)kUbxFixGnssDeadRe:
            d->fix_quality = (uint8_t)kGpsFixGnss;
            break;
        default: /* no fix, or time only */
            d->fix_quality = (uint8_t)kGpsFixNone;
            break;
    }

    d->satellites = p->num_sv;

    d->lat_1e7 = p->lat_1e7;
    d->lon_1e7 = p->lon_1e7;
    d->alt_mm  = p->hmsl_mm; /* height above mean sea level, as GGA reports */

    d->speed_mm_s = (p->gspeed_mm_s < 0) ? 0U : (uint32_t)p->gspeed_mm_s;

    /* headMot is degrees * 1e5; the struct carries degrees * 1e3. */
    {
        int32_t course_1e3 = p->head_mot_1e5 / 100;

        if (course_1e3 < 0) {
            course_1e3 += 360000;
        }
        if ((course_1e3 >= 0) && (course_1e3 < 360000)) {
            d->course_1e3 = (uint32_t)course_1e3;
        }
    }

    d->h_acc_mm = p->h_acc_mm;
    d->v_acc_mm = p->v_acc_mm;
    d->pdop_1e2 = p->pdop_1e2;

    /* Only publish a date the receiver says is resolved, or the clock reads
     * 2000-00-00 before the first fix. valid bit0 is validDate, bit1 validTime. */
    if ((p->valid & 0x01U) != 0U) {
        d->year  = p->year;
        d->month = p->month;
        d->day   = p->day;
    }
    if ((p->valid & 0x02U) != 0U) {
        d->hour   = p->hour;
        d->minute = p->minute;
        d->second = p->second;
    }
}

uint32_t gps_m10_build_fast_config(gps_m10_cfg_step_t *steps, uint32_t max_steps,
                                   uint16_t nav_period_ms, bool single_gnss)
{
    uint32_t n = 0U;

    if ((steps == NULL) || (max_steps < GPS_CFG_STEP_MAX) || (nav_period_ms == 0U)) {
        return 0U;
    }

    /*
     * A period under 100 ms needs single-GNSS operation, so refuse the
     * combination rather than build frames the receiver will reject - or
     * worse, accept while silently clamping the rate.
     */
    if ((nav_period_ms < 100U) && !single_gnss) {
        return 0U;
    }

    /*
     * 1. Constellations, first. GPS, SBAS and QZSS stay on: they share the GPS
     *    L1 C/A signal and do not count against the single-GNSS rate limit, so
     *    leaving them enabled is free satellite count. Galileo, BeiDou and
     *    GLONASS are what hold the receiver to 10 Hz.
     *
     *    Every value is written explicitly rather than only the three being
     *    switched off, so the result does not depend on what the module was
     *    configured with beforehand.
     */
    if (single_gnss) {
        const gps_ubx_cfg_item_t items[] = {
            {UBX_KEY_SIGNAL_GPS_ENA, 1U},
            {UBX_KEY_SIGNAL_SBAS_ENA, 1U},
            {UBX_KEY_SIGNAL_QZSS_ENA, 1U},
            {UBX_KEY_SIGNAL_GAL_ENA, 0U},
            {UBX_KEY_SIGNAL_BDS_ENA, 0U},
            {UBX_KEY_SIGNAL_GLO_ENA, 0U},
        };

        steps[n].name = "gnss";
        steps[n].len  = gps_ubx_build_valset(steps[n].frame, sizeof(steps[n].frame),
                                             UBX_LAYER_RAM, items, ARRAY_COUNT(items));
        if (steps[n].len == 0U) {
            return 0U;
        }
        /* The GNSS subsystem restarts and re-acquires on the remaining
         * signals. Give it a moment before pushing the next frame. */
        steps[n].settle_ms = 500U;
        n++;
    }

    /* 2. Navigation period. RATE_NAV = 1 means one solution per measurement. */
    {
        const gps_ubx_cfg_item_t items[] = {
            {UBX_KEY_RATE_MEAS, nav_period_ms},
            {UBX_KEY_RATE_NAV, 1U},
        };

        steps[n].name      = "rate";
        steps[n].settle_ms = 0U;
        steps[n].len       = gps_ubx_build_valset(steps[n].frame, sizeof(steps[n].frame),
                                                  UBX_LAYER_RAM, items, ARRAY_COUNT(items));
        if (steps[n].len == 0U) {
            return 0U;
        }
        n++;
    }

    /* 3. One NAV-PVT per navigation solution on UART1. */
    {
        const gps_ubx_cfg_item_t items[] = {
            {UBX_KEY_MSGOUT_UBX_NAV_PVT_UART1, 1U},
        };

        steps[n].name      = "nav-pvt";
        steps[n].settle_ms = 0U;
        steps[n].len       = gps_ubx_build_valset(steps[n].frame, sizeof(steps[n].frame),
                                                  UBX_LAYER_RAM, items, ARRAY_COUNT(items));
        if (steps[n].len == 0U) {
            return 0U;
        }
        n++;
    }

    /*
     * 4. Silence NMEA. Six keys, one frame: these all live in the same group
     * and either all work or all do not, so splitting them further would buy
     * no extra diagnosis.
     */
    {
        const gps_ubx_cfg_item_t items[] = {
            {UBX_KEY_MSGOUT_NMEA_GGA_UART1, 0U},
            {UBX_KEY_MSGOUT_NMEA_RMC_UART1, 0U},
            {UBX_KEY_MSGOUT_NMEA_GSV_UART1, 0U},
            {UBX_KEY_MSGOUT_NMEA_GSA_UART1, 0U},
            {UBX_KEY_MSGOUT_NMEA_GLL_UART1, 0U},
            {UBX_KEY_MSGOUT_NMEA_VTG_UART1, 0U},
        };

        steps[n].name      = "nmea-off";
        steps[n].settle_ms = 0U;
        steps[n].len       = gps_ubx_build_valset(steps[n].frame, sizeof(steps[n].frame),
                                                  UBX_LAYER_RAM, items, ARRAY_COUNT(items));
        if (steps[n].len == 0U) {
            return 0U;
        }
        n++;
    }

    return n;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

void gps_m10_init(gps_m10_t *gps)
{
    (void)memset(gps, 0, sizeof(*gps));
    gps_ubx_init(&gps->ubx);
}

bool gps_m10_feed(gps_m10_t *gps, const uint8_t *buf, size_t len)
{
    bool parsed = false;

    if ((gps == NULL) || (buf == NULL)) {
        return false;
    }

    for (size_t i = 0U; i < len; i++) {
        char c = (char)buf[i];

        gps->data.bytes_seen++;

        /*
         * UBX first. The parser claims a byte only while it is inside a frame
         * or looking at a sync character, so NMEA sees everything else. The two
         * cannot be confused: UBX sync is 0xB5 0x62 and NMEA is pure ASCII, so
         * a 0xB5 arriving mid-sentence means that sentence was already corrupt.
         */
        if (gps_ubx_feed_byte(&gps->ubx, buf[i])) {
            uint32_t seen = gps->ubx.pvt_count;

            if (seen != gps->data.pvt_count) {
                gps->data.pvt_count = seen;
                absorb_nav_pvt(&gps->ubx.pvt, &gps->data);
                parsed = true;
            }
            gps->data.ubx_frames_ok   = gps->ubx.frames_ok;
            gps->data.ubx_csum_errors = gps->ubx.checksum_errors;
            continue;
        }

        if (c == '$') {
            /* Resynchronise unconditionally: a '$' can only start a sentence,
             * so seeing one mid-line means the previous one was truncated. */
            gps->in_sentence = true;
            gps->len         = 0U;
            continue;
        }

        if (!gps->in_sentence) {
            continue;
        }

        if ((c == '\r') || (c == '\n')) {
            if (gps->len > 0U) {
                gps->line[gps->len] = '\0';
                if (nmea_checksum_ok(gps->line, gps->len)) {
                    gps->data.sentences_ok++;
                    nmea_dispatch(gps);
                    parsed = true;
                } else {
                    gps->data.checksum_errors++;
                }
            }
            gps->in_sentence = false;
            gps->len         = 0U;
            continue;
        }

        if (gps->len < (GPS_NMEA_MAX_LEN - 1U)) {
            gps->line[gps->len++] = c;
        } else {
            /* Overlong: drop it rather than wrap and corrupt the next one. */
            gps->in_sentence = false;
            gps->len         = 0U;
        }
    }

    return parsed;
}

const gps_m10_data_t *gps_m10_data(const gps_m10_t *gps)
{
    return &gps->data;
}
