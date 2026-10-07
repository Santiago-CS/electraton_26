/*
 * gps_ubx.c - u-blox UBX binary protocol: framing, CFG-VALSET and NAV-PVT.
 * See gps_ubx.h for scope and for the warning about the configuration keys.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "gps_ubx.h"

/*******************************************************************************
 * Little-endian field readers
 *
 * UBX is little-endian throughout, which happens to match the M33, but reading
 * byte by byte is free here and keeps the payload buffer unaligned-safe: it
 * sits at an arbitrary offset inside a frame that arrived over DMA.
 ******************************************************************************/

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)rd_u32(p);
}

/*******************************************************************************
 * Checksum
 *
 * 8-bit Fletcher over class, id, the two length bytes and the payload. Not over
 * the sync characters - a frequent source of wrong implementations.
 ******************************************************************************/

static void ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0U;
    uint8_t b = 0U;

    for (uint16_t i = 0U; i < len; i++) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }

    *ck_a = a;
    *ck_b = b;
}

/*******************************************************************************
 * Builders
 ******************************************************************************/

uint8_t ubx_key_value_size(uint32_t key)
{
    switch ((key >> 28) & 0x7U) {
        case 0x1U: /* single bit, stored and sent as one byte */
        case 0x2U: /* U1 / I1 / E1 / X1 */
            return 1U;
        case 0x3U: /* U2 / I2 / X2 */
            return 2U;
        case 0x4U: /* U4 / I4 / X4 / R4 */
            return 4U;
        case 0x5U: /* U8 / I8 / X8 / R8 */
            return 8U;
        default:
            return 0U; /* not a size the configuration database defines */
    }
}

/*
 * Wrap a payload that is already sitting at out[UBX_HEADER_BYTES] - both
 * builders assemble in place rather than through a scratch buffer.
 */
static uint16_t ubx_finish(uint8_t *out, uint8_t msg_class, uint8_t msg_id, uint16_t len)
{
    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = msg_class;
    out[3] = msg_id;
    out[4] = (uint8_t)(len & 0xFFU);
    out[5] = (uint8_t)(len >> 8);

    /* From class through the end of the payload: skip the two sync bytes. */
    ubx_checksum(&out[2], (uint16_t)(len + 4U), &out[UBX_HEADER_BYTES + len],
                 &out[UBX_HEADER_BYTES + len + 1U]);

    return (uint16_t)(UBX_OVERHEAD_BYTES + len);
}

uint16_t gps_ubx_build(uint8_t *out, size_t out_cap, uint8_t msg_class, uint8_t msg_id,
                       const uint8_t *payload, uint16_t len)
{
    if ((out == NULL) || (out_cap < ((size_t)UBX_OVERHEAD_BYTES + (size_t)len))) {
        return 0U;
    }

    if ((len > 0U) && (payload != NULL)) {
        (void)memcpy(&out[UBX_HEADER_BYTES], payload, len);
    } else if (len > 0U) {
        return 0U;
    } else {
        /* zero-length payload: a poll request */
    }

    return ubx_finish(out, msg_class, msg_id, len);
}

uint16_t gps_ubx_build_valset(uint8_t *out, size_t out_cap, uint8_t layers,
                              const gps_ubx_cfg_item_t *items, uint32_t count)
{
    uint16_t len = 4U; /* version, layers, 2 reserved */
    uint8_t *p;

    if ((out == NULL) || (items == NULL) || (count == 0U)) {
        return 0U;
    }

    /* Size the payload up front: a half-written frame must never be sent. */
    for (uint32_t i = 0U; i < count; i++) {
        uint8_t vsize = ubx_key_value_size(items[i].key);

        if (vsize == 0U) {
            return 0U; /* malformed key - see the warning in gps_ubx.h */
        }
        len = (uint16_t)(len + 4U + vsize);
    }

    if (out_cap < ((size_t)UBX_OVERHEAD_BYTES + (size_t)len)) {
        return 0U;
    }

    p = &out[UBX_HEADER_BYTES];

    *p++ = 0x00U;   /* version 0: no transaction */
    *p++ = layers;
    *p++ = 0x00U;   /* reserved */
    *p++ = 0x00U;

    for (uint32_t i = 0U; i < count; i++) {
        uint8_t  vsize = ubx_key_value_size(items[i].key);
        uint32_t key   = items[i].key;
        uint64_t value = items[i].value;

        *p++ = (uint8_t)(key & 0xFFU);
        *p++ = (uint8_t)((key >> 8) & 0xFFU);
        *p++ = (uint8_t)((key >> 16) & 0xFFU);
        *p++ = (uint8_t)((key >> 24) & 0xFFU);

        for (uint8_t b = 0U; b < vsize; b++) {
            *p++ = (uint8_t)((value >> (8U * b)) & 0xFFU);
        }
    }

    return ubx_finish(out, UBX_CLASS_CFG, UBX_ID_VALSET, len);
}

/*******************************************************************************
 * Receiver
 ******************************************************************************/

static void ubx_decode_nav_pvt(const uint8_t *p, gps_ubx_nav_pvt_t *o)
{
    o->itow_ms = rd_u32(&p[0]);
    o->year    = rd_u16(&p[4]);
    o->month   = p[6];
    o->day     = p[7];
    o->hour    = p[8];
    o->minute  = p[9];
    o->second  = p[10];
    o->valid   = p[11];
    /* p[12] tAcc and p[16] nano are not used here. */
    o->fix_type = p[20];
    o->fix_ok   = (p[21] & 0x01U) != 0U; /* flags bit0 gnssFixOK */
    o->num_sv   = p[23];

    o->lon_1e7   = rd_i32(&p[24]);
    o->lat_1e7   = rd_i32(&p[28]);
    o->height_mm = rd_i32(&p[32]);
    o->hmsl_mm   = rd_i32(&p[36]);
    o->h_acc_mm  = rd_u32(&p[40]);
    o->v_acc_mm  = rd_u32(&p[44]);

    o->vel_n_mm_s   = rd_i32(&p[48]);
    o->vel_e_mm_s   = rd_i32(&p[52]);
    o->vel_d_mm_s   = rd_i32(&p[56]);
    o->gspeed_mm_s  = rd_i32(&p[60]);
    o->head_mot_1e5 = rd_i32(&p[64]);
    o->s_acc_mm_s   = rd_u32(&p[68]);
    o->head_acc_1e5 = rd_u32(&p[72]);
    o->pdop_1e2     = rd_u16(&p[76]);

    /*
     * The tail of the payload - flags3, headVeh, magDec, magAcc - is left
     * alone on purpose. Its layout shifted between protocol generations
     * (flags3 widened from X1 to X2), and nothing here needs those fields, so
     * decoding them would be inventing risk for no gain. Everything above is
     * at an offset that has been stable since the protocol was introduced.
     */
}

static void ubx_dispatch(gps_ubx_t *ubx)
{
    if ((ubx->msg_class == UBX_CLASS_NAV) && (ubx->msg_id == UBX_ID_PVT) &&
        (ubx->length >= UBX_NAV_PVT_PAYLOAD_LEN)) {
        ubx_decode_nav_pvt(ubx->payload, &ubx->pvt);
        ubx->pvt_count++;
        return;
    }

    if ((ubx->msg_class == UBX_CLASS_ACK) && (ubx->length >= 2U) &&
        ((ubx->msg_id == UBX_ID_ACK) || (ubx->msg_id == UBX_ID_NAK))) {
        bool is_ack = (ubx->msg_id == UBX_ID_ACK);

        if (is_ack) {
            ubx->ack_count++;
        } else {
            ubx->nak_count++;
        }

        /* The payload names the message being acknowledged, so an ACK that
         * belongs to something else cannot satisfy our wait. */
        if (ubx->ack_pending && (ubx->payload[0] == ubx->ack_wait_class) &&
            (ubx->payload[1] == ubx->ack_wait_id)) {
            ubx->ack_ok      = is_ack;
            ubx->ack_pending = false;
        }
    }
}

void gps_ubx_init(gps_ubx_t *ubx)
{
    (void)memset(ubx, 0, sizeof(*ubx));
    ubx->state = kUbxSync1;
}

bool gps_ubx_feed_byte(gps_ubx_t *ubx, uint8_t byte)
{
    switch (ubx->state) {
        case kUbxSync1:
            if (byte == UBX_SYNC1) {
                ubx->state = kUbxSync2;
                return true;
            }
            return false; /* not ours - hand it back for NMEA */

        case kUbxSync2:
            if (byte == UBX_SYNC2) {
                ubx->state = kUbxClass;
                ubx->ck_a  = 0U;
                ubx->ck_b  = 0U;
            } else if (byte == UBX_SYNC1) {
                /* B5 B5 62: stay armed on the second one. */
            } else {
                ubx->state = kUbxSync1;
                return false;
            }
            return true;

        case kUbxClass:
            ubx->msg_class = byte;
            ubx->ck_a      = (uint8_t)(ubx->ck_a + byte);
            ubx->ck_b      = (uint8_t)(ubx->ck_b + ubx->ck_a);
            ubx->state     = kUbxId;
            return true;

        case kUbxId:
            ubx->msg_id = byte;
            ubx->ck_a   = (uint8_t)(ubx->ck_a + byte);
            ubx->ck_b   = (uint8_t)(ubx->ck_b + ubx->ck_a);
            ubx->state  = kUbxLenLo;
            return true;

        case kUbxLenLo:
            ubx->length = byte;
            ubx->ck_a   = (uint8_t)(ubx->ck_a + byte);
            ubx->ck_b   = (uint8_t)(ubx->ck_b + ubx->ck_a);
            ubx->state  = kUbxLenHi;
            return true;

        case kUbxLenHi:
            ubx->length = (uint16_t)(ubx->length | ((uint16_t)byte << 8));
            ubx->ck_a   = (uint8_t)(ubx->ck_a + byte);
            ubx->ck_b   = (uint8_t)(ubx->ck_b + ubx->ck_a);
            ubx->index  = 0U;
            /* A frame we have no room for is still walked to its checksum, so
             * the parser stays in sync instead of resynchronising on whatever
             * byte inside it happens to look like 0xB5. */
            ubx->oversize = (ubx->length > UBX_MAX_PAYLOAD);
            ubx->state    = (ubx->length == 0U) ? kUbxCkA : kUbxPayload;
            return true;

        case kUbxPayload:
            if (!ubx->oversize) {
                ubx->payload[ubx->index] = byte;
            }
            ubx->index++;
            ubx->ck_a = (uint8_t)(ubx->ck_a + byte);
            ubx->ck_b = (uint8_t)(ubx->ck_b + ubx->ck_a);
            if (ubx->index >= ubx->length) {
                ubx->state = kUbxCkA;
            }
            return true;

        case kUbxCkA:
            ubx->rx_ck_a = byte;
            ubx->state   = kUbxCkB;
            return true;

        case kUbxCkB:
        default:
            if ((ubx->rx_ck_a == ubx->ck_a) && (byte == ubx->ck_b)) {
                ubx->frames_ok++;
                if (ubx->oversize) {
                    ubx->dropped_oversize++;
                } else {
                    ubx_dispatch(ubx);
                }
            } else {
                ubx->checksum_errors++;
            }
            ubx->state = kUbxSync1;
            return true;
    }
}

void gps_ubx_expect_ack(gps_ubx_t *ubx, uint8_t msg_class, uint8_t msg_id)
{
    ubx->ack_wait_class = msg_class;
    ubx->ack_wait_id    = msg_id;
    ubx->ack_ok         = false;
    ubx->ack_pending    = true;
}

bool gps_ubx_cfg_pending(const gps_ubx_t *ubx)
{
    return ubx->ack_pending;
}
