/*
 * pmac_telem.h - motor controller telemetry, Linux -> M33 over RPMsg.
 *
 * Shared by both ends: the M33 firmware (comms/pmac_link.c) and the Linux
 * daemon that reads can0 and decodes the PMAC with speaker/pmac.c
 * (vtx/can2m33/can2m33.c). Change it in one place and rebuild both.
 *
 * One message per RPMsg packet, sent by the daemon at a fixed rate. Both
 * cores are little-endian with IEEE-754 floats, and the struct is packed, so
 * it goes on the wire as-is. Values are already in engineering units - the
 * decoding is done once, on Linux, by the decoder that was checked against
 * the real controller.
 *
 * The RPMsg channel is announced by the M33 as "rpmsg-raw", which the Linux
 * rpmsg_char driver binds to and exposes as /dev/rpmsgN. Every write() there
 * is one message, so message boundaries survive - unlike a tty.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PMAC_TELEM_H
#define PMAC_TELEM_H

#include <stdint.h>

#define PMAC_TELEM_CHANNEL  "rpmsg-raw"
#define PMAC_TELEM_EPT_ADDR 30U       /* the M33's endpoint address */

#define PMAC_TELEM_MAGIC    0x4D50U   /* "PM" little-endian */
#define PMAC_TELEM_VERSION  1U

/* Index into pmac_telem_t.v[]. */
enum {
    PMAC_TV_SPEED_RPM,   /* 606Ch velocity actual, rpm                     */
    PMAC_TV_TORQUE_PU,   /* 2020h/6 torque, fraction of rated (-1..1 ish)  */
    PMAC_TV_BATT_A,      /* 4600h/2 battery current, A                     */
    PMAC_TV_CAP_V,       /* 4600h/3 DC link capacitor voltage, V           */
    PMAC_TV_MOTOR_C,     /* motor temperature, degC                        */
    PMAC_TV_HEATSINK_C,  /* 4600h/4 heatsink temperature, degC             */
    PMAC_TV_IQ_A,        /* 4600h/8 Iq actual, A                           */
    PMAC_TV_ID_A,        /* 4600h/7 Id actual, A                           */
    PMAC_TV_MOD_PCT,     /* 4600h/11 modulation, %                         */
    PMAC_TV_COUNT
};

#define PMAC_NMT_NONE 0xFFU           /* no heartbeat seen */

#if defined(__GNUC__)
#define PMAC_TELEM_PACKED __attribute__((packed))
#else
#error "pmac_telem_t needs a packed struct"
#endif

typedef struct PMAC_TELEM_PACKED {
    uint16_t magic;      /* PMAC_TELEM_MAGIC                                 */
    uint8_t  version;    /* PMAC_TELEM_VERSION                               */
    uint8_t  nmt;        /* controller heartbeat state: 0x05 Operational,
                            0x7F Pre-op, 0x04 Stopped; PMAC_NMT_NONE        */
    uint32_t seq;        /* +1 per message, so the M33 can count gaps        */
    uint32_t valid;      /* bit n set: v[n] was received and is fresh        */
    uint16_t fault_id;   /* Sevcon fault id of an active EMCY, 0 = none      */
    uint16_t emcy_code;  /* CANopen emergency error code, 0 = none           */
    float    v[PMAC_TV_COUNT];
} pmac_telem_t;

/* 16-byte header + 9 floats. Pinned so a change on one side cannot silently
 * shift every field on the other. */
_Static_assert(sizeof(pmac_telem_t) == 16U + 4U * PMAC_TV_COUNT, "pmac_telem_t layout");

#endif /* PMAC_TELEM_H */
