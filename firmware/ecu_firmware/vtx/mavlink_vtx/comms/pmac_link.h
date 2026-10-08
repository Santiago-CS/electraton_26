/*
 * pmac_link.h - motor controller telemetry from Linux, over RPMsg.
 *
 * Linux reads can0, decodes the PMAC and sends one pmac_telem_t per RPMsg
 * message (vtx/can2m33). This side announces the channel, keeps the latest
 * message, and hands copies to whoever builds MAVLink.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PMAC_LINK_H
#define PMAC_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "pmac_telem.h"

/*! @brief Create the task that brings up RPMsg and announces the channel.
 *         Returns false if the task could not be created. */
bool pmac_link_start(void);

/*!
 * @brief Copy of the latest message.
 * @return its age in milliseconds, or UINT32_MAX if nothing has arrived since
 *         power-up (Linux side not running, or the channel not up yet).
 */
uint32_t pmac_link_latest(pmac_telem_t *out);

/* Counters, for a debugger or a status message. */
typedef struct {
    bool     announced; /*!< channel announced to Linux */
    uint32_t rx;        /*!< good messages */
    uint32_t bad;       /*!< wrong length, magic or version - dropped */
    uint32_t gaps;      /*!< messages missed, from the sequence number */
} pmac_link_stats_t;

void pmac_link_stats(pmac_link_stats_t *out);

#endif /* PMAC_LINK_H */
