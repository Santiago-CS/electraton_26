/*
 * can_feed.h - motor-controller feedback -> virtual engine rpm / load.
 */
#ifndef CAN_FEED_H
#define CAN_FEED_H

#include <stdint.h>
#include "pmac.h"

typedef struct {
    float rpm;        /* smoothed virtual engine rpm  */
    float load;       /* 0..1, acoustic load          */
    float motor_rpm;  /* smoothed motor rpm           */
    float torque;     /* -1..1, negative = regen      */
    int   gear;       /* virtual gear, 1-based        */
    int   shifting;   /* duck the audio while true    */
    int   live;       /* 0 = CAN timeout, fade out    */

    /* Controller status, filled by canfeed_update() only (not by
     * canfeed_map(), so drive_kbd.c leaves these alone). */
    int      nmt;     /* last heartbeat state: 0x05 Operational,
                         0x7F Pre-op, 0x04 Stopped, -1 none seen   */
    uint16_t fault;   /* Sevcon fault id of an active EMCY, 0 = none */
} Feed;

/* Real car: start the SocketCAN reader thread (it decodes every PMAC frame
 * into the shared pmac_state), then call canfeed_update() once per audio
 * block. node = controller node id (1). */
int  canfeed_start(const char *ifname, int node);
void canfeed_update(Feed *out, float dt);

/* The mapping on its own: motor rpm + torque (-1..1) in, virtual engine out.
 * canfeed_update() calls this with the CAN values; drive_kbd.c calls it with
 * simulated values, so both run the identical gearbox. */
void canfeed_map(float motor_rpm, float torque, int live, Feed *out, float dt);

/* Optional observer, called on the CAN thread for every frame (and once a
 * second with FR_TICK while the bus is silent), with the decoder state and
 * the latest engine values. The state is locked during the call: keep it
 * short and never call back into canfeed_*. Runs on the CAN thread, never
 * the audio thread, so printing from here cannot cause an audio dropout. */
typedef void (*canfeed_hook)(const pmac_state *st, const Feed *engine,
                             frame_kind k, int which, double t,
                             uint32_t id, int len, const uint8_t *d);
void canfeed_set_hook(canfeed_hook h);

/* Log replay without a socket: init the decoder, then feed frames and run
 * the gearbox on the log's own clock. */
void canfeed_init(int node);
void canfeed_frame(uint32_t id, int len, const uint8_t *data, double t);
void canfeed_update_at(Feed *out, float dt, double now);

/* Copy of the decoder state, e.g. for a summary at exit. */
void   canfeed_snapshot(pmac_state *out);
double canfeed_now(void);      /* the monotonic clock the thread stamps with */

#endif
