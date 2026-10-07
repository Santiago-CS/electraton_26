/*
 * can_feed.h - motor-controller feedback -> virtual engine rpm / load.
 */
#ifndef CAN_FEED_H
#define CAN_FEED_H

typedef struct {
    float rpm;        /* smoothed virtual engine rpm  */
    float load;       /* 0..1, acoustic load          */
    float motor_rpm;  /* smoothed motor rpm           */
    float torque;     /* -1..1, negative = regen      */
    int   gear;       /* virtual gear, 1-based        */
    int   shifting;   /* duck the audio while true    */
    int   live;       /* 0 = CAN timeout, fade out    */
} Feed;

/* Real car: start the SocketCAN reader, then call canfeed_update() once per
 * audio block. */
int  canfeed_start(const char *ifname);
void canfeed_update(Feed *out, float dt);

/* The mapping on its own: motor rpm + torque (-1..1) in, virtual engine out.
 * canfeed_update() calls this with the CAN values; drive_kbd.c calls it with
 * simulated values, so both run the identical gearbox. */
void canfeed_map(float motor_rpm, float torque, int live, Feed *out, float dt);

#endif
