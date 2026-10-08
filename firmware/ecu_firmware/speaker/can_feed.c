/*
 * can_feed.c - CANopen feedback source for the engine sampler.
 *
 * One reader thread decodes every frame from the PMAC controller (pmac.c)
 * into a shared state. The audio loop takes motor rpm and torque from it once
 * per block, smooths them to audio control rate and runs the virtual gearbox
 * that turns them into an engine rpm + load for the sampler to play.
 *
 * Nothing here ever transmits. Other programs (candump, ...) can read can0 at
 * the same time: SocketCAN gives every raw socket its own copy of each frame.
 */

#include "can_feed.h"

#include <errno.h>
#include <math.h>
#include <net/if.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

/* ===== 1. which decoded signals drive the engine ===== */

/* From the TPDO map in pmac.c, checked against the real controller:
 *   SIG_SPEED  - 606Ch velocity, TPDO5 0x454 bytes 1-4, int32, rpm
 *   SIG_TORQUE - 2020h/6 actual torque, TPDO4 0x271 bytes 5-6, int16,
 *                thousandths of rated torque -> about -1..1
 * Both arrive every 40 ms (SYNC-triggered). */
#define ENG_RPM_SIG   SIG_SPEED
#define ENG_TRQ_SIG   SIG_TORQUE

/* ===== 2. virtual gearbox - this is what stops it droning ===== */

#define N_GEARS     4
#define ENG_IDLE    720.0f
#define ENG_REDLINE 6600.0f
#define MOTOR_MAX   6000.0f     /* top motor rpm in this drivetrain */

/* motor rpm at which each gear upshifts, and the engine rpm multiplier.
 * Scaled x1.2 from the original 5000 rpm tuning (1500/2800/3900 rpm,
 * 4.33/2.32/1.67/1.30), so each gear covers the same fraction of the speed
 * range and the shifts sound the same. At MOTOR_MAX in 4th the engine sits at
 * 6000 x 1.08 = 6480 rpm, just under ENG_REDLINE: keep MOTOR_MAX * GRATIO[3]
 * below ENG_REDLINE or the top of the speed range drones on the limiter. */
static const float UPSHIFT[N_GEARS] = { 1800.0f, 3360.0f, 4680.0f, 1e9f };
static const float GRATIO [N_GEARS] = { 3.61f,   1.93f,   1.39f,   1.08f };
static const float DOWNSHIFT_HYST   = 0.72f;
static const float SHIFT_LIGHT      = 0.60f;  /* upshift point at zero load, fraction of UPSHIFT[] */

/* Smoothing time constants. This controller sends rpm and torque every
 * 40 ms (25 Hz), the low end of the usual TPDO range, so the filter is
 * longer than one frame period. A shorter one settles within each frame and
 * turns every new frame into a small pitch step. */
#define RPM_TAU     0.060f
#define TRQ_TAU     0.060f
#define STALE_S     0.25        /* rpm OR torque this old = DEAD */

/* ===== 3. shared state, written by the CAN thread ===== */

static pthread_mutex_t lk = PTHREAD_MUTEX_INITIALIZER;
static pmac_state   st;                 /* everything decoded so far        */
static Feed         s_engine;           /* last engine values, for the hook */
static canfeed_hook s_hook = NULL;
static int sock_fd = -1;

double canfeed_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

void canfeed_init(int node)
{
    pthread_mutex_lock(&lk);
    pmac_init(&st, node);
    memset(&s_engine, 0, sizeof s_engine);
    s_engine.gear = 1;              /* what canfeed_map() starts from */
    s_engine.rpm  = ENG_IDLE;
    s_engine.nmt  = -1;
    pthread_mutex_unlock(&lk);
}

void canfeed_set_hook(canfeed_hook h)
{
    pthread_mutex_lock(&lk);
    s_hook = h;
    pthread_mutex_unlock(&lk);
}

void canfeed_snapshot(pmac_state *out)
{
    pthread_mutex_lock(&lk);
    *out = st;
    pthread_mutex_unlock(&lk);
}

void canfeed_frame(uint32_t id, int len, const uint8_t *d, double t)
{
    int which = -1;
    pthread_mutex_lock(&lk);
    frame_kind k = pmac_feed(&st, t, id, len, d, &which);
    if (s_hook)
        s_hook(&st, &s_engine, k, which, t, id, len, d);
    pthread_mutex_unlock(&lk);
}

static void notify(frame_kind k, uint32_t id, double t)
{
    pthread_mutex_lock(&lk);
    if (k == FR_CANERR)
        st.n_canerr++;
    if (s_hook)
        s_hook(&st, &s_engine, k, -1, t, id, 0, NULL);
    pthread_mutex_unlock(&lk);
}

static void *can_thread(void *arg)
{
    (void)arg;
    struct can_frame fr;
    for (;;) {
        ssize_t n = read(sock_fd, &fr, sizeof fr);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            notify(FR_TICK, 0, canfeed_now());        /* 1 s without a frame */
            continue;
        }
        if (n != (ssize_t)sizeof fr) {
            if (n < 0 && errno == EINTR) continue;
            usleep(1000);
            continue;
        }
        if (fr.can_id & CAN_ERR_FLAG) {
            notify(FR_CANERR, fr.can_id & CAN_ERR_MASK, canfeed_now());
            continue;
        }
        if (fr.can_id & (CAN_RTR_FLAG | CAN_EFF_FLAG)) continue;
        canfeed_frame(fr.can_id & CAN_SFF_MASK, fr.can_dlc > 8 ? 8 : fr.can_dlc,
                      fr.data, canfeed_now());
    }
    return NULL;
}

int canfeed_start(const char *ifname, int node)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    canfeed_init(node);

    sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd < 0) { perror("socket"); return -1; }

    memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) { perror(ifname); return -1; }

    memset(&addr, 0, sizeof addr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
        perror("bind"); return -1;
    }

    /* All frames (the decoder wants every TPDO, SYNC, heartbeat and EMCY),
     * plus controller error frames so a dead bus is reported, and a 1 s
     * receive timeout so silence is reported too. */
    can_err_mask_t emask = CAN_ERR_MASK;
    setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &emask, sizeof emask);
    struct timeval tv = { 1, 0 };
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    pthread_t t;
    if (pthread_create(&t, NULL, can_thread, NULL)) { perror("pthread"); return -1; }
    pthread_detach(t);
    return 0;
}

/* ===== 4. feedback -> virtual engine, called once per audio block ===== */

void canfeed_update_at(Feed *o, float dt, double now)
{
    float raw_rpm, raw_trq;
    double stamp;

    pthread_mutex_lock(&lk);
    raw_rpm = st.have[ENG_RPM_SIG] ? (float)st.v[ENG_RPM_SIG] : 0.0f;
    raw_trq = st.have[ENG_TRQ_SIG] ? (float)st.v[ENG_TRQ_SIG] : 0.0f;
    /* The older of the two, so both must be fresh. They come in different
     * TPDOs (0x454 and 0x271): with the newer one, a lost speed TPDO would
     * stay LIVE on fresh torque and hold the pitch at the last speed instead
     * of fading out. A signal never received has t_sig 0, so it is DEAD. */
    stamp   = fmin(st.t_sig[ENG_RPM_SIG], st.t_sig[ENG_TRQ_SIG]);
    o->nmt   = st.nmt_state;
    o->fault = (st.n_emcy && st.emcy_code) ? st.fault_id : 0;
    pthread_mutex_unlock(&lk);

    int live = (stamp > 0.0) && (now - stamp < STALE_S);   /* failsafe */
    canfeed_map(raw_rpm, raw_trq, live, o, dt);

    pthread_mutex_lock(&lk);
    s_engine = *o;
    pthread_mutex_unlock(&lk);
}

void canfeed_update(Feed *o, float dt)
{
    canfeed_update_at(o, dt, canfeed_now());
}

void canfeed_map(float raw_rpm, float raw_trq, int live, Feed *o, float dt)
{
    static float f_mrpm = 0.0f, f_trq = 0.0f, f_erpm = ENG_IDLE;
    static int   gear = 1;
    static float shift_t = 0.0f;

    o->live = live;
    if (!live) { raw_rpm = 0.0f; raw_trq = 0.0f; }

    if (raw_rpm < 0.0f) raw_rpm = -raw_rpm;                /* reverse sounds the same */

    /* PDOs land every 40 ms here, audio control runs at ~190 Hz. Without this
     * smoothing you get a zipper on every frame boundary. */
    f_mrpm += (1.0f - expf(-dt / RPM_TAU)) * (raw_rpm - f_mrpm);
    f_trq  += (1.0f - expf(-dt / TRQ_TAU)) * (raw_trq - f_trq);

    /* --- gear selection with hysteresis ---
     * Shift points scale with load like a real automatic: gentle driving
     * upshifts early (SHIFT_LIGHT of the table), full torque holds to the
     * table value. Flooring it after a light-load upshift puts you under the
     * downshift line -> kickdown, which is the behaviour you want. */
    float ld = f_trq < 0.0f ? 0.0f : (f_trq > 1.0f ? 1.0f : f_trq);
    float sh = SHIFT_LIGHT + (1.0f - SHIFT_LIGHT) * ld;
    if (shift_t > 0.0f) shift_t -= dt;
    else {
        if (gear < N_GEARS && f_mrpm > UPSHIFT[gear - 1] * sh) { gear++; shift_t = 0.13f; }
        else if (gear > 1 && f_mrpm < UPSHIFT[gear - 2] * sh * DOWNSHIFT_HYST) { gear--; shift_t = 0.09f; }
    }
    o->gear = gear;
    o->shifting = shift_t > 0.0f;

    /* --- virtual engine rpm --- */
    float erpm = f_mrpm * GRATIO[gear - 1];

    /* clutch slip at low speed: let it rev against the load instead of
       sitting at idle while the motor is barely turning */
    float launch = ENG_IDLE + fmaxf(0.0f, f_trq) * 1800.0f;
    if (erpm < launch) erpm = launch;

    if (erpm > ENG_REDLINE) erpm = ENG_REDLINE;

    /* slew limit so a dropped or jittery frame cannot step the pitch */
    float max_step = 9000.0f * dt;          /* rpm per second */
    float d = erpm - f_erpm;
    if (d >  max_step) d =  max_step;
    if (d < -max_step) d = -max_step;
    f_erpm += d;

    o->motor_rpm = f_mrpm;
    o->torque    = f_trq;
    o->rpm       = f_erpm;

    /* --- acoustic load from real torque, not pedal position ---
     * positive torque = pulling = on-throttle layer
     * negative torque = regen  = overrun layer
     * This is strictly better than a pedal: it captures the controller
     * current-limiting, thermal foldback and traction events for free. */
    float load = f_trq;
    if (load < 0.0f) load = 0.0f;
    if (load > 1.0f) load = 1.0f;
    if (o->shifting) load *= 0.25f;
    o->load = load;
}
