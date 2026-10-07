/*
 * can_feed.c - CANopen TPDO feedback source for the engine sampler.
 *
 * Reads velocity and torque from a Sevcon-family PMAC controller over SocketCAN
 * on its own thread, smooths them to audio control rate, and exposes a virtual
 * engine rpm + load for the sampler to play.
 *
 * standalone test:  gcc -O2 -DCANFEED_TEST -o canfeed can_feed.c -lm -lpthread
 *                   ./canfeed can0
 *
 * IMPORTANT: the byte offsets below are placeholders. Take the real ones from
 * your TPDO configuration spreadsheet - which PDO carries which object, and at
 * what offset. Do not trust the defaults here.
 */

#include "can_feed.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */

#include <errno.h>
#include <math.h>
#include <net/if.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/* ===== 1. PDO field description - EDIT FROM YOUR SPREADSHEET ===== */

#define NODE_ID   0x01

typedef struct {
    uint32_t cob;     /* full 11-bit COB-ID of the PDO carrying this object */
    int      off;     /* byte offset inside the 8-byte payload              */
    int      len;     /* 1, 2 or 4                                          */
    int      sgn;     /* 1 = signed                                         */
    float    scale;   /* raw -> engineering units                           */
} Field;

/* CANopen TPDO COB-IDs: TPDO1 0x180, TPDO2 0x280, TPDO3 0x380, TPDO4 0x480 (+node) */

/* 0x606C velocity actual value, CiA-402. Sevcon reports this in rpm by
 * default, but check 0x608F/0x6096 scaling in your config - some setups
 * report in internal units. */
static const Field F_RPM = { 0x180 + NODE_ID, 0, 4, 1, 1.0f };

/* 0x6077 torque actual value, per-thousand of rated torque.
 * If your PDO carries 0x6078 current actual value instead, change the scale
 * to 1/Imax and it works the same. */
static const Field F_TRQ = { 0x180 + NODE_ID, 4, 2, 1, 0.001f };

/* ===== 2. virtual gearbox - this is what stops it droning ===== */

#define N_GEARS     4
#define ENG_IDLE    720.0f
#define ENG_REDLINE 6600.0f
#define MOTOR_MAX   5000.0f     /* ME1302 practical ceiling, adjust to yours */

/* motor rpm at which each gear upshifts, and the engine rpm multiplier */
static const float UPSHIFT[N_GEARS] = { 1500.0f, 2800.0f, 3900.0f, 1e9f };
static const float GRATIO [N_GEARS] = { 4.33f,   2.32f,   1.67f,   1.30f };
static const float DOWNSHIFT_HYST   = 0.72f;
static const float SHIFT_LIGHT      = 0.60f;  /* upshift point at zero load, fraction of UPSHIFT[] */

/* ===== 3. shared state, written by the CAN thread ===== */

static pthread_mutex_t lk = PTHREAD_MUTEX_INITIALIZER;
static volatile float  s_rpm = 0.0f, s_trq = 0.0f;
static volatile double s_stamp = 0.0;
static int sock_fd = -1;

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int32_t extract(const uint8_t *d, const Field *f)
{
    uint32_t v = 0;
    for (int i = 0; i < f->len; i++) v |= (uint32_t)d[f->off + i] << (8 * i);  /* LE */
    if (f->sgn) {
        if (f->len == 1) return (int8_t)v;
        if (f->len == 2) return (int16_t)v;
        return (int32_t)v;
    }
    return (int32_t)v;
}

static void *can_thread(void *arg)
{
    (void)arg;
    struct can_frame fr;
    for (;;) {
        ssize_t n = read(sock_fd, &fr, sizeof fr);
        if (n != sizeof fr) {
            if (errno == EINTR) continue;
            usleep(1000);
            continue;
        }
        if (fr.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) continue;
        uint32_t id = fr.can_id & CAN_SFF_MASK;

        pthread_mutex_lock(&lk);
        if (id == F_RPM.cob && fr.can_dlc >= F_RPM.off + F_RPM.len)
            s_rpm = extract(fr.data, &F_RPM) * F_RPM.scale;
        if (id == F_TRQ.cob && fr.can_dlc >= F_TRQ.off + F_TRQ.len)
            s_trq = extract(fr.data, &F_TRQ) * F_TRQ.scale;
        if (id == F_RPM.cob || id == F_TRQ.cob) s_stamp = now_s();
        pthread_mutex_unlock(&lk);
    }
    return NULL;
}

int canfeed_start(const char *ifname)
{
    struct sockaddr_can addr = {0};
    struct ifreq ifr;

    sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd < 0) { perror("socket"); return -1; }

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = 0;
    if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); return -1; }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_fd, (struct sockaddr *)&addr, sizeof addr) < 0) {
        perror("bind"); return -1;
    }

    /* only wake the thread for the PDOs we care about */
    struct can_filter flt[2] = {
        { .can_id = F_RPM.cob, .can_mask = CAN_SFF_MASK },
        { .can_id = F_TRQ.cob, .can_mask = CAN_SFF_MASK },
    };
    int nf = (F_RPM.cob == F_TRQ.cob) ? 1 : 2;
    setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_FILTER, flt, nf * sizeof(struct can_filter));

    pthread_t t;
    if (pthread_create(&t, NULL, can_thread, NULL)) { perror("pthread"); return -1; }
    pthread_detach(t);
    return 0;
}

/* ===== 4. feedback -> virtual engine, called once per audio block ===== */

void canfeed_update(Feed *o, float dt)
{
    float raw_rpm, raw_trq; double stamp;
    pthread_mutex_lock(&lk);
    raw_rpm = s_rpm; raw_trq = s_trq; stamp = s_stamp;
    pthread_mutex_unlock(&lk);

    int live = (stamp > 0.0) && (now_s() - stamp < 0.25);   /* 250 ms failsafe */
    canfeed_map(raw_rpm, raw_trq, live, o, dt);
}

void canfeed_map(float raw_rpm, float raw_trq, int live, Feed *o, float dt)
{
    static float f_mrpm = 0.0f, f_trq = 0.0f, f_erpm = ENG_IDLE;
    static int   gear = 1;
    static float shift_t = 0.0f;

    o->live = live;
    if (!live) { raw_rpm = 0.0f; raw_trq = 0.0f; }

    if (raw_rpm < 0.0f) raw_rpm = -raw_rpm;                /* reverse sounds the same */

    /* PDOs land at 10-100 Hz, audio control runs at ~190 Hz. Without this
     * smoothing you get a zipper on every frame boundary. tau ~35 ms. */
    float a = 1.0f - expf(-dt / 0.035f);
    f_mrpm += a * (raw_rpm - f_mrpm);
    f_trq  += (1.0f - expf(-dt / 0.050f)) * (raw_trq - f_trq);

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

/* ===== standalone test ===== */
#ifdef CANFEED_TEST
int main(int argc, char **argv)
{
    const char *ifn = argc > 1 ? argv[1] : "can0";
    if (canfeed_start(ifn)) return 1;
    printf("listening on %s (rpm cob 0x%03X, trq cob 0x%03X)\n",
           ifn, F_RPM.cob, F_TRQ.cob);
    Feed f;
    for (;;) {
        canfeed_update(&f, 0.05f);
        printf("\r%s motor %7.1f rpm  trq %+6.3f  ->  gear %d  engine %7.1f rpm  load %.2f %s   ",
               f.live ? "LIVE" : "DEAD", f.motor_rpm, f.torque, f.gear, f.rpm, f.load,
               f.shifting ? "[shift]" : "       ");
        fflush(stdout);
        usleep(50000);
    }
}
#endif
