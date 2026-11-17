/*
 * can2m33.c - forward the PMAC motor controller's telemetry from can0 to the
 * M33, which sends it to the ground over the VTX's MAVLink link.
 *
 *   can0 --SocketCAN--> pmac.c decode --> pmac_telem_t --RPMsg--> M33
 *
 * The decoding is speaker/pmac.c, the same decoder the engine-sound program
 * uses, checked against the real controller. The message is
 * mavlink_vtx/comms/pmac_telem.h, shared with the M33 firmware.
 *
 * The M33 announces an RPMsg channel named "rpmsg-raw"; the kernel's
 * rpmsg_char driver turns it into /dev/rpmsgN, found here by name, where each
 * write() is one message. The daemon only ever reads can0, never transmits,
 * so it can run next to the speaker program: SocketCAN gives every raw socket
 * its own copy of each frame.
 *
 * Survives the M33 being restarted (remoteproc stop/start): a failed write
 * drops the device and it is looked up again once a second.
 *
 * Build (see README.md):
 *   gcc -O2 -Wall -Wextra -I../mavlink_vtx/comms -I../../speaker \
 *       -o can2m33 can2m33.c ../../speaker/pmac.c
 */

#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "pmac.h"
#include "pmac_telem.h"

/* A decoded value older than this is not marked valid. The controller sends
 * every TPDO each 40 ms, so this is ~12 missed cycles. */
#define VALUE_STALE_S  0.5
/* The heartbeat is much slower than the TPDOs. */
#define NMT_STALE_S    5.0
#define RESCAN_S       1.0

/* pmac_telem_t.v[] slot -> pmac.c signal. */
static const int k_map[PMAC_TV_COUNT] = {
    [PMAC_TV_SPEED_RPM]  = SIG_SPEED,
    [PMAC_TV_TORQUE_PU]  = SIG_TORQUE,
    [PMAC_TV_BATT_A]     = SIG_BATT_I,
    [PMAC_TV_CAP_V]      = SIG_CAP_V,
    [PMAC_TV_MOTOR_C]    = SIG_MOTOR_TEMP,
    [PMAC_TV_HEATSINK_C] = SIG_HEATSINK_TEMP,
    [PMAC_TV_IQ_A]       = SIG_IQ,
    [PMAC_TV_ID_A]       = SIG_ID,
    [PMAC_TV_MOD_PCT]    = SIG_MODULATION,
};

static volatile sig_atomic_t stop = 0;
static void on_signal(int sig) { (void)sig; stop = 1; }

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void usage(const char *me)
{
    fprintf(stderr,
"usage: %s [options]\n"
"  -i <if>    CAN interface (default can0)\n"
"  -n <id>    PMAC node id (default 1)\n"
"  -r <hz>    messages per second to the M33, 1..50 (default 10)\n"
"  -d <dev>   RPMsg device instead of searching for the \"%s\" channel\n"
"  -v         one status line per second\n"
"\n"
"The CAN interface must already be up, e.g.\n"
"  ip link set can0 up type can bitrate 500000\n"
"and the M33 must be running mavlink_vtx with rpmsg_char loaded\n"
"  (modprobe rpmsg_char).\n", me, PMAC_TELEM_CHANNEL);
}

/* ---------------------------------------------------------------- CAN --- */

static int can_open(const char *ifname)
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (fd < 0) { perror("socket"); return -1; }
    memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { perror(ifname); close(fd); return -1; }
    memset(&addr, 0, sizeof addr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof addr) < 0) { perror("bind"); close(fd); return -1; }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    return fd;
}

/* Read every frame waiting. Returns frames read, or -1 if the interface went
 * away (it is not reopened: an interface that vanished needs a restart). */
static int can_drain(int fd, pmac_state *st, unsigned long *nframes)
{
    struct can_frame fr;
    int n = 0;

    for (;;) {
        ssize_t r = read(fd, &fr, sizeof fr);
        if (r < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return n;
            perror("can read");
            return -1;
        }
        if (r != (ssize_t)sizeof fr) continue;
        if (fr.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) continue;
        pmac_feed(st, now_s(), fr.can_id & CAN_SFF_MASK, fr.can_dlc > 8 ? 8 : fr.can_dlc,
                  fr.data, NULL);
        n++;
        (*nframes)++;
    }
}

/* -------------------------------------------------------------- RPMsg --- */

static int read_attr(const char *dir, const char *attr, char *buf, size_t len)
{
    char path[512];
    FILE *f;

    snprintf(path, sizeof path, "%s/%s", dir, attr);
    f = fopen(path, "r");
    if (!f) return -1;
    if (!fgets(buf, (int)len, f)) { fclose(f); return -1; }
    fclose(f);
    buf[strcspn(buf, "\n")] = '\0';
    return 0;
}

/* Find /dev/rpmsgN for the M33's channel: name "rpmsg-raw" and, where the
 * kernel exposes it, destination PMAC_TELEM_EPT_ADDR. */
static int rpmsg_find(char *dev, size_t len)
{
    DIR *d = opendir("/sys/class/rpmsg");
    struct dirent *e;
    int found = -1;

    if (!d) return -1;
    while ((e = readdir(d)) != NULL && found < 0) {
        char dir[300], name[64], dst[32];
        if (strncmp(e->d_name, "rpmsg", 5) || e->d_name[5] < '0' || e->d_name[5] > '9')
            continue;                               /* skips rpmsg_ctrlN */
        snprintf(dir, sizeof dir, "/sys/class/rpmsg/%s", e->d_name);
        if (read_attr(dir, "name", name, sizeof name) || strcmp(name, PMAC_TELEM_CHANNEL))
            continue;
        if (!read_attr(dir, "dst", dst, sizeof dst) &&
            strtoul(dst, NULL, 0) != PMAC_TELEM_EPT_ADDR)
            continue;
        snprintf(dev, len, "/dev/%s", e->d_name);
        found = 0;
    }
    closedir(d);
    return found;
}

/* ------------------------------------------------------------- message --- */

static void build(pmac_telem_t *m, const pmac_state *st, double t, uint32_t seq)
{
    memset(m, 0, sizeof *m);
    m->magic   = PMAC_TELEM_MAGIC;
    m->version = PMAC_TELEM_VERSION;
    m->seq     = seq;
    m->nmt     = (st->nmt_state >= 0 && t - st->t_hb < NMT_STALE_S)
                     ? (uint8_t)st->nmt_state : PMAC_NMT_NONE;
    /* As can_feed.c reads it: an EMCY with code 0000 means "cleared". */
    if (st->n_emcy && st->emcy_code) {
        m->fault_id  = st->fault_id;
        m->emcy_code = st->emcy_code;
    }
    for (int i = 0; i < PMAC_TV_COUNT; i++) {
        int s = k_map[i];
        m->v[i] = (float)st->v[s];
        if (st->have[s] && t - st->t_sig[s] < VALUE_STALE_S)
            m->valid |= 1UL << i;
    }
}

int main(int argc, char **argv)
{
    const char *ifn = "can0", *fixed_dev = NULL;
    int node = 1, verbose = 0;
    double rate = 10.0;

    for (int a = 1; a < argc; a++) {
        if      (!strcmp(argv[a], "-i") && a + 1 < argc) ifn = argv[++a];
        else if (!strcmp(argv[a], "-n") && a + 1 < argc) node = atoi(argv[++a]);
        else if (!strcmp(argv[a], "-r") && a + 1 < argc) rate = atof(argv[++a]);
        else if (!strcmp(argv[a], "-d") && a + 1 < argc) fixed_dev = argv[++a];
        else if (!strcmp(argv[a], "-v")) verbose = 1;
        else { usage(argv[0]); return strcmp(argv[a], "-h") && strcmp(argv[a], "--help") ? 2 : 0; }
    }
    if (node < 1 || node > 127 || !(rate >= 1.0 && rate <= 50.0)) { usage(argv[0]); return 2; }

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    int can_fd = can_open(ifn);
    if (can_fd < 0) return 1;

    pmac_state st;
    pmac_init(&st, node);

    char dev[300] = "";
    int rp_fd = -1;
    int waiting_said = 0;
    uint32_t seq = 0;
    unsigned long nframes = 0, nsent = 0, nfail = 0;
    const double period = 1.0 / rate;
    double t_next = now_s(), t_rescan = 0.0, t_status = now_s() + 1.0;

    fprintf(stderr, "can2m33: %s node %d -> M33 at %.0f Hz\n", ifn, node, rate);

    while (!stop) {
        double t = now_s();
        int wait_ms = (int)((t_next - t) * 1000.0);
        struct pollfd p = { .fd = can_fd, .events = POLLIN };

        if (wait_ms > 0 && poll(&p, 1, wait_ms) > 0 && can_drain(can_fd, &st, &nframes) < 0)
            break;

        t = now_s();
        if (t < t_next) continue;
        t_next += period;
        if (t_next < t) t_next = t + period;       /* fell behind: do not burst */

        if (rp_fd < 0 && t >= t_rescan) {
            t_rescan = t + RESCAN_S;
            if (fixed_dev) snprintf(dev, sizeof dev, "%s", fixed_dev);
            if (fixed_dev || !rpmsg_find(dev, sizeof dev)) {
                rp_fd = open(dev, O_WRONLY);
                if (rp_fd < 0 && !waiting_said)
                    fprintf(stderr, "can2m33: cannot open %s: %s\n", dev, strerror(errno));
            }
            if (rp_fd >= 0) {
                fprintf(stderr, "can2m33: sending to %s\n", dev);
                waiting_said = 0;
            } else if (!waiting_said) {
                fprintf(stderr, "can2m33: waiting for the M33's \"%s\" channel"
                        " (M33 running mavlink_vtx? modprobe rpmsg_char?)\n", PMAC_TELEM_CHANNEL);
                waiting_said = 1;
            }
        }

        if (rp_fd >= 0) {
            pmac_telem_t m;
            build(&m, &st, t, seq);
            if (write(rp_fd, &m, sizeof m) == (ssize_t)sizeof m) {
                seq++;
                nsent++;
            } else {
                /* The M33 was stopped or restarted: the channel and its
                 * /dev node are gone. Look again once a second. */
                fprintf(stderr, "can2m33: write to %s failed (%s), looking for the channel again\n",
                        dev, strerror(errno));
                close(rp_fd);
                rp_fd = -1;
                nfail++;
            }
        }

        if (verbose && t >= t_status) {
            pmac_telem_t m;
            build(&m, &st, t, seq);
            t_status = t + 1.0;
            printf("%s  frames %lu  sent %lu  fail %lu  valid 0x%03X  nmt %s  speed %.0f rpm"
                   "  torque %+.3f  Vcap %.1f V  Ibat %.1f A  fault %u\n",
                   rp_fd >= 0 ? "LINK" : "----", nframes, nsent, nfail, (unsigned)m.valid,
                   pmac_nmt_name(m.nmt == PMAC_NMT_NONE ? -1 : m.nmt),
                   m.v[PMAC_TV_SPEED_RPM], m.v[PMAC_TV_TORQUE_PU], m.v[PMAC_TV_CAP_V],
                   m.v[PMAC_TV_BATT_A], (unsigned)m.fault_id);
            fflush(stdout);
        }
    }

    if (rp_fd >= 0) close(rp_fd);
    close(can_fd);
    fprintf(stderr, "can2m33: stopped, %lu CAN frames, %lu messages sent\n", nframes, nsent);
    return 0;
}
