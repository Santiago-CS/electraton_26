/*
 * can2m33_test.c - mock motor controller data to the M33 over RPMsg, no CAN.
 *
 * Tests the RPMsg -> M33 -> MAVLink -> ground station path on its own: sends
 * the same pmac_telem_t messages can2m33 does, filled with a simulated drive
 * instead of decoded CAN. If the ground station shows "PMAC LIVE" with these
 * numbers moving, everything after can0 works.
 *
 * The simulated drive repeats every 30 s: accelerate 0 -> 5000 rpm, cruise,
 * regen back to 0, stand still. Torque, currents, voltage sag and modulation
 * follow it; the temperatures creep up over the run.
 *
 *   gcc -O2 -Wall -Wextra -o can2m33_test can2m33_test.c       (next to pmac_telem.h)
 *   ./can2m33_test             live values  -> ground: PMAC LIVE ...
 *   ./can2m33_test -n          no CAN data  -> ground: PMAC Linux up, NO CAN
 *   ./can2m33_test -f 0x42     plus a fault -> ground: ... FAULT 0x0042
 *   stop it                                 -> ground: PMAC NO DATA within 1 s
 */

#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "pmac_telem.h"

#define CYCLE_S 30.0

static volatile sig_atomic_t stop = 0;
static void on_signal(int sig) { (void)sig; stop = 1; }

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static int read_attr(const char *dir, const char *attr, char *buf, size_t len)
{
    char path[512];
    FILE *f;

    snprintf(path, sizeof path, "%s/%s", dir, attr);
    if (!(f = fopen(path, "r"))) return -1;
    if (!fgets(buf, (int)len, f)) { fclose(f); return -1; }
    fclose(f);
    buf[strcspn(buf, "\n")] = '\0';
    return 0;
}

/* Same lookup as can2m33: /dev/rpmsgN whose channel is "rpmsg-raw" and, where
 * the kernel shows it, destination PMAC_TELEM_EPT_ADDR. */
static int rpmsg_find(char *dev, size_t len)
{
    DIR *d = opendir("/sys/class/rpmsg");
    struct dirent *e;
    int found = -1;

    if (!d) return -1;
    while ((e = readdir(d)) != NULL && found < 0) {
        char dir[300], name[64], dst[32];
        if (strncmp(e->d_name, "rpmsg", 5) || e->d_name[5] < '0' || e->d_name[5] > '9')
            continue;
        snprintf(dir, sizeof dir, "/sys/class/rpmsg/%s", e->d_name);
        if (read_attr(dir, "name", name, sizeof name) || strcmp(name, PMAC_TELEM_CHANNEL))
            continue;
        if (!read_attr(dir, "dst", dst, sizeof dst) && strtoul(dst, NULL, 0) != PMAC_TELEM_EPT_ADDR)
            continue;
        snprintf(dev, len, "/dev/%s", e->d_name);
        found = 0;
    }
    closedir(d);
    return found;
}

/* One point of the drive cycle at time t into the run. */
static void simulate(pmac_telem_t *m, double t)
{
    double c = fmod(t, CYCLE_S), rpm, trq;

    if (c < 10.0) {                         /* accelerate */
        rpm = 5000.0 * (c / 10.0);
        trq = 0.80 - 0.04 * c;
    } else if (c < 18.0) {                  /* cruise */
        rpm = 5000.0 + 150.0 * sin((c - 10.0) * 1.3);
        trq = 0.20 + 0.05 * sin((c - 10.0) * 2.1);
    } else if (c < 26.0) {                  /* regen down to 0 */
        rpm = 5000.0 * (1.0 - (c - 18.0) / 8.0);
        trq = -0.45;
    } else {                                /* parked */
        rpm = 0.0;
        trq = 0.0;
    }

    double batt = 180.0 * trq * (rpm + 300.0) / 5300.0;   /* A, negative = regen */
    m->v[PMAC_TV_SPEED_RPM]  = (float)rpm;
    m->v[PMAC_TV_TORQUE_PU]  = (float)trq;
    m->v[PMAC_TV_BATT_A]     = (float)batt;
    m->v[PMAC_TV_CAP_V]      = (float)(50.4 - 0.02 * batt);
    m->v[PMAC_TV_MOTOR_C]    = (float)(30.0 + 25.0 * (1.0 - exp(-t / 300.0)));
    m->v[PMAC_TV_HEATSINK_C] = (float)(28.0 + 15.0 * (1.0 - exp(-t / 400.0)));
    m->v[PMAC_TV_IQ_A]       = (float)(250.0 * trq);
    m->v[PMAC_TV_ID_A]       = (float)(rpm > 3500.0 ? -0.04 * (rpm - 3500.0) : 0.0);
    m->v[PMAC_TV_MOD_PCT]    = (float)(10.0 + 85.0 * rpm / 5000.0);
}

int main(int argc, char **argv)
{
    const char *fixed_dev = NULL;
    double rate = 10.0;
    int no_can = 0, verbose = 1;
    unsigned fault = 0;

    for (int a = 1; a < argc; a++) {
        if      (!strcmp(argv[a], "-r") && a + 1 < argc) rate = atof(argv[++a]);
        else if (!strcmp(argv[a], "-d") && a + 1 < argc) fixed_dev = argv[++a];
        else if (!strcmp(argv[a], "-f") && a + 1 < argc) fault = (unsigned)strtoul(argv[++a], NULL, 0);
        else if (!strcmp(argv[a], "-n")) no_can = 1;
        else if (!strcmp(argv[a], "-q")) verbose = 0;
        else {
            fprintf(stderr,
                "usage: %s [-r hz] [-d /dev/rpmsgN] [-n] [-f fault_id] [-q]\n"
                "  -r  messages per second (default 10, like can2m33)\n"
                "  -d  RPMsg device instead of searching for the \"%s\" channel\n"
                "  -n  send 'no CAN data' (valid = 0): ground shows Linux up, NO CAN\n"
                "  -f  report this Sevcon fault id, e.g. -f 0x42\n"
                "  -q  no status line\n", argv[0], PMAC_TELEM_CHANNEL);
            return 2;
        }
    }
    if (!(rate >= 1.0 && rate <= 50.0)) { fprintf(stderr, "rate 1..50\n"); return 2; }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    char dev[300] = "";
    int fd = -1, said = 0;
    uint32_t seq = 0;
    unsigned long sent = 0;
    const double t0 = now_s(), period = 1.0 / rate;
    double t_next = t0, t_status = t0;

    fprintf(stderr, "can2m33_test: MOCK data, no CAN, %.0f Hz%s%s\n", rate,
            no_can ? ", reporting no CAN data" : "", fault ? ", with a fault" : "");

    while (!stop) {
        double t = now_s();
        if (t < t_next) { usleep((useconds_t)((t_next - t) * 1e6)); continue; }
        t_next += period;

        if (fd < 0) {
            if (fixed_dev) snprintf(dev, sizeof dev, "%s", fixed_dev);
            if ((fixed_dev || !rpmsg_find(dev, sizeof dev)) && (fd = open(dev, O_WRONLY)) >= 0) {
                fprintf(stderr, "can2m33_test: sending to %s\n", dev);
                said = 0;
            } else {
                if (!said)
                    fprintf(stderr, "can2m33_test: waiting for the M33's \"%s\" channel"
                            " (M33 running mavlink_vtx? modprobe rpmsg_char?)\n", PMAC_TELEM_CHANNEL);
                said = 1;
                usleep(1000000);
                t_next = now_s();
                continue;
            }
        }

        pmac_telem_t m;
        memset(&m, 0, sizeof m);
        m.magic     = PMAC_TELEM_MAGIC;
        m.version   = PMAC_TELEM_VERSION;
        m.seq       = seq;
        m.nmt       = 0x05;                                  /* Operational */
        m.fault_id  = (uint16_t)fault;
        m.emcy_code = fault ? 0x1000 : 0;                    /* generic error */
        m.valid     = no_can ? 0 : (1UL << PMAC_TV_COUNT) - 1U;
        simulate(&m, t - t0);

        if (write(fd, &m, sizeof m) != (ssize_t)sizeof m) {
            fprintf(stderr, "can2m33_test: write to %s failed (%s), looking again\n", dev, strerror(errno));
            close(fd);
            fd = -1;
            continue;
        }
        seq++;
        sent++;

        if (verbose && t >= t_status) {
            t_status = t + 1.0;
            printf("sent %5lu  speed %5.0f rpm  torque %+.2f  Ibat %+6.1f A  Vcap %.1f V"
                   "  Tmot %.1f C%s\n", sent, m.v[PMAC_TV_SPEED_RPM], m.v[PMAC_TV_TORQUE_PU],
                   m.v[PMAC_TV_BATT_A], m.v[PMAC_TV_CAP_V], m.v[PMAC_TV_MOTOR_C],
                   no_can ? "  (sent as NOT valid)" : "");
            fflush(stdout);
        }
    }

    if (fd >= 0) close(fd);
    fprintf(stderr, "can2m33_test: stopped after %lu messages\n", sent);
    return 0;
}
