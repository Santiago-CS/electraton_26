/*
 * drive_kbd.c - keyboard test drive for the engine sampler, desktop Linux.
 *
 * build: gcc -O2 -o drive_kbd drive_kbd.c can_feed.c -lasound -lm -lpthread
 * run  : ./drive_kbd                (from the folder that contains samples/)
 *
 * Simulates what the PMAC controller would report over CAN (motor rpm and
 * torque) from the arrow keys, and pushes it through the SAME canfeed_map()
 * gearbox and render() sampler the car uses. What you hear here is what the
 * car will play; only the source of motor rpm/torque is fake.
 *
 * keys
 *   Up     hold = full throttle
 *   Down   hold = brake (regen)
 *   Right  part-throttle lever +10 %   (stays set, like cruise)
 *   Left   part-throttle lever -10 %
 *   Space  lever back to 0
 *   n      neutral on/off  (free-rev the engine, the car coasts)
 *   + / -  volume
 *   q      quit            (Ctrl-C also works)
 *
 * A terminal has no key-release event, only key auto-repeat. So "held" means
 * "a repeat arrived recently". A single tap therefore gives a ~0.6 s blip,
 * which is what bridges the gap before auto-repeat starts.
 */

#define SAMPLER_NO_MAIN
#include "sampler.c"          /* sample bank, render(), engine_step() */
#include "can_feed.h"

#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* ================= simulated drivetrain =================
 * Stands in for motor + controller + vehicle. Scalar, one state each:
 *   motor torque  (first-order current loop, tau 40 ms)
 *   motor rpm     (d rpm/dt = K_ACC*torque - drag)
 */
#define M_BASE   2500.0f   /* rpm where constant torque turns into constant power */
#define M_MAX    5000.0f   /* motor ceiling, match MOTOR_MAX in can_feed.c         */
#define K_ACC    1000.0f   /* rpm/s at full torque: sets how fast the "car" is    */
#define REGEN_LIFT  0.10f  /* regen torque when off the pedal                      */
#define REGEN_BRAKE 0.90f  /* regen torque with Down held                          */

static float m_rpm = 0.0f;     /* motor rpm           */
static float m_trq = 0.0f;     /* motor torque, -1..1 */

static void drivetrain_step(float pedal, int brake, float dt)
{
    float tmax = (m_rpm < M_BASE) ? 1.0f : M_BASE / m_rpm;
    float cmd  = pedal * tmax;

    if (m_rpm > M_MAX)                      cmd = 0.0f;
    if (pedal < 0.02f && m_rpm > 30.0f)     cmd = -REGEN_LIFT;
    if (brake)                              cmd = (m_rpm > 30.0f) ? -REGEN_BRAKE : 0.0f;

    m_trq += (1.0f - expf(-dt / 0.040f)) * (cmd - m_trq);

    float acc = K_ACC * m_trq - 0.03f * m_rpm - 2.5e-5f * m_rpm * m_rpm;
    m_rpm += acc * dt;
    if (m_rpm < 0.0f) m_rpm = 0.0f;
}

/* ================= per-block simulation =================
 * Kept separate from the terminal/ALSA code so it can be driven by a script
 * for offline testing.
 */
typedef struct {
    float pedal;      /* 0..1 */
    int   brake;
    int   neutral;
} Input;

static void sim_block(const Input *in, Feed *fb, float dt)
{
    static int was_neutral = 0;

    if (in->neutral) {
        /* clutch in: car coasts, engine free-revs on its own plant */
        if (!was_neutral) w = fb->rpm / 9.5493f;          /* no pitch jump */
        drivetrain_step(0.0f, in->brake, dt);
        engine_step((int)(in->pedal * 255.0f), dt);
        Feed dummy;
        canfeed_map(m_rpm, m_trq, 1, &dummy, dt);        /* keep gearbox tracking */
        fb->rpm       = w * 9.5493f;
        fb->load      = u_f;
        fb->motor_rpm = m_rpm;
        fb->torque    = m_trq;
        fb->gear      = 0;
        fb->shifting  = 0;
        fb->live      = 1;
    } else {
        drivetrain_step(in->pedal, in->brake, dt);
        canfeed_map(m_rpm, m_trq, 1, fb, dt);
    }
    was_neutral = in->neutral;
}

#ifndef DRIVE_KBD_NO_MAIN

/* ================= terminal ================= */

static struct termios tio_orig;
static int tio_saved = 0;
static volatile sig_atomic_t quit = 0;

static void term_restore(void)
{
    if (!tio_saved) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &tio_orig);
    tio_saved = 0;
    printf("\033[?25h\n");           /* cursor back on */
    fflush(stdout);
}

static void on_signal(int s) { (void)s; quit = 1; }

static int term_raw(void)
{
    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "run this in a terminal (stdin is not a tty)\n");
        return -1;
    }
    tcgetattr(STDIN_FILENO, &tio_orig);
    tio_saved = 1;
    struct termios t = tio_orig;
    t.c_lflag &= ~(ICANON | ECHO);   /* keep ISIG so Ctrl-C still works */
    t.c_cc[VMIN]  = 0;               /* non-blocking read */
    t.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    printf("\033[?25l");             /* hide cursor */
    return 0;
}

static double now_mono(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* ================= keyboard state ================= */

#define HOLD_FIRST  0.60   /* covers the auto-repeat start delay      */
#define HOLD_REPEAT 0.12   /* release detected this long after repeat */

static double up_until = 0.0, dn_until = 0.0;
static float  lever = 0.0f;      /* part-throttle lever, 0..1 */
static int    neutral = 0;
static float  volume = 1.0f;

static void press(double *until, double t)
{
    *until = t + ((t < *until) ? HOLD_REPEAT : HOLD_FIRST);
}

static void poll_keys(double t)
{
    unsigned char b[128];
    ssize_t n = read(STDIN_FILENO, b, sizeof b);
    for (ssize_t i = 0; i < n; i++) {
        if (b[i] == 27 && i + 2 < n && (b[i + 1] == '[' || b[i + 1] == 'O')) {
            switch (b[i + 2]) {
            case 'A': press(&up_until, t); break;                       /* up    */
            case 'B': press(&dn_until, t); break;                       /* down  */
            case 'C': lever = fminf(1.0f, lever + 0.1f); break;         /* right */
            case 'D': lever = fmaxf(0.0f, lever - 0.1f); break;         /* left  */
            }
            i += 2;
            continue;
        }
        switch (b[i]) {
        case 'q': case 'Q': quit = 1; break;
        case 'n': case 'N': neutral = !neutral; break;
        case ' ': lever = 0.0f; break;
        case '+': case '=': volume = fminf(2.0f, volume + 0.1f); break;
        case '-': case '_': volume = fmaxf(0.1f, volume - 0.1f); break;
        }
    }
}

/* ================= display ================= */

static void status(const Feed *fb, float pedal, int brake)
{
    enum { W = 32 };
    char bar[W * 12 + 1];
    int  filled = (int)(fb->rpm / 7000.0f * W + 0.5f);
    int  red    = (int)(6000.0f / 7000.0f * W);
    char *p = bar;
    for (int i = 0; i < W; i++) {
        if (i == red) p += sprintf(p, "\033[31m");
        *p++ = (i < filled) ? '#' : '.';
    }
    p += sprintf(p, "\033[0m");
    *p = 0;

    char gear[8];
    if (fb->gear == 0) snprintf(gear, sizeof gear, "N");
    else               snprintf(gear, sizeof gear, "%d", fb->gear);

    printf("\r\033[K gear %s  %5.0f rpm [%s]  pedal %3.0f%%  motor %4.0f rpm  %s%s vol %.1f  tone %+5.1f dB",
           gear, fb->rpm, bar, pedal * 100.0f, fb->motor_rpm,
           brake ? "BRAKE " : "      ",
           fb->shifting ? "SHIFT " : "      ",
           volume, tone_shelf_db_now());
    fflush(stdout);
}

/* ================= main ================= */

int main(void)
{
    const char *dir = getenv("V8_PACK") ? getenv("V8_PACK") : "samples";
    if (load_pack(dir)) {
        fprintf(stderr, "no sample pack in '%s'. Run from the speaker folder, "
                        "or set V8_PACK=/path/to/pack\n", dir);
        return 1;
    }

    snd_pcm_t *pcm;
    int err;
    const char *dev = getenv("ALSA_DEV") ? getenv("ALSA_DEV") : "default";
    if ((err = snd_pcm_open(&pcm, dev, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "open %s: %s\n", dev, snd_strerror(err));
        return 1;
    }
    if ((err = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE,
                                  SND_PCM_ACCESS_RW_INTERLEAVED,
                                  1, SR, 1, 30000)) < 0) {
        fprintf(stderr, "params: %s\n", snd_strerror(err));
        return 1;
    }

    if (term_raw()) return 1;
    atexit(term_restore);
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    printf("\n  Up: throttle (hold)   Down: brake (hold)   Right/Left: part-throttle lever\n"
           "  Space: lever to 0   n: neutral   +/-: volume   q: quit\n\n");

    const float Ts = (float)PERIOD / SR;
    float   buf[PERIOD];
    int16_t out[PERIOD];
    float   pedal = 0.0f, fade = 0.0f;
    Feed    fb = { .rpm = 720.0f };
    int     blk = 0;

    while (!quit) {
        double t = now_mono();
        poll_keys(t);

        int   up    = t < up_until;
        int   brake = t < dn_until;
        float want  = up ? 1.0f : lever;
        if (brake) want = 0.0f;

        /* pedal travel: ~0.25 s to floor, ~0.12 s to lift */
        float rate = (want > pedal ? 4.0f : 8.0f) * Ts;
        if      (want > pedal + rate) pedal += rate;
        else if (want < pedal - rate) pedal -= rate;
        else                          pedal  = want;

        Input in = { pedal, brake, neutral };
        sim_block(&in, &fb, Ts);

        render(buf, PERIOD, fb.rpm, fb.load);

        float target = fb.shifting ? 0.35f : 1.0f;
        for (int i = 0; i < PERIOD; i++) {
            fade += (target - fade) * 0.004f;
            float s = buf[i] * 1.75f * fade * volume;
            if (s >  1.0f) s =  1.0f;
            if (s < -1.0f) s = -1.0f;
            out[i] = (int16_t)(s * 30000.0f);
        }

        snd_pcm_sframes_t n = snd_pcm_writei(pcm, out, PERIOD);
        if (n < 0) snd_pcm_recover(pcm, (int)n, 1);

        if (++blk % 8 == 0) status(&fb, pedal, brake);
    }

    snd_pcm_drop(pcm);
    snd_pcm_close(pcm);
    term_restore();
    return 0;
}

#endif /* DRIVE_KBD_NO_MAIN */
