/*
 * sampler.c - RPM-indexed crossfading engine sound sampler.
 *
 * Realtime (ALSA):  gcc -O2 -o sampler sampler.c -lasound -lm
 * Offline render :  gcc -O2 -DOFFLINE -o sampler_off sampler.c -lm
 *                   ./sampler_off demo.wav
 *
 * Runtime chain:
 *   throttle 0..255 -> torque -> crank inertia -> rpm
 *   rpm  -> pick bracketing sample pair, equal-power crossfade
 *   rpm  -> per-loop resample ratio so BOTH loops land on the exact target pitch
 *   load -> blend on-throttle vs off-throttle layer
 *
 * The resample ratio is the part people skip. If you only crossfade two
 * fixed-pitch loops you hear the pitch step at every sample point. Stretching
 * each loop to the true target rpm removes that completely.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef OFFLINE
#include <alsa/asoundlib.h>
#endif

#define SR         48000
#define PERIOD     256
#define MAX_PTS    32
#define RATIO_MIN  0.55f
#define RATIO_MAX  1.80f

/* ================= sample bank ================= */

typedef struct {
    float *d;
    int    n;
} Loop;

typedef struct {
    float rpm;
    Loop  on, off;
    float pos_on, pos_off;    /* persistent phase, survives index changes */
} Point;

static Point pts[MAX_PTS];
static int   npts = 0;

static int load_wav(const char *path, Loop *lp)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return -1; }

    uint8_t hdr[12];
    if (fread(hdr, 1, 12, f) != 12 || memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) {
        fprintf(stderr, "%s: not a RIFF/WAVE file\n", path); fclose(f); return -1;
    }

    int ch = 1, bits = 16;
    long data_off = 0; uint32_t data_len = 0;

    for (;;) {
        uint8_t c[8];
        if (fread(c, 1, 8, f) != 8) break;
        uint32_t sz = c[4] | (c[5] << 8) | (c[6] << 16) | ((uint32_t)c[7] << 24);
        if (!memcmp(c, "fmt ", 4)) {
            uint8_t fmt[16];
            if (fread(fmt, 1, 16, f) != 16) { fclose(f); return -1; }
            ch   = fmt[2] | (fmt[3] << 8);
            bits = fmt[14] | (fmt[15] << 8);
            if (sz > 16) fseek(f, sz - 16, SEEK_CUR);
        } else if (!memcmp(c, "data", 4)) {
            data_off = ftell(f); data_len = sz;
            break;
        } else {
            fseek(f, (long)sz + (sz & 1), SEEK_CUR);
        }
    }
    if (!data_off || bits != 16) {
        fprintf(stderr, "%s: need 16-bit PCM\n", path); fclose(f); return -1;
    }

    int frames = (int)(data_len / (2 * ch));
    int16_t *raw = malloc(data_len);
    fseek(f, data_off, SEEK_SET);
    if (fread(raw, 1, data_len, f) != data_len) {
        fprintf(stderr, "%s: short read\n", path);
        free(raw); fclose(f); return -1;
    }
    fclose(f);

    lp->n = frames;
    lp->d = malloc(sizeof(float) * frames);
    for (int i = 0; i < frames; i++) {
        float s = 0.0f;
        for (int c2 = 0; c2 < ch; c2++) s += raw[i * ch + c2];
        lp->d[i] = s / (ch * 32768.0f);      /* downmix to mono */
    }
    free(raw);
    return 0;
}

static void tone_init(void);      /* defined with the tone stage below */

static int load_pack(const char *dir)
{
    char mpath[512];
    snprintf(mpath, sizeof mpath, "%s/manifest.txt", dir);
    FILE *m = fopen(mpath, "r");
    if (!m) { fprintf(stderr, "cannot open %s\n", mpath); return -1; }

    char line[512];
    while (fgets(line, sizeof line, m) && npts < MAX_PTS) {
        if (line[0] == '#' || line[0] == '\n') continue;
        float rpm; char a[192], b[192];
        if (sscanf(line, "%f %191s %191s", &rpm, a, b) != 3) continue;

        char pa[512], pb[512];
        snprintf(pa, sizeof pa, "%s/%s", dir, a);
        snprintf(pb, sizeof pb, "%s/%s", dir, b);

        Point *p = &pts[npts];
        p->rpm = rpm;
        if (load_wav(pa, &p->on) || load_wav(pb, &p->off)) { fclose(m); return -1; }
        p->pos_on = p->pos_off = 0.0f;
        npts++;
    }
    fclose(m);
    printf("loaded %d rpm points (%.0f .. %.0f)\n", npts, pts[0].rpm, pts[npts - 1].rpm);
    tone_init();       /* here, so a bad TONE_* value is reported before the
                        * status line starts overwriting itself */
    return npts >= 2 ? 0 : -1;
}

/* ================= engine plant (scalar) ================= */

static float w   = 75.0f;    /* crank speed [rad/s] */
static float u_f = 0.0f;     /* filtered throttle 0..1 */
static float cut = 0.0f;

static const float J      = 0.32f;     /* V8 + drivetrain inertia */
static const float T_MAX  = 520.0f;    /* N m */
static const float W_PK   = 440.0f;    /* peak torque ~4200 rpm */
static const float B_FRIC = 0.09f;
static const float K_DRAG = 7.0e-4f;
static const float W_IDLE = 76.0f;     /* ~725 rpm */
static const float W_MAX  = 712.0f;    /* 6800 rpm limiter */

static void engine_step(int throttle, float Ts)
{
    float u = throttle * (1.0f / 255.0f);
    u_f += 0.16f * (u - u_f);

    float r = (w - W_PK) / W_PK;
    float g = 1.0f - 0.75f * r * r;
    if (g < 0.20f) g = 0.20f;

    float Te = u_f * T_MAX * g;

    if (w < W_IDLE * 1.3f) {
        float e = W_IDLE - w;
        if (e > 0.0f) Te += 9.0f * e;
    }
    if (w > W_MAX) cut = 0.055f;
    if (cut > 0.0f) { Te = 0.0f; cut -= Ts; }

    float Tl = B_FRIC * w + K_DRAG * w * w;
    w += (Ts / J) * (Te - Tl);
    if (w < 50.0f) w = 50.0f;
}

/* ================= voice playback ================= */

static inline float tap(const Loop *lp, float pos)
{
    int   i0 = (int)pos;
    float fr = pos - (float)i0;
    int   i1 = i0 + 1; if (i1 >= lp->n) i1 = 0;
    return lp->d[i0] * (1.0f - fr) + lp->d[i1] * fr;
}

/* accumulate one looping voice into buf with linearly ramped gain and ratio */
static void voice(float *buf, int n, const Loop *lp, float *pos,
                  float g0, float g1, float r0, float r1)
{
    if (g0 < 1e-4f && g1 < 1e-4f) {
        /* still advance the phase so it stays coherent when it fades back in */
        *pos = fmodf(*pos + 0.5f * (r0 + r1) * n, (float)lp->n);
        return;
    }
    float p = *pos;
    for (int i = 0; i < n; i++) {
        float t = (float)i / (float)n;
        float g = g0 + (g1 - g0) * t;
        float r = r0 + (r1 - r0) * t;
        buf[i] += tap(lp, p) * g;
        p += r;
        if (p >= (float)lp->n) p -= (float)lp->n;
    }
    *pos = p;
}

/* ================= overrun crackle =================
 * Generated live rather than baked into the loops: a pop baked into a 1.2 s
 * loop comes back at the same spot every 1.2 s, which the ear catches at once.
 * Fires off-throttle above ~1500 rpm, most at high rpm and closed throttle.
 * Set CRACKLE_RATE to 0 to disable.
 */
#ifndef CRACKLE_RATE
#define CRACKLE_RATE 8.0f          /* pops per second at redline, fully closed */
#endif

static inline float frand01(void) { return (float)rand() / (float)RAND_MAX; }

static void crackle(float *buf, int n, float rpm, float load)
{
    static int   left = 0;          /* samples left in the current pop */
    static float amp = 0.0f, env = 0.0f, lp = 0.0f, k_lp = 0.5f;

    float rn  = (rpm - 1500.0f) / 5000.0f;
    if (rn < 0.0f) rn = 0.0f;
    if (rn > 1.0f) rn = 1.0f;
    float off = 1.0f - load;
    float p   = CRACKLE_RATE * rn * off * off * off * (float)n / SR;

    if (left == 0 && frand01() < p) {
        left = (int)(0.004f * SR);                 /* 4 ms burst          */
        amp  = 0.25f + 0.25f * frand01();
        env  = 1.0f;
        k_lp = 0.35f + 0.6f * frand01();           /* each pop a bit different */
    }
    if (left == 0) return;

    const float decay = expf(-1.0f / (0.0007f * SR));   /* 0.7 ms */
    for (int i = 0; i < n && left > 0; i++, left--) {
        float x = (2.0f * frand01() - 1.0f) * env * amp;
        lp += k_lp * (x - lp);
        buf[i] += lp;
        env *= decay;
    }
}

/* ================= output tone stage =================
 * The packs are bright: in the stock pack the top octave sits only ~10 dB
 * below the midrange, partly the synthesised exhaust rasp itself and partly
 * linear-interpolation aliasing from voice() playing a loop above its own
 * pitch. On a small hard-mounted speaker, and especially on the i.MX93 MQS
 * output, that reads as a harsh fizz.
 *
 * But a flat cut is the wrong shape. That rasp IS the character when the
 * engine is pulling; it only grates when it sits there unchanging at idle.
 * Which is also how a real engine behaves — muffled at idle, brightness
 * arriving with revs and load — so the fix is a tone stage that opens up
 * instead of a filter that is always closed.
 *
 * brightness b, 0 at idle and 1 revving hard, drives a high shelf and a
 * low-pass. Both ends of both are tunable because the right amount depends on
 * the speaker and the enclosure, not on the sample pack:
 *
 *   TONE_SHELF_HZ   shelf corner, Hz                       (2000)
 *   TONE_IDLE_DB    shelf gain at b=0, dB                  (-14)
 *   TONE_REV_DB     shelf gain at b=1, dB                  (-2)
 *   TONE_LPF_LO_HZ  low-pass corner at b=0, Hz             (5000)
 *   TONE_LPF_HI_HZ  low-pass corner at b=1, Hz, 0 = open   (16000)
 *   TONE_RPM_LO     rpm where it starts opening            (1000)
 *   TONE_RPM_HI     rpm where it is fully open             (4200)
 *   TONE_LOAD_W     how much load darkens it, 0..1         (0.5)
 *   TONE_TAU        brightness smoothing, seconds          (0.12)
 *
 * Setting the older TONE_SHELF_DB / TONE_LPF_HZ pins both ends of that
 * control to one value, which gives back the previous static behaviour.
 */

typedef struct {
    float b0, b1, b2, a1, a2;   /* normalised by a0 */
    float x1, x2, y1, y2;
    int   on;
} Biquad;

static Biquad tone_shelf, tone_lpf;
static int    tone_ready = 0;

/* voicing, resolved once at startup */
static float t_shelf_hz, t_idle_db, t_rev_db;
static float t_lpf_lo, t_lpf_hi, t_rpm_lo, t_rpm_hi, t_load_w, t_tau;

/* live state */
static float tone_b    = -1.0f;   /* smoothed brightness; <0 = not seeded yet */
static float tone_b_cf = -2.0f;   /* brightness the coefficients were built at */

static void bq_highshelf(Biquad *q, float f0, float db)
{
    float A     = powf(10.0f, db / 40.0f);
    float w0    = 2.0f * (float)M_PI * f0 / SR;
    float cw    = cosf(w0);
    float alpha = sinf(w0) * 0.5f * sqrtf(2.0f);     /* shelf slope S = 1 */
    float beta  = 2.0f * sqrtf(A) * alpha;

    float a0 =        (A + 1.0f) - (A - 1.0f) * cw + beta;
    q->b0    =   A * ((A + 1.0f) + (A - 1.0f) * cw + beta) / a0;
    q->b1    = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cw) / a0;
    q->b2    =   A * ((A + 1.0f) + (A - 1.0f) * cw - beta) / a0;
    q->a1    =  2.0f * ((A - 1.0f) - (A + 1.0f) * cw) / a0;
    q->a2    =       ((A + 1.0f) - (A - 1.0f) * cw - beta) / a0;
}

static void bq_lowpass(Biquad *q, float f0)
{
    float w0    = 2.0f * (float)M_PI * f0 / SR;
    float cw    = cosf(w0);
    float alpha = sinf(w0) * 0.5f * sqrtf(2.0f);     /* Butterworth, Q = 1/sqrt2 */

    float a0 = 1.0f + alpha;
    q->b0    = (1.0f - cw) * 0.5f / a0;
    q->b1    = (1.0f - cw) / a0;
    q->b2    = q->b0;
    q->a1    = -2.0f * cw / a0;
    q->a2    = (1.0f - alpha) / a0;
}

static float env_f(const char *name, float dflt)
{
    const char *s = getenv(name);
    if (!s || !*s) return dflt;
    char *end;
    float v = strtof(s, &end);
    if (end == s) {
        fprintf(stderr, "%s: not a number ('%s'), using %g\n", name, s, dflt);
        return dflt;
    }
    return v;
}

/* 0 means "no low-pass". A corner just under Nyquist is transparent and keeps
 * the filter running, so the sweep never has to switch a biquad on or off
 * mid-stream and click. */
#define TONE_OPEN_HZ (0.45f * SR)

static float lpf_arg(const char *name, float dflt)
{
    float v = env_f(name, dflt);
    if (v <= 0.0f || v > TONE_OPEN_HZ) return TONE_OPEN_HZ;
    if (v < 200.0f) {
        fprintf(stderr, "%s: %g Hz is too low to be meant, using 200\n", name, v);
        return 200.0f;
    }
    return v;
}

static void tone_init(void)
{
    t_shelf_hz = env_f("TONE_SHELF_HZ", 2000.0f);
    t_idle_db  = env_f("TONE_IDLE_DB",  -14.0f);
    t_rev_db   = env_f("TONE_REV_DB",    -2.0f);
    t_lpf_lo   = lpf_arg("TONE_LPF_LO_HZ", 5000.0f);
    t_lpf_hi   = lpf_arg("TONE_LPF_HI_HZ", 16000.0f);
    t_rpm_lo   = env_f("TONE_RPM_LO",  1000.0f);
    t_rpm_hi   = env_f("TONE_RPM_HI",  4200.0f);
    t_load_w   = env_f("TONE_LOAD_W",     0.5f);
    t_tau      = env_f("TONE_TAU",       0.12f);

    /* the old static knobs pin both ends */
    if (getenv("TONE_SHELF_DB"))
        t_idle_db = t_rev_db = env_f("TONE_SHELF_DB", -10.0f);
    if (getenv("TONE_LPF_HZ"))
        t_lpf_lo = t_lpf_hi = lpf_arg("TONE_LPF_HZ", 7000.0f);

    if (t_rpm_hi < t_rpm_lo + 1.0f) {
        fprintf(stderr, "TONE_RPM_HI (%g) must exceed TONE_RPM_LO (%g), "
                        "using %g\n", t_rpm_hi, t_rpm_lo, t_rpm_lo + 1.0f);
        t_rpm_hi = t_rpm_lo + 1.0f;
    }
    if (t_load_w < 0.0f) t_load_w = 0.0f;
    if (t_load_w > 1.0f) t_load_w = 1.0f;
    if (t_tau    < 0.0f) t_tau    = 0.0f;

    memset(&tone_shelf, 0, sizeof tone_shelf);
    memset(&tone_lpf,   0, sizeof tone_lpf);
    tone_b = tone_b_cf = -1.0f;

    tone_shelf.on = (t_idle_db != 0.0f || t_rev_db != 0.0f)
                    && t_shelf_hz >= 20.0f && t_shelf_hz < 0.5f * SR;
    tone_lpf.on   = (t_lpf_lo < TONE_OPEN_HZ || t_lpf_hi < TONE_OPEN_HZ);
    tone_ready    = 1;

    if (!tone_shelf.on && !tone_lpf.on) { printf("tone: flat (original voicing)\n"); return; }

    int dyn = (t_idle_db != t_rev_db) || (t_lpf_lo != t_lpf_hi);
    printf("tone: ");
    if (!dyn) {
        if (tone_shelf.on) printf("shelf %+.1f dB above %.0f Hz", t_idle_db, t_shelf_hz);
        if (tone_lpf.on)   printf("%slow-pass %.0f Hz", tone_shelf.on ? ", " : "", t_lpf_lo);
        printf(" (static)\n");
        return;
    }
    printf("idle");
    if (tone_shelf.on) printf(" %+.1f dB", t_idle_db);
    if (tone_lpf.on)   printf(" / %.0f Hz", t_lpf_lo);
    printf("  ->  rev");
    if (tone_shelf.on) printf(" %+.1f dB", t_rev_db);
    if (tone_lpf.on)   printf(" / %.0f Hz", t_lpf_hi);
    printf("   over %.0f-%.0f rpm, load weight %.2f\n", t_rpm_lo, t_rpm_hi, t_load_w);
}

static void bq_run(Biquad *q, float *buf, int n)
{
    if (!q->on) return;
    for (int i = 0; i < n; i++) {
        float x = buf[i];
        float y = q->b0 * x + q->b1 * q->x1 + q->b2 * q->x2
                            - q->a1 * q->y1 - q->a2 * q->y2;
        q->x2 = q->x1; q->x1 = x;
        q->y2 = q->y1; q->y1 = y;
        buf[i] = y;
    }
}

/* Exposed so a UI can show what the ear is hearing. */
float tone_brightness(void) { return tone_b < 0.0f ? 0.0f : tone_b; }

float tone_shelf_db_now(void)
{
    if (!tone_shelf.on) return 0.0f;
    return t_idle_db + (t_rev_db - t_idle_db) * tone_brightness();
}

static void tone_run(float *buf, int n, float rpm, float load)
{
    if (!tone_ready) tone_init();
    if (!tone_shelf.on && !tone_lpf.on) return;

    /* rpm sets the envelope, load scales it: overrun at 4000 rpm should not be
     * as bright as full throttle at 4000 rpm. */
    float b = (rpm - t_rpm_lo) / (t_rpm_hi - t_rpm_lo);
    if (b < 0.0f) b = 0.0f;
    if (b > 1.0f) b = 1.0f;
    b *= b * (3.0f - 2.0f * b);                       /* smoothstep */
    if (load < 0.0f) load = 0.0f;
    if (load > 1.0f) load = 1.0f;
    b *= 1.0f - t_load_w + t_load_w * load;

    /* Smoothed, because a shift drops rpm by ~1500 in one block and stepping
     * the coefficients that far at once is audible as a click. */
    if (tone_b < 0.0f) tone_b = b;                    /* seed, no ramp in */
    else {
        float k = (t_tau > 0.0f)
                ? 1.0f - expf(-((float)n / SR) / t_tau) : 1.0f;
        tone_b += (b - tone_b) * k;
    }

    if (fabsf(tone_b - tone_b_cf) > 1e-4f) {          /* ~0.002 dB, inaudible */
        if (tone_shelf.on)
            bq_highshelf(&tone_shelf, t_shelf_hz,
                         t_idle_db + (t_rev_db - t_idle_db) * tone_b);
        if (tone_lpf.on)
            bq_lowpass(&tone_lpf, t_lpf_lo + (t_lpf_hi - t_lpf_lo) * tone_b);
        tone_b_cf = tone_b;
    }

    bq_run(&tone_shelf, buf, n);
    bq_run(&tone_lpf,   buf, n);
}

/* previous-block gains/ratios, so every block ramps from the last one */
static float pg[MAX_PTS][2], pr[MAX_PTS];
static int   have_prev = 0;

static void render(float *buf, int n, float rpm, float load)
{
    memset(buf, 0, sizeof(float) * n);

    if (rpm < pts[0].rpm)          rpm = pts[0].rpm;
    if (rpm > pts[npts - 1].rpm)   rpm = pts[npts - 1].rpm;

    int i = 0;
    while (i < npts - 2 && rpm > pts[i + 1].rpm) i++;

    float t  = (rpm - pts[i].rpm) / (pts[i + 1].rpm - pts[i].rpm);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    /* equal-power rpm crossfade */
    float glo = cosf(t * (float)M_PI * 0.5f);
    float ghi = sinf(t * (float)M_PI * 0.5f);

    /* equal-power load blend between the two layers */
    float gon  = sqrtf(load);
    float goff = sqrtf(1.0f - load);

    float gain[MAX_PTS][2], ratio[MAX_PTS];
    for (int k = 0; k < npts; k++) {
        gain[k][0] = gain[k][1] = 0.0f;
        ratio[k] = rpm / pts[k].rpm;
        if (ratio[k] < RATIO_MIN) ratio[k] = RATIO_MIN;
        if (ratio[k] > RATIO_MAX) ratio[k] = RATIO_MAX;
    }
    gain[i][0]     = glo * gon;   gain[i][1]     = glo * goff;
    gain[i + 1][0] = ghi * gon;   gain[i + 1][1] = ghi * goff;

    if (!have_prev) {
        memcpy(pg, gain, sizeof pg);
        memcpy(pr, ratio, sizeof pr);
        have_prev = 1;
    }

    for (int k = 0; k < npts; k++) {
        voice(buf, n, &pts[k].on,  &pts[k].pos_on,
              pg[k][0], gain[k][0], pr[k], ratio[k]);
        voice(buf, n, &pts[k].off, &pts[k].pos_off,
              pg[k][1], gain[k][1], pr[k], ratio[k]);
    }

    memcpy(pg, gain, sizeof pg);
    memcpy(pr, ratio, sizeof pr);

    crackle(buf, n, rpm, load);

    tone_run(buf, n, rpm, load);
}

/* ================= throttle source ================= */

#ifdef OFFLINE
/* scripted drive for the offline demo, incl. fake upshifts */
static int demo_throttle(float t, float rpm, int *shift_hold)
{
    static float hold = 0.0f;
    if (hold > 0.0f) { hold -= (float)PERIOD / SR; *shift_hold = 1; return 0; }
    *shift_hold = 0;

    if (rpm > 6500.0f && t > 6.0f && t < 24.0f) {   /* upshift */
        w *= 0.70f;
        hold = 0.12f;
        return 0;
    }
    if (t <  2.0f) return 0;                        /* idle */
    if (t <  2.35f) return 210;                     /* blip */
    if (t <  3.4f) return 0;
    if (t <  3.9f) return 255;                      /* hard blip */
    if (t <  5.5f) return 0;
    if (t < 22.0f) return 255;                      /* pull through the gears */
    if (t < 26.0f) return 0;                        /* overrun */
    if (t < 28.0f) return 110;                      /* part throttle cruise */
    return 0;
}
#elif !defined(SAMPLER_NO_MAIN)
static int read_pedal(void)
{
    /* IIO ADC on the i.MX95:
     *   FILE *f = fopen("/sys/bus/iio/devices/iio:device0/in_voltage0_raw","r");
     *   int raw; fscanf(f,"%d",&raw); fclose(f); return raw >> 4;
     */
    static int v = 0, dir = 1;
    v += dir * 2;
    if (v > 255) { v = 255; dir = -1; }
    if (v < 0)   { v = 0;   dir =  1; }
    return v;
}
#endif

/* ================= main ================= */
/* Define SAMPLER_NO_MAIN to reuse the sample bank and render() from another
 * program (drive_kbd.c does this). */
#ifndef SAMPLER_NO_MAIN
int main(int argc, char **argv)
{
#ifndef OFFLINE
    (void)argc; (void)argv;
#endif
    const char *dir = getenv("V8_PACK") ? getenv("V8_PACK") : "samples";
    if (load_pack(dir)) return 1;

    float buf[PERIOD];
    const float Ts = (float)PERIOD / SR;

#ifdef OFFLINE
    const char *outp = argc > 1 ? argv[1] : "demo.wav";
    const float dur = 30.0f;
    int total = (int)(dur * SR);
    int16_t *acc = malloc(sizeof(int16_t) * total);
    int wrote = 0;

    for (float t = 0.0f; wrote + PERIOD <= total; t += Ts) {
        int sh = 0;
        int thr = demo_throttle(t, w * 9.5493f, &sh);
        engine_step(thr, Ts);
        float rpm = w * 9.5493f;
        render(buf, PERIOD, rpm, u_f);
        float mg = sh ? 0.35f : 1.0f;                 /* duck during the shift */
        for (int i = 0; i < PERIOD; i++) {
            float s = buf[i] * 1.75f * mg;
            if (s >  1.0f) s =  1.0f;
            if (s < -1.0f) s = -1.0f;
            acc[wrote + i] = (int16_t)(s * 30000.0f);
        }
        wrote += PERIOD;
    }

    FILE *f = fopen(outp, "wb");
    uint32_t dl = wrote * 2, rl = 36 + dl;
    uint8_t h[44] = {0};
    memcpy(h, "RIFF", 4); memcpy(h + 4, &rl, 4); memcpy(h + 8, "WAVEfmt ", 8);
    uint32_t sz16 = 16; uint16_t f1 = 1, ch = 1, bps = 16, ba = 2;
    uint32_t sr = SR, br = SR * 2;
    memcpy(h + 16, &sz16, 4); memcpy(h + 20, &f1, 2); memcpy(h + 22, &ch, 2);
    memcpy(h + 24, &sr, 4);   memcpy(h + 28, &br, 4); memcpy(h + 32, &ba, 2);
    memcpy(h + 34, &bps, 2);  memcpy(h + 36, "data", 4); memcpy(h + 40, &dl, 4);
    fwrite(h, 1, 44, f);
    fwrite(acc, 2, wrote, f);
    fclose(f);
    printf("wrote %s (%.1f s)\n", outp, (float)wrote / SR);
    return 0;

#else
    snd_pcm_t *pcm;
    int err;
    const char *dev = getenv("ALSA_DEV") ? getenv("ALSA_DEV") : "default";

    if ((err = snd_pcm_open(&pcm, dev, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "open %s: %s\n", dev, snd_strerror(err)); return 1;
    }
    if ((err = snd_pcm_set_params(pcm, SND_PCM_FORMAT_S16_LE,
                                  SND_PCM_ACCESS_RW_INTERLEAVED,
                                  1, SR, 1, 20000)) < 0) {
        fprintf(stderr, "params: %s\n", snd_strerror(err)); return 1;
    }

    int16_t out[PERIOD];
    for (;;) {
        engine_step(read_pedal(), Ts);
        render(buf, PERIOD, w * 9.5493f, u_f);
        for (int i = 0; i < PERIOD; i++) {
            float s = buf[i] * 1.75f;
            if (s >  1.0f) s =  1.0f;
            if (s < -1.0f) s = -1.0f;
            out[i] = (int16_t)(s * 30000.0f);
        }
        snd_pcm_sframes_t n = snd_pcm_writei(pcm, out, PERIOD);
        if (n < 0) snd_pcm_recover(pcm, (int)n, 1);
    }
    return 0;
#endif
}
#endif /* SAMPLER_NO_MAIN */
