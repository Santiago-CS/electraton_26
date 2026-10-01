/*
 * engine_sim.c - procedural 4-cylinder engine sound driven by a 0..255 throttle
 *
 * build: gcc -O2 -o engine_sim engine_sim.c -lasound -lm
 * run  : ./engine_sim            (ALSA "default"; use plughw:0,0 to force the SAI codec)
 *
 * Model chain:
 *   throttle(0..255) -> torque -> 1st-order crank inertia -> rpm
 *   rpm -> firing frequency -> impulse train -> fixed exhaust resonators
 *   throttle -> brightness / amplitude / intake noise (independent of pitch)
 */

#include <alsa/asoundlib.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define SR            48000
#define PERIOD        256                  /* frames per block -> 5.3 ms control rate */
#define CYLINDERS     4
#define FIRE_PER_REV  (CYLINDERS / 2.0f)   /* 4-stroke: half the cylinders fire per rev */

/* ------------------------------------------------------------------ */
/* 1. Engine plant - single state variable, scalar throughout          */
/* ------------------------------------------------------------------ */

static float w   = 80.0f;    /* crank speed [rad/s], ~765 rpm */
static float u_f = 0.0f;     /* filtered throttle, 0..1 */
static float cut = 0.0f;     /* rev-limiter fuel-cut timer [s] */

static const float J      = 0.18f;     /* rotating inertia      [kg m^2]  */
static const float T_MAX  = 220.0f;    /* peak engine torque    [N m]     */
static const float W_PK   = 450.0f;    /* speed at peak torque  [rad/s]   */
static const float B_FRIC = 0.045f;    /* linear friction       [N m s]   */
static const float K_DRAG = 4.5e-4f;   /* quadratic road load   [N m s^2] */
static const float W_IDLE = 78.5f;     /* idle target, 750 rpm            */
static const float W_MAX  = 733.0f;    /* limiter, 7000 rpm               */

static void engine_step(uint8_t throttle, float Ts)
{
    float u = throttle * (1.0f / 255.0f);

    /* pedal travel + manifold filling lag: the pedal is never a step input */
    u_f += 0.15f * (u - u_f);

    /* normalised torque curve, parabola peaking at W_PK */
    float r = (w - W_PK) / W_PK;
    float g = 1.0f - 0.8f * r * r;
    if (g < 0.15f) g = 0.15f;

    float Te = u_f * T_MAX * g;

    /* idle air control: proportional, additive only */
    if (w < W_IDLE * 1.3f) {
        float e = W_IDLE - w;
        if (e > 0.0f) Te += 6.0f * e;
    }

    /* rev limiter: 60 ms hard fuel cut -> the classic bouncing-off-the-limiter sound */
    if (w > W_MAX) cut = 0.06f;
    if (cut > 0.0f) { Te = 0.0f; cut -= Ts; }

    float Tl = B_FRIC * w + K_DRAG * w * w;   /* friction + drivetrain/road load */

    /* forward Euler:  w[k+1] = w[k] + (Ts/J)(Te - Tl) */
    w += (Ts / J) * (Te - Tl);
    if (w < 45.0f) w = 45.0f;                 /* stall floor */
}

/* ------------------------------------------------------------------ */
/* 2. Two-pole resonators (exhaust / intake formants)                  */
/* ------------------------------------------------------------------ */

typedef struct { float b0, a1, a2, z1, z2; } BP;

static BP bp0, bp1, bp2, bpn;

static void bp_set(BP *f, float fc, float q)
{
    float wc = 2.0f * (float)M_PI * fc / (float)SR;
    float r  = expf(-wc / (2.0f * q));
    f->a1 = 2.0f * r * cosf(wc);
    f->a2 = -r * r;
    f->b0 = 1.0f - r;
    f->z1 = f->z2 = 0.0f;
}

static inline float bp_run(BP *f, float x)
{
    float y = f->b0 * x + f->a1 * f->z1 + f->a2 * f->z2;
    f->z2 = f->z1;
    f->z1 = y;
    return y;
}

/* ------------------------------------------------------------------ */
/* 3. Synthesis                                                        */
/* ------------------------------------------------------------------ */

static float phase = 0.0f;   /* 0..1 between firing events */
static float lp    = 0.0f;   /* brightness one-pole */

static inline float frand(void)
{
    return (float)rand() * (2.0f / (float)RAND_MAX) - 1.0f;
}

static void render(int16_t *out, int n, float f_fire, float load, float rpm_n)
{
    float inc = f_fire / (float)SR;

    for (int i = 0; i < n; i++) {
        float exc = 0.0f;

        phase += inc;
        if (phase >= 1.0f) {
            phase -= 1.0f;
            /* cylinder-to-cylinder amplitude scatter.
               Without this jitter the result is a synth buzz, not an engine. */
            float jitter = 0.82f + 0.35f * fabsf(frand());
            exc = jitter * (0.55f + 0.45f * load);
        }

        /* exhaust body: fixed formants, excited by the impulse train.
           Pitch follows the pulse rate; the formants stay put, like a real pipe. */
        float y  = bp_run(&bp0, exc);
        y       += bp_run(&bp1, exc) * 0.60f;
        y       += bp_run(&bp2, exc) * 0.35f;

        /* intake / turbulence: grows with both load and speed */
        float nz = frand() * (0.05f + 0.25f * load) * (0.30f + 0.70f * rpm_n);
        y += bp_run(&bpn, nz) * 0.8f;

        /* brightness opens with throttle - this is what "on throttle" sounds like */
        float a = 0.05f + 0.55f * load;
        lp += a * (y - lp);

        float s = lp * (0.25f + 0.75f * load) * 0.9f;
        if (s >  1.0f) s =  1.0f;
        if (s < -1.0f) s = -1.0f;

        out[i] = (int16_t)(s * 30000.0f);
    }
}

/* ------------------------------------------------------------------ */
/* 4. Pedal input - replace with your own source                       */
/* ------------------------------------------------------------------ */

static uint8_t read_pedal(void)
{
    /* e.g. an IIO ADC channel on the i.MX95:
     *   FILE *f = fopen("/sys/bus/iio/devices/iio:device0/in_voltage0_raw", "r");
     *   int raw; fscanf(f, "%d", &raw); fclose(f);
     *   return (uint8_t)(raw >> 4);            // 12-bit -> 8-bit
     *
     * Demo: slow triangle sweep so you can hear the whole range.
     */
    static int v = 0, dir = 1;
    v += dir * 2;
    if (v > 255) { v = 255; dir = -1; }
    if (v < 0)   { v = 0;   dir =  1; }
    return (uint8_t)v;
}

/* ------------------------------------------------------------------ */

int main(void)
{
    snd_pcm_t *pcm;
    int err;

    if ((err = snd_pcm_open(&pcm, "default", SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "open: %s\n", snd_strerror(err));
        return 1;
    }
    if ((err = snd_pcm_set_params(pcm,
                                  SND_PCM_FORMAT_S16_LE,
                                  SND_PCM_ACCESS_RW_INTERLEAVED,
                                  1,          /* mono */
                                  SR,
                                  1,          /* allow resampling */
                                  20000)) < 0) {   /* 20 ms latency */
        fprintf(stderr, "params: %s\n", snd_strerror(err));
        return 1;
    }

    /* exhaust formants - tune these to change the car's character */
    bp_set(&bp0, 105.0f, 6.0f);
    bp_set(&bp1, 250.0f, 5.0f);
    bp_set(&bp2, 720.0f, 3.0f);
    bp_set(&bpn, 1800.0f, 1.2f);   /* intake hiss */

    int16_t buf[PERIOD];
    const float Ts = (float)PERIOD / (float)SR;

    for (;;) {
        engine_step(read_pedal(), Ts);

        float rpm    = w * 9.5493f;                                  /* rad/s -> rpm */
        float f_fire = (w / (2.0f * (float)M_PI)) * FIRE_PER_REV;    /* Hz */
        float rpm_n  = (rpm - 700.0f) / 6300.0f;
        if (rpm_n < 0.0f) rpm_n = 0.0f;
        if (rpm_n > 1.0f) rpm_n = 1.0f;

        render(buf, PERIOD, f_fire, u_f, rpm_n);

        snd_pcm_sframes_t w2 = snd_pcm_writei(pcm, buf, PERIOD);
        if (w2 < 0) snd_pcm_recover(pcm, (int)w2, 1);
    }

    snd_pcm_close(pcm);
    return 0;
}
