
#include <math.h>
#include <string.h>
#include <stdint.h>

#define SR 48000.0f
#define BLOCK 32
#define PI 3.14159265f
#define TWOPI 6.2831853f
#define CVOK 6
#define CVTOM 7                   /* tom pitch in Hz (voices 1, 3, 5) */
#define CVAT 8                    /* amp segment: time (samples), up, target; then the filter's */

/* the voices one by one, for the AU plugin's separate outputs (core/src/
   rytm_dsp_taps.c defines these); empty here and for the Python */
#ifndef RYTM_VOICE_OUT
#define RYTM_VOICE_FIELDS
#define RYTM_DSP_FIELDS
#define RYTM_VOICE_OUT(v, a, b)
#define RYTM_VOICE_BEGIN(d, i)
#define RYTM_VOICE_END(d, i)
#endif

/* the FPGA's generator bank (frame words 180 + 4g): the firmware sends
   segments -- [flags, start<<8 | target (signed, /127), mode:4 flags:8 rate:20]
   -- and the bank runs them sample by sample; the machine models read `lev` */
#define NGEN 32
typedef struct { float lev, T, tgt, start, Tlong, relT; int up, on, rel; } gen_t;

typedef struct {
    int on, model, track, idx;
    float p[32];
    float vel, t;                 /* seconds since trigger */
    float ph[8];                  /* oscillator phases */
    float mph;                    /* modulator phase */
    float s1, s2;                 /* main SVF */
    float a1, a2, b1, b2;         /* two auxiliary SVFs (noise shaping) */
    float hx, hy;                 /* one-pole states */
    float rnd[8];                 /* per-note random values */
    float dust;
    uint32_t rng;
    float fade;                   /* the previous note's tail, faded out */
    float last;
    float cv[20];
    float tzr, tzi, strike;       /* tom resonator state; this sample's strike */
    float aenv, fenv;             /* the FPGA's envelope generators (frame-driven) */
    int aseen, fseen;
    gen_t *g;                     /* XD voices: sweep osc1, sweep osc2, VCA osc1, VCA osc2 */
    float mf[6];                  /* metal voices: the six oscillators' Hz from the frame (0 = none) */
    int ext_seen;                 /* the DSP sends this note a signal (noise, click, clap) */
    float vca[2];                 /* XD voices: the VCAs as gates (xd_vca) */
    RYTM_VOICE_FIELDS
} voice_t;

#define DMAX 96000                /* 2 s at 48 kHz */
#define NCOMB 8
#define NAP 4


typedef struct {
    voice_t v[8];
    gen_t g[NGEN];
    uint16_t gprev[4 * NGEN]; int gprev_ok;   /* the generator blocks of the last frame */
    float fx[32];
    float in[8][BLOCK];           /* the firmware's sample layer per voice (TX slots 8-15) */
    float strike[8][BLOCK];       /* the toms' strikes (TX slots 0-2 -> voices 1, 3, 5) */
    float dl[DMAX], dr[DMAX];
    int dpos;
    float dlp_l, dlp_r, dhp_l, dhp_r;
    float dtime_s;
    float pre[9600]; int prepos;
    float comb[2][NCOMB][2400]; int clen[2][NCOMB], cpos[2][NCOMB]; float cdamp[2][NCOMB];
    float ap[2][NAP][700]; int alen[2][NAP], apos[2][NAP];
    float rhp, rsh_l, rsh_r;
    float cenv, sc_lp;
    RYTM_DSP_FIELDS
} dsp_t;

static float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static float kn(float v) { return clampf(v, 0, 127) / 127.0f; }
static float ktime(float v, float lo, float hi) { return lo * powf(hi / lo, kn(v)); }
static float frand(uint32_t *s) { *s = *s * 1664525u + 1013904223u; return ((*s >> 8) * (1.0f / 8388608.0f)) - 1.0f; }
static float edec(float t, float tdec) { return expf(-t * 4.6f / (tdec > 1e-4f ? tdec : 1e-4f)); }
static float ahd(float t, float hold, float tdec) { return t < hold ? 1.0f : edec(t - hold, tdec); }
static float semis(float v, float per) { return powf(2.0f, (v - 64.0f) * per / 12.0f); }
static float adv(float *ph, float f) { *ph += f / SR; *ph -= floorf(*ph); return *ph; }
static float sine(float ph) { return sinf(TWOPI * ph); }
static float tri(float ph) { return 4.0f * fabsf(ph - 0.5f) - 1.0f; }
static float saw(float ph) { return 2.0f * ph - 1.0f; }
static float pulse(float ph, float pw) { return ph < pw ? 1.0f : -1.0f; }

/* The XD voices (BD, SD, RS): two linear VCOs. Frame words 45+4k / 46+4k
   are their codes (cv[14], cv[15]); f = XD_HZ * code -- 0.0637 Hz a code
   puts SD CLASSIC's defaults on the 808's 238/476 Hz, RS CLASSIC's on
   455/1667 Hz, the BD's on C2 (docs/CALIBRAZIONE.md). The sweeps are the
   generators 0/1 of the voice, added to the code (level x 127 x 256); the
   VCAs are generators 2/3. */
#define XD_HZ 0.06369f
#define CVXD 14
static int xd_on(voice_t *v, int o)
{
    return v->g && (v->idx == 0 || v->idx == 2 || v->idx == 7) && v->cv[CVOK] > 0 && v->cv[CVXD + o] > 0;
}
static float xd_hz(voice_t *v, int o)
{
    return XD_HZ * fabsf(v->cv[CVXD + o] + 32512.0f * v->g[o].lev);
}
/* The XD VCAs (generators 2/3 of the voice) as the firmware uses them: full
   from the trigger, then an exponential release (tau = T) once a release
   segment (target 0x80) arrives -- after the machine's hold (BD PLASTIC ~21 ms,
   BD CLASSIC HLD) or at once (SD HARD). The other segments on them are not
   levels: BD CLASSIC loads "127 -> 0 in one sample" at the trigger and still
   sounds (docs/CALIBRAZIONE.md). Called once a sample per oscillator. */
static float xd_vca(voice_t *v, int o)
{
    gen_t *g = &v->g[2 + o];
    if (g->rel) v->vca[o] *= expf(-1.0f / (g->relT > 1 ? g->relT : 1));
    return v->vca[o];
}

/* The metal voices' decays: the time (to -40 dB, 4.6 T) of the machine's
   decay generator (CH voice from g12, CY voice from g17; relative index per
   machine, docs/CALIBRAZIONE.md), or `fallback` (s) without one. The shape
   stays the model's: the generators' extra flags (0x61) are not decoded. */
static float metal_dec(const voice_t *v, int rel, float fallback)
{
    if (!v->g || v->cv[CVOK] <= 0 || v->g[rel].Tlong <= 0) return fallback;
    return 4.6f * v->g[rel].Tlong / SR;          /* to -40 dB, tau = T */
}

/* The toms (BT, LT, MT on voices 1, 3, 5): the resonator the analog
   calibration was run on (tools/calloop.py Circuits, docs/CALIBRAZIONE.md
   TOMS) -- f from the pitch word (cv[CVTOM]), damping alpha = f * 2^(cv/3200)
   - 120 /s from word 60+2k (cv[16]; the firmware moves it in time for DEC and
   raises it to choke), struck by TX slot 0-2 (a ramp, ~11 ms), a limiter at
   |z| = 1. The strike's gain is not fixed by the calibration (it compares
   amplitudes): TOM_STRIKE. */
#define CVDAMP 16
#define TOM_SLOPE 1010.0f
#define TOM_OUT 3.0f                /* level: chosen so a tom peaks like the XD voices in a render (declared) */
#define TOM_STRIKE 0.025f           /* the runtime strike (~0.38 FS, 11 ms) rings a 120 Hz tom to ~full scale */
static int tom_on(const voice_t *v)
{
    return (v->idx == 1 || v->idx == 3 || v->idx == 5) && v->cv[CVOK] > 0 && v->cv[CVTOM] > 1;
}
static float tom_sample(voice_t *v)
{
    /* alpha: 0.32 /s (tau = 150000 samples, the calibration's target) at the
       CV the calibration found, D(f) = 3200 log2(120.3 / f), then doubling
       every TOM_SLOPE codes (FITTED: the calibration does not fix it; 1010
       gives ~0.25 s after the firmware's ~27 ms hold at default DEC) */
    float f = v->cv[CVTOM], D = 3200.0f * log2f(120.3f / f);
    float alpha = 0.32f * powf(2.0f, (v->cv[CVDAMP] - D) / TOM_SLOPE);
    float r = expf(-alpha / SR), c = cosf(TWOPI * f / SR), s = sinf(TWOPI * f / SR);
    float zr = r * (v->tzr * c - v->tzi * s) + TOM_STRIKE * v->strike, zi = r * (v->tzr * s + v->tzi * c);
    float m = zr * zr + zi * zi;
    if (m > 1.0f) { m = 1.0f / sqrtf(m); zr *= m; zi *= m; }
    if (m < 1e-12f) zr += 1e-6f;                 /* circuit noise: self-oscillation can start */
    v->tzr = zr; v->tzi = zi;
    return zr;
}

/* Chamberlin SVF, 2x oversampled. mode: 0 LP2 1 LP1 2 BP 3 HP1 4 HP2 5 BS 6 PK */
static float svf(float *s1, float *s2, float x, float cut, float q, int mode)
{
    float f = 2.0f * sinf(PI * clampf(cut, 20.0f, SR / 4.5f) / (2.0f * SR));
    float damp = 1.0f / (q > 0.5f ? q : 0.5f), low = *s1, band = *s2, high = 0;
    int k;
    for (k = 0; k < 2; k++) {
        low += f * band;
        high = x - low - damp * band;
        band += f * high;
    }
    *s1 = low; *s2 = band;
    switch (mode) {
    case 0: return low;
    case 1: return 0.5f * (low + band * damp * 0.5f);
    case 2: return band;
    case 3: return 0.5f * (high + band * damp * 0.5f);
    case 4: return high;
    case 5: return low + high;
    default: return x + band * damp * 2.0f;
    }
}
static float bp(float *s1, float *s2, float x, float f, float q) { return svf(s1, s2, x, f, q, 2); }
static float hp(float *s1, float *s2, float x, float f) { return svf(s1, s2, x, f, 0.7f, 4); }
static float lp(float *s1, float *s2, float x, float f, float q) { return svf(s1, s2, x, f, q, 0); }

/* the 808's six square oscillators, as frequency ratios */
static const float METAL[6] = {1.0f, 1.4829f, 1.8003f, 2.5463f, 2.6304f, 3.8967f};
static float metal6(voice_t *v, float base, float pw)
{
    float s = 0; int k;
    /* the firmware's own six frequencies when it sends them (frame 308+ / 320+:
       0.02313 Hz a code, CH/OH/CY CLASSIC on the 808's 205.3..800 Hz) */
    for (k = 0; k < 6; k++)
        s += pulse(adv(&v->ph[k], v->mf[0] > 0 && v->cv[CVOK] > 0 ? v->mf[k] : base * METAL[k]), pw);
    return s / 6.0f;
}

/* one sample of the voice's source (before the filter) */
static float source(voice_t *v)
{
    float *p = v->p, t = v->t, x = 0, n = frand(&v->rng);
    switch (v->model) {
    case 0: case 26: { /* BD HARD: LEV TUN DEC HLD SWT SNP WAV TIC; BD SHARP: LEV TUN DEC HLD SWT SWD WAV TIC */
        float swt = ktime(p[4], 0.001f, 0.12f);
        float depth = v->model == 0 ? 3.2f : 1.0f + 4.0f * kn(p[5]);
        int fr = xd_on(v, 1);
        float f = fr ? xd_hz(v, 1) : 46.0f * semis(p[1], 0.5f) * powf(2.0f, depth * expf(-t / swt));
        float ph = adv(&v->ph[0], f), b;
        int wav = (int)(p[6] + 0.5f);
        b = wav == 0 ? sine(ph) : wav == 1 ? sine(ph) + 0.35f * sine(2 * ph) : tri(ph);
        x = b * (fr ? xd_vca(v, 1) : ahd(t, 0.6f * kn(p[3]) * kn(p[3]), ktime(p[2], 0.02f, 6.0f)));
        if (v->model == 0 && !v->ext_seen) x += pulse(adv(&v->ph[1], f * 7.0f), 0.5f) * expf(-t / 0.0025f) * kn(p[5]) * 0.6f;
        if (!v->ext_seen) x += n * expf(-t / 0.0009f) * kn(p[7]) * 0.9f;
        break; }
    case 1: { /* BD CLASSIC (909): LEV TUN DEC HLD SWT SWD WAV TRA */
        float swt = ktime(p[4], 0.002f, 0.15f), swd = 4.0f * kn(p[5]);
        int fr = xd_on(v, 1);
        float f = fr ? xd_hz(v, 1) : 50.0f * semis(p[1], 0.5f) * powf(2.0f, swd * expf(-t / swt));
        float ph = adv(&v->ph[0], f), b;
        int wav = (int)(p[6] + 0.5f);
        b = wav == 0 ? sine(ph) : wav == 1 ? tri(ph) : clampf(sine(ph) * 2.2f, -1, 1);
        x = b * (fr ? xd_vca(v, 1) : ahd(t, 0.6f * kn(p[3]) * kn(p[3]), ktime(p[2], 0.03f, 6.0f)));
        v->hy = 0.8f * (v->hy + n - v->hx); v->hx = n;
        if (!v->ext_seen) x += (v->hy * 0.6f + pulse(adv(&v->ph[1], 1800.0f), 0.5f) * 0.4f) * expf(-t / 0.002f) * kn(p[7]);
        break; }
    case 13: { /* BD FM: LEV TUN DEC FMA SWT FMS FMD FMT */
        float swt = ktime(p[4], 0.002f, 0.12f);
        int fr = xd_on(v, 0) && xd_on(v, 1);
        float f = fr ? xd_hz(v, 0) : 52.0f * semis(p[1], 0.5f) * powf(2.0f, 2.5f * expf(-t / swt));
        float mf = fr ? xd_hz(v, 1) : f * semis(p[7], 1.0f) * 1.5f * powf(2.0f, 2.0f * kn(p[5]) * expf(-t / 0.03f));
        float idx = 6.0f * kn(p[3]) * (fr ? xd_vca(v, 1) : edec(t, ktime(p[6], 0.005f, 2.0f)));
        float m = sine(adv(&v->mph, mf));
        x = sinf(TWOPI * adv(&v->ph[0], f) + idx * m) * (fr ? xd_vca(v, 0) : edec(t, ktime(p[2], 0.03f, 6.0f)));
        break; }
    case 21: { /* BD PLASTIC: LEV TUN DEC TYP MOD SWT SWD TIC (TUN: 0.62 semitone a step, from the frame) */
        float swt = ktime(p[5], 0.002f, 0.2f), swd = 4.0f * kn(p[6]);
        int fr = xd_on(v, 0);
        float f = fr ? xd_hz(v, 0) : 50.0f * semis(p[1], 0.62f) * powf(2.0f, swd * expf(-t / swt));
        float f2 = xd_on(v, 1) ? xd_hz(v, 1) : f * 2.0f;
        float ph = adv(&v->ph[0], f), b = sine(ph);
        if (p[3] >= 0.5f) b = sinf(TWOPI * ph + 2.5f * kn(p[4]) * sine(adv(&v->ph[1], f2)));
        else b = b * (1.0f - kn(p[4])) + clampf(b * (1 + 6 * kn(p[4])), -1, 1) * kn(p[4]);
        x = b * (fr ? xd_vca(v, 0) : edec(t, ktime(p[2], 0.03f, 6.0f)));
        if (!v->ext_seen) x += n * expf(-t / 0.0012f) * kn(p[7]) * 0.8f;
        break; }
    case 22: { /* BD SILKY: LEV TUN DEC HLD SWT SWD DUS CLK */
        float swt = ktime(p[4], 0.003f, 0.2f), swd = 3.0f * kn(p[5]);
        int fr = xd_on(v, 0);
        float f = fr ? xd_hz(v, 0) : 44.0f * semis(p[1], 0.5f) * powf(2.0f, swd * expf(-t / swt));
        x = sine(adv(&v->ph[0], f)) * (fr ? xd_vca(v, 0) : ahd(t, 0.5f * kn(p[3]) * kn(p[3]), ktime(p[2], 0.05f, 8.0f)));
        if (frand(&v->rng) > 1.0f - 0.004f * kn(p[6])) v->dust = frand(&v->rng);
        v->dust *= 0.93f;
        x += v->dust * kn(p[6]) * 0.7f;
        x += (t < 0.0006f ? 1.0f : 0.0f) * kn(p[7]);
        break; }
    case 30: { /* BD ACOUSTIC: LEV TUN DEC HLD SWT SWD WAV IMP */
        float swt = ktime(p[4], 0.002f, 0.15f), swd = 2.5f * kn(p[5]);
        int fr = xd_on(v, 1);
        float f = fr ? xd_hz(v, 1) : 55.0f * semis(p[1], 0.5f) * powf(2.0f, swd * expf(-t / swt));
        float ph = adv(&v->ph[0], f), b;
        int wav = (int)(p[6] + 0.5f);
        b = wav == 0 ? sine(ph) : wav == 1 ? 0.7f * sine(ph) + 0.3f * sine(adv(&v->ph[1], f * 1.59f)) : tri(ph);
        x = b * (fr ? xd_vca(v, 1) : ahd(t, 0.4f * kn(p[3]) * kn(p[3]), ktime(p[2], 0.04f, 5.0f)));
        if (!v->ext_seen) x += bp(&v->a1, &v->a2, n, 2200.0f, 1.2f) * expf(-t / 0.012f) * kn(p[7]) * 1.5f;
        break; }
    case 2: { /* SD HARD: LEV TUN DEC SWD TIC NOD NOL SWT */
        float swt = ktime(p[7], 0.002f, 0.1f), swd = 2.0f * kn(p[3]);
        int fr = xd_on(v, 0) && xd_on(v, 1);
        float f = fr ? xd_hz(v, 0) : 190.0f * semis(p[1], 0.5f) * powf(2.0f, swd * expf(-t / swt));
        float f2 = fr ? xd_hz(v, 1) : f * 1.62f;
        float e = fr ? 1.0f : edec(t, ktime(p[2], 0.02f, 2.0f));
        float tone = 0.65f * sine(adv(&v->ph[0], f)) * (fr ? xd_vca(v, 0) : e)
                     + 0.35f * sine(adv(&v->ph[1], f2)) * (fr ? xd_vca(v, 1) : e);
        float nz = hp(&v->a1, &v->a2, n, 1800.0f);
        x = tone;
        if (!v->ext_seen) x += nz * edec(t, ktime(p[5], 0.02f, 2.0f)) * kn(p[6]) * 1.2f + n * expf(-t / 0.0008f) * kn(p[4]);
        break; }
    case 3: { /* SD CLASSIC (808): LEV TUN DEC DET SNP NOD NOL BAL */
        int fr = xd_on(v, 0) && xd_on(v, 1);
        float f = fr ? xd_hz(v, 0) : 180.0f * semis(p[1], 0.5f), f2 = fr ? xd_hz(v, 1) : f * (1.47f + 0.6f * kn(p[3]));
        float o1 = sine(adv(&v->ph[0], f)) * (fr ? xd_vca(v, 0) : edec(t, ktime(p[2], 0.02f, 1.5f)));
        float o2 = sine(adv(&v->ph[1], f2)) * (fr ? xd_vca(v, 1) : edec(t, ktime(p[2], 0.015f, 1.0f)));
        float bal = kn(p[7]);
        float nz = hp(&v->a1, &v->a2, n, 900.0f + 4000.0f * kn(p[4]));
        x = (o1 * (1 - bal) + o2 * bal) * 0.9f;
        if (!v->ext_seen) x += nz * edec(t, ktime(p[5], 0.02f, 1.5f)) * kn(p[6]) * 1.2f;
        break; }
    case 14: { /* SD FM: LEV TUN DEC FMT FMD NOD NOL FMA */
        int fr = xd_on(v, 0) && xd_on(v, 1);
        float f = fr ? xd_hz(v, 0) : 200.0f * semis(p[1], 0.5f);
        float idx = 5.0f * kn(p[7]) * (fr ? xd_vca(v, 1) : edec(t, ktime(p[4], 0.004f, 1.0f)));
        float m = sine(adv(&v->mph, fr ? xd_hz(v, 1) : f * semis(p[3], 1.0f) * 1.41f));
        x = sinf(TWOPI * adv(&v->ph[0], f) + idx * m) * (fr ? xd_vca(v, 0) : edec(t, ktime(p[2], 0.02f, 1.5f)));
        if (!v->ext_seen) x += hp(&v->a1, &v->a2, n, 2500.0f) * edec(t, ktime(p[5], 0.02f, 1.5f)) * kn(p[6]) * 1.2f;
        break; }
    case 23: { /* SD NATURAL: LEV TUN BDY DEC BAL LPF HPF RES */
        int fr = xd_on(v, 1);
        float f = fr ? xd_hz(v, 1) : 175.0f * semis(p[1], 0.5f);
        float body = (0.6f * sine(adv(&v->ph[0], f)) + 0.4f * sine(adv(&v->ph[1], f * 2.3f)))
                     * (fr ? xd_vca(v, 1) : edec(t, ktime(p[2], 0.01f, 0.8f)));
        float nz = lp(&v->b1, &v->b2, n, 300.0f * powf(60.0f, kn(p[5])), 0.6f + 6.0f * kn(p[7]));
        nz = hp(&v->a1, &v->a2, nz, 40.0f * powf(100.0f, kn(p[6])));
        x = body * (1 - kn(p[4])) * 1.2f + nz * kn(p[4]) * 1.4f * edec(t, ktime(p[3], 0.02f, 1.5f));
        break; }
    case 31: { /* SD ACOUSTIC: LEV TUN BDY NOD NOL HLD SWD IMP */
        int fr = xd_on(v, 0);
        float f = fr ? xd_hz(v, 0) : 185.0f * semis(p[1], 0.5f) * powf(2.0f, 1.5f * kn(p[6]) * expf(-t / 0.01f));
        float body = sine(adv(&v->ph[0], f)) * (fr ? xd_vca(v, 0) : ahd(t, 0.2f * kn(p[5]) * kn(p[5]), ktime(p[2], 0.01f, 1.0f)));
        float nz = bp(&v->a1, &v->a2, n, 4500.0f, 0.5f);
        if (fr && xd_on(v, 1)) body += 0.5f * sine(adv(&v->ph[1], xd_hz(v, 1))) * xd_vca(v, 1);
        x = body;
        if (!v->ext_seen) x += nz * edec(t, ktime(p[3], 0.02f, 1.5f)) * kn(p[4]) * 1.3f
            + bp(&v->b1, &v->b2, n, 1200.0f, 1.0f) * expf(-t / 0.006f) * kn(p[7]) * 2.0f;
        break; }
    case 4: { /* RS HARD: LEV TUN DEC SWD TIC NOL SYM SWT */
        float swt = ktime(p[7], 0.001f, 0.05f);
        int fr = xd_on(v, 0);
        float f = fr ? xd_hz(v, 0) : 900.0f * semis(p[1], 0.5f) * powf(2.0f, 2.0f * kn(p[3]) * expf(-t / swt));
        x = pulse(adv(&v->ph[0], f), 0.1f + 0.8f * kn(p[6])) * (fr ? xd_vca(v, 0) : edec(t, ktime(p[2], 0.01f, 0.6f)));
        if (fr && xd_on(v, 1)) x = 0.6f * x + 0.4f * pulse(adv(&v->ph[1], xd_hz(v, 1)), 0.1f + 0.8f * kn(p[6])) * xd_vca(v, 1);
        if (!v->ext_seen) x += hp(&v->a1, &v->a2, n, 3000.0f) * edec(t, 0.03f) * kn(p[5]) + n * expf(-t / 0.0006f) * kn(p[4]);
        break; }
    case 5: { /* RS CLASSIC (808): LEV T1 DEC BAL T2 SYM NOL TIC */
        int fr = xd_on(v, 0) && xd_on(v, 1);
        float f1 = fr ? xd_hz(v, 1) : 455.0f * semis(p[1], 0.5f), f2 = fr ? xd_hz(v, 0) : 1667.0f * semis(p[4], 0.5f), pw = 0.1f + 0.8f * kn(p[5]);
        float o = pulse(adv(&v->ph[0], f1), pw) * (1 - kn(p[3])) * (fr ? xd_vca(v, 1) : 1.0f)
                  + pulse(adv(&v->ph[1], f2), pw) * kn(p[3]) * (fr ? xd_vca(v, 0) : 1.0f);
        x = bp(&v->a1, &v->a2, o, 1100.0f, 1.5f) * 2.0f * (fr ? 1.0f : edec(t, ktime(p[2], 0.005f, 0.5f)));
        if (!v->ext_seen) x += hp(&v->b1, &v->b2, n, 5000.0f) * edec(t, 0.02f) * kn(p[6]) + n * expf(-t / 0.0005f) * kn(p[7]);
        break; }
    case 6: { /* CP CLASSIC: LEV TON NOD NUM RAT NOL RND CPD */
        /* the clap is the DSP's (TX slot 8+v, no frame word changes with any
           parameter): nothing of ours on top of it */
        int k, num = 1 + (int)(kn(p[3]) * 7.0f + 0.5f);
        if (v->ext_seen) { x = 0; break; }
        {
        float rate = ktime(p[4], 0.004f, 0.04f), e = 0, cpd = ktime(p[7], 0.002f, 0.05f);
        for (k = 0; k < num; k++) {
            float t0 = k * rate * (1.0f + kn(p[6]) * v->rnd[k & 7]);
            if (t >= t0) e += expf(-(t - t0) / cpd);
        }
        e += edec(t, ktime(p[2], 0.02f, 2.0f)) * kn(p[5]) * 0.8f * (t > num * rate ? 1.0f : 0.0f);
        x = bp(&v->a1, &v->a2, n, 600.0f * powf(8.0f, kn(p[1])), 1.8f) * e * 1.6f;
        }
        break; }
    case 7: { /* BT CLASSIC: LEV TUN DEC - NOL SNP SWD - */
        float f = 64.0f * semis(p[1], 0.5f) * powf(2.0f, 1.2f * kn(p[6]) * expf(-t / 0.04f));
        if (tom_on(v)) { x = tom_sample(v) * TOM_OUT; break; }   /* the circuit, as the firmware drives it */
        x = sine(adv(&v->ph[0], f)) * edec(t, ktime(p[2], 0.05f, 3.0f));
        x += lp(&v->a1, &v->a2, n, 900.0f, 0.7f) * edec(t, 0.12f) * kn(p[4]) * 0.8f;
        x += pulse(adv(&v->ph[1], f * 6.0f), 0.5f) * expf(-t / 0.002f) * kn(p[5]) * 0.6f;
        break; }
    case 8: { /* XT CLASSIC: LEV TUN DEC SWD SWT NOD NOL TON */
        static const float base[12] = {80, 80, 80, 80, 80, 100, 140, 190, 80, 80, 80, 80};
        float swt = ktime(p[4], 0.004f, 0.3f);
        float f = base[v->track % 12] * semis(p[1], 0.5f) * powf(2.0f, 1.5f * kn(p[3]) * expf(-t / swt));
        if (tom_on(v)) { x = tom_sample(v) * TOM_OUT; break; }
        x = sine(adv(&v->ph[0], f)) * edec(t, ktime(p[2], 0.04f, 3.0f));
        x += bp(&v->a1, &v->a2, n, 400.0f * powf(12.0f, kn(p[7])), 0.8f) * edec(t, ktime(p[5], 0.01f, 1.0f)) * kn(p[6]);
        break; }
    case 9: case 10: case 17: case 18: { /* CH/OH CLASSIC: LEV TUN DEC COL; CH/OH METAL: LEV TUN DEC */
        int open = v->model == 10 || v->model == 18, metal = v->model >= 17;
        float base = 205.0f * semis(p[1], 0.5f), m;
        if (metal) m = metal6(v, base * 1.3f, 0.5f) * (0.5f + 0.5f * pulse(adv(&v->ph[6], base * 3.1f), 0.5f));
        else m = metal6(v, base, 0.5f);
        m = m * (1.0f - (metal ? 0.15f : kn(p[3]))) + n * (metal ? 0.15f : kn(p[3]));
        m = hp(&v->a1, &v->a2, bp(&v->b1, &v->b2, m, 9000.0f, 0.6f), 6000.0f);
        x = m * 4.0f * edec(t, metal_dec(v, metal ? 0 : open ? 3 : 2,
                                         open ? ktime(p[2], 0.08f, 3.0f) : ktime(p[2], 0.01f, 0.4f)));
        break; }
    case 11: case 19: { /* CY CLASSIC: LEV TUN DEC COL TON; CY METAL: LEV TUN DEC TON TRD */
        float base = 300.0f * semis(p[1], 0.5f), m = metal6(v, base, 0.5f);
        float col = v->model == 11 ? kn(p[3]) : 0.2f, ton = v->model == 11 ? kn(p[4]) : kn(p[3]);
        m = m * (1 - col) + n * col;
        m = bp(&v->b1, &v->b2, m, 3000.0f + 7000.0f * ton, 0.5f) + hp(&v->a1, &v->a2, m, 8000.0f);
        x = m * 1.6f * edec(t, v->model == 19 ? metal_dec(v, 1, ktime(p[2], 0.2f, 6.0f)) : ktime(p[2], 0.2f, 6.0f));
        if (v->model == 19) x += n * edec(t, ktime(p[4], 0.002f, 0.1f)) * 0.6f;
        break; }
    case 12: case 20: { /* CB CLASSIC/METAL: LEV TUN DEC DET PW1 PW2 */
        int fr = v->mf[0] > 0 && v->mf[1] > 0 && v->cv[CVOK] > 0;   /* CB CLASSIC: 524/774 Hz from the frame (808: 540/800) */
        float f1 = fr ? v->mf[0] : 540.0f * semis(p[1], 0.5f), f2 = fr ? v->mf[1] : f1 * (1.48f + 0.3f * (kn(p[3]) - 0.5f));
        float o = pulse(adv(&v->ph[0], f1), 0.1f + 0.8f * kn(p[4])) + pulse(adv(&v->ph[1], f2), 0.1f + 0.8f * kn(p[5]));
        if (v->model == 20) o *= 0.6f + 0.4f * sine(adv(&v->ph[2], f1 * 2.7f));
        x = bp(&v->a1, &v->a2, o, 900.0f, 1.2f) * 0.55f * edec(t, metal_dec(v, v->model == 20 ? 5 : 6, ktime(p[2], 0.03f, 1.5f)));
        break; }
    case 24: { /* HH BASIC: LEV TUN DEC TON TRD RST */
        float base = 250.0f * semis(p[1], 0.5f), m = metal6(v, base, 0.5f) * 0.6f + n * 0.4f;
        m = hp(&v->a1, &v->a2, m, 2500.0f + 9000.0f * kn(p[3]));
        x = m * 2.0f * (edec(t, metal_dec(v, 1, ktime(p[2], 0.01f, 2.0f))) + edec(t, ktime(p[4], 0.001f, 0.05f)) * 0.5f);
        break; }
    case 25: { /* CY RIDE: LEV TUN DEC TYP HIT C1 C2 C3 */
        float base = 400.0f * semis(p[1], 0.5f);
        float c1 = sine(adv(&v->ph[0], base)) + sine(adv(&v->ph[1], base * 1.37f));
        float c2 = pulse(adv(&v->ph[2], base * 2.11f), 0.5f) + pulse(adv(&v->ph[3], base * 2.93f), 0.5f);
        float c3 = hp(&v->a1, &v->a2, n, 7000.0f);
        float m = c1 * kn(p[5]) * 0.4f + c2 * kn(p[6]) * 0.3f + c3 * kn(p[7]);
        m = hp(&v->b1, &v->b2, m, p[3] >= 0.5f ? 3500.0f : 1500.0f);
        x = 0.6f * (m * edec(t, metal_dec(v, 3, ktime(p[2], 0.25f, 6.0f))) + n * edec(t, ktime(p[4], 0.002f, 0.08f)) * 0.7f);
        break; }
    case 33: { /* HH LAB: LEV OSC1 DEC OSC2 OSC3 OSC4 OSC5 OSC6 */
        static const int slot[6] = {1, 3, 4, 5, 6, 7};
        float s = 0; int k;
        for (k = 0; k < 6; k++)
            s += pulse(adv(&v->ph[k], v->mf[k] > 0 && v->cv[CVOK] > 0 ? v->mf[k] : 80.0f * powf(2.0f, p[slot[k]] / 16.0f)), 0.5f);
        x = hp(&v->a1, &v->a2, s / 6.0f, 5000.0f) * 2.2f * edec(t, metal_dec(v, 1, ktime(p[2], 0.01f, 2.0f)));
        break; }
    case 15: { /* NOISE: LEV LPF DEC HPF LPQ ATK SWT SWD */
        if (v->ext_seen) { x = 0; break; }      /* the DSP's (no frame word changes) */
        float lpf = 60.0f * powf(300.0f, kn(p[1])) * powf(2.0f, 8.0f * (kn(p[7]) - 0.5f) * expf(-t / ktime(p[6], 0.005f, 2.0f)));
        float y = hp(&v->a1, &v->a2, n, 20.0f * powf(500.0f, kn(p[3])));
        float atk = ktime(p[5], 0.0005f, 1.0f) * kn(p[5]);
        y = lp(&v->b1, &v->b2, y, lpf, 0.6f + 12.0f * kn(p[4]) * kn(p[4]));
        x = y * (atk > 0.0006f && t < atk ? t / atk : 1.0f) * edec(t, ktime(p[2], 0.01f, 4.0f));
        break; }
    case 16: { /* IMPULSE: LEV ATK DEC - - - - POL */
        if (v->ext_seen) { x = 0; break; }      /* the DSP's (no frame word changes) */
        float atk = ktime(p[1], 0.0002f, 0.05f) * kn(p[1]);
        float e = atk > 0.0003f && t < atk ? t / atk : edec(t - atk, ktime(p[2], 0.0005f, 0.3f));
        x = (p[7] >= 0.5f ? -1.0f : 1.0f) * e * (t < atk + 0.002f ? 1.0f : 0.3f);
        break; }
    case 28: { /* SY DUAL VCO: LEV TUN DEC DET DEC2 BAL BND CFG */
        int fr = xd_on(v, 0) && xd_on(v, 1);
        float f = fr ? xd_hz(v, 0) : 110.0f * semis(p[1], 0.5f) * powf(2.0f, 2.0f * (kn(p[6]) - 0.5f) * expf(-t / 0.08f));
        float f2 = fr ? xd_hz(v, 1) : f * semis(p[3], 0.25f);
        int cfg = (int)(p[7] + 0.5f) % 4;
        float o1 = (cfg & 1) ? pulse(adv(&v->ph[0], f), 0.5f) : saw(adv(&v->ph[0], f));
        float o2 = (cfg & 2) ? pulse(adv(&v->ph[1], f2), 0.5f) : saw(adv(&v->ph[1], f2));
        x = o1 * (1 - kn(p[5])) * (fr ? xd_vca(v, 0) : edec(t, ktime(p[2], 0.02f, 6.0f)))
            + o2 * kn(p[5]) * (fr ? xd_vca(v, 1) : edec(t, ktime(p[4], 0.02f, 6.0f)));
        x *= 0.8f;
        break; }
    case 29: { /* SY CHIP: LEV TUN DEC OF2 OF3 OF4 WAV SPD */
        float step = ktime(p[7], 0.01f, 0.25f), off[4], f, ph;
        int k = (int)(t / step) & 3, wav = (int)(p[6] + 0.5f) % 4;
        off[0] = 0; off[1] = p[3] - 64.0f; off[2] = p[4] - 64.0f; off[3] = p[5] - 64.0f;
        f = xd_on(v, 0) ? xd_hz(v, 0) : 220.0f * semis(p[1], 0.5f) * powf(2.0f, off[k] / 12.0f);   /* the firmware steps the code */
        ph = adv(&v->ph[0], f);
        x = (wav == 0 ? pulse(ph, 0.5f) : wav == 1 ? pulse(ph, 0.25f) : wav == 2 ? tri(ph) : saw(ph)) * 0.6f;
        x *= xd_on(v, 0) ? xd_vca(v, 0) : edec(t, ktime(p[2], 0.02f, 6.0f));
        break; }
    case 32: { /* SY RAW: LEV TUNE DEC2 DET NLEV WAV1 WAV2 BAL */
        int fr = xd_on(v, 0) && xd_on(v, 1);
        float f = fr ? xd_hz(v, 0) : 110.0f * semis(p[1], 0.5f), f2 = fr ? xd_hz(v, 1) : f * semis(p[3], 0.25f), a, b, o1, o2;
        int w1 = (int)(p[5] + 0.5f) % 4, w2 = (int)(p[6] + 0.5f) % 4;
        a = adv(&v->ph[0], f); b = adv(&v->ph[1], f2);
        o1 = w1 == 0 ? sine(a) : w1 == 1 ? tri(a) : w1 == 2 ? saw(a) : pulse(a, 0.5f);
        o2 = w2 == 0 ? sine(b) : w2 == 1 ? tri(b) : w2 == 2 ? saw(b) : pulse(b, 0.5f);
        x = (o1 * (1 - kn(p[7])) * (fr ? xd_vca(v, 0) : 1.0f)
             + o2 * kn(p[7]) * (fr ? xd_vca(v, 1) : edec(t, ktime(p[2], 0.02f, 6.0f)))) * 0.7f;
        if (!v->ext_seen) x += n * kn(p[4]) * 0.5f;
        break; }
    default: x = 0;
    }
    return x;
}

/* one sample of an FPGA envelope segment: seg = {time in samples, up, target}.
   The firmware sends the time as log2(samples) in 4.16 (frame +2/+3: mode
   nibble, bit 20 = down, 20-bit rate) and runs attack -> hold -> decay itself:
   up = a linear ramp over the time, down = exponential with that constant. */
static float segment(float e, const float *seg)
{
    float T = seg[0] > 1.0f ? seg[0] : 1.0f, tgt = seg[2];
    if (seg[1] > 0.5f) { e += 1.0f / T; if (e > tgt) e = tgt > e - 1.0f / T ? tgt : e - 1.0f / T; }
    else e += (tgt - e) / T;
    return e;
}

/* filter (with its ADSR), amp envelope (AHD), overdrive, VCA and pan */
static void voice_sample(voice_t *v, float ext, float *l, float *r, float *dsend, float *rsend)
{
    float *p = v->p, *cv = v->cv, x, t = v->t;
    int ok = cv[CVOK] > 0;
    float fatk = ktime(p[8], 0.0005f, 4.0f) * kn(p[8]), fdec = ktime(p[9], 0.005f, 8.0f);
    float fsus = kn(p[10]), frel = ktime(p[11], 0.005f, 8.0f);
    float ffreq = ok ? cv[3] : p[12], freso = ok ? cv[4] : p[13];
    float fenvd = ((ok ? cv[5] : p[15]) - 64.0f) / 64.0f;
    float aatk = ktime(p[16], 0.0003f, 4.0f) * kn(p[16]);
    float ahold = p[17] >= 127 ? 60.0f : ktime(p[17], 0.001f, 4.0f);
    float adec = p[18] >= 127 ? 60.0f : ktime(p[18], 0.012f, 10.0f);
    float ovr = ok ? cv[2] : p[19];
    float fenv, cut, q, env, gl, gr;
    int mode = ((int)p[14]) % 7;
    /* the level knob, then the sample layer: the firmware's own playback, summed before the filter */
    if (ext != 0) v->ext_seen = 1;
    x = source(v) * kn(p[0]) + ext;
    fenv = fatk > 0.0006f && t < fatk ? t / fatk : fsus + (1 - fsus) * expf(-(t - fatk) * 4.6f / fdec);
    if (t > ahold + aatk + adec) fenv *= expf(-(t - ahold - aatk - adec) * 4.6f / frel);
    /* once the firmware has sent a segment, a zero command holds the level */
    if (ok && cv[CVAT + 3] > 0) v->fseen = 1;
    if (ok && v->fseen) fenv = v->fenv = cv[CVAT + 3] > 0 ? segment(v->fenv, cv + CVAT + 3) : v->fenv;
    cut = 20.0f * powf(1000.0f, kn(ffreq)) * powf(2.0f, fenvd * 6.0f * fenv);
    q = 0.5f + powf(kn(freso), 2) * 18.0f;
    if (!(mode == 0 && ffreq >= 126 && fabsf(fenvd) < 0.05f))
        x = svf(&v->s1, &v->s2, x, cut, q, mode);
    env = aatk > 0.0004f && t < aatk ? t / aatk : 1.0f;
    if (t > ahold + aatk) env *= expf(-(t - ahold - aatk) * 4.6f / adec);
    if (ok && cv[CVAT] > 0) v->aseen = 1;
    if (ok && v->aseen) env = v->aenv = cv[CVAT] > 0 ? segment(v->aenv, cv + CVAT) : v->aenv;
    x *= env;
    if (ovr > 0.5f) { float dr = 1 + 8 * kn(ovr); x = tanhf(x * dr) / tanhf(dr); }
    x *= 0.25f + 0.75f * v->vel / 127.0f;
    if (ok) { gl = cv[0]; gr = cv[1]; }
    else {
        float pan = (p[22] - 64.0f) / 64.0f, vol = kn(p[23]) * 0.5f;
        gl = vol * cosf((pan + 1) * PI / 4); gr = vol * sinf((pan + 1) * PI / 4);
    }
    if (v->fade > 1e-4f) { x = x * (1 - v->fade) + v->last * v->fade; v->fade *= 0.93f; }
    v->last = x;
    *l += x * gl; *r += x * gr;
    RYTM_VOICE_OUT(v, x * gl, x * gr);
    *dsend += x * (gl + gr) * kn(p[20]);
    *rsend += x * (gl + gr) * kn(p[21]);
    v->t += 1.0f / SR;
    if ((v->t > 30.0f && !v->aseen) || (env < 1e-5f && v->t > 0.05f)) v->on = 0;
}

void rytm_init(dsp_t *d)
{
    static const int cl[NCOMB] = {1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617};
    static const int al[NAP] = {556, 441, 341, 225};
    int i, c;
    memset(d, 0, sizeof(*d));
    for (c = 0; c < 2; c++) {
        for (i = 0; i < NCOMB; i++) d->clen[c][i] = (int)((cl[i] + c * 23) * 48000.0f / 44100.0f);
        for (i = 0; i < NAP; i++) d->alen[c][i] = (int)((al[i] + c * 23) * 48000.0f / 44100.0f);
    }
    for (i = 0; i < 8; i++) { d->v[i].rng = 0x1234567u + i * 7919u; d->v[i].idx = i; }
    d->v[0].g = d->g + 0; d->v[2].g = d->g + 4; d->v[7].g = d->g + 8;
    d->v[4].g = d->g + 12; d->v[6].g = d->g + 17;     /* the metal voices' generators */
    d->dtime_s = 6000;
}

void rytm_trigger(dsp_t *d, int vi, const float *p, float vel)
{
    voice_t *v = &d->v[vi];
    int k;
    v->fade = v->on ? 1.0f : 0.0f;
    memcpy(v->p, p, sizeof(v->p));
    v->model = (int)p[24];
    v->track = (int)p[25];
    v->vel = vel;
    v->t = 0;
    memset(v->ph, 0, sizeof(v->ph));
    v->mph = 0;
    v->s1 = v->s2 = v->a1 = v->a2 = v->b1 = v->b2 = v->hx = v->hy = v->dust = 0;
    v->aenv = v->fenv = 0;
    v->aseen = v->fseen = 0;
    v->ext_seen = 0;
    v->vca[0] = v->vca[1] = 1.0f;
    if (v->g && (vi == 0 || vi == 2 || vi == 7)) v->g[2].rel = v->g[3].rel = 0;
    for (k = 0; k < 8; k++) v->rnd[k] = frand(&v->rng);
    v->on = 1;
}

int rytm_size(void) { return (int)sizeof(dsp_t); }

void rytm_set_fx(dsp_t *d, const float *fx) { memcpy(d->fx, fx, sizeof(d->fx)); }

void rytm_set_input(dsp_t *d, int vi, const float *x) { memcpy(d->in[vi], x, sizeof(d->in[vi])); }

void rytm_gen(dsp_t *d, int gi, int w0, int w1, int w2, int w3)
{
    gen_t *g;
    int mode = (w2 >> 12) & 15, rate = ((w2 & 15) << 16) | (w3 & 0xFFFF);
    if (gi < 0 || gi >= NGEN) return;
    g = &d->g[gi];
    g->on = (w0 & 0x4000) != 0 || g->on;
    if (w2 == 0 && w3 == 0) return;                       /* no segment: hold */
    if (mode & 8 && mode & 4) g->lev = g->start = (float)(signed char)((w1 >> 8) & 0xFF) / 127.0f;
    /* target byte, signed /127; 0x80 (-128) = release to zero: exponential with
       tau = T, the reading the amp/filter envelopes use too (their 0x80). Read
       as a discharge toward -1 stopping at 0 it made the BD's decay T ln 2 --
       a 43 Hz BD PLASTIC at DEC 97 lasted ~90 ms (docs/CALIBRAZIONE.md) */
    g->tgt = (w1 & 0xFF) >= 0x80 ? 0.0f : (float)(w1 & 0xFF) / 127.0f;
    g->T = powf(2.0f, rate / 65536.0f);
    g->up = !((w2 >> 4) & 1);
    if ((w1 & 0xFF) >= 0x80 && g->T > 240.0f) g->Tlong = g->T;   /* a release (> 5 ms): the DEC's time */
    if (mode & 8 && (w1 & 0xFF) == 0x80) { g->rel = 1; g->relT = g->T; }   /* a release segment starts */
}

static void gens_step(dsp_t *d)
{
    int k;
    for (k = 0; k < NGEN; k++) {
        gen_t *g = &d->g[k];
        if (!g->on || g->T <= 0) continue;
        if (g->up) { g->lev += 1.0f / g->T; if (g->lev > g->tgt) g->lev = g->tgt; }
        else g->lev += (g->tgt - g->lev) / (g->T > 1 ? g->T : 1);
    }
}

float rytm_gen_level(dsp_t *d, int gi) { return gi >= 0 && gi < NGEN ? d->g[gi].lev : 0; }

/* One frame of the FPGA exchange (words 0..331, 16 bits each), once a tick:
   what emu/rytmsynth.py voice_cvs / gens / metal did in Python, in C (the
   per-tick cost on the emulator thread; docs/CALIBRAZIONE.md). Constants as
   there: mixer 5+v/13+v, OVR 21+v, cutoff 29+v, resonance 37+v, env depth
   148+4v, toms 59..64, XD codes 45+4k/46+4k, envelopes 84+4v / 148+4v,
   generators 180+4g, metal 308..331. */
#define GAIN_UNIT 42000.0f
#define OVR_0 12516.0f
#define OVR_127 52652.0f
#define FRQ_0 (-19654.0f)
#define FRQ_127 19890.0f
#define RES_0 6193.0f
#define RES_127 26326.0f
#define ENV_0 (-20022.0f)
#define ENV_127 19878.0f
#define TOM_F0 (48000.0f / 2627.0f)
#define METAL_HZ (205.3f / 8876.0f)
static float knob(float x, float a, float b) { return clampf((x - a) * 127.0f / (b - a), 0, 127); }
static void seg_cv(float *c, int tgt_word, int hi, int lo)
{
    int rate = ((hi & 0xF) << 16) | lo, tg = tgt_word & 0xFF;
    c[0] = hi ? powf(2.0f, rate / 65536.0f) : 0.0f;
    c[1] = ((hi >> 4) & 1) == 0;
    c[2] = tg < 0x80 ? tg / 127.0f : 0.0f;
}
void rytm_frame(dsp_t *d, const uint16_t *w, int n)
{
    int v, k, g;
    if (n < 332) return;
    for (v = 0; v < 8; v++) {
        float *c = d->v[v].cv;
        c[0] = w[5 + v] / GAIN_UNIT;
        c[1] = w[13 + v] / GAIN_UNIT;
        c[2] = knob(w[21 + v], OVR_0, OVR_127);
        c[3] = knob((int16_t)w[29 + v], FRQ_0, FRQ_127);
        c[4] = knob(w[37 + v], RES_0, RES_127);
        c[5] = knob((int16_t)w[148 + 4 * v], ENV_0, ENV_127);
        c[CVOK] = 1.0f;
        seg_cv(c + CVAT, w[85 + 4 * v], w[86 + 4 * v], w[87 + 4 * v]);
        seg_cv(c + CVAT + 3, w[149 + 4 * v], w[150 + 4 * v], w[151 + 4 * v]);
    }
    for (k = 0; k < 3; k++) {
        voice_t *t = &d->v[1 + 2 * k], *x = &d->v[k == 0 ? 0 : k == 1 ? 2 : 7];
        int p = w[59 + 2 * k];
        t->cv[CVTOM] = TOM_F0 * sqrtf((p > 1 ? p : 1) / 256.0f);
        t->cv[16] = (int16_t)w[60 + 2 * k];
        x->cv[14] = (int16_t)w[45 + 4 * k];
        x->cv[15] = (int16_t)w[46 + 4 * k];
    }
    for (g = 0; g < NGEN; g++) {
        const uint16_t *b = w + 180 + 4 * g;
        uint16_t *q = d->gprev + 4 * g;
        if (!d->gprev_ok || b[0] != q[0] || b[1] != q[1] || b[2] != q[2] || b[3] != q[3] || (b[2] & 0x8000))
            rytm_gen(d, g, b[0], b[1], b[2], b[3]);
        q[0] = b[0]; q[1] = b[1]; q[2] = b[2]; q[3] = b[3];
    }
    d->gprev_ok = 1;
    for (k = 0; k < 6; k++) {
        d->v[4].mf[k] = (((w[308 + 2 * k] & 1) << 16) | w[309 + 2 * k]) * METAL_HZ;
        d->v[6].mf[k] = (((w[320 + 2 * k] & 1) << 16) | w[321 + 2 * k]) * METAL_HZ;
    }
}

void rytm_set_strike(dsp_t *d, int vi, const float *x) { memcpy(d->strike[vi], x, sizeof(d->strike[vi])); }

void rytm_set_metal(dsp_t *d, int vi, const float *f) { memcpy(d->v[vi].mf, f, sizeof(d->v[vi].mf)); }

void rytm_set_cv(dsp_t *d, int vi, const float *cv) { memcpy(d->v[vi].cv, cv, sizeof(d->v[vi].cv)); }

static float dist(float x, float amt, float sym)
{
    float g = 1.0f + amt * 20.0f, b = (sym - 0.5f) * 0.8f;
    return (tanhf((x + b) * g) - tanhf(b * g)) / tanhf(g);
}

void rytm_render(dsp_t *d, float *out)
{
    int i, k, c;
    float *fx = d->fx;
    float bpm = fx[29] > 20 ? fx[29] : 120;
    float dt_target = clampf((fx[0] + 1) / 128.0f * (60.0f / bpm) * 4.0f * SR, 32, DMAX - 2);
    float fb = clampf(kn(fx[3]) * 1.05f, 0, 1.02f), dvol = kn(fx[7]) * 1.2f, dov = kn(fx[8]);
    float wid = kn(fx[2]), drev = kn(fx[6]);
    float dlpf = 1.0f - expf(-2 * PI * (150.0f * powf(130.0f, kn(fx[5]))) / SR);
    float dhpf = 1.0f - expf(-2 * PI * (20.0f * powf(100.0f, kn(fx[4]))) / SR);
    int pre = (int)(kn(fx[10]) * 9000);
    float rfb = 0.70f + 0.28f * kn(fx[11]);
    float rdamp = 0.05f + 0.5f * (1.0f - kn(fx[15]));
    float rhpf = 1.0f - expf(-2 * PI * (20.0f * powf(50.0f, kn(fx[14]))) / SR);
    float shf = 1.0f - expf(-2 * PI * (200.0f * powf(40.0f, kn(fx[12]))) / SR);
    float shg = powf(10.0f, (fx[13] - 64.0f) / 64.0f * 0.6f);
    float rvol = kn(fx[16]) * 0.6f;
    float amt = kn(fx[18]), sym = kn(fx[19]);
    float thr = powf(10.0f, -30.0f * (1.0f - kn(fx[21])) / 20.0f);
    float catk = expf(-1.0f / (SR * ktime(fx[22], 0.0001f, 0.03f)));
    float crel = expf(-1.0f / (SR * ktime(fx[23], 0.01f, 1.0f)));
    float ratio = 1.0f + (fx[24] >= 3 ? 19.0f : fx[24] * 3.0f);
    float mup = powf(10.0f, kn(fx[26]) * 24.0f / 20.0f), cmix = kn(fx[27]), cvol = kn(fx[28]);
    int comp_on = fx[21] < 127 || fx[24] > 0;
    for (i = 0; i < BLOCK; i++) {
        float l = 0, r = 0, ds = 0, rs = 0, yl, yr, rin, rl = 0, rr = 0;
        gens_step(d);
        RYTM_VOICE_BEGIN(d, i);
        for (k = 0; k < 8; k++) if (d->v[k].on) { d->v[k].strike = d->strike[k][i]; voice_sample(&d->v[k], d->in[k][i], &l, &r, &ds, &rs); }
        RYTM_VOICE_END(d, i);
        /* delay: smoothed time, HP/LP in the loop, overdrive, pingpong, width */
        {
            float pos, frac, fl, fr;
            int ip, ip2;
            d->dtime_s += (dt_target - d->dtime_s) * 0.0005f;
            pos = d->dpos - d->dtime_s; if (pos < 0) pos += DMAX;
            ip = (int)pos; frac = pos - ip; ip2 = (ip + 1) % DMAX;
            yl = d->dl[ip] * (1 - frac) + d->dl[ip2] * frac;
            yr = d->dr[ip] * (1 - frac) + d->dr[ip2] * frac;
            d->dlp_l += (yl - d->dlp_l) * dlpf; d->dlp_r += (yr - d->dlp_r) * dlpf;
            d->dhp_l += (d->dlp_l - d->dhp_l) * dhpf; d->dhp_r += (d->dlp_r - d->dhp_r) * dhpf;
            fl = d->dlp_l - d->dhp_l; fr = d->dlp_r - d->dhp_r;
            if (dov > 0.01f) { float g = 1 + dov * 6; fl = tanhf(fl * g) / tanhf(g); fr = tanhf(fr * g) / tanhf(g); }
            if (fx[1] >= 64) { float tmp = fl; fl = fr; fr = tmp; }
            d->dl[d->dpos] = tanhf(ds + fl * fb);
            d->dr[d->dpos] = tanhf((fx[1] >= 64 ? 0.0f : ds) + fr * fb);
            d->dpos = (d->dpos + 1) % DMAX;
            { float m = 0.5f * (yl + yr), s = 0.5f * (yl - yr) * wid; yl = m + s; yr = m - s; }
            rs += 0.5f * (yl + yr) * drev;
        }
        /* reverb: predelay, HP, 8 damped combs + 4 allpasses per side, shelf */
        d->pre[d->prepos] = rs;
        rin = d->pre[(d->prepos - pre + 9600) % 9600];
        d->prepos = (d->prepos + 1) % 9600;
        d->rhp += (rin - d->rhp) * rhpf; rin -= d->rhp;
        for (c = 0; c < 2; c++) {
            float acc = 0, y;
            for (k = 0; k < NCOMB; k++) {
                float *b = d->comb[c][k]; int q = d->cpos[c][k];
                y = b[q];
                d->cdamp[c][k] = y * (1 - rdamp) + d->cdamp[c][k] * rdamp;
                b[q] = rin * 0.015f + d->cdamp[c][k] * rfb;
                d->cpos[c][k] = (q + 1) % d->clen[c][k];
                acc += y;
            }
            for (k = 0; k < NAP; k++) {
                float *b = d->ap[c][k]; int q = d->apos[c][k];
                float bo = b[q];
                b[q] = acc + bo * 0.5f;
                acc = bo - acc;
                d->apos[c][k] = (q + 1) % d->alen[c][k];
            }
            if (c == 0) { d->rsh_l += (acc - d->rsh_l) * shf; rl = acc + (shg - 1) * (acc - d->rsh_l); }
            else { d->rsh_r += (acc - d->rsh_r) * shf; rr = acc + (shg - 1) * (acc - d->rsh_r); }
        }
        rl *= rvol; rr *= rvol;
        /* main path: distortion, compressor; delay/reverb through them when routed there */
        if (fx[9] >= 0.5f) { l += yl * dvol; r += yr * dvol; }
        if (fx[17] >= 0.5f) { l += rl; r += rr; }
        if (amt > 0.004f) { l = dist(l, amt, sym); r = dist(r, amt, sym); }
        if (comp_on) {
            float sc = 0.5f * (fabsf(l) + fabsf(r)), g;
            d->sc_lp += (sc - d->sc_lp) * 0.05f;
            if (fx[25] > 0.5f) sc = fabsf(sc - d->sc_lp);
            d->cenv = sc > d->cenv ? catk * d->cenv + (1 - catk) * sc : crel * d->cenv + (1 - crel) * sc;
            g = d->cenv > thr ? powf(d->cenv / thr, 1.0f / ratio - 1.0f) : 1.0f;
            l = (l * (1 - cmix) + l * g * mup * cmix) * cvol;
            r = (r * (1 - cmix) + r * g * mup * cmix) * cvol;
        }
        if (fx[9] < 0.5f) { l += yl * dvol; r += yr * dvol; }
        if (fx[17] < 0.5f) { l += rl; r += rr; }
        out[2 * i] = tanhf(l * 1.2f) / tanhf(1.2f);
        out[2 * i + 1] = tanhf(r * 1.2f) / tanhf(1.2f);
    }
    memset(d->in, 0, sizeof(d->in));     /* consumed: a block not refreshed is silence */
    memset(d->strike, 0, sizeof(d->strike));
}
