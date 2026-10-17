/* dncore: riga di comando di prova del motore del Digitone II (dalla dncore della
 * Syntakt). Lo stato del DSP (DNDS, tools/dsp_export.py) in DNCORE_DSP_STATE o --dsp.
 *
 *   dncore play --syx SYX --state F.dtst --instrs N [--scenario S] [--trace OUT]
 *               [--ram-every K] [--raw OUT.f32] [--save OUT.dtst] [--dsp F.dnds]
 *                                    il worker della GUI, con la traccia di
 *                                    tools/dncore_guiref.py
 *   dncore rt-test --syx SYX --state F.dtst [--seconds S] [--rate R] [--frames N]
 *                  [--realtime] [--dump OUT.f32] [--dsp F.dnds]
 *                                    il motore in tempo reale (rt.c del Digitakt)
 *                                    con blocchi fissi e casuali, e a 48 kHz contro
 *                                    il motore diretto: uscita stereo identica
 */
#include "dncore.h"
#include "sha256.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

static uint8_t *read_file(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *p = NULL;
    long n;
    if (!f)
        return NULL;
    if (fseek(f, 0, SEEK_END) == 0 && (n = ftell(f)) >= 0 && fseek(f, 0, SEEK_SET) == 0) {
        p = (uint8_t *)malloc(n ? (size_t)n : 1);
        if (p && fread(p, 1, (size_t)n, f) != (size_t)n) {
            free(p);
            p = NULL;
        }
        *len = (size_t)n;
    }
    fclose(f);
    return p;
}

static int load_fw(const char *path, dt_firmware *fw)
{
    char err[256] = "";
    size_t len = 0;
    uint8_t *syx = read_file(path, &len);
    int r;
    if (!syx) {
        fprintf(stderr, "dncore: cannot read %s: %s\n", path, strerror(errno));
        return -1;
    }
    r = dt_fw_extract(syx, len, fw, err, sizeof err);
    free(syx);
    if (r) {
        fprintf(stderr, "dncore: %s: %s\n", path, err);
        return -1;
    }
    return 0;
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int hexval(int c)
{
    return c >= '0' && c <= '9' ? c - '0' : (c | 32) >= 'a' && (c | 32) <= 'f' ? (c | 32) - 'a' + 10 : -1;
}

/* la riga `ram` di tools/dncore_guiref.py: registri e impronta della memoria */
static void ram_line(FILE *out, dt_engine *e, const char *tag)
{
    static const char *names[18] = {"d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
                                    "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "pc", "sr"};
    uint32_t regs[18], nreg = 0;
    uint8_t dig[32];
    char hx[65];
    int i;
    dt_machine_regs(dt_engine_machine(e), regs);
    dn_engine_digest(e, dig, &nreg);
    dt_sha256_hex(dig, hx);
    hx[16] = 0;
    fprintf(out, "ram %s", tag);
    for (i = 0; i < 18; i++)
        fprintf(out, " %s=%08x", names[i], regs[i]);
    fprintf(out, " %s regions=%u\n", hx, nreg);
}

static int dsp_arg(const char *path)
{
    size_t n = 0;
    uint8_t *p = read_file(path, &n);
    if (!p) {
        fprintf(stderr, "dncore: cannot read %s\n", path);
        return -1;
    }
    dn_set_dsp_state(p, n);
    free(p);
    return 0;
}

typedef struct {
    float *buf;                  /* stereo interleaved */
    size_t n, cap;               /* frame */
} collect_t;

static void collect_sink(void *ctx, const float *mix, const float *tracks, size_t n)
{
    collect_t *c = (collect_t *)ctx;
    size_t i;
    if (c->n + n > c->cap) {
        c->cap = (c->n + n) * 2;
        c->buf = (float *)realloc(c->buf, c->cap * 2 * sizeof(float));
    }
    for (i = 0; i < n; i++) {
        c->buf[2 * (c->n + i)] = mix[i];
        c->buf[2 * (c->n + i) + 1] = tracks[i];
    }
    c->n += n;
}

typedef struct {
    int64_t at;
    int kind;                    /* 0 midi, 1 press, 2 release, 3 encoder, 4 rpc */
    int a, b;
    uint8_t data[256];
    size_t n;
} ev_t;

static int cmd_play(int argc, char **argv)
{
    const char *syx = NULL, *state = NULL, *scen = NULL, *trace = NULL, *save = NULL, *raw = NULL;
    const char *screen = NULL;
    double limit_d = 0;
    collect_t col = {NULL, 0, 0};
    int64_t limit, clk_from = 0, clk_every = 0, clk_until = 0, clk_next = 0;
    int ram_every = 25, i, have_clock = 0, last = 0;
    ev_t *evs = NULL;
    size_t nev = 0, cap = 0, iev = 0;
    dt_firmware fw;
    dt_engine *e;
    char err[256] = "", line[2048];
    FILE *out = stdout;
    int64_t chunk = 0;
    unsigned rpc_seq = 0;
    int quiet = 0;                  /* niente traccia: il DSP resta in parallelo fra i giri */
    double t0, t1;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
        else if (!strcmp(argv[i], "--instrs") && i + 1 < argc) limit_d = strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--scenario") && i + 1 < argc) scen = argv[++i];
        else if (!strcmp(argv[i], "--trace") && i + 1 < argc) trace = argv[++i];
        else if (!strcmp(argv[i], "--ram-every") && i + 1 < argc) ram_every = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--save") && i + 1 < argc) save = argv[++i];
        else if (!strcmp(argv[i], "--raw") && i + 1 < argc) raw = argv[++i];
        else if (!strcmp(argv[i], "--screen") && i + 1 < argc) screen = argv[++i];
        else if (!strcmp(argv[i], "--quiet")) quiet = 1;
        else if (!strcmp(argv[i], "--dsp") && i + 1 < argc) {
            if (dsp_arg(argv[++i]))
                return 1;
        } else {
            fprintf(stderr, "dncore play: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state || limit_d <= 0) {
        fprintf(stderr, "uso: dncore play --syx SYX --state F.dtst --instrs N [--scenario S] [--trace OUT] [--ram-every K]\n");
        return 2;
    }
    limit = (int64_t)limit_d;
    if (scen) {
        FILE *f = fopen(scen, "r");
        if (!f) {
            fprintf(stderr, "dncore: cannot read %s\n", scen);
            return 1;
        }
        while (fgets(line, sizeof line, f)) {
            char *h = strchr(line, '#'), w[5][300];
            int nw;
            if (h)
                *h = 0;
            nw = sscanf(line, "%299s %299s %299s %299s %299s", w[0], w[1], w[2], w[3], w[4]);
            if (nw <= 0)
                continue;
            if (!strcmp(w[0], "midi_clock") && nw >= 4) {
                have_clock = 1;
                clk_from = (int64_t)strtod(w[1], NULL);
                clk_every = (int64_t)strtod(w[2], NULL);
                clk_until = (int64_t)strtod(w[3], NULL);
                clk_next = clk_from;
            } else if (!strcmp(w[0], "at") && nw >= 4) {
                ev_t ev;
                memset(&ev, 0, sizeof ev);
                ev.at = (int64_t)strtod(w[1], NULL);
                if (!strcmp(w[2], "midi")) {
                    size_t k;
                    ev.kind = 0;
                    for (k = 0; w[3][2 * k] && w[3][2 * k + 1] && k < sizeof ev.data; k++)
                        ev.data[k] = (uint8_t)(hexval(w[3][2 * k]) << 4 | hexval(w[3][2 * k + 1]));
                    ev.n = k;
                } else if (!strcmp(w[2], "press")) {
                    ev.kind = 1; ev.a = atoi(w[3]);
                } else if (!strcmp(w[2], "release")) {
                    ev.kind = 2; ev.a = atoi(w[3]);
                } else if (!strcmp(w[2], "encoder") && nw >= 5) {
                    ev.kind = 3; ev.a = atoi(w[3]); ev.b = atoi(w[4]);
                } else if (!strcmp(w[2], "rpc")) {     /* RpcLink.send(id, corpo) */
                    size_t k;
                    ev.kind = 4;
                    ev.a = (int)strtol(w[3], NULL, 0);
                    for (k = 0; nw >= 5 && w[4][2 * k] && w[4][2 * k + 1] && k < sizeof ev.data; k++)
                        ev.data[k] = (uint8_t)(hexval(w[4][2 * k]) << 4 | hexval(w[4][2 * k + 1]));
                    ev.n = k;
                } else
                    continue;
                if (nev == cap) {
                    cap = cap ? 2 * cap : 64;
                    evs = (ev_t *)realloc(evs, cap * sizeof *evs);
                }
                evs[nev++] = ev;
            }
        }
        fclose(f);
        {
            size_t a, b;                          /* sorted((at, indice)): stabile */
            for (a = 1; a < nev; a++)
                for (b = a; b > 0 && evs[b - 1].at > evs[b].at; b--) {
                    ev_t t = evs[b]; evs[b] = evs[b - 1]; evs[b - 1] = t;
                }
        }
    }
    if (trace && !(out = fopen(trace, "w"))) {
        fprintf(stderr, "dncore: cannot write %s\n", trace);
        return 1;
    }
    if (load_fw(syx, &fw))
        return 1;
    t0 = now_s();
    e = dt_engine_new(&fw, state, NULL, err, sizeof err);
    if (!e) {
        fprintf(stderr, "dncore: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    t1 = now_s();
    fprintf(stderr, "dncore: motore pronto in %.2f s\n", t1 - t0);
    if (raw)
        dt_engine_set_audio_sink(e, collect_sink, &col, 1);
    t0 = now_s();
    for (;;) {
        int64_t n = dt_engine_instrs(e);
        if (chunk > 0 && !quiet) {
            dt_engine_trace(e, "chunk", chunk - 1, line, sizeof line);
            fprintf(out, "%s\n", line);
            if (ram_every && (chunk - 1) % ram_every == 0) {
                char tag[32];
                snprintf(tag, sizeof tag, "%lld", (long long)(chunk - 1));
                ram_line(out, e, tag);
            }
        }
        if (n >= limit)
            last = 1;
        while (iev < nev && evs[iev].at <= n) {
            ev_t *v = &evs[iev++];
            if (v->kind == 0)
                dt_engine_midi_in(e, v->data, v->n);
            else if (v->kind == 1)
                dt_engine_panel(e, DT_PANEL_PRESS, v->a, 0);
            else if (v->kind == 2)
                dt_engine_panel(e, DT_PANEL_RELEASE, v->a, 0);
            else if (v->kind == 3)
                dt_engine_panel(e, DT_PANEL_ENCODER, v->a, v->b);
            else {                                /* transfer.request(seq, id, corpo) */
                uint8_t req[5 + sizeof v->data];
                rpc_seq++;
                req[0] = (uint8_t)(rpc_seq >> 8); req[1] = (uint8_t)rpc_seq; req[2] = 0; req[3] = 0;
                req[4] = (uint8_t)v->a;
                memcpy(req + 5, v->data, v->n);
                dt_engine_rpc(e, req, 5 + v->n);
            }
        }
        if (have_clock) {
            static const uint8_t f8 = 0xF8;
            while (clk_next <= n && clk_next <= clk_until) {
                dt_engine_midi_in(e, &f8, 1);
                clk_next += clk_every;
            }
        }
        chunk++;
        if (dt_engine_chunk(e) || last)
            break;
    }
    t1 = now_s();
    dt_engine_trace(e, "end", chunk - 1, line, sizeof line);
    fprintf(out, "%s\n", line);
    ram_line(out, e, "end");
    fprintf(stderr, "dncore: %lld istruzioni in %.2f s (%.2fx il tempo reale), %s\n",
            (long long)dt_engine_instrs(e), t1 - t0, dt_engine_instrs(e) / 150e6 / (t1 - t0),
            dt_engine_status(e));
    if (out != stdout)
        fclose(out);
    if (screen) {                         /* lo schermo del firmware (128x64) in ASCII, e i LED accesi */
        static uint8_t fb[128 * 64];
        uint8_t leds[DN_LEDS];
        FILE *sf = fopen(screen, "w");
        int x, y;
        dt_engine_screen(e, fb);
        dn_engine_leds(e, leds);
        if (sf) {
            for (y = 0; y < 64; y += 2) {
                for (x = 0; x < 128; x++) {
                    int up = fb[y * 128 + x] != 0, dn = fb[(y + 1) * 128 + x] != 0;
                    fputs(up && dn ? "\xe2\x96\x88" : up ? "\xe2\x96\x80" : dn ? "\xe2\x96\x84" : " ", sf);
                }
                fputc('\n', sf);
            }
            fprintf(sf, "leds:");
            for (x = 0; x < DN_LEDS; x++)
                fprintf(sf, " %02x", leds[x]);
            fputc('\n', sf);
            fclose(sf);
        }
    }
    if (raw) {
        FILE *rf = fopen(raw, "wb");
        if (rf) {
            fwrite(col.buf, 2 * sizeof(float), col.n, rf);
            fclose(rf);
        }
        free(col.buf);
    }
    if (save) {
        int r = dt_engine_save(e, save, 1);
        fprintf(stderr, "dncore: stato salvato in %s (%d)\n", save, r);
    }
    dt_engine_free(e);
    dt_fw_free(&fw);
    free(evs);
    return 0;
}

/* ------------------------------------------------------------ rt-test */
typedef struct {
    int64_t h;
    uint8_t d[3];
    int n;
} rtev;

/* gli eventi del test: START, clock a 120 BPM (ogni 1000 campioni a 48 kHz), note */
static size_t rt_events(rtev *ev, size_t cap, int64_t H)
{
    size_t n = 0;
    int64_t h;
    ev[n].h = 4800; ev[n].d[0] = 0xFA; ev[n].n = 1; n++;
    for (h = 4800; h < H - 4800 && n < cap - 40; h += 1000) {
        ev[n].h = h; ev[n].d[0] = 0xF8; ev[n].n = 1; n++;
    }
    for (h = 9600; h < H && n < cap; h += 7919) {
        ev[n].h = h; ev[n].d[0] = (uint8_t)(0x90 | (h / 7919) % 16);
        ev[n].d[1] = (uint8_t)(48 + h % 24); ev[n].d[2] = 100; ev[n].n = 3; n++;
    }
    {
        size_t a, b;
        for (a = 1; a < n; a++)
            for (b = a; b > 0 && ev[b - 1].h > ev[b].h; b--) {
                rtev t = ev[b]; ev[b] = ev[b - 1]; ev[b - 1] = t;
            }
    }
    return n;
}

static int g_rt_realtime = 0;
static uint32_t g_rt_frames = 0;

static void sleep_s(double s)
{
    struct timespec ts;
    if (s <= 0)
        return;
    ts.tv_sec = (time_t)s;
    ts.tv_nsec = (long)((s - (double)ts.tv_sec) * 1e9);
    nanosleep(&ts, NULL);
}

/* -> l'uscita stereo interleaved (H frame) */
static float *rt_run(const uint8_t *syx, size_t len, const char *state, int64_t H, int random_blocks,
                     double rate, int64_t *E, int64_t *X, int64_t *underruns)
{
    double t0;
    char err[256] = "";
    dt_rt *rt = dt_rt_new(syx, len, state, rate, g_rt_frames ? g_rt_frames : g_rt_realtime ? 256 : 2048,
                          err, sizeof err);
    float *out, *l, *r, *o[2];
    rtev ev[4096];
    size_t nev = rt_events(ev, 4096, (int64_t)(H * 48000.0 / rate)), iev = 0;
    int64_t c = 0, i;
    unsigned seed = 12345;
    if (!rt) {
        fprintf(stderr, "dncore: %s\n", err);
        return NULL;
    }
    dt_rt_offsets(rt, E, X);
    t0 = now_s();                         /* l'host comincia a rendere quando il plugin c'e' */
    l = (float *)calloc((size_t)H, sizeof(float));
    r = (float *)calloc((size_t)H, sizeof(float));
    while (c < H) {
        uint32_t n = g_rt_frames ? g_rt_frames : 256;
        if (random_blocks) {
            seed = seed * 1103515245u + 12345u;
            n = 16 + (seed >> 8) % (g_rt_realtime ? 241 : 2033);
        }
        if (c + n > H)
            n = (uint32_t)(H - c);
        while (iev < nev && (int64_t)(ev[iev].h * rate / 48000.0) < c + n) {
            int64_t hh = (int64_t)(ev[iev].h * rate / 48000.0);
            dt_rt_midi(rt, (uint32_t)(hh > c ? hh - c : 0), ev[iev].d, (size_t)ev[iev].n);
            iev++;
        }
        o[0] = l + c;
        o[1] = r + c;
        if (g_rt_realtime) {
            sleep_s(t0 + (double)c / rate - now_s());
            dt_rt_render(rt, n, o, 2, 0);
        } else
            dt_rt_render(rt, n, o, 2, 1);
        c += n;
    }
    *underruns = dt_rt_underruns(rt);
    fprintf(stderr, "rt: ritardo massimo %lld campioni, %s\n", (long long)dt_rt_max_lag(rt), dt_rt_status(rt));
    dt_rt_free(rt);
    out = (float *)malloc((size_t)H * 2 * sizeof(float));
    for (i = 0; i < H; i++) {
        out[2 * i] = l[i];
        out[2 * i + 1] = r[i];
    }
    free(l);
    free(r);
    return out;
}

static int cmd_rt_test(int argc, char **argv)
{
    const char *syx_path = NULL, *state = NULL, *dump = NULL;
    double secs = 6, rate = 48000;
    int64_t H, E, X, E2, X2, u1, u2, i, bad = 0, firstbad = -1;
    size_t len;
    uint8_t *syx;
    float *a, *b;
    collect_t col = {NULL, 0, 0};
    dt_firmware fw;
    dt_engine *e;
    char err[256] = "";
    rtev ev[4096];
    size_t nev, iev = 0;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx_path = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--rate") && i + 1 < argc) rate = atof(argv[++i]);
        else if (!strcmp(argv[i], "--realtime")) g_rt_realtime = 1;
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) g_rt_frames = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dump") && i + 1 < argc) dump = argv[++i];
        else if (!strcmp(argv[i], "--dsp") && i + 1 < argc) {
            if (dsp_arg(argv[++i]))
                return 1;
        }
    }
    if (!syx_path || !state) {
        fprintf(stderr, "uso: dncore rt-test --syx SYX --state F.dtst [--seconds S] [--rate R]\n");
        return 2;
    }
    syx = read_file(syx_path, &len);
    if (!syx)
        return 1;
    H = (int64_t)(secs * rate);
    a = rt_run(syx, len, state, H, 0, rate, &E, &X, &u1);
    b = rt_run(syx, len, state, H, 1, rate, &E2, &X2, &u2);
    if (!a || !b)
        return 1;
    for (i = 0; i < 2 * H; i++)
        if (memcmp(&a[i], &b[i], 4)) {
            if (firstbad < 0)
                firstbad = i / 2;
            bad++;
        }
    if (dump) {
        FILE *f = fopen(dump, "wb");
        if (f) {
            fwrite(a, 2 * sizeof(float), (size_t)H, f);
            fclose(f);
        }
    }
    printf("rt blocchi fissi contro casuali: %lld frame, %lld valori diversi (primo frame %lld), buchi %lld/%lld\n",
           (long long)H, (long long)bad, (long long)firstbad, (long long)u1, (long long)u2);
    if (rate == 48000.0) {
        if (load_fw(syx_path, &fw))
            return 1;
        e = dt_engine_new(&fw, state, NULL, err, sizeof err);
        if (!e) {
            fprintf(stderr, "dncore: %s\n", err);
            return 1;
        }
        dt_engine_set_audio_sink(e, collect_sink, &col, 1);
        nev = rt_events(ev, 4096, H);
        while ((int64_t)col.n < H + E) {
            int64_t t48 = dt_engine_instrs(e) / 3125;
            if (getenv("DN_LAG") && t48 - (int64_t)col.n > 1000)
                fprintf(stderr, "lag %lld a t48=%lld\n", (long long)(t48 - (int64_t)col.n), (long long)t48);
            while (iev < nev && ev[iev].h + E <= t48) {
                dt_engine_midi_in(e, ev[iev].d, (size_t)ev[iev].n);
                iev++;
            }
            if (dt_engine_chunk(e))
                break;
        }
        bad = 0;
        firstbad = -1;
        for (i = 0; i < H; i++) {
            int64_t k = i + E - X, ch;
            for (ch = 0; ch < 2; ch++) {
                float want = k >= 0 && k < (int64_t)col.n ? col.buf[2 * k + ch] : 0.0f;
                if (memcmp(&a[2 * i + ch], &want, 4)) {
                    if (firstbad < 0)
                        firstbad = i;
                    bad++;
                }
            }
        }
        printf("rt contro motore diretto: %lld valori diversi (primo frame %lld); E=%lld X=%lld\n",
               (long long)bad, (long long)firstbad, (long long)E, (long long)X);
        {
            double peak = 0;
            for (i = 0; i < 2 * H; i++)
                if (fabs(a[i]) > peak)
                    peak = fabs(a[i]);
            printf("picco %.4f\n", peak);
        }
        dt_engine_free(e);
        dt_fw_free(&fw);
    }
    free(a);
    free(b);
    free(syx);
    free(col.buf);
    return bad ? 1 : 0;
}

/* ------------------------------------------------------------ multi */
/* K motori nello stesso processo, ciascuno nel suo thread (e col suo thread del DSP),
   con lo stesso stato e lo stesso scenario semplice (START e clock a 120 BPM, note):
   le righe finali della traccia devono essere uguali fra loro e a un motore solo */
typedef struct {
    const dt_firmware *fw;
    const char *state;
    int64_t limit;
    char line[2048];
    double secs;
    int rc;
} multi_arg;

static void *multi_main(void *p)
{
    multi_arg *a = (multi_arg *)p;
    char err[256] = "";
    dt_engine *e = dt_engine_new(a->fw, a->state, NULL, err, sizeof err);
    int64_t clk = 0, chunk = 0;
    double t0;
    if (!e) {
        snprintf(a->line, sizeof a->line, "errore: %s", err);
        a->rc = 1;
        return NULL;
    }
    t0 = now_s();
    {
        static const uint8_t fa = 0xFA;
        dt_engine_midi_in(e, &fa, 1);
    }
    for (;;) {
        int64_t n = dt_engine_instrs(e);
        static const uint8_t f8 = 0xF8;
        while (clk <= n) {
            dt_engine_midi_in(e, &f8, 1);
            clk += 3125000;
        }
        if (chunk % 400 == 0) {
            uint8_t note[3] = {0x90, (uint8_t)(48 + chunk / 400 % 24), 100};
            dt_engine_midi_in(e, note, 3);
        }
        chunk++;
        if (dt_engine_chunk(e) || n >= a->limit)
            break;
    }
    a->secs = now_s() - t0;
    dt_engine_trace(e, "end", chunk, a->line, sizeof a->line);
    dt_engine_free(e);
    return NULL;
}

static int cmd_multi(int argc, char **argv)
{
    const char *syx_path = NULL, *state = NULL;
    double limit_d = 300e6;
    int k = 2, i, bad = 0;
    dt_firmware fw;
    multi_arg one, *as;
    pthread_t *th;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx_path = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
        else if (!strcmp(argv[i], "--instrs") && i + 1 < argc) limit_d = strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--instances") && i + 1 < argc) k = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dsp") && i + 1 < argc) {
            if (dsp_arg(argv[++i]))
                return 1;
        }
    }
    if (!syx_path || !state || load_fw(syx_path, &fw))
        return 2;
    memset(&one, 0, sizeof one);
    one.fw = &fw;
    one.state = state;
    one.limit = (int64_t)limit_d;
    multi_main(&one);
    printf("uno solo: %.2f s\n%s\n", one.secs, one.line);
    as = (multi_arg *)calloc((size_t)k, sizeof *as);
    th = (pthread_t *)calloc((size_t)k, sizeof *th);
    for (i = 0; i < k; i++) {
        as[i] = one;
        as[i].line[0] = 0;
        pthread_create(&th[i], NULL, multi_main, &as[i]);
    }
    for (i = 0; i < k; i++) {
        pthread_join(th[i], NULL);
        printf("istanza %d: %.2f s, %s\n", i, as[i].secs, strcmp(as[i].line, one.line) ? "DIVERSA" : "uguale");
        bad += strcmp(as[i].line, one.line) != 0;
    }
    free(as);
    free(th);
    dt_fw_free(&fw);
    return bad ? 1 : 0;
}

/* ------------------------------------------------------------ data-test */
/* il +Drive con l'API Data dal motore in tempo reale (come scratch/dn_data_client_probe.py,
   solo letture): ls di /, /soundbanks, /soundbanks/A, /projects e lettura di A/1 */
static volatile int g_data_run = 1;

static void *data_render(void *arg)
{
    dt_rt *rt = (dt_rt *)arg;
    float l[256], r[256], *o[2] = {l, r};
    while (g_data_run) {
        dt_rt_render(rt, 256, o, 2, 0);
        sleep_s(256.0 / 48000.0);
    }
    return NULL;
}

static int cmd_data_test(int argc, char **argv)
{
    const char *syx_path = NULL, *state = NULL;
    static const char *paths[4] = {"/", "/soundbanks", "/soundbanks/A", "/projects"};
    size_t len, k, i;
    uint8_t *syx;
    char err[256] = "";
    dt_rt *rt;
    pthread_t th;
    for (i = 0; i < (size_t)argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < (size_t)argc) syx_path = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < (size_t)argc) state = argv[++i];
        else if (!strcmp(argv[i], "--dsp") && i + 1 < (size_t)argc) {
            if (dsp_arg(argv[++i]))
                return 1;
        }
    }
    if (!syx_path || !state || !(syx = read_file(syx_path, &len)))
        return 2;
    rt = dt_rt_new(syx, len, state, 48000, 256, err, sizeof err);
    if (!rt) {
        fprintf(stderr, "dncore: %s\n", err);
        return 1;
    }
    pthread_create(&th, NULL, data_render, rt);
    sleep_s(1.0);
    for (k = 0; k < 4; k++) {
        dn_data_entry *v = NULL;
        size_t n = 0;
        double t0 = now_s();
        if (dn_data_ls(rt, paths[k], &v, &n, err, sizeof err)) {
            printf("[dc] %s: errore %s\n", paths[k], err);
            continue;
        }
        printf("[dc] %s: %zu voci (%.2f s); prime:", paths[k], n, now_s() - t0);
        for (i = 0; i < n && i < 6; i++)
            printf(" (%s,%d,%u,%u,%d)", v[i].name, v[i].type, v[i].index, v[i].size, v[i].valid);
        printf("\n");
        free(v);
    }
    {
        uint8_t *d = NULL;
        size_t dn = 0;
        if (dn_data_read(rt, "/soundbanks/A/1", &d, &dn, err, sizeof err))
            printf("[dc] A/1: errore %s\n", err);
        else {
            int hidden = 0;
            for (i = 0; i + 12 <= dn; i++)
                if (!memcmp(d + i, "HIDDEN TEARS", 12))
                    hidden = 1;
            printf("[dc] A/1: %zu byte, contiene HIDDEN TEARS: %s\n", dn, hidden ? "True" : "False");
            free(d);
        }
    }
    g_data_run = 0;
    pthread_join(th, NULL);
    printf("[dc] %s\n", dt_rt_status(rt));
    dt_rt_free(rt);
    free(syx);
    return 0;
}

/* dncore coldboot --syx SYX --out F.dtsz [--instrs N] [--rung N] [--no-compress]
   la sessione iniziale dal solo firmware (dn_coldboot_session, docs/COLDBOOT.md);
   dncore coldboot --syx SYX --out F.dtst --rung-only N [--factory-index]
   solo un gradino della scala di emu/dspboot.py (boot*M.snap), senza motore */
static volatile int g_cb_cancel;

static void cb_progress(void *ctx, int pct)
{
    static int last = -10;
    (void)ctx;
    if (pct < last + 10 && pct < 100)
        return;
    last = pct;
    fprintf(stderr, "[coldboot] %3d%%\n", pct);
}

static int cmd_coldboot(int argc, char **argv)
{
    const char *syx_path = NULL, *out_path = NULL;
    int64_t instrs = 0;
    int compress = 1, factory = 0, rung_only = 0;
    uint64_t point = 0;
    dt_firmware fw;
    uint8_t *out = NULL;
    size_t n = 0, len = 0;
    char err[256] = "";
    double t0;
    int i, r;
    FILE *f;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx_path = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out_path = argv[++i];
        else if (!strcmp(argv[i], "--instrs") && i + 1 < argc) instrs = (int64_t)strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--rung-only") && i + 1 < argc) { rung_only = 1; point = (uint64_t)strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--no-compress")) compress = 0;
        else if (!strcmp(argv[i], "--factory-index")) factory = 1;
        else if (!strcmp(argv[i], "--dsp") && i + 1 < argc) {
            if (dsp_arg(argv[++i]))
                return 1;
        } else {
            fprintf(stderr, "dncore coldboot: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx_path || !out_path) {
        fprintf(stderr, "uso: dncore coldboot --syx SYX --out F.dtsz [--instrs N] [--no-compress]\n"
                        "     dncore coldboot --syx SYX --out F.dtst --rung-only N [--factory-index]\n");
        return 2;
    }
    if (load_fw(syx_path, &fw))
        return 1;
    t0 = now_s();
    if (rung_only)
        r = dn_coldboot_rung(&fw, point, compress, factory ? DT_COLDBOOT_FACTORY_INDEX : 0, &out, &len,
                             err, sizeof err);
    else
        r = dn_coldboot_session(&fw, instrs ? instrs : DN_COLDBOOT_INSTRS, compress, &out, &len, &g_cb_cancel,
                                cb_progress, NULL, err, sizeof err);
    if (r) {
        fprintf(stderr, "dncore coldboot: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    f = fopen(out_path, "wb");
    if (!f || fwrite(out, 1, len, f) != len) {
        fprintf(stderr, "dncore coldboot: cannot write %s\n", out_path);
        return 1;
    }
    fclose(f);
    (void)n;
    printf("[coldboot] %s: %zu byte in %.1f s\n", out_path, len, now_s() - t0);
    free(out);
    dt_fw_free(&fw);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "multi"))
        return cmd_multi(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "data-test"))
        return cmd_data_test(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "play"))
        return cmd_play(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "rt-test"))
        return cmd_rt_test(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "coldboot"))
        return cmd_coldboot(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "version")) {
        printf("dncore %s (dtcore %s)\n", DNCORE_VERSION, dt_version());
        return 0;
    }
    fprintf(stderr, "uso: dncore play|rt-test|data-test|version ...\n");
    return 2;
}
