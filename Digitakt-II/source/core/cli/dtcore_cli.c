/* dtcore: riga di comando di prova della libreria (M1).
 *
 *   dtcore sections SYX              una riga JSON per sezione (id, nome, sha256)
 *   dtcore extract SYX OUTDIR        come `python -m emu.extract SYX -o OUTDIR`
 *   dtcore run --syx SYX --state F.dtst --count N [--idle-yield N]
 *                                    esegue N istruzioni e stampa un JSON con
 *                                    registri, impronta della memoria e contatori,
 *                                    lo stesso di tools/dtcore_ref.py
 *   dtcore play --syx SYX --state F.dtst --instrs N [--scenario S] [--trace OUT]
 *               [--ram-every K]          il worker della GUI, con la traccia di
 *                                    tools/dtcore_guiref.py
 */
#include "dtcore.h"
#include "../src/sha256.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

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
        fprintf(stderr, "dtcore: cannot read %s: %s\n", path, strerror(errno));
        return -1;
    }
    r = dt_fw_extract(syx, len, fw, err, sizeof err);
    free(syx);
    if (r) {
        fprintf(stderr, "dtcore: %s: %s\n", path, err);
        return -1;
    }
    return 0;
}

static void hex(const uint8_t d[32], char out[65])
{
    dt_sha256_hex(d, out);
}

static int cmd_sections(int argc, char **argv)
{
    dt_firmware fw;
    int i;
    char hx[65];
    if (argc != 1) {
        fprintf(stderr, "uso: dtcore sections SYX\n");
        return 2;
    }
    if (load_fw(argv[0], &fw))
        return 1;
    hex(fw.syx_sha256, hx);
    printf("{\"syx_sha256\": \"%s\", \"sections\": %d}\n", hx, fw.count);
    for (i = 0; i < fw.count; i++) {
        dt_sha256 h;
        uint8_t d[32];
        dt_sha256_init(&h);
        dt_sha256_update(&h, fw.sec[i].data, fw.sec[i].len);
        dt_sha256_final(&h, d);
        hex(d, hx);
        printf("{\"id\": %u, \"name\": \"section_%u_%s.bin\", \"kind\": \"%s\", \"len\": %zu, "
               "\"dest\": %u, \"sha256\": \"%s\"}\n",
               fw.sec[i].id, fw.sec[i].id, dt_fw_section_name(fw.sec[i].id),
               fw.sec[i].packed ? "packed" : "raw", fw.sec[i].len, fw.sec[i].dest, hx);
    }
    dt_fw_free(&fw);
    return 0;
}

static int cmd_extract(int argc, char **argv)
{
    dt_firmware fw;
    char path[4096], hx[65];
    FILE *f;
    int i;
    if (argc != 2) {
        fprintf(stderr, "uso: dtcore extract SYX OUTDIR\n");
        return 2;
    }
    if (load_fw(argv[0], &fw))
        return 1;
    for (i = 0; i < fw.count; i++) {
        snprintf(path, sizeof path, "%s/section_%u_%s.bin", argv[1], fw.sec[i].id,
                 dt_fw_section_name(fw.sec[i].id));
        f = fopen(path, "wb");
        if (!f || fwrite(fw.sec[i].data, 1, fw.sec[i].len, f) != fw.sec[i].len) {
            fprintf(stderr, "dtcore: cannot write %s\n", path);
            if (f)
                fclose(f);
            dt_fw_free(&fw);
            return 1;
        }
        fclose(f);
        printf("  %-26s %-7s %9zu bytes  dest=%#010x\n", strrchr(path, '/') + 1,
               fw.sec[i].packed ? "packed" : "raw", fw.sec[i].len, fw.sec[i].dest);
    }
    snprintf(path, sizeof path, "%s/.source-sha256", argv[1]);
    f = fopen(path, "w");
    if (f) {
        hex(fw.syx_sha256, hx);
        fprintf(f, "%s\n", hx);
        fclose(f);
    }
    dt_fw_free(&fw);
    return 0;
}

static double now_s(void)
{
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (double)ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int cmd_run(int argc, char **argv)
{
    const char *syx = NULL, *state = NULL;
    unsigned long long count = 0;
    dt_machine_config cfg = {20000, NULL, NULL};
    dt_firmware fw;
    dt_machine *m;
    dt_counters c;
    char err[256] = "", hx[65];
    uint32_t regs[18], nreg = 0;
    uint64_t nbytes = 0;
    uint8_t dig[32];
    double t0, t1;
    int i, r;
    static const char *names[18] = {"d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
                                    "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "pc", "sr"};
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc)
            syx = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc)
            state = argv[++i];
        else if (!strcmp(argv[i], "--count") && i + 1 < argc)
            count = strtoull(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--idle-yield") && i + 1 < argc)
            cfg.idle_yield = (int32_t)strtol(argv[++i], NULL, 0);
        else {
            fprintf(stderr, "dtcore run: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state) {
        fprintf(stderr, "uso: dtcore run --syx SYX --state F.dtst --count N [--idle-yield N]\n");
        return 2;
    }
    if (load_fw(syx, &fw))
        return 1;
    m = dt_machine_new(&fw, &cfg, err, sizeof err);
    if (!m) {
        fprintf(stderr, "dtcore: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    t0 = now_s();
    if ((r = dt_machine_load_state(m, state, err, sizeof err)) != 0) {
        fprintf(stderr, "dtcore: %s: %s\n", state, err);
        dt_machine_free(m);
        dt_fw_free(&fw);
        return 1;
    }
    t1 = now_s();
    fprintf(stderr, "dtcore: stato caricato in %.2f s\n", t1 - t0);
    t0 = now_s();
    r = count ? dt_machine_run_count(m, count) : 0;
    t1 = now_s();
    dt_machine_regs(m, regs);
    dt_machine_counters(m, &c);
    dt_machine_digest(m, dig, &nreg, &nbytes);
    hex(dig, hx);
    printf("{\"impl\": \"c\", \"count\": %llu, \"uc_status\": %d, \"regs\": {", count, r);
    for (i = 0; i < 18; i++)
        printf("%s\"%s\": %u", i ? ", " : "", names[i], regs[i]);
    printf("}, \"mem_sha256\": \"%s\", \"regions\": %u, \"bytes\": %llu, "
           "\"movec_count\": %llu, \"idle_spins\": %llu, \"halt_vec\": %d, "
           "\"movec_sites\": %d, \"idle_sites\": %d, \"mmio_hooks\": %d}\n",
           hx, nreg, (unsigned long long)nbytes, (unsigned long long)c.movec,
           (unsigned long long)c.idle_spins, c.halt_vec, c.movec_sites, c.idle_sites,
           c.mmio_hooks);
    fprintf(stderr, "dtcore: %llu istruzioni in %.3f s (%.1f M istr/s), %llu eccezioni, %llu RTE, "
                    "%llu pagine mappate a richiesta\n",
            count, t1 - t0, (t1 > t0 ? count / (t1 - t0) : 0) / 1e6,
            (unsigned long long)c.raised, (unsigned long long)c.rte,
            (unsigned long long)c.faults_mapped);
    dt_machine_free(m);
    dt_fw_free(&fw);
    return r ? 1 : 0;
}

typedef struct {
    int64_t at;
    int kind;                     /* 0 midi, 1 press, 2 release, 3 encoder, 4 rpc */
    int a, b;
    uint8_t data[256];
    size_t n;
} ev_t;

static int hexval(int c)
{
    return c >= '0' && c <= '9' ? c - '0' : (c | 32) >= 'a' && (c | 32) <= 'f' ? (c | 32) - 'a' + 10 : -1;
}

static void ram_line(FILE *out, dt_engine *e, const char *tag)
{
    static const char *names[18] = {"d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7",
                                    "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "pc", "sr"};
    uint32_t regs[18], nreg = 0;
    uint64_t nbytes = 0;
    uint8_t dig[32];
    char hx[65];
    int i;
    dt_machine *m = dt_engine_machine(e);
    dt_machine_regs(m, regs);
    dt_machine_digest(m, dig, &nreg, &nbytes);
    dt_sha256_hex(dig, hx);
    hx[16] = 0;
    fprintf(out, "ram %s", tag);
    for (i = 0; i < 18; i++)
        fprintf(out, " %s=%08x", names[i], regs[i]);
    fprintf(out, " %s regions=%u\n", hx, nreg);
}

/* ------------------------------------------------------------ rt-test */
typedef struct {
    float *buf;
    size_t n, cap;
} collect_t;

static void collect_sink(void *ctx, const float *mix, const float *tracks, size_t n)
{
    collect_t *c = (collect_t *)ctx;
    (void)tracks;
    if (c->n + n > c->cap) {
        c->cap = (c->n + n) * 2;
        c->buf = (float *)realloc(c->buf, c->cap * sizeof(float));
    }
    memcpy(c->buf + c->n, mix, n * sizeof(float));
    c->n += n;
}

static int cmd_play(int argc, char **argv)
{
    const char *syx = NULL, *state = NULL, *scen = NULL, *trace = NULL, *save = NULL, *raw = NULL;
    double limit_d = 0;
    collect_t col = {NULL, 0, 0};
    int64_t limit, clk_from = 0, clk_every = 0, clk_until = 0, clk_next = 0;
    int ram_every = 25, i, have_clock = 0, last = 0;
    ev_t *evs = NULL;
    size_t nev = 0, cap = 0, iev = 0;
    dt_firmware fw;
    dt_engine *e;
    char err[256] = "", line[1024];
    FILE *out = stdout;
    int64_t chunk = 0;
    unsigned rpc_seq = 0;
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
        else {
            fprintf(stderr, "dtcore play: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state || limit_d <= 0) {
        fprintf(stderr, "uso: dtcore play --syx SYX --state F.dtst --instrs N [--scenario S] [--trace OUT] [--ram-every K]\n");
        return 2;
    }
    limit = (int64_t)limit_d;
    if (scen) {
        FILE *f = fopen(scen, "r");
        if (!f) {
            fprintf(stderr, "dtcore: cannot read %s\n", scen);
            return 1;
        }
        while (fgets(line, sizeof line, f)) {
            char *h = strchr(line, '#'), w[5][128];
            int nw;
            if (h)
                *h = 0;
            nw = sscanf(line, "%127s %127s %127s %127s %127s", w[0], w[1], w[2], w[3], w[4]);
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
                } else if (!strcmp(w[2], "rpc")) {
                    size_t k;
                    ev.kind = 4;
                    ev.a = (int)strtol(w[3], NULL, 0);
                    for (k = 0; nw >= 5 && w[4][2 * k] && w[4][2 * k + 1] && k < sizeof ev.data; k++)
                        ev.data[k] = (uint8_t)(hexval(w[4][2 * k]) << 4 | hexval(w[4][2 * k + 1]));
                    ev.n = k;
                } else if (!strcmp(w[2], "encoder") && nw >= 5) {
                    ev.kind = 3; ev.a = atoi(w[3]); ev.b = atoi(w[4]);
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
        /* sorted((at, indice)): stabile per istante */
        {
            size_t a, b;
            for (a = 1; a < nev; a++)
                for (b = a; b > 0 && evs[b - 1].at > evs[b].at; b--) {
                    ev_t t = evs[b]; evs[b] = evs[b - 1]; evs[b - 1] = t;
                }
        }
    }
    if (trace && !(out = fopen(trace, "w"))) {
        fprintf(stderr, "dtcore: cannot write %s\n", trace);
        return 1;
    }
    if (load_fw(syx, &fw))
        return 1;
    t0 = now_s();
    e = dt_engine_new(&fw, state, NULL, err, sizeof err);
    if (!e) {
        fprintf(stderr, "dtcore: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    t1 = now_s();
    fprintf(stderr, "dtcore: motore pronto in %.2f s\n", t1 - t0);
    if (raw)
        dt_engine_set_audio_sink(e, collect_sink, &col, 0);
    t0 = now_s();
    for (;;) {
        int64_t n = dt_engine_instrs(e);
        if (chunk > 0) {
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
            else if (v->kind == 4) {             /* transfer.request(seq, rid, body) */
                uint8_t pay[5 + sizeof v->data];
                rpc_seq++;
                pay[0] = (uint8_t)(rpc_seq >> 8); pay[1] = (uint8_t)rpc_seq;
                pay[2] = 0; pay[3] = 0; pay[4] = (uint8_t)v->a;
                memcpy(pay + 5, v->data, v->n);
                dt_engine_rpc(e, pay, 5 + v->n);
            }
            else
                dt_engine_panel(e, DT_PANEL_ENCODER, v->a, v->b);
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
    fprintf(stderr, "dtcore: %lld istruzioni in %.2f s (%.1f M istr/s), %s\n",
            (long long)dt_engine_instrs(e), t1 - t0, dt_engine_instrs(e) / (t1 - t0) / 1e6,
            dt_engine_status(e));
    if (out != stdout)
        fclose(out);
    if (raw) {
        FILE *rf = fopen(raw, "wb");
        if (rf) {
            fwrite(col.buf, sizeof(float), col.n, rf);
            fclose(rf);
        }
        free(col.buf);
    }
    if (save) {
        int r = dt_engine_save(e, save, 0);
        fprintf(stderr, "dtcore: stato salvato in %s (%d)\n", save, r);
    }
    dt_engine_free(e);
    dt_fw_free(&fw);
    free(evs);
    return 0;
}

typedef struct {
    int64_t h;
    uint8_t d[3];
    int n;
} rtev;

/* --notes-only: solo le note (un host VST3 non porta Start ne' clock: vst/tools/vst3render.cpp) */
static int g_rt_notes_only = 0;

/* gli eventi del test: START, clock a 120 BPM (ogni 1000 campioni a 48 kHz), note */
static size_t rt_events(rtev *ev, size_t cap, int64_t H)
{
    size_t n = 0;
    int64_t h;
    if (!g_rt_notes_only) {
        ev[n].h = 4800; ev[n].d[0] = 0xFA; ev[n].n = 1; n++;
    }
    for (h = 4800; !g_rt_notes_only && h < H - 4800 && n < cap - 40; h += 1000) {
        ev[n].h = h; ev[n].d[0] = 0xF8; ev[n].n = 1; n++;
    }
    for (h = 9600; h < H && n < cap; h += 7919) {
        ev[n].h = h; ev[n].d[0] = (uint8_t)(0x90 | (h / 7919) % 16);
        ev[n].d[1] = (uint8_t)(48 + h % 24); ev[n].d[2] = 100; ev[n].n = 3; n++;
    }
    /* in ordine di istante */
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

static float *rt_run(const uint8_t *syx, size_t len, const char *state, int64_t H, int random_blocks,
                     double rate, int64_t *E, int64_t *X, int64_t *underruns)
{
    double t0 = now_s();
    char err[256] = "";
    dt_rt *rt = dt_rt_new(syx, len, state, rate, g_rt_frames ? g_rt_frames : g_rt_realtime ? 256 : 2048,
                          err, sizeof err);
    float *out, *o[1];
    rtev ev[4096];
    size_t nev = rt_events(ev, 4096, (int64_t)(H * 48000.0 / rate)), iev = 0;
    int64_t c = 0;
    unsigned seed = 12345;
    if (!rt) {
        fprintf(stderr, "dtcore: %s\n", err);
        return NULL;
    }
    dt_rt_offsets(rt, E, X);
    out = (float *)calloc((size_t)H, sizeof(float));
    while (c < H) {
        uint32_t n = g_rt_frames ? g_rt_frames : 256;
        if (random_blocks) {
            seed = seed * 1103515245u + 12345u;
            n = 16 + (seed >> 8) % (g_rt_realtime ? 241 : 2033);
        }
        if (c + n > H)
            n = (uint32_t)(H - c);
        /* eventi a istanti a 48 kHz: qui l'host e' a `rate` */
        while (iev < nev && (int64_t)(ev[iev].h * rate / 48000.0) < c + n) {
            int64_t hh = (int64_t)(ev[iev].h * rate / 48000.0);
            dt_rt_midi(rt, (uint32_t)(hh > c ? hh - c : 0), ev[iev].d, (size_t)ev[iev].n);
            iev++;
        }
        o[0] = out + c;
        if (g_rt_realtime) {                      /* l'host: un blocco quando tocca */
            sleep_s(t0 + (double)c / rate - now_s());
            dt_rt_render(rt, n, o, 1, 0);
        } else
            dt_rt_render(rt, n, o, 1, 1);
        c += n;
    }
    *underruns = dt_rt_underruns(rt);
    fprintf(stderr, "rt: ritardo massimo del sintetizzatore %lld campioni\n", (long long)dt_rt_max_lag(rt));
    dt_rt_free(rt);
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
        else if (!strcmp(argv[i], "--notes-only")) g_rt_notes_only = 1;
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) g_rt_frames = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dump") && i + 1 < argc) dump = argv[++i];
    }
    if (!syx_path || !state) {
        fprintf(stderr, "uso: dtcore rt-test --syx SYX --state F.dtst [--seconds S] [--rate R]\n");
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
    for (i = 0; i < H; i++)
        if (memcmp(&a[i], &b[i], 4)) {
            if (firstbad < 0)
                firstbad = i;
            bad++;
        }
    if (dump) {
        FILE *f = fopen(dump, "wb");
        if (f) {
            fwrite(a, sizeof(float), (size_t)H, f);
            fclose(f);
        }
    }
    printf("rt blocchi fissi contro casuali: %lld campioni, %lld diversi (primo %lld), buchi %lld/%lld\n",
           (long long)H, (long long)bad, (long long)firstbad, (long long)u1, (long long)u2);
    if (rate == 48000.0) {
        /* il motore diretto, gli eventi agli stessi confini */
        if (load_fw(syx_path, &fw))
            return 1;
        e = dt_engine_new(&fw, state, NULL, err, sizeof err);
        if (!e) {
            fprintf(stderr, "dtcore: %s\n", err);
            return 1;
        }
        dt_engine_set_audio_sink(e, collect_sink, &col, 0);
        nev = rt_events(ev, 4096, H);
        while ((int64_t)col.n < H + E) {
            int64_t t48 = dt_engine_instrs(e) / 3125;
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
            int64_t k = i + E - X;
            float want = k >= 0 && k < (int64_t)col.n ? col.buf[k] : 0.0f;
            if (memcmp(&a[i], &want, 4)) {
                if (firstbad < 0)
                    firstbad = i;
                bad++;
            }
        }
        printf("rt contro motore diretto: %lld diversi (primo %lld); E=%lld X=%lld\n", (long long)bad,
               (long long)firstbad, (long long)E, (long long)X);
        {
            double peak = 0;
            for (i = 0; i < H; i++)
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

static int cmd_coldboot(int argc, char **argv)
{
    const char *syx = NULL, *outp = NULL;
    double point = 0, gui = 0;
    dt_firmware fw;
    uint8_t *buf = NULL;
    size_t n = 0;
    char err[256] = "";
    double t0;
    int i, r, flags = 0;
    FILE *f;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--factory-index")) flags |= DT_COLDBOOT_FACTORY_INDEX;
        else if (!strcmp(argv[i], "--emmc-dt2")) flags |= DT_COLDBOOT_EMMC_DT2;
        else if (!strcmp(argv[i], "--format-drive")) flags |= DT_COLDBOOT_FORMAT_DRIVE;
        else if (!strcmp(argv[i], "--plus-drive")) flags |= DT_COLDBOOT_PLUSDRIVE;
        else if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--point") && i + 1 < argc) point = strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) outp = argv[++i];
        else if (!strcmp(argv[i], "--gui") && i + 1 < argc) gui = strtod(argv[++i], NULL);
    }
    if (!syx || !outp || point < 1) {
        fprintf(stderr, "uso: dtcore coldboot --syx SYX --point N [--gui M] [--factory-index] [--emmc-dt2]\n"
                        "                     [--format-drive | --plus-drive] --out F.dtst\n"
                        "  --gui M: poi M istruzioni del motore della GUI (dt_coldboot_session)\n"
                        "  --factory-index: indice di fabbrica vuoto sull'eMMC (i164)\n"
                        "  --emmc-dt2: la eMMC del Digitakt II, il firmware la riconosce (i212)\n"
                        "  --format-drive: dopo la sessione FORMAT +DRIVE dal menu (i212; implica --emmc-dt2)\n"
                        "  --plus-drive: le tre insieme, lo stato iniziale del plugin\n");
        return 2;
    }
    if (load_fw(syx, &fw))
        return 1;
    t0 = now_s();
    if ((flags & DT_COLDBOOT_FORMAT_DRIVE) && gui <= 0)
        gui = (double)DT_COLDBOOT_GUI;
    r = gui > 0 ? dt_coldboot_session_ex(&fw, (uint64_t)point, (int64_t)gui, 0, flags, &buf, &n, NULL, err, sizeof err)
                : dt_coldboot_ex(&fw, (uint64_t)point, 0, flags, &buf, &n, err, sizeof err);
    if (r) {
        fprintf(stderr, "dtcore: boot freddo: %s (%d)\n", err, r);
        return 1;
    }
    f = fopen(outp, "wb");
    if (!f || fwrite(buf, 1, n, f) != n)
        return 1;
    fclose(f);
    fprintf(stderr, "dtcore: boot freddo fino a %.0f istruzioni in %.1f s -> %s\n", point, now_s() - t0, outp);
    free(buf);
    dt_fw_free(&fw);
    return 0;
}

#include <pthread.h>
static volatile int g_render_run = 1;

static void *render_loop(void *arg)
{
    dt_rt *rt = (dt_rt *)arg;
    static float buf[256];
    float *o[1] = {buf};
    while (g_render_run)
        dt_rt_render(rt, 256, o, 1, 1);           /* l'host: il tempo del motore avanza */
    return NULL;
}

static int cmd_drive_test(int argc, char **argv)
{
    const char *syx_path = NULL, *state = NULL, *wavp = NULL;
    size_t len, wl = 0, n, i, cl;
    uint8_t *syx, *wav, *blob;
    char err[256] = "";
    dt_rt *rt;
    dt_drive_entry *ent;
    pthread_t th;
    int r = 0;
    for (i = 0; i < (size_t)argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < (size_t)argc) syx_path = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < (size_t)argc) state = argv[++i];
        else if (!strcmp(argv[i], "--wav") && i + 1 < (size_t)argc) wavp = argv[++i];
    }
    if (!syx_path || !state || !wavp)
        return 2;
    syx = read_file(syx_path, &len);
    wav = read_file(wavp, &wl);
    rt = dt_rt_new(syx, len, state, 48000, 256, err, sizeof err);
    if (!rt) {
        fprintf(stderr, "%s\n", err);
        return 1;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    pthread_create(&th, NULL, render_loop, rt);
#define STEP(call, what) do { double t0 = now_s(); int rr = (call); printf("%-28s %s (%.2f s) %s\n", what, rr ? "ERRORE" : "ok", now_s() - t0, rr ? err : ""); if (rr) r = 1; } while (0)
    STEP(dt_drive_ls(rt, "/", &ent, &n, err, sizeof err), "ls /");
    for (i = 0; i < n; i++)
        printf("    %c %-24s %u\n", ent[i].type, ent[i].name, ent[i].size);
    free(ent);
    {
        char line[1024];
        dt_engine_trace(dt_rt_engine(rt), "dopo-ls", 0, line, sizeof line);
        printf("%s\n", line);
    }
    STEP(dt_drive_mkdir(rt, "/dtcore_test", err, sizeof err), "mkdir /dtcore_test");
    STEP(dt_wav2dt(wav, wl, &blob, &cl, err, sizeof err), "wav2dt");
    STEP(dt_drive_put(rt, "/dtcore_test/kick", blob, cl, NULL, NULL, err, sizeof err), "put /dtcore_test/kick");
    STEP(dt_drive_ls(rt, "/dtcore_test", &ent, &n, err, sizeof err), "ls /dtcore_test");
    for (i = 0; i < n; i++)
        printf("    %c %-24s %u (atteso %zu)\n", ent[i].type, ent[i].name, ent[i].size, cl);
    free(ent);
    {   /* lo stato salvato (quello del progetto della DAW) riaperto in un motore nuovo */
        uint8_t *st = NULL;
        size_t sl = 0;
        dt_rt *rt2;
        STEP(dt_rt_save(rt, &st, &sl), "dt_rt_save");
        g_render_run = 0;
        pthread_join(th, NULL);
        dt_rt_free(rt);
        rt2 = st ? dt_rt_new_mem(syx, len, st, sl, 48000, 256, err, sizeof err) : NULL;
        free(st);
        STEP(rt2 ? 0 : -1, "dt_rt_new_mem");
        if (!rt2) {
            free(blob);
            return 1;
        }
        rt = rt2;
        g_render_run = 1;
        pthread_create(&th, NULL, render_loop, rt);
    }
    STEP(dt_drive_ls(rt, "/dtcore_test", &ent, &n, err, sizeof err), "ls /dtcore_test (ripreso)");
    for (i = 0; i < n; i++)
        printf("    %c %-24s %u (atteso %zu)\n", ent[i].type, ent[i].name, ent[i].size, cl);
    if (n != 1 || ent[0].size != cl)
        r = 1;
    free(ent);
    STEP(dt_drive_rm(rt, "/dtcore_test/kick", err, sizeof err), "rm /dtcore_test/kick");
    STEP(dt_drive_rmdir(rt, "/dtcore_test", err, sizeof err), "rmdir /dtcore_test");
    g_render_run = 0;
    pthread_join(th, NULL);
    dt_rt_free(rt);
    free(blob);
    return r;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "drive-test"))
        return cmd_drive_test(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "coldboot"))
        return cmd_coldboot(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "rt-test"))
        return cmd_rt_test(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "play"))
        return cmd_play(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "sections"))
        return cmd_sections(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "extract"))
        return cmd_extract(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "run"))
        return cmd_run(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "--version")) {
        printf("dtcore %s\n", dt_version());
        return 0;
    }
    fprintf(stderr, "uso: dtcore sections|extract|run ... (dtcore %s)\n", dt_version());
    return 2;
}
