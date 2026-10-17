/* arcore: riga di comando di prova della libreria del Analog Rytm MKII.
 *
 *   arcore sections SYX              una riga JSON per sezione (id, nome, sha256)
 *   arcore extract SYX OUTDIR        come `python -m emu.extract SYX -o OUTDIR`
 *   arcore run --syx SYX --state F.dtst --count N
 *                                    la macchina di base: N istruzioni esatte
 *   arcore play --syx SYX --state F.dtst --instrs N [--scenario S] [--trace OUT]
 *               [--ram-every K] [--audio OUT.f32] [--save OUT.dtst]
 *                                    il worker della GUI v2 --audio, con la
 *                                    traccia di tools/arcore_ref.py
 *   arcore rt-test --syx SYX --state F.dtst [--seconds S] [--rate R] [--frames N]
 *               [--realtime] [--dump OUT.f32]
 *                                    il motore in tempo reale (dt_rt): blocchi
 *                                    fissi = casuali = motore diretto
 */
#include "arcore.h"
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
        fprintf(stderr, "arcore: cannot read %s: %s\n", path, strerror(errno));
        return -1;
    }
    r = dt_fw_extract(syx, len, fw, err, sizeof err);
    free(syx);
    if (r) {
        fprintf(stderr, "arcore: %s: %s\n", path, err);
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
        fprintf(stderr, "uso: arcore sections SYX\n");
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
        fprintf(stderr, "uso: arcore extract SYX OUTDIR\n");
        return 2;
    }
    if (load_fw(argv[0], &fw))
        return 1;
    for (i = 0; i < fw.count; i++) {
        snprintf(path, sizeof path, "%s/section_%u_%s.bin", argv[1], fw.sec[i].id,
                 dt_fw_section_name(fw.sec[i].id));
        f = fopen(path, "wb");
        if (!f || fwrite(fw.sec[i].data, 1, fw.sec[i].len, f) != fw.sec[i].len) {
            fprintf(stderr, "arcore: cannot write %s\n", path);
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
    dt_machine_config cfg = {.idle_yield = 20000};
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
            fprintf(stderr, "arcore run: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state) {
        fprintf(stderr, "uso: arcore run --syx SYX --state F.dtst --count N [--idle-yield N]\n");
        return 2;
    }
    if (load_fw(syx, &fw))
        return 1;
    m = dt_machine_new(&fw, &cfg, err, sizeof err);
    if (!m) {
        fprintf(stderr, "arcore: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    t0 = now_s();
    if ((r = dt_machine_load_state(m, state, err, sizeof err)) != 0) {
        fprintf(stderr, "arcore: %s: %s\n", state, err);
        dt_machine_free(m);
        dt_fw_free(&fw);
        return 1;
    }
    t1 = now_s();
    fprintf(stderr, "arcore: stato caricato in %.2f s\n", t1 - t0);
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
    fprintf(stderr, "arcore: %llu istruzioni in %.3f s (%.1f M istr/s), %llu eccezioni, %llu RTE, "
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
    int kind;                     /* 0 midi, 1 press, 2 release, 3 encoder, 4 raw, 5 pad, 6 padup */
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

static void file_sink(void *ctx, const float *lr, const float *voices, size_t n)
{
    (void)voices;
    fwrite(lr, sizeof(float), 2 * n, (FILE *)ctx);
}

static size_t parse_hex(const char *s, uint8_t *out, size_t cap)
{
    size_t k;
    for (k = 0; s[2 * k] && s[2 * k + 1] && k < cap; k++)
        out[k] = (uint8_t)(hexval(s[2 * k]) << 4 | hexval(s[2 * k + 1]));
    return k;
}

/* lo scenario di tools/arcore_ref.py */
static ev_t *read_scenario(const char *scen, size_t *pnev, int *have_clock, int64_t clk[3])
{
    char line[1024];
    ev_t *evs = NULL;
    size_t nev = 0, cap = 0;
    FILE *f = fopen(scen, "r");
    if (!f) {
        fprintf(stderr, "arcore: cannot read %s\n", scen);
        return NULL;
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
            *have_clock = 1;
            clk[0] = (int64_t)strtod(w[1], NULL);
            clk[1] = (int64_t)strtod(w[2], NULL);
            clk[2] = (int64_t)strtod(w[3], NULL);
        } else if (!strcmp(w[0], "at") && nw >= 4) {
            ev_t ev;
            memset(&ev, 0, sizeof ev);
            ev.at = (int64_t)strtod(w[1], NULL);
            if (!strcmp(w[2], "midi")) {
                ev.n = parse_hex(w[3], ev.data, sizeof ev.data);
            } else if (!strcmp(w[2], "press")) {
                ev.kind = 1; ev.a = (int)strtol(w[3], NULL, 0);
            } else if (!strcmp(w[2], "release")) {
                ev.kind = 2; ev.a = (int)strtol(w[3], NULL, 0);
            } else if (!strcmp(w[2], "encoder") && nw >= 5) {
                ev.kind = 3; ev.a = (int)strtol(w[3], NULL, 0); ev.b = atoi(w[4]);
            } else if (!strcmp(w[2], "raw")) {
                ev.kind = 4; ev.n = parse_hex(w[3], ev.data, sizeof ev.data);
            } else if (!strcmp(w[2], "pad") && nw >= 5) {
                ev.kind = 5; ev.a = atoi(w[3]); ev.b = atoi(w[4]);
            } else if (!strcmp(w[2], "padup")) {
                ev.kind = 6; ev.a = atoi(w[3]);
            } else {
                fprintf(stderr, "arcore: scenario: evento sconosciuto %s\n", w[2]);
                fclose(f);
                free(evs);
                return NULL;
            }
            if (nev == cap) {
                cap = cap ? 2 * cap : 64;
                evs = (ev_t *)realloc(evs, cap * sizeof *evs);
            }
            evs[nev++] = ev;
        }
    }
    fclose(f);
    {                                             /* sorted((at, indice)): stabile */
        size_t a, b;
        for (a = 1; a < nev; a++)
            for (b = a; b > 0 && evs[b - 1].at > evs[b].at; b--) {
                ev_t t = evs[b]; evs[b] = evs[b - 1]; evs[b - 1] = t;
            }
    }
    *pnev = nev;
    if (!evs)
        evs = (ev_t *)calloc(1, sizeof *evs);
    return evs;
}

/* il ciclo di tools/arcore_ref.py: al confine di ogni giro la riga del giro
   precedente, poi gli eventi dovuti; arrivato al limite il worker fa ancora
   un giro (stop_flag si guarda in cima al ciclo) */
static int cmd_play(int argc, char **argv)
{
    const char *syx = NULL, *state = NULL, *scen = NULL, *trace = NULL, *save = NULL, *audio = NULL;
    double limit_d = 0;
    int64_t limit, clk[3] = {0, 0, 0}, clk_next = 0;
    int ram_every = 25, i, have_clock = 0, stop = 0;
    ev_t *evs = NULL;
    size_t nev = 0, iev = 0;
    dt_firmware fw;
    dt_engine *e;
    char err[256] = "", line[1024];
    FILE *out = stdout, *af = NULL;
    int64_t chunk = 0;
    double t0, t1;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
        else if (!strcmp(argv[i], "--instrs") && i + 1 < argc) limit_d = strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--scenario") && i + 1 < argc) scen = argv[++i];
        else if (!strcmp(argv[i], "--trace") && i + 1 < argc) trace = argv[++i];
        else if (!strcmp(argv[i], "--ram-every") && i + 1 < argc) ram_every = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--save") && i + 1 < argc) save = argv[++i];
        else if (!strcmp(argv[i], "--audio") && i + 1 < argc) audio = argv[++i];
        else {
            fprintf(stderr, "arcore play: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state || limit_d <= 0) {
        fprintf(stderr, "uso: arcore play --syx SYX --state F.dtst --instrs N [--scenario S] [--trace OUT]"
                        " [--ram-every K] [--audio OUT.f32] [--save OUT.dtst]\n");
        return 2;
    }
    limit = (int64_t)limit_d;
    if (scen && !(evs = read_scenario(scen, &nev, &have_clock, clk)))
        return 1;
    clk_next = clk[0];
    if (trace && !(out = fopen(trace, "w"))) {
        fprintf(stderr, "arcore: cannot write %s\n", trace);
        return 1;
    }
    if (load_fw(syx, &fw))
        return 1;
    t0 = now_s();
    e = dt_engine_new(&fw, state, NULL, err, sizeof err);
    if (!e) {
        fprintf(stderr, "arcore: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    if (audio) {
        af = fopen(audio, "wb");
        if (!af) {
            fprintf(stderr, "arcore: cannot write %s\n", audio);
            return 1;
        }
        dt_engine_set_audio_sink(e, file_sink, af);
    }
    t1 = now_s();
    fprintf(stderr, "arcore: motore pronto in %.2f s\n", t1 - t0);
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
            stop = 1;
        while (iev < nev && evs[iev].at <= n) {
            ev_t *v = &evs[iev++];
            switch (v->kind) {
            case 0: dt_engine_midi_in(e, v->data, v->n); break;
            case 1: dt_engine_panel(e, DT_PANEL_PRESS, v->a, 0); break;
            case 2: dt_engine_panel(e, DT_PANEL_RELEASE, v->a, 0); break;
            case 3: dt_engine_panel(e, DT_PANEL_ENCODER, v->a, v->b); break;
            case 4: dt_engine_panel_raw(e, v->data, v->n); break;
            case 5: dt_engine_panel(e, DT_PANEL_PAD, v->a, v->b); break;
            default: dt_engine_panel(e, DT_PANEL_PAD, v->a, 0); break;
            }
        }
        if (have_clock) {
            static const uint8_t f8 = 0xF8;
            while (clk_next <= n && clk_next <= clk[2]) {
                dt_engine_midi_in(e, &f8, 1);
                clk_next += clk[1];
            }
        }
        chunk++;
        if (dt_engine_chunk(e) || stop)
            break;
    }
    t1 = now_s();
    dt_engine_trace(e, "end", chunk - 1, line, sizeof line);
    fprintf(out, "%s\n", line);
    ram_line(out, e, "end");
    fprintf(stderr, "arcore: %lld istruzioni in %.2f s (%.1f M istr/s, %.0f%% del tempo reale), %s\n",
            (long long)dt_engine_instrs(e), t1 - t0, dt_engine_instrs(e) / (t1 - t0) / 1e6,
            dt_engine_instrs(e) / 60e6 / (t1 - t0) * 100, dt_engine_status(e));
    if (out != stdout)
        fclose(out);
    if (af)
        fclose(af);
    if (save) {
        int r = dt_engine_save(e, save, 0);
        fprintf(stderr, "arcore: stato salvato in %s (%d)\n", save, r);
    }
    dt_engine_free(e);
    dt_fw_free(&fw);
    free(evs);
    return 0;
}

/* ------------------------------------------------------------ rt-test */
typedef struct {
    float *buf;                   /* stereo interlacciato */
    size_t n, cap;                /* frame */
} collect_t;

static void collect_sink(void *ctx, const float *lr, const float *voices, size_t n)
{
    (void)voices;
    collect_t *c = (collect_t *)ctx;
    if (c->n + n > c->cap) {
        c->cap = (c->n + n) * 2;
        c->buf = (float *)realloc(c->buf, 2 * c->cap * sizeof(float));
    }
    memcpy(c->buf + 2 * c->n, lr, 2 * n * sizeof(float));
    c->n += n;
}

typedef struct {
    int64_t h;
    uint8_t d[3];
    int n;
} rtev;

/* gli eventi del test: START, clock a 120 BPM (ogni 1000 campioni a 48 kHz),
   note sui canali 1-12 (le tracce del Rytm), un note off ciascuna */
/* --notes-only: solo le note (un host VST3 non porta Start ne' clock: vst/tools/vst3render.cpp) */
static int g_rt_notes_only = 0;

static size_t rt_events(rtev *ev, size_t cap, int64_t H)
{
    size_t n = 0;
    int64_t h;
    if (!g_rt_notes_only) {
        ev[n].h = 4800; ev[n].d[0] = 0xFA; ev[n].n = 1; n++;
    }
    for (h = 4800; !g_rt_notes_only && h < H - 4800 && n < cap - 80; h += 1000) {
        ev[n].h = h; ev[n].d[0] = 0xF8; ev[n].n = 1; n++;
    }
    for (h = 9600; h < H && n < cap - 2; h += 7919) {
        uint8_t ch = (uint8_t)((h / 7919) % 12), note = (uint8_t)(36 + h % 24);
        ev[n].h = h; ev[n].d[0] = (uint8_t)(0x90 | ch); ev[n].d[1] = note; ev[n].d[2] = 100; ev[n].n = 3; n++;
        ev[n].h = h + 3000; ev[n].d[0] = (uint8_t)(0x80 | ch); ev[n].d[1] = note; ev[n].d[2] = 0; ev[n].n = 3; n++;
    }
    {                                             /* in ordine di istante, stabile */
        size_t a, b;
        for (a = 1; a < n; a++)
            for (b = a; b > 0 && ev[b - 1].h > ev[b].h; b--) {
                rtev t = ev[b]; ev[b] = ev[b - 1]; ev[b - 1] = t;
            }
    }
    return n;
}

static int g_rt_realtime = 0;
static int g_rt_bus = 0;                    /* --bus K: il paio di canali 2K, 2K+1 (0 il mix, 1-8 le voci) */
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

/* -> H frame stereo interlacciati */
static float *rt_run(const uint8_t *syx, size_t len, const char *state, int64_t H, int random_blocks,
                     double rate, int64_t *E, int64_t *X, int64_t *underruns)
{
    double t0 = now_s();
    char err[256] = "";
    dt_rt *rt = dt_rt_new(syx, len, state, rate, g_rt_frames ? g_rt_frames : g_rt_realtime ? 256 : 2048,
                          err, sizeof err);
    float *out, *o[DT_RT_CHANNELS], *tmp[DT_RT_CHANNELS];
    static rtev ev[8192];
    size_t nev = rt_events(ev, 8192, (int64_t)(H * 48000.0 / rate)), iev = 0;
    int64_t c = 0, i;
    unsigned seed = 12345;
    if (!rt) {
        fprintf(stderr, "arcore: %s\n", err);
        return NULL;
    }
    dt_rt_offsets(rt, E, X);
    out = (float *)calloc((size_t)H * 2, sizeof(float));
    for (i = 0; i < DT_RT_CHANNELS; i++)
        tmp[i] = (float *)calloc(4096, sizeof(float));
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
        for (i = 0; i < DT_RT_CHANNELS; i++)
            o[i] = tmp[i];
        if (g_rt_realtime) {                      /* l'host: un blocco quando tocca */
            sleep_s(t0 + (double)c / rate - now_s());
            dt_rt_render(rt, n, o, DT_RT_CHANNELS, 0);
        } else
            dt_rt_render(rt, n, o, DT_RT_CHANNELS, 1);
        for (i = 0; i < n; i++) {
            out[2 * (c + i)] = tmp[2 * g_rt_bus][i];
            out[2 * (c + i) + 1] = tmp[2 * g_rt_bus + 1][i];
        }
        c += n;
    }
    *underruns = dt_rt_underruns(rt);
    fprintf(stderr, "rt: %s; ritardo massimo %lld\n", dt_rt_status(rt), (long long)dt_rt_max_lag(rt));
    dt_rt_free(rt);
    for (i = 0; i < DT_RT_CHANNELS; i++)
        free(tmp[i]);
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
    static rtev ev[8192];
    size_t nev, iev = 0;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx_path = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--rate") && i + 1 < argc) rate = atof(argv[++i]);
        else if (!strcmp(argv[i], "--realtime")) g_rt_realtime = 1;
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) g_rt_frames = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dump") && i + 1 < argc) dump = argv[++i];
        else if (!strcmp(argv[i], "--bus") && i + 1 < argc) g_rt_bus = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--notes-only")) g_rt_notes_only = 1;
    }
    if (!syx_path || !state) {
        fprintf(stderr, "uso: arcore rt-test --syx SYX --state F.dtst [--seconds S] [--rate R] [--frames N]"
                        " [--realtime] [--dump OUT.f32]\n");
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
            fwrite(a, sizeof(float), (size_t)(2 * H), f);
            fclose(f);
        }
    }
    printf("rt blocchi fissi contro casuali: %lld frame stereo, %lld valori diversi (primo frame %lld), "
           "buchi %lld/%lld\n", (long long)H, (long long)bad, (long long)firstbad, (long long)u1,
           (long long)u2);
    if (rate == 48000.0 && g_rt_bus == 0) {
        /* il motore diretto, gli eventi agli stessi confini */
        int64_t bad2 = 0, first2 = -1;
        if (load_fw(syx_path, &fw))
            return 1;
        e = dt_engine_new(&fw, state, NULL, err, sizeof err);
        if (!e) {
            fprintf(stderr, "arcore: %s\n", err);
            return 1;
        }
        dt_engine_set_audio_sink(e, collect_sink, &col);
        nev = rt_events(ev, 8192, H);
        while ((int64_t)col.n < H + E) {
            int64_t t48 = dt_engine_instrs(e) / 1250;
            while (iev < nev && ev[iev].h + E <= t48) {
                dt_engine_midi_in(e, ev[iev].d, (size_t)ev[iev].n);
                iev++;
            }
            if (dt_engine_chunk(e))
                break;
        }
        for (i = 0; i < H; i++) {
            int64_t k = i + E - X;
            int c;
            for (c = 0; c < 2; c++) {
                float want = k >= 0 && k < (int64_t)col.n ? col.buf[2 * k + c] : 0.0f;
                if (memcmp(&a[2 * i + c], &want, 4)) {
                    if (first2 < 0)
                        first2 = i;
                    bad2++;
                }
            }
        }
        printf("rt contro motore diretto: %lld valori diversi (primo frame %lld); E=%lld X=%lld\n",
               (long long)bad2, (long long)first2, (long long)E, (long long)X);
        {
            double peak = 0;
            for (i = 0; i < 2 * H; i++)
                if (fabs(a[i]) > peak)
                    peak = fabs(a[i]);
            printf("picco %.4f\n", peak);
        }
        bad += bad2;
        dt_engine_free(e);
        dt_fw_free(&fw);
    }
    free(a);
    free(b);
    free(syx);
    free(col.buf);
    return bad ? 1 : 0;
}

/* ------------------------------------------------------------- drive */
#include <pthread.h>
static volatile int g_pump_quit = 0;

/* l'host: un blocco da 256 ogni 5,3 ms, come una DAW che suona */
static void *render_pump(void *arg)
{
    dt_rt *rt = (dt_rt *)arg;
    float buf[2][256], *o[2] = {buf[0], buf[1]};
    double t0 = now_s(), c = 0;
    while (!g_pump_quit) {
        dt_rt_render(rt, 256, o, 2, 0);
        c += 256;
        sleep_s(t0 + c / 48000.0 - now_s());
    }
    return NULL;
}

/* arcore drive --syx SYX --state F [--wav OUT.bin WAV] [--put WAV FOLDER] [--ls PATH]...:
   il client del +Drive (drive.c) sul motore in tempo reale, come il pannello +DRIVE */
static int cmd_drive(int argc, char **argv)
{
    const char *syx_path = NULL, *state = NULL;
    size_t len;
    uint8_t *syx;
    dt_rt *rt;
    char err[256] = "";
    int i, rc = 0;
    pthread_t pump;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx_path = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
    }
    if (!syx_path || !state || !(syx = read_file(syx_path, &len)))
        return 2;
    rt = dt_rt_new(syx, len, state, 48000, 256, err, sizeof err);
    if (!rt) {
        fprintf(stderr, "arcore: %s\n", err);
        return 1;
    }
    pthread_create(&pump, NULL, render_pump, rt);
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--ls") && i + 1 < argc) {
            dt_drive_entry *v = NULL;
            size_t n = 0, k;
            if (dt_drive_ls(rt, argv[++i], &v, &n, err, sizeof err)) { fprintf(stderr, "%s\n", err); rc = 1; continue; }
            printf("ls %s: %zu voci\n", argv[i], n);
            for (k = 0; k < n; k++)
                printf("  %c %8u %s\n", v[k].type, v[k].size, v[k].name);
            free(v);
        } else if (!strcmp(argv[i], "--wav") && i + 2 < argc) {
            size_t wl, bl;
            uint8_t *w = read_file(argv[i + 2], &wl), *b = NULL;
            if (!w || dt_wav2rytm(w, wl, &b, &bl, err, sizeof err)) { fprintf(stderr, "%s\n", err); rc = 1; }
            else { FILE *f = fopen(argv[i + 1], "wb"); fwrite(b, 1, bl, f); fclose(f); printf("wav %s -> %zu byte\n", argv[i + 2], bl); }
            free(w); free(b);
            i += 2;
        } else if (!strcmp(argv[i], "--put") && i + 2 < argc) {
            size_t wl, bl;
            uint8_t *w = read_file(argv[i + 1], &wl), *b = NULL;
            const char *base = strrchr(argv[i + 1], '/');
            char name[64], path[512];
            double t0 = now_s();
            snprintf(name, sizeof name, "%.32s", base ? base + 1 : argv[i + 1]);
            if (strrchr(name, '.')) *strrchr(name, '.') = 0;
            snprintf(path, sizeof path, "%s/%s", argv[i + 2], name);
            if (!w || dt_wav2rytm(w, wl, &b, &bl, err, sizeof err)
                || dt_drive_put(rt, path, b, bl, NULL, NULL, err, sizeof err)) { fprintf(stderr, "%s\n", err); rc = 1; }
            else printf("put %s (%zu byte) in %.2f s\n", path, bl, now_s() - t0);
            free(w); free(b);
            i += 2;
        } else if (!strcmp(argv[i], "--mkdir") && i + 1 < argc) {
            if (dt_drive_mkdir(rt, argv[++i], err, sizeof err)) { fprintf(stderr, "%s\n", err); rc = 1; }
            else printf("mkdir %s\n", argv[i]);
        } else if (!strcmp(argv[i], "--rmdir") && i + 1 < argc) {
            if (dt_drive_rmdir(rt, argv[++i], err, sizeof err)) { fprintf(stderr, "%s\n", err); rc = 1; }
            else printf("rmdir %s\n", argv[i]);
        } else if (!strcmp(argv[i], "--rm") && i + 1 < argc) {
            if (dt_drive_rm(rt, argv[++i], err, sizeof err)) { fprintf(stderr, "%s\n", err); rc = 1; }
            else printf("rm %s\n", argv[i]);
        }
    }
    g_pump_quit = 1;
    pthread_join(pump, NULL);
    dt_rt_free(rt);
    free(syx);
    return rc;
}

/* ------------------------------------------------------------ coldboot */
typedef struct {
    double t0, tphase, tlast;
    char phase[64], text[160];
    int calls, cancel_at;           /* --cancel-at K: alla K-esima chiamata di progress si interrompe */
    volatile int cancel;
} cb_ctx;

/* una riga per fase: il testo e' "fase: dettaglio"; a ogni cambio di fase si
   stampa l'ultimo dettaglio della precedente e il tempo che ha richiesto */
static void cb_progress(void *c0, double frac, const char *what)
{
    cb_ctx *c = (cb_ctx *)c0;
    char ph[64];
    if (c->cancel_at && ++c->calls >= c->cancel_at)
        c->cancel = 1;
    const char *colon = strchr(what, ':');
    size_t n = colon ? (size_t)(colon - what) : strlen(what);
    double t = now_s();
    if (n >= sizeof ph)
        n = sizeof ph - 1;
    memcpy(ph, what, n);
    ph[n] = 0;
    if (strcmp(ph, c->phase)) {
        if (c->phase[0])
            printf("  %-34s %7.1f s   (%s)\n", c->phase, t - c->tphase, c->text);
        snprintf(c->phase, sizeof c->phase, "%s", ph);
        c->tphase = t;
    }
    snprintf(c->text, sizeof c->text, "%s", colon ? colon + 1 + (colon[1] == ' ') : "");
    if (t - c->tlast >= 10.0) {
        c->tlast = t;
        fprintf(stderr, "  [%5.1f%%] %s\n", frac * 100.0, what);
    }
    fflush(stdout);
}

/* arcore coldboot --syx F.syx --out STATO.dtsz [--no-compress] [--point N] [--cancel-at K]
   il primo avvio dal solo firmware (dt_coldboot_session); con --point N solo
   il boot freddo fino a N istruzioni (dt_coldboot, lo stato del gradino) */
static int cmd_coldboot(int argc, char **argv)
{
    const char *syx = NULL, *outp = NULL;
    int compress = 1, i, r, cancel_at = 0;
    double point = 0, t0;
    dt_firmware fw;
    uint8_t *st = NULL;
    size_t n = 0;
    char err[256] = "", ver[32];
    cb_ctx c;
    FILE *f;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) outp = argv[++i];
        else if (!strcmp(argv[i], "--no-compress")) compress = 0;
        else if (!strcmp(argv[i], "--point") && i + 1 < argc) point = strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--cancel-at") && i + 1 < argc) cancel_at = atoi(argv[++i]);
        else {
            fprintf(stderr, "arcore coldboot: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !outp) {
        fprintf(stderr, "uso: arcore coldboot --syx F.syx --out STATO.dtsz [--no-compress] [--point N]\n");
        return 2;
    }
    if (load_fw(syx, &fw))
        return 1;
    i = dt_fw_supported(&fw, ver, sizeof ver);
    printf("firmware: OS %s, %s\n", ver[0] ? ver : "?", i ? "supportato" : "NON supportato");
    memset(&c, 0, sizeof c);
    c.cancel_at = cancel_at;
    c.t0 = c.tphase = c.tlast = t0 = now_s();
    if (point > 0)
        r = dt_coldboot(&fw, (uint64_t)point, compress, &st, &n, err, sizeof err);
    else
        r = dt_coldboot_session(&fw, compress, &st, &n, &c.cancel, cb_progress, &c, err, sizeof err);
    if (c.phase[0])
        printf("  %-34s %7.1f s   (%s)\n", c.phase, now_s() - c.tphase, c.text);
    if (r) {
        fprintf(stderr, "arcore: %s (%d)\n", err, r);
        dt_fw_free(&fw);
        return 1;
    }
    f = fopen(outp, "wb");
    if (!f || fwrite(st, 1, n, f) != n) {
        fprintf(stderr, "arcore: cannot write %s\n", outp);
        if (f)
            fclose(f);
        free(st);
        dt_fw_free(&fw);
        return 1;
    }
    fclose(f);
    printf("stato: %s, %zu byte, totale %.1f s\n", outp, n, now_s() - t0);
    free(st);
    dt_fw_free(&fw);
    return 0;
}

/* ---------------------------------------------------------------- shot */
#include <zlib.h>

static void png_chunk(FILE *f, const char *t, const uint8_t *d, uint32_t n)
{
    uint8_t h[8] = {(uint8_t)(n >> 24), (uint8_t)(n >> 16), (uint8_t)(n >> 8), (uint8_t)n, (uint8_t)t[0],
                    (uint8_t)t[1], (uint8_t)t[2], (uint8_t)t[3]};
    uLong c = crc32(0, h + 4, 4);
    uint8_t cb[4];
    fwrite(h, 1, 8, f);
    if (n) {
        fwrite(d, 1, n, f);
        c = crc32(c, d, n);
    }
    cb[0] = (uint8_t)(c >> 24); cb[1] = (uint8_t)(c >> 16); cb[2] = (uint8_t)(c >> 8); cb[3] = (uint8_t)c;
    fwrite(cb, 1, 4, f);
}

/* un'immagine in scala di grigi 8 bit -> PNG */
static int write_gray_png(const char *path, const uint8_t *g, int w, int h)
{
    int y;
    size_t rowlen = (size_t)w + 1;
    uint8_t *raw = (uint8_t *)malloc(rowlen * (size_t)h), *zb;
    uLongf zn;
    FILE *f;
    uint8_t ihdr[13] = {0, 0, (uint8_t)(w >> 8), (uint8_t)w, 0, 0, (uint8_t)(h >> 8), (uint8_t)h, 8, 0, 0, 0, 0};
    if (!raw)
        return -1;
    for (y = 0; y < h; y++) {
        raw[y * rowlen] = 0;
        memcpy(raw + y * rowlen + 1, g + (size_t)y * (size_t)w, (size_t)w);
    }
    zn = compressBound((uLong)(rowlen * (size_t)h));
    zb = (uint8_t *)malloc(zn);
    if (!zb || compress2(zb, &zn, raw, (uLong)(rowlen * (size_t)h), 6) != Z_OK) {
        free(raw);
        free(zb);
        return -1;
    }
    f = fopen(path, "wb");
    if (!f) {
        free(raw);
        free(zb);
        return -1;
    }
    fwrite("\x89PNG\r\n\x1a\n", 1, 8, f);
    png_chunk(f, "IHDR", ihdr, 13);
    png_chunk(f, "IDAT", zb, (uint32_t)zn);
    png_chunk(f, "IEND", NULL, 0);
    fclose(f);
    free(raw);
    free(zb);
    return 0;
}

/* lo schermo 128x64 (0/1) in un PNG a toni di grigio, ingrandito `z` volte */
static int write_png(const char *path, const uint8_t fb[128 * 64], int z)
{
    int w = 128 * z, h = 64 * z, x, y, r;
    uint8_t *g = (uint8_t *)malloc((size_t)w * (size_t)h);
    if (!g)
        return -1;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
            g[(size_t)y * (size_t)w + (size_t)x] = fb[(y / z) * 128 + x / z] ? 0xE8 : 0x10;
    r = write_gray_png(path, g, w, h);
    free(g);
    return r;
}

/* un foglio di schermate (2x, `cols` per riga, un filo di grigio fra l'una e l'altra) */
static int write_sheet(const char *path, const uint8_t *frames, int n, int cols)
{
    int rows = (n + cols - 1) / cols, cw = 256 + 6, ch = 128 + 6, w = cols * cw, h = rows * ch, i, x, y;
    uint8_t *g = (uint8_t *)malloc((size_t)w * (size_t)h);
    int r;
    if (!g)
        return -1;
    memset(g, 0x80, (size_t)w * (size_t)h);
    for (i = 0; i < n; i++)
        for (y = 0; y < 128; y++)
            for (x = 0; x < 256; x++)
                g[(size_t)((i / cols) * ch + y) * (size_t)w + (size_t)((i % cols) * cw + x)] =
                    frames[(size_t)i * 8192 + (size_t)(y / 2) * 128 + (size_t)(x / 2)] ? 0xE8 : 0x10;
    r = write_gray_png(path, g, w, h);
    free(g);
    return r;
}

/* arcore shot --syx SYX --state F.dtst --instrs N [--scenario S] --png OUT.png [--save OUT.dtst]
   [--ascii] [--png-every K DIR]
   [--sheet OUT.png COLS] [--every K]: il motore per N istruzioni (con gli ingressi
   dello scenario di `play`), poi lo schermo; con --png-every K DIR un PNG di
   ogni schermata nuova (al massimo uno ogni K istruzioni) */
static int cmd_shot(int argc, char **argv)
{
    const char *syx = NULL, *state = NULL, *scen = NULL, *png = NULL, *save = NULL, *dir = NULL, *sheet = NULL;
    uint8_t *frames = NULL;
    int nframes = 0, sheet_cols = 3;
    double limit_d = 0, every = 0;
    int ascii = 0, i, clockf = 0;
    int64_t clk[3] = {0, 0, 0}, last_png = -1;
    ev_t *evs = NULL;
    size_t nev = 0, iev = 0;
    dt_firmware fw;
    dt_engine *e;
    char err[256] = "";
    uint8_t fb[128 * 64];
    uint64_t ver = 0, lastver = 0;
    int nshot = 0;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
        else if (!strcmp(argv[i], "--instrs") && i + 1 < argc) limit_d = strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--scenario") && i + 1 < argc) scen = argv[++i];
        else if (!strcmp(argv[i], "--png") && i + 1 < argc) png = argv[++i];
        else if (!strcmp(argv[i], "--save") && i + 1 < argc) save = argv[++i];
        else if (!strcmp(argv[i], "--ascii")) ascii = 1;
        else if (!strcmp(argv[i], "--sheet") && i + 2 < argc) { sheet = argv[++i]; sheet_cols = atoi(argv[++i]); }
        else if (!strcmp(argv[i], "--every") && i + 1 < argc) every = strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--png-every") && i + 2 < argc) { every = strtod(argv[++i], NULL); dir = argv[++i]; }
        else {
            fprintf(stderr, "arcore shot: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state || limit_d <= 0) {
        fprintf(stderr, "uso: arcore shot --syx SYX --state F.dtst --instrs N [--scenario S] [--png OUT.png] "
                        "[--save OUT.dtst] [--ascii] [--png-every K DIR]\n");
        return 2;
    }
    if (scen && !(evs = read_scenario(scen, &nev, &clockf, clk)))
        return 1;
    if (load_fw(syx, &fw))
        return 1;
    e = dt_engine_new(&fw, state, NULL, err, sizeof err);
    if (!e) {
        fprintf(stderr, "arcore: %s\n", err);
        return 1;
    }
    for (;;) {
        int64_t n = dt_engine_instrs(e);
        while (iev < nev && evs[iev].at <= n) {
            ev_t *v = &evs[iev++];
            switch (v->kind) {
            case 0: dt_engine_midi_in(e, v->data, v->n); break;
            case 1: dt_engine_panel(e, DT_PANEL_PRESS, v->a, 0); break;
            case 2: dt_engine_panel(e, DT_PANEL_RELEASE, v->a, 0); break;
            case 3: dt_engine_panel(e, DT_PANEL_ENCODER, v->a, v->b); break;
            case 4: dt_engine_panel_raw(e, v->data, v->n); break;
            case 5: dt_engine_panel(e, DT_PANEL_PAD, v->a, v->b); break;
            default: dt_engine_panel(e, DT_PANEL_PAD, v->a, 0); break;
            }
        }
        if ((dir || sheet) && (ver = dt_engine_screen(e, fb)) != lastver && n - last_png >= (int64_t)every) {
            if (dir) {
                char path[4096];
                snprintf(path, sizeof path, "%s/s%04d_%dM.png", dir, nshot++, (int)(n / 1000000));
                write_png(path, fb, 3);
            }
            if (sheet && nframes < 90) {
                frames = (uint8_t *)realloc(frames, (size_t)(nframes + 1) * 8192);
                memcpy(frames + (size_t)nframes * 8192, fb, 8192);
                fprintf(stderr, "  schermata %d a %lld M istruzioni\n", nframes, (long long)(n / 1000000));
                nframes++;
            }
            lastver = ver;
            last_png = n;
        }
        if (n >= (int64_t)limit_d)
            break;
        if (dt_engine_chunk(e)) {
            fprintf(stderr, "arcore: motore fermo: %s\n", dt_engine_status(e));
            break;
        }
    }
    dt_engine_screen(e, fb);
    if (sheet && nframes && write_sheet(sheet, frames, nframes, sheet_cols))
        fprintf(stderr, "arcore: cannot write %s\n", sheet);
    free(frames);
    if (png && write_png(png, fb, 4))
        fprintf(stderr, "arcore: cannot write %s\n", png);
    if (ascii) {
        int x, y;
        for (y = 0; y < 64; y += 2) {
            for (x = 0; x < 128; x++)
                putchar(fb[y * 128 + x] && fb[(y + 1) * 128 + x] ? '#' : fb[y * 128 + x] ? '\'' : fb[(y + 1) * 128 + x] ? ',' : ' ');
            putchar('\n');
        }
    }
    fprintf(stderr, "arcore: %lld istruzioni, %s\n", (long long)dt_engine_instrs(e), dt_engine_status(e));
    if (save)
        dt_engine_save(e, save, 0);
    dt_engine_free(e);
    dt_fw_free(&fw);
    free(evs);
    return 0;
}

/* ------------------------------------------------------------ calibrate */
typedef struct {
    dt_engine *e;
    double t0;
    char last[96];
    int64_t i0;
} cal_ctx;

static void cal_progress(void *c0, double frac, const char *what)
{
    cal_ctx *c = (cal_ctx *)c0;
    if (strcmp(what, c->last)) {
        snprintf(c->last, sizeof c->last, "%s", what);
        printf("  %5.1f%%  %7.1f s  %8.1f M istr  %s\n", frac * 100.0, now_s() - c->t0,
               (double)(dt_engine_instrs(c->e) - c->i0) / 1e6, what);
        fflush(stdout);
    }
}

#define CAL_REC_LEN 0x1b072u
#define CAL_REC_ADDR 0x4196E0A8u
#define CAL_ERR_ADDR 0x42FB4224u

/* arcore calibrate --syx SYX --state IN.dtst --out OUT.dtst [--record F] [--ref F] */
static int cmd_calibrate(int argc, char **argv)
{
    const char *syx = NULL, *state = NULL, *out = NULL, *rec_out = NULL, *ref = NULL;
    dt_firmware fw;
    dt_engine *e;
    cal_ctx cc;
    char err[256] = "";
    uint8_t *rec, *errs;
    volatile int cancel = 0;
    double t0, t1;
    int i, rc;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--record") && i + 1 < argc) rec_out = argv[++i];
        else if (!strcmp(argv[i], "--ref") && i + 1 < argc) ref = argv[++i];
        else {
            fprintf(stderr, "arcore calibrate: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state) {
        fprintf(stderr, "uso: arcore calibrate --syx SYX --state IN.dtst [--out OUT.dtst] [--record REC.bin] [--ref REF.bin]\n");
        return 2;
    }
    if (load_fw(syx, &fw))
        return 1;
    e = dt_engine_new(&fw, state, NULL, err, sizeof err);
    if (!e) {
        fprintf(stderr, "arcore: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    memset(&cc, 0, sizeof cc);
    cc.e = e;
    cc.i0 = dt_engine_instrs(e);
    t0 = cc.t0 = now_s();
    rc = dt_engine_calibrate(e, &cancel, cal_progress, &cc, err, sizeof err);
    t1 = now_s();
    printf("calibrazione: %s%s%s  %.1f s reali, %.1f M istruzioni (%.1f s emulati)\n", rc ? "ERRORE " : "ok",
           rc ? err : "", "", t1 - t0, (double)(dt_engine_instrs(e) - cc.i0) / 1e6,
           (double)(dt_engine_instrs(e) - cc.i0) / 60e6);
    /* i record di errore dei controlli: lavoro, poi codici di 4 lettere */
    errs = (uint8_t *)malloc(0x3C * 14);
    if (errs && !dt_machine_mem_read(dt_engine_machine(e), CAL_ERR_ADDR, errs, 0x3C * 14)) {
        int nerr = 0;
        for (i = 0; i < 14; i++) {
            const uint8_t *r = errs + 0x3C * i;
            int k, any = 0;
            for (k = 0; k < 0x3C; k++) any |= r[k];
            if (!any) continue;                                              /* record vuoto: nessun lavoro */
            any = 0;
            for (k = 8; k < 0x3C; k++) any |= r[k];
            any |= !(r[4] == 0 && r[5] == 0 && r[6] == 0 && r[7] == 1);        /* word1 = 1: lavoro superato */
            if (!any) continue;
            nerr++;
            printf("  errore %d: lavoro %u:", i, (r[0] << 24) | (r[1] << 16) | (r[2] << 8) | r[3]);
            for (k = 1; k < 15; k++) {
                const uint8_t *w = r + 4 * k;
                if (!(w[0] | w[1] | w[2] | w[3])) continue;
                if (w[0] >= 0x20 && w[0] < 0x7f && w[1] >= 0x20 && w[1] < 0x7f)
                    printf(" %c%c%c%c", w[0], w[1], w[2], w[3]);
                else
                    printf(" %02x%02x%02x%02x", w[0], w[1], w[2], w[3]);
            }
            printf("\n");
        }
        printf("  record di errore: %d\n", nerr);
    }
    free(errs);
    rec = (uint8_t *)malloc(CAL_REC_LEN);
    if (rec && !dt_machine_mem_read(dt_engine_machine(e), CAL_REC_ADDR, rec, CAL_REC_LEN)) {
        uint32_t w10 = (rec[0x10] << 24) | (rec[0x11] << 16) | (rec[0x12] << 8) | rec[0x13];
        printf("  record RBOW: \"%.4s\" campo $10 = %08x (%s), %u byte\n", rec, w10, (w10 >> 16) ? "valido" : "NON valido",
               CAL_REC_LEN);
        if (rec_out) {
            FILE *f = fopen(rec_out, "wb");
            if (f) { fwrite(rec, 1, CAL_REC_LEN, f); fclose(f); }
        }
        if (ref) {
            size_t rl = 0;
            uint8_t *r2 = read_file(ref, &rl);
            if (!r2 || rl != CAL_REC_LEN)
                fprintf(stderr, "arcore: riferimento %s illeggibile o di misura diversa\n", ref);
            else {
                size_t nb = 0, nw = 0, k, first = (size_t)-1;
                long maxd = 0;
                for (k = 0; k < CAL_REC_LEN; k++)
                    if (rec[k] != r2[k]) { nb++; if (first == (size_t)-1) first = k; }
                for (k = 0; k + 3 < CAL_REC_LEN; k += 4) {
                    int32_t a = (int32_t)((rec[k] << 24) | (rec[k + 1] << 16) | (rec[k + 2] << 8) | rec[k + 3]);
                    int32_t b = (int32_t)((r2[k] << 24) | (r2[k + 1] << 16) | (r2[k + 2] << 8) | r2[k + 3]);
                    if (a != b) { long d = (long)a - (long)b; nw++; if (d < 0) d = -d; if (d > maxd) maxd = d; }
                }
                printf("  confronto col riferimento: %zu byte diversi su %u, %zu parole, scarto massimo %ld, primo a +%#zx\n",
                       nb, CAL_REC_LEN, nw, maxd, first == (size_t)-1 ? 0 : first);
            }
            free(r2);
        }
    }
    free(rec);
    if (out) {
        int sr = dt_engine_save(e, out, 0);
        printf("stato salvato in %s (%d)\n", out, sr);
    }
    {
        uint8_t scr[128 * 64];
        int x, y, lit = 0;
        dt_engine_screen(e, scr);
        for (y = 0; y < 64; y++)
            for (x = 0; x < 128; x++) lit += scr[y * 128 + x] != 0;
        printf("  schermo: %d pixel accesi, stato motore: %s\n", lit, dt_engine_status(e));
    }
    dt_engine_free(e);
    dt_fw_free(&fw);
    return rc ? 1 : 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "shot"))
        return cmd_shot(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "coldboot"))
        return cmd_coldboot(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "calibrate"))
        return cmd_calibrate(argc - 2, argv + 2);
    if (argc >= 2 && !strcmp(argv[1], "drive"))
        return cmd_drive(argc - 2, argv + 2);
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
        printf("arcore %s (Unicorn m68k)\n", dt_version());
        return 0;
    }
    fprintf(stderr, "uso: arcore sections|extract|run|play ... (vedi il sorgente)\n");
    return 2;
}
