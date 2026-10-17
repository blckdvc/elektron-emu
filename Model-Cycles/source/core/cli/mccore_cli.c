/* mccore: riga di comando di prova della libreria del Model:Cycles.
 *
 *   mccore sections SYX              una riga JSON per sezione (id, nome, sha256)
 *   mccore extract SYX OUTDIR        come `python -m emu.extract SYX -o OUTDIR`
 *   mccore run --syx SYX --state F.dtst --count N
 *                                    la macchina di base: N istruzioni esatte
 *   mccore play --syx SYX --state F.dtst --instrs N [--scenario S] [--trace OUT]
 *               [--ram-every K] [--audio OUT.f32] [--save OUT.dtst]
 *                                    il thread del motore della GUI, con la
 *                                    traccia di tools/mccore_ref.py
 *   mccore rt-test --syx SYX --state F.dtst [--seconds S] [--rate R] [--frames N]
 *               [--realtime] [--dump OUT.f32]
 *                                    il motore in tempo reale (dt_rt): blocchi
 *                                    fissi = casuali = motore diretto
 */
#include "mccore.h"
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
        fprintf(stderr, "mccore: cannot read %s: %s\n", path, strerror(errno));
        return -1;
    }
    r = dt_fw_extract(syx, len, fw, err, sizeof err);
    free(syx);
    if (r) {
        fprintf(stderr, "mccore: %s: %s\n", path, err);
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
        fprintf(stderr, "uso: mccore sections SYX\n");
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
        fprintf(stderr, "uso: mccore extract SYX OUTDIR\n");
        return 2;
    }
    if (load_fw(argv[0], &fw))
        return 1;
    for (i = 0; i < fw.count; i++) {
        snprintf(path, sizeof path, "%s/section_%u_%s.bin", argv[1], fw.sec[i].id,
                 dt_fw_section_name(fw.sec[i].id));
        f = fopen(path, "wb");
        if (!f || fwrite(fw.sec[i].data, 1, fw.sec[i].len, f) != fw.sec[i].len) {
            fprintf(stderr, "mccore: cannot write %s\n", path);
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
    dt_machine_config cfg = {20000};
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
            fprintf(stderr, "mccore run: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state) {
        fprintf(stderr, "uso: mccore run --syx SYX --state F.dtst --count N [--idle-yield N]\n");
        return 2;
    }
    if (load_fw(syx, &fw))
        return 1;
    m = dt_machine_new(&fw, &cfg, err, sizeof err);
    if (!m) {
        fprintf(stderr, "mccore: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    t0 = now_s();
    if ((r = dt_machine_load_state(m, state, err, sizeof err)) != 0) {
        fprintf(stderr, "mccore: %s: %s\n", state, err);
        dt_machine_free(m);
        dt_fw_free(&fw);
        return 1;
    }
    t1 = now_s();
    fprintf(stderr, "mccore: stato caricato in %.2f s\n", t1 - t0);
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
    fprintf(stderr, "mccore: %llu istruzioni in %.3f s (%.1f M istr/s), %llu eccezioni, %llu RTE, "
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
    int kind;                     /* 0 midi, 1 press, 2 release, 3 turn, 4 pad */
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

static void file_sink(void *ctx, const float *lr, size_t n)
{
    fwrite(lr, sizeof(float), 2 * n, (FILE *)ctx);
}

/* lo scenario di tools/mccore_ref.py */
static ev_t *read_scenario(const char *scen, size_t *pnev, int *have_clock, int64_t clk[3])
{
    char line[1024];
    ev_t *evs = NULL;
    size_t nev = 0, cap = 0;
    FILE *f = fopen(scen, "r");
    if (!f) {
        fprintf(stderr, "mccore: cannot read %s\n", scen);
        return NULL;
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
            *have_clock = 1;
            clk[0] = (int64_t)strtod(w[1], NULL);
            clk[1] = (int64_t)strtod(w[2], NULL);
            clk[2] = (int64_t)strtod(w[3], NULL);
        } else if (!strcmp(w[0], "at") && nw >= 4) {
            ev_t ev;
            memset(&ev, 0, sizeof ev);
            ev.at = (int64_t)strtod(w[1], NULL);
            if (!strcmp(w[2], "midi")) {
                size_t k;
                for (k = 0; w[3][2 * k] && w[3][2 * k + 1] && k < sizeof ev.data; k++)
                    ev.data[k] = (uint8_t)(hexval(w[3][2 * k]) << 4 | hexval(w[3][2 * k + 1]));
                ev.n = k;
            } else if (!strcmp(w[2], "press")) {
                ev.kind = 1; ev.a = (int)strtol(w[3], NULL, 0);
            } else if (!strcmp(w[2], "release")) {
                ev.kind = 2; ev.a = (int)strtol(w[3], NULL, 0);
            } else if (!strcmp(w[2], "turn") && nw >= 5) {
                ev.kind = 3; ev.a = (int)strtol(w[3], NULL, 0); ev.b = atoi(w[4]);
            } else if (!strcmp(w[2], "pad") && nw >= 5) {
                ev.kind = 4; ev.a = atoi(w[3]); ev.b = atoi(w[4]);
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

static int cmd_play(int argc, char **argv)
{
    const char *syx = NULL, *state = NULL, *scen = NULL, *trace = NULL, *save = NULL, *audio = NULL;
    double limit_d = 0;
    int64_t limit, clk[3] = {0, 0, 0}, clk_next = 0;
    int ram_every = 25, i, have_clock = 0;
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
            fprintf(stderr, "mccore play: argomento sconosciuto %s\n", argv[i]);
            return 2;
        }
    }
    if (!syx || !state || limit_d <= 0) {
        fprintf(stderr, "uso: mccore play --syx SYX --state F.dtst --instrs N [--scenario S] [--trace OUT]"
                        " [--ram-every K] [--audio OUT.f32] [--save OUT.dtst]\n");
        return 2;
    }
    limit = (int64_t)limit_d;
    if (scen && !(evs = read_scenario(scen, &nev, &have_clock, clk)))
        return 1;
    clk_next = clk[0];
    if (trace && !(out = fopen(trace, "w"))) {
        fprintf(stderr, "mccore: cannot write %s\n", trace);
        return 1;
    }
    if (load_fw(syx, &fw))
        return 1;
    t0 = now_s();
    e = dt_engine_new(&fw, state, NULL, err, sizeof err);
    if (!e) {
        fprintf(stderr, "mccore: %s\n", err);
        dt_fw_free(&fw);
        return 1;
    }
    if (audio) {
        af = fopen(audio, "wb");
        if (!af) {
            fprintf(stderr, "mccore: cannot write %s\n", audio);
            return 1;
        }
        dt_engine_set_audio_sink(e, file_sink, af);
        dt_engine_set_volume(e, 1.0);
    }
    t1 = now_s();
    fprintf(stderr, "mccore: motore pronto in %.2f s\n", t1 - t0);
    ram_line(out, e, "init");
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
            break;
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
            else
                dt_engine_panel(e, DT_PANEL_PAD, v->a, v->b);
        }
        if (have_clock) {
            static const uint8_t f8 = 0xF8;
            while (clk_next <= n && clk_next <= clk[2]) {
                dt_engine_midi_in(e, &f8, 1);
                clk_next += clk[1];
            }
        }
        chunk++;
        if (dt_engine_chunk(e))
            break;
    }
    t1 = now_s();
    dt_engine_trace(e, "end", chunk - 1, line, sizeof line);
    fprintf(out, "%s\n", line);
    ram_line(out, e, "end");
    fprintf(stderr, "mccore: %lld istruzioni in %.2f s (%.1f M istr/s, %.0f%% del tempo reale), %s\n",
            (long long)dt_engine_instrs(e), t1 - t0, dt_engine_instrs(e) / (t1 - t0) / 1e6,
            dt_engine_instrs(e) / 40e6 / (t1 - t0) * 100, dt_engine_status(e));
    if (out != stdout)
        fclose(out);
    if (af)
        fclose(af);
    if (save) {
        int r = dt_engine_save(e, save, 0);
        fprintf(stderr, "mccore: stato salvato in %s (%d)\n", save, r);
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

static void collect_sink(void *ctx, const float *lr, size_t n)
{
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
   note sui canali 1-6 (le tracce del Cycles), un note off ciascuna */
static int g_notes_only = 0;                 /* --notes-only: senza START e clock (i plugin hanno la loro Sync DAW) */
static double g_volume = 1.0;                /* --volume */

static size_t rt_events(rtev *ev, size_t cap, int64_t H)
{
    size_t n = 0;
    int64_t h;
    if (!g_notes_only) {
        ev[n].h = 4800; ev[n].d[0] = 0xFA; ev[n].n = 1; n++;
        for (h = 4800; h < H - 4800 && n < cap - 80; h += 1000) {
            ev[n].h = h; ev[n].d[0] = 0xF8; ev[n].n = 1; n++;
        }
    }
    for (h = 9600; h < H && n < cap - 2; h += 7919) {
        uint8_t ch = (uint8_t)((h / 7919) % 6), note = (uint8_t)(48 + h % 24);
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
    float *out, *o[2], *tmp[2];
    static rtev ev[8192];
    size_t nev = rt_events(ev, 8192, (int64_t)(H * 48000.0 / rate)), iev = 0;
    int64_t c = 0, i;
    unsigned seed = 12345;
    if (!rt) {
        fprintf(stderr, "mccore: %s\n", err);
        return NULL;
    }
    dt_rt_set_volume(rt, g_volume);
    dt_rt_offsets(rt, E, X);
    out = (float *)calloc((size_t)H * 2, sizeof(float));
    tmp[0] = (float *)calloc(4096, sizeof(float));
    tmp[1] = (float *)calloc(4096, sizeof(float));
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
        o[0] = tmp[0];
        o[1] = tmp[1];
        if (g_rt_realtime) {                      /* l'host: un blocco quando tocca */
            sleep_s(t0 + (double)c / rate - now_s());
            dt_rt_render(rt, n, o, 2, 0);
        } else
            dt_rt_render(rt, n, o, 2, 1);
        for (i = 0; i < n; i++) {
            out[2 * (c + i)] = tmp[0][i];
            out[2 * (c + i) + 1] = tmp[1][i];
        }
        c += n;
    }
    *underruns = dt_rt_underruns(rt);
    fprintf(stderr, "rt: %s; ritardo massimo %lld\n", dt_rt_status(rt), (long long)dt_rt_max_lag(rt));
    dt_rt_free(rt);
    free(tmp[0]);
    free(tmp[1]);
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
        else if (!strcmp(argv[i], "--notes-only")) g_notes_only = 1;
        else if (!strcmp(argv[i], "--volume") && i + 1 < argc) g_volume = atof(argv[++i]);
    }
    if (!syx_path || !state) {
        fprintf(stderr, "uso: mccore rt-test --syx SYX --state F.dtst [--seconds S] [--rate R] [--frames N]"
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
    if (rate == 48000.0) {
        /* il motore diretto, gli eventi agli stessi confini */
        int64_t bad2 = 0, first2 = -1;
        if (load_fw(syx_path, &fw))
            return 1;
        e = dt_engine_new(&fw, state, NULL, err, sizeof err);
        if (!e) {
            fprintf(stderr, "mccore: %s\n", err);
            return 1;
        }
        dt_engine_set_audio_sink(e, collect_sink, &col);
        dt_engine_set_volume(e, g_volume);
        nev = rt_events(ev, 8192, H);
        while ((int64_t)col.n < H + E) {
            int64_t t48 = dt_engine_instrs(e) * 3 / 2500;
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

/* ----------------------------------------------------------- coldboot */
/* mccore coldboot --syx SYX --out F.dtst [--point N] [--session INSTRS] [--compress]
   --point: solo il boot freddo fino a N istruzioni dal reset (come cold200M.snap, predefinito 200M);
   --session: poi il motore per INSTRS istruzioni (lo stato del plugin, dt_coldboot_session) */
static int cmd_coldboot(int argc, char **argv)
{
    const char *syx = NULL, *out = NULL;
    uint64_t point = DT_COLDBOOT_RUNG;
    int64_t session = 0;
    int compress = 0, i;
    dt_firmware fw;
    uint8_t *buf = NULL;
    size_t n = 0;
    char err[256] = "";
    int r;
    struct timespec t0, t1;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--point") && i + 1 < argc) point = (uint64_t)strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--session") && i + 1 < argc) session = (int64_t)strtod(argv[++i], NULL);
        else if (!strcmp(argv[i], "--compress")) compress = 1;
    }
    if (!syx || !out) {
        fprintf(stderr, "uso: mccore coldboot --syx SYX --out F.dtst [--point N] [--session INSTRS] [--compress]\n");
        return 2;
    }
    if (load_fw(syx, &fw))
        return 1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    if (session > 0)
        r = dt_coldboot_session(&fw, point, session, compress, &buf, &n, NULL, NULL, err, sizeof err);
    else
        r = dt_coldboot(&fw, point, compress, &buf, &n, NULL, NULL, err, sizeof err);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    dt_fw_free(&fw);
    if (r) {
        fprintf(stderr, "mccore coldboot: %s (%d)\n", err, r);
        return 1;
    }
    {
        FILE *f = fopen(out, "wb");
        if (!f || fwrite(buf, 1, n, f) != n) {
            fprintf(stderr, "mccore coldboot: cannot write %s\n", out);
            return 1;
        }
        fclose(f);
    }
    printf("coldboot: %llu istruzioni%s, %zu byte, %.1f s\n", (unsigned long long)point,
           session > 0 ? " + sessione" : "", n,
           (double)(t1.tv_sec - t0.tv_sec) + 1e-9 * (double)(t1.tv_nsec - t0.tv_nsec));
    free(buf);
    return 0;
}

/* ------------------------------------------------------------- show */
/* mccore show --syx SYX --state F.dtst [--instrs N]: carica lo stato, esegue N istruzioni (giri da
   5 ms), stampa lo schermo OLED in ASCII e lo stato del +Drive (firma 0xBEEFBACE, archivio preset) */
static uint32_t mem32(dt_engine *e, uint32_t a)
{
    uint8_t b[4] = {0, 0, 0, 0};
    dt_machine_mem_read(dt_engine_machine(e), a, b, 4);
    return (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
}

static int cmd_show(int argc, char **argv)
{
    const char *syx = NULL, *state = NULL;
    int64_t limit = 0;
    int i, x, y;
    dt_firmware fw;
    char err[256] = "";
    dt_engine *e;
    uint8_t scr[128 * 64];
    uint32_t fs;
    for (i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
        else if (!strcmp(argv[i], "--instrs") && i + 1 < argc) limit = (int64_t)strtod(argv[++i], NULL);
    }
    if (!syx || !state) {
        fprintf(stderr, "uso: mccore show --syx SYX --state F.dtst [--instrs N]\n");
        return 2;
    }
    if (load_fw(syx, &fw))
        return 1;
    e = dt_engine_new(&fw, state, NULL, err, sizeof err);
    if (!e) {
        fprintf(stderr, "mccore show: %s\n", err);
        return 1;
    }
    while (dt_engine_instrs(e) < limit)
        if (dt_engine_chunk(e)) {
            fprintf(stderr, "mccore show: %s\n", dt_engine_status(e));
            return 1;
        }
    dt_engine_screen(e, scr);
    for (y = 0; y < 64; y += 2) {
        for (x = 0; x < 128; x++) {
            int up = scr[y * 128 + x], dn = scr[(y + 1) * 128 + x];
            fputs(up && dn ? "#" : up ? "'" : dn ? "." : " ", stdout);
        }
        fputc('\n', stdout);
    }
    fs = mem32(e, 0x40fe4248u);
    printf("istruzioni %lld, +Drive: firma %s, archivio preset %s\n", (long long)dt_engine_instrs(e),
           fs && mem32(e, fs + 0x14ab4u) == 0xBEEFBACEu ? "ok" : "NO",
           mem32(e, 0x40f07cf4u) ? "aperto" : "CHIUSO");
    dt_engine_free(e);
    dt_fw_free(&fw);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "show"))
        return cmd_show(argc - 2, argv + 2);
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
        printf("mccore %s (Unicorn m68k)\n", dt_version());
        return 0;
    }
    fprintf(stderr, "uso: mccore sections|extract|run|play|rt-test ... (vedi il sorgente)\n");
    return 2;
}
