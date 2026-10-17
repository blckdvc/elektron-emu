/* dsptool: prove dell'interprete del DSP (docs/DSP-INTERPRETE.md).
 *
 *   dsptool boot --syx F.syx --out S.dnds [--cycles N]     boot del DSP dalla sezione 7
 *   dsptool feed S.dnds FRAMES.bin DA N [--native]         N frame; hash di audio e memoria
 *   dsptool gensrc S.dnds OUTDIR                           i sorgenti C del DSP (traduttore del plugin)
 */
#include "dncore.h"
#include "dn_dsp.h"
#include "sha256.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static double now_s(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }

static uint8_t *read_file(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    uint8_t *p = NULL;
    long n;
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) == 0 && (n = ftell(f)) >= 0 && fseek(f, 0, SEEK_SET) == 0) {
        p = (uint8_t *)malloc(n ? (size_t)n : 1);
        if (p && fread(p, 1, (size_t)n, f) != (size_t)n) { free(p); p = NULL; }
        *len = (size_t)n;
    }
    fclose(f);
    return p;
}

static void prog(void *ctx, int pct, const char *what) { (void)ctx; fprintf(stderr, "\r%s %3d%%", what, pct); }

static int cmd_boot(int argc, char **argv)
{
    const char *syx = NULL, *out = NULL;
    int64_t cycles = 2000000000LL;
    size_t n;
    for (int i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "--syx") && i + 1 < argc) syx = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--cycles") && i + 1 < argc) cycles = atoll(argv[++i]);
    }
    if (!syx || !out) { fprintf(stderr, "uso: dsptool boot --syx F.syx --out S.dnds [--cycles N]\n"); return 2; }
    uint8_t *buf = read_file(syx, &n);
    char err[256] = "";
    dt_firmware fw;
    if (!buf || dt_fw_extract(buf, n, &fw, err, sizeof err)) { fprintf(stderr, "dsptool: %s\n", err); return 1; }
    const dt_section *s7 = dt_fw_section(&fw, 7);
    if (!s7) { fprintf(stderr, "dsptool: manca la sezione 7\n"); return 1; }
    uint8_t *dnds; size_t dlen;
    double t0 = now_s();
    if (dn_dsp_boot(s7->data, s7->len, cycles, NULL, prog, NULL, &dnds, &dlen, err, sizeof err)) { fprintf(stderr, "\ndsptool: %s\n", err); return 1; }
    fprintf(stderr, "\nboot del DSP: %.1f s, %zu byte\n", now_s() - t0, dlen);
    FILE *f = fopen(out, "wb");
    fwrite(dnds, 1, dlen, f);
    fclose(f);
    return 0;
}

static void hex(const uint8_t *h, char out[65]) { for (int i = 0; i < 32; i++) sprintf(out + 2 * i, "%02x", h[i]); }

static int cmd_feed(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "uso: dsptool feed S.dnds FRAMES.bin DA N [--native]\n"); return 2; }
    size_t sl, fl;
    uint8_t *st = read_file(argv[0], &sl), *fr = read_file(argv[1], &fl);
    long da = atol(argv[2]), count = atol(argv[3]);
    char err[256] = "";
    if (!st || !fr) { fprintf(stderr, "dsptool: file illeggibile\n"); return 1; }
    for (int i = 4; i < argc; i++)
        if (!strcmp(argv[i], "--native")) {
#ifdef DN_STATIC_DSP
            dn_dsp_set_ops(dnn_dsp_get_ops());
#else
            fprintf(stderr, "dsptool: build senza il DSP nativo\n"); return 1;
#endif
        } else if (!strcmp(argv[i], "--interp")) dn_dsp_set_ops(dni_dsp_get_ops());
        else if (!strcmp(argv[i], "--dylib") && i + 1 < argc) {
            if (dn_dsp_native_load(argv[++i], err, sizeof err)) { fprintf(stderr, "dsptool: %s\n", err); return 1; }
        }
    /* FRAMES.bin: <u32 BE lunghezza><frame>, ripetuto (dndsp_feed.c) */
    const uint8_t *pos[100000]; uint32_t lens[100000]; long nfr = 0;
    for (size_t o = 0; o + 4 <= fl && nfr < 100000;) {
        uint32_t n = (uint32_t)fr[o] << 24 | fr[o + 1] << 16 | fr[o + 2] << 8 | fr[o + 3];
        if (o + 4 + n > fl) break;
        pos[nfr] = fr + o + 4; lens[nfr] = n; nfr++; o += 4 + n;
    }
    dn_dsp *d = dn_dsp_new(st, sl, err, sizeof err);
    if (!d) { fprintf(stderr, "dsptool: %s\n", err); return 1; }
    fprintf(stderr, "motore: %s, %ld frame nel file\n", dn_dsp_backend_name(), nfr);
    dt_sha256 sa; uint8_t sum[32]; char hx[65];
    dt_sha256_init(&sa);
    static int32_t pcm[2 * 4096];
    uint64_t samples = 0;
    FILE *dump = getenv("DSP_DUMP") ? fopen(getenv("DSP_DUMP"), "wb") : NULL;
    double t0 = now_s();
    long analyze_at = getenv("DSP_ANALYZE_AT") ? atol(getenv("DSP_ANALYZE_AT")) : -1;
    for (long i = da; i < da + count && i < nfr; i++) {
        if (i - da == analyze_at) { double a0 = now_s(); dn_dsp_analyze(d); fprintf(stderr, "analisi: %.3f s\n", now_s() - a0); }
        int r = dn_dsp_frame(d, pos[i], lens[i]);
        if (r < 0) { fprintf(stderr, "frame %ld: %d %s\n", i, r, dn_dsp_error(d)); return 1; }
        int got = dn_dsp_pull(d, pcm, 4096);
        dt_sha256_update(&sa, (const uint8_t *)pcm, (size_t)got * 8);
        samples += (uint64_t)got;
        if (dump) fwrite(pcm, 8, (size_t)got, dump);
    }
    double dt = now_s() - t0;
    if (dump) fclose(dump);
    dt_sha256_final(&sa, sum); hex(sum, hx);
    printf("audio %s  campioni %llu  cicli %lld\n", hx, (unsigned long long)samples, (long long)dn_dsp_cycles(d));
    dt_sha256 sm; dt_sha256_init(&sm);
    for (int w = 0; w < 3; w++) { size_t n; const uint8_t *m = dn_dsp_memory(d, w, &n); dt_sha256_update(&sm, m, n); }
    dt_sha256_final(&sm, sum); hex(sum, hx);
    printf("memoria %s\n", hx);
    fprintf(stderr, "%.2f s, %.0f frame/s (servono 1500)\n", dt, count / dt);
    dn_dsp_free(d);
#ifdef DNI_PROF
    { extern void dni_prof_dump(void); dni_prof_dump(); }
#endif
    return 0;
}

static int gen_sink(void *ctx, const char *name, const char *data, size_t len)
{
    char path[1200];
    FILE *f;
    snprintf(path, sizeof path, "%s/%s", (const char *)ctx, name);
    f = fopen(path, "wb");
    if (!f) return -1;
    fwrite(data, 1, len, f);
    return fclose(f);
}

static int cmd_gensrc(int argc, char **argv)
{
    size_t n;
    uint8_t *st;
    char err[256] = "";
    double t0 = now_s();
    if (argc < 2) { fprintf(stderr, "uso: dsptool gensrc S.dnds OUTDIR\n"); return 2; }
    st = read_file(argv[0], &n);
    if (!st) { fprintf(stderr, "dsptool: file illeggibile\n"); return 1; }
    if (dn_dsp_gensrc(st, n, gen_sink, argv[1], err, sizeof err)) { fprintf(stderr, "dsptool: %s\n", err); return 1; }
    fprintf(stderr, "sorgenti scritti in %s (%.2f s)\n", argv[1], now_s() - t0);
    return 0;
}

double dni_bench(int kind, int spec, int64_t iters);
static int cmd_bench(void)
{
    static const char *names[] = {"NOP", "EXEC mulf", "EXEC addf", "OP_3 load", "OP_3 load+mulf", "5MOVE R3=R4", "OP_4 load", "MEM15 load"};
    for (int k = 0; k < 8; k++) printf("%-16s generico %.2f ns   specializzato %.2f ns\n", names[k], dni_bench(k, 0, 2000000), dni_bench(k, 1, 2000000));
    printf("misto (200 istr, 7 tipi a caso) generico %.2f ns   specializzato %.2f ns\n", dni_bench(100, 0, 200000), dni_bench(100, 1, 200000));
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "uso: dsptool boot|feed ...\n"); return 2; }
    if (!strcmp(argv[1], "boot")) return cmd_boot(argc - 2, argv + 2);
    if (!strcmp(argv[1], "bench")) return cmd_bench();
    if (!strcmp(argv[1], "feed")) return cmd_feed(argc - 2, argv + 2);
    if (!strcmp(argv[1], "gensrc")) return cmd_gensrc(argc - 2, argv + 2);
    fprintf(stderr, "dsptool: comando sconosciuto\n");
    return 2;
}
