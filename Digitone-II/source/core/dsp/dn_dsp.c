/* DSP del Digitone II senza Python: vedi dn_dsp.h e docs/DSP-EMULATOR.md ("Il DSP senza
 * Python per il plugin"). C11; si compila con -I <cartella dei sorgenti generati> (rt.h,
 * dn_key.h) e si lega con dn_rtmain.c, i dsp_NNN.c generati e -lz.
 *
 * dnn_dsp_frame riproduce DspSynth.on_frame(live=False) / Engine.run_native: il frame
 * aspetta in `pending`; rt_run ritorna 5 quando il DSP vuole un frame: la prima volta lo si
 * spinge, la seconda ci si ferma (il chiamante dara' il prossimo); 2 = limite, 4 = errore.
 */
#include "dn_dsp.h"   /* dn_dsp_ops */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef DN_EXT_INFLATE
/* libreria costruita dal plugin: niente zlib da collegare, il plugin passa la sua uncompress
   (dnn_set_uncompress, subito dopo il caricamento) */
typedef int (*dnn_uncompress_fn)(unsigned char *dst, unsigned long *dstlen, const unsigned char *src, unsigned long srclen);
static dnn_uncompress_fn g_uncompress;
#define Z_OK 0
typedef unsigned long uLongf;
#define uncompress(d, dl, s, sl) (g_uncompress ? g_uncompress((d), (dl), (s), (sl)) : -1)
#else
#include <zlib.h>
#endif

#include "rt.h"
#include "dn_key.h"

/* dsp_main.c (via dn_rtmain.c) e periph.h */
int rt_run(St *st, int64_t limit);
int rt_push_frame(St *st, const uint8_t *fr, uint32_t n);
int rt_pull_audio(St *st, int32_t *out, int max);
uint32_t rt_reply(St *st, uint8_t *out, uint32_t n);
void *rt_periph_new(void);
size_t dn__seg_bytes(void);
size_t dn__st_size(void);
size_t dn__periph_size(void);
void dn__init_seg(St *st, void *tab);
int dn__periph_nsparse(void *per);
uint32_t dn__periph_sparse_base(void *per, int k);
uint8_t **dn__periph_sparse_slot(void *per, int k);

typedef struct dnn_dsp dnn_dsp;
dnn_dsp *dnn_dsp_new(const uint8_t *dnds, size_t len, char *err, size_t errlen);
void dnn_dsp_free(dnn_dsp *d);
int dnn_dsp_frame(dnn_dsp *d, const uint8_t *frame, size_t n);
int dnn_dsp_pull(dnn_dsp *d, int32_t *out, int max);
uint32_t dnn_dsp_reply(dnn_dsp *d, uint8_t *out, uint32_t n);
const char *dnn_dsp_error(const dnn_dsp *d);
int64_t dnn_dsp_cycles(const dnn_dsp *d);
const char *dnn_dsp_key(void);
const uint8_t *dnn_dsp_memory(const dnn_dsp *d, int which, size_t *len);
const dn_dsp_ops *dnn_dsp_get_ops(void);

#define DN_L1_SIZE 0x400000u
#define DN_L2_SIZE 0x100000u
#define DN_DDR_SIZE 0x4000000u
#define DN_SPARSE 64
#define DN_PAGE 0x10000u

struct dnn_dsp {
    St st;
    void *per;                       /* Periph (periph.h), malloc */
    uint8_t *l1, *l2, *ddr;
    void *seg;                       /* Seg[65536] di questa istanza */
    char err[160];
};

static void seterr(char *err, size_t n, const char *fmt, ...) {
    if (!err || !n) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(err, n, fmt, ap);
    va_end(ap);
}

static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

typedef struct { const uint8_t *p, *end; } Rd;

/* il prossimo blocco, che deve avere il tag `tag` e (se raw > 0) quella lunghezza; lo
 * scrive in dst (raw_len byte). 0 = ok */
static int chunk(Rd *r, const char *tag, uint8_t *dst, size_t raw, char *err, size_t errlen) {
    if (r->end - r->p < 16) { seterr(err, errlen, "dnds: manca il blocco %.4s", tag); return -1; }
    const uint8_t *h = r->p;
    uint32_t flags = rd32(h + 4), rlen = rd32(h + 8), slen = rd32(h + 12);
    if (memcmp(h, tag, 4)) { seterr(err, errlen, "dnds: blocco %.4s al posto di %.4s", (const char *)h, tag); return -1; }
    if ((size_t)(r->end - r->p - 16) < slen) { seterr(err, errlen, "dnds: blocco %.4s troncato", tag); return -1; }
    if (rlen != raw) { seterr(err, errlen, "dnds: blocco %.4s di %u byte, attesi %zu", tag, rlen, raw); return -1; }
    const uint8_t *src = h + 16;
    if (flags & 1) {
        uLongf got = (uLongf)raw;
        if (uncompress(dst, &got, src, slen) != Z_OK || got != raw) {
            seterr(err, errlen, "dnds: blocco %.4s: zlib", tag);
            return -1;
        }
    } else {
        if (slen != raw) { seterr(err, errlen, "dnds: blocco %.4s: lunghezze", tag); return -1; }
        memcpy(dst, src, raw);
    }
    r->p = src + slen;
    return 0;
}

const char *dnn_dsp_key(void) { return DN_DSP_KEY; }

#ifdef DN_EXT_INFLATE
#if defined(_WIN32)
#define DN_EXPORT __declspec(dllexport)
#else
#define DN_EXPORT __attribute__((visibility("default")))
#endif
DN_EXPORT void dnn_set_uncompress(dnn_uncompress_fn f) { g_uncompress = f; }
#endif

dnn_dsp *dnn_dsp_new(const uint8_t *buf, size_t len, char *err, size_t errlen) {
    seterr(err, errlen, "%s", "");
    if (len < 48 || memcmp(buf, "DNDS", 4)) { seterr(err, errlen, "dnds: non e' un file DNDS"); return 0; }
    if (rd32(buf + 4) != 1) { seterr(err, errlen, "dnds: versione %u, attesa 1", rd32(buf + 4)); return 0; }
    if (memcmp(buf + 8, DN_DSP_KEY, 16)) {
        seterr(err, errlen, "dnds: stato fatto con i sorgenti %.16s, la libreria e' %s", (const char *)buf + 8, DN_DSP_KEY);
        return 0;
    }
    uint32_t st_size = rd32(buf + 24), per_size = rd32(buf + 28);
    if (st_size != dn__st_size() || st_size != sizeof(St) || per_size != dn__periph_size()) {
        seterr(err, errlen, "dnds: St %u / Periph %u byte, la libreria %zu / %zu", st_size, per_size,
               dn__st_size(), dn__periph_size());
        return 0;
    }
    if (rd32(buf + 32) != DN_L1_SIZE || rd32(buf + 36) != DN_L2_SIZE || rd32(buf + 40) != DN_DDR_SIZE) {
        seterr(err, errlen, "dnds: dimensioni della memoria diverse");
        return 0;
    }
    dnn_dsp *d = calloc(1, sizeof *d);
    if (!d) { seterr(err, errlen, "memoria"); return 0; }
    d->per = rt_periph_new();
    d->l1 = malloc(DN_L1_SIZE);
    d->l2 = malloc(DN_L2_SIZE);
    d->ddr = malloc(DN_DDR_SIZE);
    d->seg = malloc(dn__seg_bytes());
    if (!d->per || !d->l1 || !d->l2 || !d->ddr || !d->seg) { seterr(err, errlen, "memoria"); goto fail; }
    Rd r = {buf + 48, buf + len};
    if (chunk(&r, "ST  ", (uint8_t *)&d->st, sizeof(St), err, errlen)) goto fail;
    if (chunk(&r, "PER ", (uint8_t *)d->per, per_size, err, errlen)) goto fail;
    for (int k = 0; k < DN_SPARSE; k++) *dn__periph_sparse_slot(d->per, k) = 0;   /* dal file: a zero */
    int ns = dn__periph_nsparse(d->per);
    if (ns < 0 || ns > DN_SPARSE) { seterr(err, errlen, "dnds: %d pagine sparse", ns); goto fail; }
    for (int k = 0; k < ns; k++) {
        uint8_t *tmp = malloc(4 + DN_PAGE);
        if (!tmp) { seterr(err, errlen, "memoria"); goto fail; }
        if (chunk(&r, "SPG ", tmp, 4 + DN_PAGE, err, errlen)) { free(tmp); goto fail; }
        if (rd32(tmp) != dn__periph_sparse_base(d->per, k)) {
            seterr(err, errlen, "dnds: pagina sparsa %d a %#x, attesa %#x", k, rd32(tmp), dn__periph_sparse_base(d->per, k));
            free(tmp);
            goto fail;
        }
        memmove(tmp, tmp + 4, DN_PAGE);                    /* la pagina (64 KB) dall'inizio */
        *dn__periph_sparse_slot(d->per, k) = tmp;
    }
    if (chunk(&r, "L1  ", d->l1, DN_L1_SIZE, err, errlen)) goto fail;
    if (chunk(&r, "L2  ", d->l2, DN_L2_SIZE, err, errlen)) goto fail;
    if (chunk(&r, "DDR ", d->ddr, DN_DDR_SIZE, err, errlen)) goto fail;
    if (r.p != r.end) { seterr(err, errlen, "dnds: %td byte in piu' in fondo", r.end - r.p); goto fail; }
    if (d->st.ddr_size != DN_DDR_SIZE) { seterr(err, errlen, "dnds: ddr_size %u", d->st.ddr_size); goto fail; }
    /* i puntatori, come Engine.__init__ + go_native */
    d->st.l1 = d->l1;
    d->st.l2 = d->l2;
    d->st.ddr = d->ddr;
    d->st.mmio_rd = 0;                                     /* con per != 0 il C non le chiama */
    d->st.mmio_wr = 0;
    d->st.per = d->per;
    dn__init_seg(&d->st, d->seg);
    return d;
fail:
    dnn_dsp_free(d);
    return 0;
}

void dnn_dsp_free(dnn_dsp *d) {
    if (!d) return;
    if (d->per) {
        int ns = dn__periph_nsparse(d->per);           /* anche le pagine nate durante l'esecuzione */
        if (ns < 0 || ns > DN_SPARSE) ns = DN_SPARSE;
        for (int k = 0; k < ns; k++) free(*dn__periph_sparse_slot(d->per, k));
        free(d->per);
    }
    free(d->l1);
    free(d->l2);
    free(d->ddr);
    free(d->seg);
    free(d);
}

int dnn_dsp_frame(dnn_dsp *d, const uint8_t *frame, size_t n) {
    St *st = &d->st;
    int pending = 1;
    int64_t target = st->cycles + ((int64_t)1 << 40);
    while (st->cycles < target) {
        int r = rt_run(st, target);
        if (r == 5) {                                   /* il DSP vuole un frame */
            if (!pending) return 0;
            pending = 0;
            rt_push_frame(st, frame, (uint32_t)n);      /* come Engine.push_frame (esito ignorato) */
        } else if (r == 2) {
            break;
        } else if (r == 4) {
            snprintf(d->err, sizeof d->err, "errore del motore C: err_code %#x a pc %#x", st->err_code, st->pc);
            return -4;
        } else {
            snprintf(d->err, sizeof d->err, "rt_run inatteso: %d (pc %#x)", r, st->pc);
            return r == 3 ? -3 : -1;
        }
    }
    return 1;
}

int dnn_dsp_pull(dnn_dsp *d, int32_t *out, int max) { return rt_pull_audio(&d->st, out, max); }

uint32_t dnn_dsp_reply(dnn_dsp *d, uint8_t *out, uint32_t n) { return rt_reply(&d->st, out, n); }

const char *dnn_dsp_error(const dnn_dsp *d) { return d->err; }

int64_t dnn_dsp_cycles(const dnn_dsp *d) { return d->st.cycles; }

const uint8_t *dnn_dsp_memory(const dnn_dsp *d, int which, size_t *len) {
    switch (which) {
    case 0: if (len) *len = DN_L1_SIZE; return d->l1;
    case 1: if (len) *len = DN_L2_SIZE; return d->l2;
    case 2: if (len) *len = DN_DDR_SIZE; return d->ddr;
    }
    if (len) *len = 0;
    return 0;
}

/* il backend nativo (libdndsp.a, o la libreria fatta sulla macchina dell'utente) per il fronte
   di dn_dspback.c; i puntatori a funzione tolgono ogni dipendenza di link dalla libreria */
static void *ops_create(const uint8_t *b, size_t n, char *err, size_t el) { return dnn_dsp_new(b, n, err, el); }
static void ops_destroy(void *d) { dnn_dsp_free((dnn_dsp *)d); }
static int ops_frame(void *d, const uint8_t *f, size_t n) { return dnn_dsp_frame((dnn_dsp *)d, f, n); }
static int ops_pull(void *d, int32_t *o, int m) { return dnn_dsp_pull((dnn_dsp *)d, o, m); }
static uint32_t ops_reply(void *d, uint8_t *o, uint32_t n) { return dnn_dsp_reply((dnn_dsp *)d, o, n); }
static const char *ops_error(const void *d) { return dnn_dsp_error((const dnn_dsp *)d); }
static int64_t ops_cycles(const void *d) { return dnn_dsp_cycles((const dnn_dsp *)d); }
static const uint8_t *ops_memory(const void *d, int w, size_t *l) { return dnn_dsp_memory((const dnn_dsp *)d, w, l); }
static const dn_dsp_ops g_ops = {"nativo", DN_DSP_KEY, ops_create, ops_destroy, ops_frame, ops_pull, ops_reply,
                                 ops_error, ops_cycles, ops_memory, 0};
#ifdef DN_EXT_INFLATE
DN_EXPORT
#endif
const dn_dsp_ops *dnn_dsp_get_ops(void) { return &g_ops; }
