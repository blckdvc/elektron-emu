/* COPIA di emu/sharc/rec/rt.h per l'interprete in C del plugin (core/dsp/dn_interp.c,
 * docs/DSP-INTERPRETE.md): stessa semantica e stessi layout di St e Periph (gli stati DNDS sono
 * intercambiabili col DSP ricompilato), con solo le modifiche di portabilita' segnate "PORT"
 * (x86-64 oltre ad arm64, nessun thread dal vivo ne' profiler). Non cambiare la semantica qui
 * senza cambiarla anche nell'originale (e viceversa).
 *
 * Runtime del SHARC+ ricompilato (docs/DSP-EMULATOR.md, "Ricompilazione statica").
 *
 * Porta 1:1 di emu/sharc/core.py, forms.py e compute.py: stesso stato, stessa
 * semantica, stessi arrotondamenti (i float si calcolano in double e si
 * arrotondano una volta a single, come fa il Python), cosi' i due motori si
 * confrontano al bit (emu/sharc/rec/bridge.py, modalita' lockstep).
 *
 * Il codice generato (gen.py) chiama queste funzioni con argomenti costanti:
 * sono static inline e il compilatore le specializza.
 */
#ifndef SHARC_RT_H
#define SHARC_RT_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#ifdef __aarch64__
#include <arm_neon.h>
#endif
#include <fenv.h>
#include <stdlib.h>
#if !defined(__clang__)           /* PORT: gcc (mingw) non ha __builtin_bitreverse32 */
static inline uint32_t dn_bitrev32(uint32_t v) {
    v = ((v >> 1) & 0x55555555u) | ((v & 0x55555555u) << 1);
    v = ((v >> 2) & 0x33333333u) | ((v & 0x33333333u) << 2);
    v = ((v >> 4) & 0x0F0F0F0Fu) | ((v & 0x0F0F0F0Fu) << 4);
    v = ((v >> 8) & 0x00FF00FFu) | ((v & 0x00FF00FFu) << 8);
    return (v >> 16) | (v << 16);
}
#define __builtin_bitreverse32 dn_bitrev32
#endif
/* PORT: FPCR.FZ (arm64) / MXCSR.FTZ+DAZ (x86-64) dietro due funzioni; i denormali a zero come f32() */
#if defined(__aarch64__)
static inline uint64_t dn_fpcr_get(void) { return __builtin_arm_rsr64("fpcr"); }
static inline void dn_fpcr_set(uint64_t v) { __builtin_arm_wsr64("fpcr", v); }
#elif defined(__x86_64__) || defined(_M_X64)
#include <xmmintrin.h>
static inline uint64_t dn_fpcr_get(void) { return (_mm_getcsr() & 0x8040u) ? (1ull << 24) : 0; }
static inline void dn_fpcr_set(uint64_t v) {
    unsigned c = _mm_getcsr() & ~0x8040u;
    if (v & (1ull << 24)) c |= 0x8040u;
    _mm_setcsr(c);
}
#else
#error "architettura non supportata"
#endif

#ifdef RT_SOFT_INLINE                   /* esperimento d19: inlining deciso dal compilatore */
#define RT_INLINE static inline
#else
#define RT_INLINE static inline __attribute__((always_inline))
#endif
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define M32 0xFFFFFFFFu
/* PORT: un errore del motore rimette subito il controllo al dispatcher (deadline = cycles), cosi' l'esecutore
   non controlla err_code a ogni istruzione */
#define DN_ERR(st, v) do { (st)->err_code = (v); (st)->deadline = (st)->cycles; } while (0)

typedef unsigned __int128 u128;
typedef __int128 s128;

/* MODE1 */
enum { BR8 = 0, BR0 = 1, SRCU = 2, SRD1H = 3, SRD1L = 4, SRD2H = 5, SRD2L = 6, SRRFH = 7,
       SRRFL = 10, NESTM = 11, IRPTEN = 12, ALUSAT = 13, SSE = 14, TRUNCATE = 15, RND32 = 16,
       PEYEN = 21, CBUFEN = 24 };
/* ASTAT */
enum { AZ = 0, AV = 1, AN = 2, AC = 3, AS = 4, AI = 5, MN = 6, MV = 7, MU = 8, MI = 9, AF = 10,
       SV = 11, SZ = 12, SS = 13, SF = 14, BTF = 18 };
#define ALU_FLAGS ((1u << AZ) | (1u << AV) | (1u << AN) | (1u << AC) | (1u << AS) | (1u << AI) | (1u << AF))
#define MUL_FLAGS ((1u << MN) | (1u << MV) | (1u << MU) | (1u << MI))
#define SH_FLAGS ((1u << SV) | (1u << SZ) | (1u << SS))

#define PCSTK_MAX 64
#define LOOP_MAX 32
#define STS_MAX 32

typedef struct {
    uint32_t r[2][16];          /* [0] banco attivo, [1] banco d'ombra (scambiati da set_mode1) */
    uint32_t astat, stky;
    uint64_t mrf_lo, mrf_hi, mrb_lo, mrb_hi;   /* MRF, MRB: 80 bit */
    uint64_t bff;
    int32_t bffwrp;
} PE;

typedef struct St St;
typedef uint32_t (*mmio_rd_t)(uint32_t addr, uint32_t n);
typedef void (*mmio_wr_t)(uint32_t addr, uint32_t n, uint32_t v);

struct St {
    PE pe[2];                   /* 0 = PEx, 1 = PEy */
    uint32_t dag[2][4][16];     /* [attivo/ombra][i, m, l, b][n] */
    uint32_t mode1, mode2, mmask, irptl, imask, imaskp, lirptl, flags;
    uint32_t ustat[4];
    uint64_t px;
    uint32_t faddr, daddr, tperiod, tcount;
    int64_t timer_due;          /* -1 = timer spento */
    uint32_t lcntr;
    uint32_t pc, next_pc;
    uint32_t pcstk[PCSTK_MAX]; int32_t npc;
    uint32_t loops[LOOP_MAX][2]; int32_t nloops;   /* [LADDR (fine | term << 24), CURLCNTR] */
    uint32_t sts[STS_MAX][3]; int32_t nsts;        /* (MODE1, ASTATX, ASTATY) */
    /* salto ritardato: destinazione, istruzioni rimaste, pc del ramo, rti a fine */
    int32_t dly_active; uint32_t dly_target; int32_t dly_left; uint32_t dly_from; int32_t dly_rti;
    int32_t idle;
    int64_t cycles;
    int64_t next_event;         /* -1 = nessun evento (lo gestisce il Python) */
    int64_t deadline;           /* il ciclo in cui tornare al dispatcher */
    int64_t limit;              /* fine del run() corrente */
    uint64_t idle_skips;
    uint32_t irq_count[64];
    uint32_t stop_reason;       /* 1 evento, 2 limite, 3 pc sconosciuto, 4 errore */
    uint32_t err_code;
    /* memoria: finestre dirette, il resto va al Python */
    uint8_t *l1;                /* sistema 0x28000000..0x28400000 */
    uint8_t *l2;                /* sistema 0x20000000..0x20100000 */
    uint8_t *ddr;               /* sistema 0x80000000..0x80000000 + ddr_size */
    uint32_t ddr_size;
    mmio_rd_t mmio_rd;
    mmio_wr_t mmio_wr;
    uint64_t mmio_calls;
    void *seg;                  /* Seg[65536], da rt_init_seg */
    void *per;                  /* periferiche in C (periph.h); 0 = callback verso il Python */
};

uint32_t periph_read(St *st, uint32_t b, uint32_t n);
void periph_write(St *st, uint32_t b, uint32_t n, uint32_t v);

/* ------------------------------------------------------------------------- utilita' */
RT_INLINE int32_t s32(uint32_t v) { return (int32_t)v; }
RT_INLINE int64_t sext(uint64_t v, int bits) {
    v &= (bits >= 64) ? ~0ull : ((1ull << bits) - 1);
    return (bits < 64 && (v >> (bits - 1)) & 1) ? (int64_t)(v - (1ull << bits)) : (int64_t)v;
}
/* I float si calcolano in single precision con FPCR.FZ attivo (rt_run): l'FPU
 * azzera i denormali in ingresso alle operazioni aritmetiche, come f32() del Python,
 * e per +, -, *, /, sqrt il risultato single coincide con il double arrotondato una
 * volta del Python (53 >= 2*24 + 2). b32f azzera i denormali in uscita e rende ogni
 * NaN 0x7FFFFFFF, come compute.b32. */
RT_INLINE float f32(uint32_t bits) { float f; memcpy(&f, &bits, 4); return f; }
RT_INLINE uint32_t b32f(float f) {
    uint32_t bits; memcpy(&bits, &f, 4);
    if (UNLIKELY(f != f)) return 0x7FFFFFFFu;
    if (((bits >> 23) & 0xFF) == 0) return bits & 0x80000000u;
    return bits;
}
/* b32a: risultato di +, -, * in single precision con FPCR.FZ attivo (rt_run): l'FPU non
   produce mai un denormale, quindi basta il NaN; come ramo improbabile, fuori dalla catena
   di dipendenze del risultato (d19). Stesso valore di b32f per questi casi. */
RT_INLINE uint32_t b32a(float f) {
    uint32_t bits; memcpy(&bits, &f, 4);
    if (UNLIKELY(f != f)) bits = 0x7FFFFFFFu;
    return bits;
}
RT_INLINE uint32_t b32(double x) {
    if (x != x) return 0x7FFFFFFFu;
    return b32f((float)x);
}
RT_INLINE double rt_ldexp(double x, int64_t e) {
    if (LIKELY(e >= -1022 && e <= 1023)) {        /* x * 2^e: un solo arrotondamento, come ldexp */
        uint64_t b = (uint64_t)(e + 1023) << 52;
        double p; memcpy(&p, &b, 8);
        return x * p;
    }
    if (e > 2000) e = 2000;
    if (e < -2000) e = -2000;
    return ldexp(x, (int)e);
}

/* -------------------------------------------------------------------- memoria */
enum { SP_NW = 0, SP_SW = 1, SP_BYTE = 2 };
#define CMMR_BASE 0x3F000000u

RT_INLINE uint32_t translate(uint32_t a, int *space) {
    if (a >= 0x00090000u && a < 0x00100000u) { *space = SP_NW; return 0x28000000u + 4 * a; }
    if (a >= 0x00120000u && a < 0x00200000u) { *space = SP_SW; return 0x28000000u + 2 * a; }
    if (a >= 0x00240000u && a < 0x00400000u) { *space = SP_BYTE; return 0x28000000u + a; }
    if (a >= 0x00B80000u && a < 0x00C00000u) { *space = SP_SW; return 0x20000000u + 2 * (a - 0x00B80000u); }
    if (a >= 0x005C0000u && a < 0x00600000u) { *space = SP_NW; return 0x20000000u + 4 * (a - 0x005C0000u); }
    if (a >= 0x00030000u && a < 0x00040000u) { *space = SP_NW; return CMMR_BASE + 4 * (a - 0x00030000u); }
    *space = SP_BYTE; return a;
}
/* segmento di 64 KB dell'indirizzo del DAG: host = base + (a << shift); base 0 =
 * MMIO o pagine sparse, che passano per translate() e le callback */
typedef struct { uint8_t *base; uint8_t space, shift, pad[6]; } Seg;
RT_INLINE int space_of(uint32_t a) { int s; translate(a, &s); return s; }

RT_INLINE uint8_t *host(St *st, uint32_t b, uint32_t n) {
    if (b - 0x28000000u < 0x400000u - n + 1) return st->l1 + (b - 0x28000000u);
    if (b - 0x20000000u < 0x100000u - n + 1) return st->l2 + (b - 0x20000000u);
    if (b - 0x80000000u < st->ddr_size - n + 1) return st->ddr + (b - 0x80000000u);
    return 0;
}
/* percorsi lenti (periferiche) fuori linea: nei nuclei dei loop (KERNEL_MODE, d19) si
   chiamano sulla St vera (k_real) dopo aver sincronizzato la copia locale */
static uint64_t __attribute__((noinline)) memload_slow(St *st, uint32_t b, uint32_t n);
static void __attribute__((noinline)) memstore_slow(St *st, uint32_t b, uint32_t n, uint64_t v);
#ifdef KERNEL_MODE
extern St *k_real;
#endif
RT_INLINE uint64_t memload(St *st, uint32_t b, uint32_t n) {
    uint8_t *p = host(st, b, n);
    if (LIKELY(p != 0)) {
        if (n == 4) { uint32_t v; memcpy(&v, p, 4); return v; }
        if (n == 2) { uint16_t v; memcpy(&v, p, 2); return v; }
        if (n == 1) return *p;
        uint64_t v; memcpy(&v, p, 8); return v;
    }
#if defined(KERNEL_MODE) && (defined(KERNEL_TRAP) || defined(KSPACE))
    __builtin_trap();
#elif defined(KERNEL_MODE) && defined(KERNEL_NOCOPY)
    return memload_slow(st, b, n);
#elif defined(KERNEL_MODE)
    memcpy(k_real, st, sizeof(St));
    uint64_t r = memload_slow(k_real, b, n);
    memcpy(st, k_real, sizeof(St));
    return r;
#else
    return memload_slow(st, b, n);
#endif
}
static uint64_t __attribute__((noinline)) memload_slow(St *st, uint32_t b, uint32_t n) {
    st->mmio_calls++;
    st->deadline = st->cycles;
    if (st->per) {
        if (n == 8) return (uint64_t)periph_read(st, b, 4) | ((uint64_t)periph_read(st, b + 4, 4) << 32);
        return periph_read(st, b, n);
    }
    uint64_t fpcr = dn_fpcr_get();
    dn_fpcr_set(fpcr & ~(1ull << 24));      /* il Python gira senza FZ */
    uint64_t r = n == 8 ? ((uint64_t)st->mmio_rd(b, 4) | ((uint64_t)st->mmio_rd(b + 4, 4) << 32)) : st->mmio_rd(b, n);
    dn_fpcr_set(fpcr);
    return r;
}
RT_INLINE void memstore(St *st, uint32_t b, uint32_t n, uint64_t v) {
    uint8_t *p = host(st, b, n);
    if (LIKELY(p != 0)) {
        if (n == 4) { uint32_t w = (uint32_t)v; memcpy(p, &w, 4); return; }
        if (n == 2) { uint16_t w = (uint16_t)v; memcpy(p, &w, 2); return; }
        if (n == 1) { *p = (uint8_t)v; return; }
        memcpy(p, &v, 8); return;
    }
#if defined(KERNEL_MODE) && (defined(KERNEL_TRAP) || defined(KSPACE))
    __builtin_trap();
#elif defined(KERNEL_MODE) && defined(KERNEL_NOCOPY)
    memstore_slow(st, b, n, v);
#elif defined(KERNEL_MODE)
    memcpy(k_real, st, sizeof(St));
    memstore_slow(k_real, b, n, v);
    memcpy(st, k_real, sizeof(St));
#else
    memstore_slow(st, b, n, v);
#endif
}
static void __attribute__((noinline)) memstore_slow(St *st, uint32_t b, uint32_t n, uint64_t v) {
    st->mmio_calls++;
    if (st->per) {
        if (n == 8) { periph_write(st, b, 4, (uint32_t)v); periph_write(st, b + 4, 4, (uint32_t)(v >> 32)); }
        else periph_write(st, b, n, (uint32_t)v);
        st->deadline = st->cycles;
        return;
    }
    uint64_t fpcr = dn_fpcr_get();
    dn_fpcr_set(fpcr & ~(1ull << 24));
    if (n == 8) { st->mmio_wr(b, 4, (uint32_t)v); st->mmio_wr(b + 4, 4, (uint32_t)(v >> 32)); }
    else st->mmio_wr(b, n, (uint32_t)v);
    dn_fpcr_set(fpcr);
    /* una scrittura di periferica puo' alzare interrupt o eventi: al dispatcher */
    st->deadline = st->cycles;
}

RT_INLINE const Seg *seg_of(St *st, uint32_t a) { return &((const Seg *)st->seg)[a >> 16]; }
/* nei nuclei dei loop (KERNEL_MODE + KSPACE, d19) lo spazio degli accessi e' noto: kacc lo
   verifica prima di ogni istruzione che accede alla memoria, altrimenti il nucleo esce */
#if defined(KERNEL_MODE) && defined(KSPACE)
#define SEG_SPACE(sg) KSPACE
#define SEG_SHIFT(sg) (KSPACE == 2 ? 0 : (KSPACE == 1 ? 1 : 2))
#else
#define SEG_SPACE(sg) ((sg)->space)
#define SEG_SHIFT(sg) ((sg)->shift)
#endif
RT_INLINE int spc(St *st, uint32_t a) { return seg_of(st, a)->space; }

/* d31: q (dentro L1/L2/DDR) e la St sono allocazioni separate. Detto al compilatore, uno store
   nella memoria del DSP (un uint8_t*, che per le regole del C puo' toccare qualunque oggetto)
   non lo costringe piu' a rileggere dalla St registri, DAG, MODE1 e tabella dei segmenti gia'
   letti. RT_SEPSTORE lo definisce gen.py (REC_SEPSTORE=0 lo spegne). */
#if defined(RT_SEPSTORE) && defined(__has_builtin)
#if __has_builtin(__builtin_assume_separate_storage)
#define SEP(q, st) __builtin_assume_separate_storage((q), (st))
#endif
#endif
#ifndef SEP
#define SEP(q, st) ((void)0)
#endif

/* d33: profilo dei valori (RT_VPROF, build di sola misura): per ogni pc (gen.py scrive st->pc
   prima di ogni istruzione) conta gli accessi e annota spazio, base nota e verso */
#ifdef RT_VPROF
extern uint32_t rt_vp_cnt[1u << 24];
extern uint8_t rt_vp_sp[1u << 24];
extern uint8_t rt_vp_rng[1u << 24];
/* intervallo dell'indirizzo, nello stesso ordine di rt_init_seg (7 = altro) */
RT_INLINE int vp_range(uint32_t a) {
    if (a - 0x00090000u < 0x00070000u) return 0;
    if (a - 0x00120000u < 0x000E0000u) return 1;
    if (a - 0x00240000u < 0x001C0000u) return 2;
    if (a - 0x00B80000u < 0x00080000u) return 3;
    if (a - 0x005C0000u < 0x00040000u) return 4;
    if (a - 0x28000000u < 0x00400000u) return 5;
    if (a - 0x20000000u < 0x00100000u) return 6;
    return 7;
}
RT_INLINE void vp_note(St *st, const Seg *sg, uint32_t addr, int write) {
    uint32_t k = st->pc & 0xFFFFFFu;
    rt_vp_cnt[k]++;
    rt_vp_sp[k] |= (uint8_t)((1u << SEG_SPACE(sg)) | (sg->base ? 8u : 16u) | (write ? 32u : 64u));
    rt_vp_rng[k] |= (uint8_t)(1u << vp_range(addr));
}
#else
#define vp_note(st, sg, a, w) ((void)0)
#endif

/* load/store con il segmento gia' noto (stessa semantica di ld/sto) */
RT_INLINE uint32_t ld_seg(St *st, const Seg *sg, uint32_t addr, int size, int sgn) {
    int sp = SEG_SPACE(sg); vp_note(st, sg, addr, 0);
    uint32_t n = sp == SP_NW ? (size == 8 ? 8 : 4) : (sp == SP_SW ? 2 : (uint32_t)size);
    uint32_t v;
    if (LIKELY(sg->base != 0)) {
        const uint8_t *q = sg->base + ((uintptr_t)addr << SEG_SHIFT(sg)); SEP(q, st);
        if (n == 4) memcpy(&v, q, 4);
        else if (n == 2) { uint16_t h; memcpy(&h, q, 2); v = h; }
        else if (n == 1) v = *q;
        else { uint64_t d; memcpy(&d, q, 8); v = (uint32_t)d; }
    } else {
        int s2; uint32_t b = translate(addr, &s2);
        v = (uint32_t)memload(st, b, n);
    }
    if (sp == SP_SW && ((st->mode1 >> SSE) & 1)) sgn = 1;
    if (sgn && n < 4) v = (uint32_t)sext(v, 8 * n);
    return v;
}
RT_INLINE void sto_seg(St *st, const Seg *sg, uint32_t addr, uint32_t v, int size) {
    int sp = SEG_SPACE(sg); vp_note(st, sg, addr, 1);
    uint32_t n = sp == SP_NW ? (size == 8 ? 8 : 4) : (sp == SP_SW ? 2 : (uint32_t)size);
    if (LIKELY(sg->base != 0)) {
        uint8_t *q = sg->base + ((uintptr_t)addr << SEG_SHIFT(sg)); SEP(q, st);
        if (n == 4) memcpy(q, &v, 4);
        else if (n == 2) { uint16_t h = (uint16_t)v; memcpy(q, &h, 2); }
        else if (n == 1) *q = (uint8_t)v;
        else { uint64_t d = v; memcpy(q, &d, 8); }
        return;
    }
    int s2; uint32_t b = translate(addr, &s2);
    memstore(st, b, n, v);
}

/* load/store del core (core.py): lo spazio decide la dimensione reale */
RT_INLINE uint32_t ld(St *st, uint32_t addr, int size, int sgn) {
    const Seg *sg = seg_of(st, addr);
    int sp = SEG_SPACE(sg); vp_note(st, sg, addr, 0);
    uint32_t n = sp == SP_NW ? (size == 8 ? 8 : 4) : (sp == SP_SW ? 2 : (uint32_t)size);
    uint32_t v;
    if (LIKELY(sg->base != 0)) {
        const uint8_t *q = sg->base + ((uintptr_t)addr << SEG_SHIFT(sg)); SEP(q, st);
        if (n == 4) memcpy(&v, q, 4);
        else if (n == 2) { uint16_t h; memcpy(&h, q, 2); v = h; }
        else if (n == 1) v = *q;
        else { uint64_t d; memcpy(&d, q, 8); v = (uint32_t)d; }
    } else {
        int s2; uint32_t b = translate(addr, &s2);
        v = (uint32_t)memload(st, b, n);
    }
    if (sp == SP_SW && ((st->mode1 >> SSE) & 1)) sgn = 1;
    if (sgn && n < 4) v = (uint32_t)sext(v, 8 * n);
    return v;
}
RT_INLINE void sto(St *st, uint32_t addr, uint32_t v, int size) {
    const Seg *sg = seg_of(st, addr);
    int sp = SEG_SPACE(sg); vp_note(st, sg, addr, 1);
    uint32_t n = sp == SP_NW ? (size == 8 ? 8 : 4) : (sp == SP_SW ? 2 : (uint32_t)size);
    if (LIKELY(sg->base != 0)) {
        uint8_t *q = sg->base + ((uintptr_t)addr << SEG_SHIFT(sg)); SEP(q, st);
        if (n == 4) memcpy(q, &v, 4);
        else if (n == 2) { uint16_t h = (uint16_t)v; memcpy(q, &h, 2); }
        else if (n == 1) *q = (uint8_t)v;
        else { uint64_t d = v; memcpy(q, &d, 8); }
        return;
    }
    int s2; uint32_t b = translate(addr, &s2);
    memstore(st, b, n, v);
}

/* ------------------------------------------------------------------- registri */
/* Come l'hardware, i banchi secondari si scambiano con quelli attivi quando cambia
 * MODE1 (set_mode1): l'accesso a un registro non guarda MODE1. */
RT_INLINE uint32_t *rref(St *st, int pe, int n) { return &st->pe[pe].r[0][n]; }
RT_INLINE uint32_t get_r(St *st, int pe, int n) { return *rref(st, pe, n); }
RT_INLINE void set_r(St *st, int pe, int n, uint32_t v) { *rref(st, pe, n) = v; }

RT_INLINE uint32_t *dref(St *st, int kind, int n) { return &st->dag[0][kind][n]; }
enum { KI = 0, KM = 1, KL = 2, KB = 3 };

/* bit di MODE1 che scelgono i banchi: (bit, primo registro, quanti, 0 dati / 1 DAG) */
static void bank_swap(St *st, uint32_t changed) {
    static const int map[6][4] = {{SRRFL, 0, 8, 0}, {SRRFH, 8, 8, 0}, {SRD1L, 0, 4, 1},
                                  {SRD1H, 4, 4, 1}, {SRD2L, 8, 4, 1}, {SRD2H, 12, 4, 1}};
    for (int k = 0; k < 6; k++) {
        if (!((changed >> map[k][0]) & 1)) continue;
        int lo = map[k][1], n = map[k][2];
        if (map[k][3] == 0) {
            for (int pe = 0; pe < 2; pe++)
                for (int i = lo; i < lo + n; i++) {
                    uint32_t t = st->pe[pe].r[0][i]; st->pe[pe].r[0][i] = st->pe[pe].r[1][i]; st->pe[pe].r[1][i] = t;
                }
        } else {
            for (int kind = 0; kind < 4; kind++)
                for (int i = lo; i < lo + n; i++) {
                    uint32_t t = st->dag[0][kind][i]; st->dag[0][kind][i] = st->dag[1][kind][i]; st->dag[1][kind][i] = t;
                }
        }
    }
}
#define BANK_BITS ((1u << SRRFL) | (1u << SRRFH) | (1u << SRD1L) | (1u << SRD1H) | (1u << SRD2L) | (1u << SRD2H))
RT_INLINE uint32_t get_dag(St *st, int kind, int n) { return *dref(st, kind, n); }
RT_INLINE void set_dag(St *st, int kind, int n, uint32_t v) { *dref(st, kind, n) = v; }

/* una scrittura che puo' rendere servibile un interrupt: controllo al dispatcher */
RT_INLINE void irq_recheck(St *st) { st->deadline = st->cycles; }

RT_INLINE void set_mode1(St *st, uint32_t v) {
    uint32_t ch = (st->mode1 ^ v) & BANK_BITS;
    if (UNLIKELY(ch)) bank_swap(st, ch);
    st->mode1 = v;
    irq_recheck(st);
}

RT_INLINE void timer_sync(St *st) {
    if ((st->mode2 >> 5) & 1) {
        if (st->timer_due < 0) st->timer_due = st->cycles + st->tcount;
    } else {
        if (st->timer_due >= 0) {
            int64_t left = st->timer_due - st->cycles;
            st->tcount = (uint32_t)(left > 0 ? left : 0);
        }
        st->timer_due = -1;
    }
    irq_recheck(st);
}

RT_INLINE uint32_t stkyx_read(St *st) {
    return (st->pe[0].stky & ~((1u << 22) | (1u << 24) | (1u << 26))) |
           ((st->npc ? 0u : 1u) << 22) | ((st->nsts ? 0u : 1u) << 24) | ((st->nloops ? 0u : 1u) << 26);
}

/* ritorna 0 e mette err_code se il codice non e' gestito */
RT_INLINE uint32_t get_ureg(St *st, int code) {
    int g = code >> 4, n = code & 15;
    switch (g) {
    case 0: return get_r(st, 0, n);
    case 1: return get_dag(st, KI, n);
    case 2: return get_dag(st, KM, n);
    case 3: return get_dag(st, KL, n);
    case 4: return get_dag(st, KB, n);
    case 5: return get_r(st, 1, n);
    }
    switch (code) {
    case 0x60: return st->faddr;
    case 0x61: return st->daddr;
    case 0x63: return st->pc;
    case 0x64: return st->npc ? st->pcstk[st->npc - 1] : 0;
    case 0x65: return (uint32_t)st->npc;
    case 0x66: return st->nloops ? st->loops[st->nloops - 1][0] : 0;
    case 0x67: return st->nloops ? st->loops[st->nloops - 1][1] : 0;
    case 0x68: return st->lcntr;
    case 0x69: return (uint32_t)st->cycles;
    case 0x6a: return (uint32_t)(st->cycles >> 32);
    case 0x6b: return (uint32_t)(st->px >> 32);
    case 0x6c: return (uint32_t)st->px;
    case 0x6d: return (uint32_t)(st->px >> 32);
    case 0x6e: return st->tperiod;
    case 0x6f:
        if (st->timer_due >= 0) { int64_t d = st->timer_due - st->cycles; return (uint32_t)(d > 0 ? d : 0); }
        return st->tcount;
    case 0x70: return st->ustat[0];
    case 0x71: return st->ustat[1];
    case 0x7e: return st->ustat[2];
    case 0x7f: return st->ustat[3];
    case 0x72: return st->mode1;
    case 0x73: return st->mmask;
    case 0x74: return st->mode2;
    case 0x75: return st->flags;
    case 0x76: return st->pe[0].astat;
    case 0x77: return st->pe[1].astat;
    case 0x78: return stkyx_read(st);
    case 0x79: return st->pe[1].stky;
    case 0x7a: return st->irptl;
    case 0x7b: return st->imask;
    case 0x7c: return st->imaskp;
    case 0x7d: return st->nsts ? st->sts[st->nsts - 1][0] : st->mode1;
    }
    DN_ERR(st, 0x10000 | (uint32_t)code);
    return 0;
}

RT_INLINE void set_ureg(St *st, int code, uint32_t v) {
    int g = code >> 4, n = code & 15;
    switch (g) {
    case 0: set_r(st, 0, n, v); return;
    case 1: set_dag(st, KI, n, v); return;
    case 2: set_dag(st, KM, n, v); return;
    case 3: set_dag(st, KL, n, v); return;
    case 4: set_dag(st, KB, n, v); set_dag(st, KI, n, v); return;
    case 5: set_r(st, 1, n, v); return;
    }
    switch (code) {
    case 0x60: st->faddr = v; return;
    case 0x61: st->daddr = v; return;
    case 0x64:
        if (st->npc) st->pcstk[st->npc - 1] = v;
        else if (st->npc < PCSTK_MAX) st->pcstk[st->npc++] = v;
        return;
    case 0x65: if ((int32_t)v < st->npc) st->npc = (int32_t)v; return;
    case 0x66: if (st->nloops) st->loops[st->nloops - 1][0] = v; return;
    case 0x67: if (st->nloops) st->loops[st->nloops - 1][1] = v; return;
    case 0x68: st->lcntr = v; return;
    case 0x6b: st->px = (uint64_t)v << 32; return;
    case 0x6c: st->px = (st->px & ~(uint64_t)M32) | v; return;
    case 0x6d: st->px = (st->px & M32) | ((uint64_t)v << 32); return;
    case 0x6e: st->tperiod = v; return;
    case 0x6f: st->tcount = v; timer_sync(st); return;
    case 0x70: st->ustat[0] = v; return;
    case 0x71: st->ustat[1] = v; return;
    case 0x7e: st->ustat[2] = v; return;
    case 0x7f: st->ustat[3] = v; return;
    case 0x72: set_mode1(st, v); return;
    case 0x73: st->mmask = v; return;
    case 0x74: st->mode2 = v; timer_sync(st); return;
    case 0x75: st->flags = v; return;
    case 0x76: st->pe[0].astat = v; return;
    case 0x77: st->pe[1].astat = v; return;
    case 0x78: st->pe[0].stky = v; return;
    case 0x79: st->pe[1].stky = v; return;
    case 0x7a: st->irptl = v; irq_recheck(st); return;
    case 0x7b: st->imask = v; irq_recheck(st); return;
    case 0x7c: st->imaskp = v; irq_recheck(st); return;
    case 0x7d:
        if (st->nsts) st->sts[st->nsts - 1][0] = v;
        else if (st->nsts < STS_MAX) {
            st->sts[st->nsts][0] = v; st->sts[st->nsts][1] = st->pe[0].astat;
            st->sts[st->nsts][2] = st->pe[1].astat; st->nsts++;
        }
        return;
    case 0x63: case 0x69: case 0x6a: return;
    }
    DN_ERR(st, 0x20000 | (uint32_t)code);
}

/* ------------------------------------------------------------------------- DAG */
RT_INLINE uint32_t circ(St *st, int n, int64_t nw, int64_t scale) {
    uint32_t ln = get_dag(st, KL, n);
    if (ln && ((st->mode1 >> CBUFEN) & 1)) {
        int64_t base = get_dag(st, KB, n);
        int64_t span = (int64_t)ln * scale;
        if (nw >= base + span) nw -= span;
        else if (nw < base) nw += span;
    }
    return (uint32_t)nw;
}

/* -> indirizzo (del DAG); aggiorna I se post. size 1, 2, 4 (8 si passa come 4) */
RT_INLINE uint32_t dag_access(St *st, int n, uint32_t mod, int post, int size) {
    uint32_t ia = get_dag(st, KI, n);
    int sp = spc(st, ia);
    int64_t scale = (sp == SP_BYTE && (size == 1 || size == 2)) ? size : (sp == SP_BYTE ? 4 : 1);
    int64_t m = (int32_t)mod;
    if (post) {
        set_dag(st, KI, n, circ(st, n, (int64_t)ia + m * scale, scale));
        return ia;
    }
    return (uint32_t)((int64_t)ia + m * scale);
}

/* come dag_access, ma ritorna anche il segmento dell'indirizzo (una ricerca sola) */
RT_INLINE uint32_t dag_access_seg(St *st, int n, uint32_t mod, int post, int size, const Seg **sgp) {
    uint32_t ia = get_dag(st, KI, n);
    const Seg *sg = seg_of(st, ia);
    int sp = SEG_SPACE(sg);
    int64_t scale = (sp == SP_BYTE && (size == 1 || size == 2)) ? size : (sp == SP_BYTE ? 4 : 1);
    int64_t m = (int32_t)mod;
    if (post) {
        set_dag(st, KI, n, circ(st, n, (int64_t)ia + m * scale, scale));
        *sgp = sg;
        return ia;
    }
    uint32_t addr = (uint32_t)((int64_t)ia + m * scale);
    *sgp = ((addr ^ ia) >> 16) ? seg_of(st, addr) : sg;
    return addr;
}

/* forms._modify: MODIFY scalato (kind 0 nessuno, 1 sw, 2 nw) */
RT_INLINE void modify(St *st, int src, int dst, uint32_t mod, int kind) {
    uint32_t ia = get_dag(st, KI, src);
    int64_t scale = 1;
    if (kind && spc(st, ia) == SP_BYTE) scale = kind == 1 ? 2 : 4;
    int64_t nw = (int64_t)ia + (int64_t)(int32_t)mod * scale;
    uint32_t ln = get_dag(st, KL, src);
    if (ln) {
        int64_t base = get_dag(st, KB, src), span = (int64_t)ln * scale;
        if (nw >= base + span) nw -= span;
        else if (nw < base) nw += span;
    }
    set_dag(st, KI, dst, (uint32_t)nw);
}

/* ------------------------------------------------------------------ condizioni */
RT_INLINE int cond(St *st, int c, int pe) {
    if (c == 31) return 1;
    uint32_t a = st->pe[pe].astat;
    int neg = c >= 16;
    c &= 15;
    int az = (a >> AZ) & 1, an = (a >> AN) & 1, av = (a >> AV) & 1, r;
    switch (c) {
    case 0: r = az; break;
    case 1: r = an && !az; break;
    case 2: r = an || az; break;
    case 3: r = (a >> AC) & 1; break;
    case 4: r = av; break;
    case 5: r = (a >> MV) & 1; break;
    case 6: r = (a >> MN) & 1; break;
    case 7: r = (a >> SV) & 1; break;
    case 8: r = (a >> SZ) & 1; break;
    case 9: case 10: case 11: case 12: r = (st->flags >> (1 + 2 * (c - 9))) & 1; break;
    case 13: r = (a >> BTF) & 1; break;
    case 14: r = (a >> 14) & 1; break;
    default: /* 15: NOT LCE, senza complemento */
        return st->nloops && st->loops[st->nloops - 1][1] != 1;
    }
    if (neg) {
        if (c == 1) return !(an && !az);
        if (c == 2) return !(an || az);
        return !r;
    }
    return r != 0;
}
#ifdef EXP_NOSIMD
RT_INLINE int simd(St *st) { (void)st; return 0; }
#else
RT_INLINE int simd(St *st) { return (st->mode1 >> PEYEN) & 1; }
#endif
/* maschera dei PE attivi: bit0 PEx, bit1 PEy (forms._pes) */
RT_INLINE int pes(St *st, int c) {
    int all = simd(st) ? 3 : 1;
    if (c == 31) return all;
    int m = cond(st, c, 0) ? 1 : 0;
    if (all & 2) m |= cond(st, c, 1) ? 2 : 0;
    return m;
}
RT_INLINE int bcond(St *st, int c) {
    if (c == 31) return 1;
    if (simd(st)) return cond(st, c, 0) && cond(st, c, 1);
    return cond(st, c, 0);
}

/* ---------------------------------------------------------------------- ALU */
typedef struct { int n; int reg[4]; uint32_t val[4]; } Wr;
RT_INLINE void W(Wr *w, int n, uint32_t v) { w->reg[w->n] = n; w->val[w->n] = v; w->n++; }
RT_INLINE void apply(St *st, int pe, Wr *w) { for (int i = 0; i < w->n; i++) set_r(st, pe, w->reg[i], w->val[i]); }

/* scrive in ASTAT i soli flag vivi (gen.py: analisi di vivezza) */
RT_INLINE void setfl(PE *p, uint32_t mask, uint32_t bits, uint32_t live) {
    mask &= live;
    if (mask) p->astat = (p->astat & ~mask) | (bits & mask);
}

RT_INLINE void alu_fixed_flags(PE *p, int64_t res, int carry, int ovf, uint32_t live) {
    uint32_t v = (uint32_t)res, fl = 0;
    if (v == 0) fl |= 1u << AZ;
    if (v & 0x80000000u) fl |= 1u << AN;
    if (ovf) { fl |= 1u << AV; if (live & (1u << AV)) p->stky |= 1u << 1; }
    if (carry) fl |= 1u << AC;
    setfl(p, ALU_FLAGS, fl, live);
}
RT_INLINE uint32_t float_flags_bits(PE *p, uint32_t bits, uint32_t live) {
    uint32_t fl = 1u << AF;
    if ((bits & 0x7FFFFFFFu) == 0) fl |= 1u << AZ;
    if (bits >> 31) fl |= 1u << AN;
    if ((bits & 0x7F800000u) == 0x7F800000u) fl |= (bits & 0x7FFFFFu) ? (1u << AI) : (1u << AV);
    setfl(p, ALU_FLAGS, fl, live);
    return bits;
}
/* risultato aritmetico (+, -, abs di +/-): niente controllo dei denormali (b32a) */
RT_INLINE uint32_t alu_float_flags_a(PE *p, float x, uint32_t live) { return float_flags_bits(p, b32a(x), live); }
RT_INLINE uint32_t alu_float_flags(PE *p, float x, uint32_t live) {
    uint32_t bits = b32f(x), fl = 1u << AF;
    if ((bits & 0x7FFFFFFFu) == 0) fl |= 1u << AZ;
    if (bits >> 31) fl |= 1u << AN;
    if ((bits & 0x7F800000u) == 0x7F800000u) fl |= (bits & 0x7FFFFFu) ? (1u << AI) : (1u << AV);
    setfl(p, ALU_FLAGS, fl, live);
    return bits;
}
/* compute._add; cin = -1 per "None" (sub: 1) */
RT_INLINE uint32_t add(St *st, PE *p, uint32_t a, uint32_t b, int cin, int sub, uint32_t live) {
    uint64_t res;
    if (sub) res = (uint64_t)a + (uint64_t)(~b & M32) + (uint64_t)(cin < 0 ? 1 : cin);
    else res = (uint64_t)a + b + (uint64_t)(cin < 0 ? 0 : cin);
    int carry = (int)(res >> 32);
    uint32_t r = (uint32_t)res;
    uint32_t bb = sub ? ~b : b;
    int ovf = ((a ^ r) & (bb ^ r) & 0x80000000u) != 0;
    if (ovf && ((st->mode1 >> ALUSAT) & 1)) r = (a & 0x80000000u) ? 0x80000000u : 0x7FFFFFFFu;
    alu_fixed_flags(p, r, carry, ovf, live);
    return r;
}
/* min/max del Python: min(a, b) = b se b < a, altrimenti a (NaN compreso) */
RT_INLINE float py_min(float a, float b) { return b < a ? b : a; }
RT_INLINE float py_max(float a, float b) { return b > a ? b : a; }
RT_INLINE double py_mind(double a, double b) { return b < a ? b : a; }
RT_INLINE double py_maxd(double a, double b) { return b > a ? b : a; }

/* compute._round_int: NaN -> 0, infinito saturato, al pari piu' vicino o troncato */
RT_INLINE int64_t round_int2(double x, int tr) {
    if (x != x) return 0;
    if (isinf(x)) return x > 0 ? 0x7FFFFFFF : -0x80000000LL;
    double v = tr ? trunc(x) : nearbyint(x);   /* FE_TONEAREST: al pari piu' vicino, come round() */
    if (v > 2147483647.0) return 0x7FFFFFFF;
    if (v < -2147483648.0) return -0x80000000LL;
    return (int64_t)v;
}

RT_INLINE void alu(St *st, int pe, Wr *w, int op, int rn, int rx, int ry, uint32_t live) {
    PE *p = &st->pe[pe];
    int ci = (p->astat >> AC) & 1;
    uint32_t X = get_r(st, pe, rx), Y = get_r(st, pe, ry);
    if (op < 0x80) {
        switch (op) {
        case 0x01: W(w, rn, add(st, p, X, Y, 0, 0, live)); return;
        case 0x02: W(w, rn, add(st, p, X, Y, 1, 1, live)); return;
        case 0x05: W(w, rn, add(st, p, X, Y, ci, 0, live)); return;
        case 0x06: W(w, rn, add(st, p, X, Y, ci, 1, live)); return;
        case 0x09: { int64_t r = ((int64_t)s32(X) + s32(Y)) >> 1; alu_fixed_flags(p, r, 0, 0, live); W(w, rn, (uint32_t)r); return; }
        case 0x0A: case 0x0B: {
            int lt, eq, gt;
            if (op == 0x0A) { int32_t a = s32(X), b = s32(Y); lt = a < b; eq = a == b; gt = a > b; }
            else { lt = X < Y; eq = X == Y; gt = X > Y; }
            setfl(p, ALU_FLAGS, (eq ? 1u << AZ : 0) | (lt ? 1u << AN : 0), live);
            if (live & 0xFF000000u) {
                uint32_t cacc = (p->astat >> 24) & 0xFF;
                cacc = (cacc >> 1) | (gt ? 0x80u : 0);
                p->astat = (p->astat & 0x00FFFFFFu) | (cacc << 24);
            }
            return;
        }
        case 0x21: alu_fixed_flags(p, X, 0, 0, live); W(w, rn, X); return;
        case 0x22: { uint32_t r = (uint32_t)(-(int64_t)s32(X)); alu_fixed_flags(p, r, 0, X == 0x80000000u, live); W(w, rn, r); return; }
        case 0x25: W(w, rn, add(st, p, X, 0, ci, 0, live)); return;
        case 0x26: W(w, rn, add(st, p, X, 0, ci, 1, live)); return;
        case 0x29: W(w, rn, add(st, p, X, 1, 0, 0, live)); return;
        case 0x2A: W(w, rn, add(st, p, X, 1, 1, 1, live)); return;
        case 0x30: {
            int64_t sx = s32(X); uint32_t v = (uint32_t)(sx < 0 ? -sx : sx);
            alu_fixed_flags(p, v, 0, X == 0x80000000u, live);
            if (live & (1u << AS)) p->astat = (p->astat & ~(1u << AS)) | ((X >> 31) << AS);
            W(w, rn, v); return;
        }
        case 0x40: { uint32_t v = X & Y; alu_fixed_flags(p, v, 0, 0, live); W(w, rn, v); return; }
        case 0x41: { uint32_t v = X | Y; alu_fixed_flags(p, v, 0, 0, live); W(w, rn, v); return; }
        case 0x42: { uint32_t v = X ^ Y; alu_fixed_flags(p, v, 0, 0, live); W(w, rn, v); return; }
        case 0x43: { uint32_t v = ~X; alu_fixed_flags(p, v, 0, 0, live); W(w, rn, v); return; }
        case 0x61: case 0x62: {
            int32_t a = s32(X), b = s32(Y), v = op == 0x61 ? (a < b ? a : b) : (a > b ? a : b);
            alu_fixed_flags(p, v, 0, 0, live); W(w, rn, (uint32_t)v); return;
        }
        case 0x63: {
            int64_t lim = s32(Y); if (lim < 0) lim = -lim;
            int64_t v = s32(X); if (v > lim) v = lim; if (v < -lim) v = -lim;
            alu_fixed_flags(p, v, 0, 0, live); W(w, rn, (uint32_t)v); return;
        }
        }
        DN_ERR(st, 0x30000 | (uint32_t)op);
        return;
    }
    float FX = f32(X), FY = f32(Y);
    int trunc_ = (st->mode1 >> TRUNCATE) & 1;
    switch (op) {
    case 0x81: W(w, rn, alu_float_flags_a(p, FX + FY, live)); return;
    case 0x82: W(w, rn, alu_float_flags_a(p, FX - FY, live)); return;
    case 0x89: W(w, rn, alu_float_flags(p, (float)(((double)FX + FY) / 2), live)); return;
    case 0x8A: {
        setfl(p, ALU_FLAGS, (1u << AF) | (FX == FY ? 1u << AZ : 0) | (FX < FY ? 1u << AN : 0), live);
        if (live & 0xFF000000u) {
            uint32_t cacc = (p->astat >> 24) & 0xFF;
            cacc = (cacc >> 1) | (FX > FY ? 0x80u : 0);
            p->astat = (p->astat & 0x00FFFFFFu) | (cacc << 24);
        }
        return;
    }
    case 0x91: W(w, rn, alu_float_flags_a(p, fabsf(FX + FY), live)); return;
    case 0x92: W(w, rn, alu_float_flags_a(p, fabsf(FX - FY), live)); return;
    case 0xA1: case 0xA5: W(w, rn, alu_float_flags(p, FX, live)); return;
    case 0xA2: W(w, rn, alu_float_flags(p, -FX, live)); return;
    case 0xAD: {
        uint32_t m = ((X & 0x7FFFFFu) | ((X & 0x7F800000u) ? 0x800000u : 0)) << 8;
        alu_fixed_flags(p, m, 0, 0, live); W(w, rn, m); return;
    }
    case 0xB0: W(w, rn, alu_float_flags(p, fabsf(FX), live)); return;
    case 0xBD: {                                  /* scalb: ldexp in double, come il Python */
        uint32_t bits = FX == FX ? b32(rt_ldexp(FX, s32(Y))) : b32f(FX);
        alu_float_flags(p, f32(bits), live);
        W(w, rn, bits);
        return;
    }
    case 0xC1: {
        uint32_t e = (X >> 23) & 0xFF;
        int64_t v = e ? (int64_t)e - 127 : -0x80000000LL;
        alu_fixed_flags(p, v, 0, 0, live); W(w, rn, (uint32_t)v); return;
    }
    case 0xC4: W(w, rn, alu_float_flags(p, FX != 0 ? (float)(1.0 / (double)FX) : copysignf(INFINITY, FX), live)); return;
    case 0xC5: W(w, rn, alu_float_flags(p, FX > 0 ? (float)(1.0 / sqrt((double)FX)) : (FX == 0 ? INFINITY : NAN), live)); return;
    case 0xC9: case 0xCD: case 0xD9: case 0xDD: {
        int64_t v;
#ifdef __aarch64__
        if (op == 0xC9 || op == 0xCD) {
            /* d19: FCVTZS/FCVTNS sul float fanno esattamente round_int2 ((double)FX esatto):
               NaN -> 0, saturazione a int32, troncamento o pari piu' vicino */
            v = (trunc_ || op == 0xCD) ? (int64_t)vcvts_s32_f32(FX) : (int64_t)vcvtns_s32_f32(FX);
        } else
#endif
        {
            double x = (double)FX;
            if (op == 0xD9 || op == 0xDD) x = rt_ldexp(x, s32(Y));
            v = round_int2(x, trunc_ || op == 0xCD || op == 0xDD);
        }
        setfl(p, ALU_FLAGS, (1u << AF) | (v == 0 ? 1u << AZ : 0) | (v < 0 ? 1u << AN : 0), live);
        W(w, rn, (uint32_t)v); return;
    }
    case 0xCA: case 0xDA: {
        double x = (double)s32(X);
        if (op == 0xDA) x = rt_ldexp(x, s32(Y));
        { uint32_t bits = b32(x); alu_float_flags(p, f32(bits), live); W(w, rn, bits); }
        return;
    }
    case 0xE0: W(w, rn, alu_float_flags(p, copysignf(fabsf(FX), FY), live)); return;
    case 0xE1: W(w, rn, alu_float_flags(p, py_min(FX, FY), live)); return;
    case 0xE2: W(w, rn, alu_float_flags(p, py_max(FX, FY), live)); return;
    case 0xE3: { float lim = fabsf(FY); W(w, rn, alu_float_flags(p, py_max(-lim, py_min(lim, FX)), live)); return; }
    }
    DN_ERR(st, 0x40000 | (uint32_t)op);
}

/* ----------------------------------------------------------------- moltiplicatore */
#define M80 ((((u128)1) << 80) - 1)
RT_INLINE s128 s80(u128 v) {
    v &= M80;
    return (v >> 79) & 1 ? (s128)v - ((s128)1 << 80) : (s128)v;
}
RT_INLINE void mr_set(PE *p, int b, s128 v) {
    u128 u = (u128)v & M80;
    if (b) { p->mrb_lo = (uint64_t)u; p->mrb_hi = (uint64_t)(u >> 64); }
    else { p->mrf_lo = (uint64_t)u; p->mrf_hi = (uint64_t)(u >> 64); }
}
RT_INLINE u128 mr_get(PE *p, int b) {
    return b ? (((u128)p->mrb_hi << 64) | p->mrb_lo) : (((u128)p->mrf_hi << 64) | p->mrf_lo);
}
RT_INLINE void mul_flags(PE *p, s128 v, int bits, uint32_t live) {
    uint32_t fl = 0;
    if (v < 0 || (bits == 32 && ((uint64_t)v & 0x80000000u))) fl |= 1u << MN;
    setfl(p, MUL_FLAGS, fl, live);
}
/* PGR 11-50: verso Rn passa solo la parte nel formato degli ingressi (bit 31-0 intero,
   63-32 frazionario), senza saturazione (solo SAT MR satura) */
RT_INLINE uint32_t mr_to_reg(s128 v80, int frac, int rnd, int sgn) {
    s128 v = sgn ? s80((u128)v80) : (s128)((u128)v80 & M80);
    if (frac) {
        if (rnd) v += (s128)1 << 31;
        return (uint32_t)(v >> 32);
    }
    return (uint32_t)v;
}

RT_INLINE void mul(St *st, int pe, Wr *w, int op, int rn, int rx, int ry, uint32_t live) {
    PE *p = &st->pe[pe];
    uint32_t X = get_r(st, pe, rx), Y = get_r(st, pe, ry);
    if (op == 0x30) {
        uint32_t bits = b32a(f32(X) * f32(Y)), fl = (bits >> 31) ? (1u << MN) : 0;
        if ((bits & 0x7F800000u) == 0x7F800000u) fl |= 1u << MV;
        setfl(p, MUL_FLAGS, fl, live);
        W(w, rn, bits);
        return;
    }
    int top = op >> 6;
    if (top == 0) {
        if ((op & 0xF0) == 0x00) {                     /* sat */
            int b = (op & 2) != 0, frac = (op >> 3) & 1;
            /* percorso veloce: MR entra in 64 bit con segno (i 16 alti sono l'estensione) */
            int64_t v64 = (int64_t)(b ? p->mrb_lo : p->mrf_lo);
            uint64_t h16 = (b ? p->mrb_hi : p->mrf_hi) & 0xFFFF;
            if (LIKELY(h16 == (v64 < 0 ? 0xFFFFu : 0u)) && !(op & 4)) {
                if (frac) {
                    if (v64 > (int64_t)0x7FFFFFFF00000000LL) v64 = (int64_t)0x7FFFFFFF00000000LL;
                    W(w, rn, (uint32_t)(int32_t)(v64 >> 32));
                } else {
                    if (v64 > 0x7FFFFFFFLL) v64 = 0x7FFFFFFFLL;
                    if (v64 < -0x80000000LL) v64 = -0x80000000LL;
                    W(w, rn, (uint32_t)v64);
                }
                return;
            }
            s128 v = s80(mr_get(p, b));
            /* PRM tab. 3-5: 7FFF FFFF FFFF FFFF / 8000 0000 0000 0000 (SF), 7FFF FFFF / 8000 0000 (SI) */
            s128 lim = frac ? (((s128)1 << 63) - 1) : (s128)0x7FFFFFFF;
            s128 lo = -lim - 1;
            if (v > lim) v = lim;
            if (v < lo) v = lo;
            if (op & 4) mr_set(p, b, v);
            else W(w, rn, mr_to_reg(v, frac, 0, 1));
            return;
        }
        if (op == 0x14 || op == 0x16) { mr_set(p, op == 0x16, 0); return; }
        if ((op & 0xF8) == 0x18) {                     /* rnd */
            int b = (op & 2) != 0;
            s128 v = s80(mr_get(p, b));
            v = (v + ((s128)1 << 31)) & ~(s128)M32;
            if (op & 4) mr_set(p, b, v);
            else W(w, rn, mr_to_reg(v, 1, 0, 1));
            return;
        }
        DN_ERR(st, 0x50000 | (uint32_t)op);
        return;
    }
    int ysgn = (op >> 5) & 1, xsgn = (op >> 4) & 1, frac = (op >> 3) & 1, dst = (op >> 1) & 3, rnd = op & 1;
    s128 a = xsgn ? (s128)s32(X) : (s128)X;
    s128 b = ysgn ? (s128)s32(Y) : (s128)Y;
    s128 pr = a * b;
    if (frac && xsgn && ysgn) pr *= 2;                /* solo SSF toglie il bit di segno doppio */
    s128 acc;
    if (top == 1) acc = pr;
    else {
        s128 base = s80(mr_get(p, dst & 1));
        acc = top == 2 ? base + pr : base - pr;
    }
    int sgn = xsgn || ysgn;
    if (dst == 2 || dst == 3) {
        mr_set(p, dst == 3, acc);
        mul_flags(p, acc, 80, live);
    } else {
        uint32_t r = mr_to_reg(acc, frac, rnd, sgn);
        mul_flags(p, (s128)s32(r), 32, live);
        W(w, rn, r);
    }
}

/* ------------------------------------------------------------------------ shifter */
RT_INLINE void sh_flags(PE *p, uint32_t v, int ovf, uint32_t live) {
    setfl(p, SH_FLAGS, (v == 0 ? 1u << SZ : 0) | (ovf ? 1u << SV : 0), live);
}
RT_INLINE void bff_sf(PE *p, uint32_t live) { if (live & (1u << SF)) p->astat = (p->astat & ~(1u << SF)) | ((p->bffwrp >= 32 ? 1u : 0) << SF); }
RT_INLINE int bitlen32(uint32_t v) { return v ? 32 - __builtin_clz(v) : 0; }

/* compute._bitfifo; hasY = 0 per "Y is None" */
RT_INLINE void bitfifo(St *st, int pe, Wr *w, int kind, int rn, uint32_t X, uint32_t RN, int hasY, uint32_t Y, uint32_t live) {
    PE *p = &st->pe[pe];
    (void)RN;
    if (kind == 0x14 || kind == 0x16) {
        uint32_t ln = (hasY ? Y : X) & 0x3F;
        uint32_t v = ln ? (uint32_t)((ln >= 64 ? p->bff : (p->bff >> (64 - ln))) & M32) : 0;
        int ovf = ln > 32 || (int32_t)ln > p->bffwrp;
        if (kind == 0x14) {
            p->bff = ln >= 64 ? 0 : (p->bff << ln);
            p->bffwrp = p->bffwrp - (int32_t)ln > 0 ? p->bffwrp - (int32_t)ln : 0;
        }
        sh_flags(p, v, ovf, live);
        bff_sf(p, live);
        W(w, rn, v);
    } else if (kind == 0x1D) {
        uint32_t ln = Y & 0x3F;
        int ovf = ln > 32 || p->bffwrp + (int32_t)ln > 64;
        if (ln && !ovf) {
            uint64_t f = X & ((1ull << ln) - 1);
            int sh = 64 - p->bffwrp - (int)ln;
            p->bff |= sh >= 0 ? (f << sh) : (f >> -sh);
            p->bffwrp += (int32_t)ln;
        }
        setfl(p, SH_FLAGS, ovf ? 1u << SV : 0, live);
        bff_sf(p, live);
    } else if (kind == 0x1C) {
        W(w, rn, (uint32_t)p->bffwrp);
    } else {
        uint32_t v = Y & 0x7F;
        p->bffwrp = v < 64 ? (int32_t)v : 64;
        setfl(p, SH_FLAGS, v > 64 ? 1u << SV : 0, live);
        bff_sf(p, live);
    }
}

/* compute.shift: op = shiftop a 8 bit; hasimm/imm per il Type 6 */
RT_INLINE void shift(St *st, int pe, Wr *w, int op, int rn, int rx, int ry, int hasimm, int64_t imm, uint32_t live) {
    PE *p = &st->pe[pe];
    uint32_t X = get_r(st, pe, rx), RN = get_r(st, pe, rn);
    uint32_t Y = hasimm ? (uint32_t)imm : get_r(st, pe, ry);
    int64_t n8 = hasimm ? imm : (int64_t)(int8_t)(Y & 0xFF);
    int kind = op >> 2;
    switch (kind) {
    case 0x00: case 0x08: {
        uint32_t v;
        if (n8 >= 32 || n8 <= -32) v = 0;
        else v = n8 >= 0 ? (X << n8) : (X >> -n8);
        if (kind == 0x08) v |= RN;
        sh_flags(p, v, 0, live); W(w, rn, v); return;
    }
    case 0x01: case 0x09: {
        int32_t sx = s32(X); uint32_t v;
        if (n8 >= 0) v = n8 < 32 ? (uint32_t)((uint64_t)(int64_t)sx << n8) : 0;
        else v = (uint32_t)(sx >> (-n8 < 31 ? -n8 : 31));
        if (kind == 0x09) v |= RN;
        sh_flags(p, v, 0, live); W(w, rn, v); return;
    }
    case 0x02: {
        int64_t k = ((n8 % 32) + 32) % 32;
        uint32_t v = k ? ((X << k) | (X >> (32 - k))) : X;
        sh_flags(p, v, 0, live); W(w, rn, v); return;
    }
    case 0x10: case 0x11: case 0x12: case 0x13: case 0x19: case 0x1B: {
        uint32_t src = hasimm ? (uint32_t)imm : Y;
        uint32_t pos = src & 0x3F, ln = (src >> 6) & 0x3F;
        uint32_t mask = ln < 32 ? (uint32_t)((1ull << ln) - 1) : M32;
        uint32_t v;
        if (kind == 0x10 || kind == 0x12) {
            v = pos < 32 ? (X >> pos) & mask : 0;
            if (kind == 0x12 && ln && ((v >> (ln - 1)) & 1)) v |= ~mask;
        } else {
            uint64_t f = X & mask;
            if ((kind == 0x13 || kind == 0x1B) && ln && ((f >> (ln - 1)) & 1)) f |= (~mask) & M32;
            v = pos < 64 ? (uint32_t)((f << pos) & M32) : 0;
            if (kind == 0x19 || kind == 0x1B) v = RN | v;
        }
        sh_flags(p, v, 0, live); W(w, rn, v); return;
    }
    case 0x30: case 0x31: case 0x32: case 0x33: {
        uint32_t bit = Y & 0x3F;
        uint32_t mask = bit < 32 ? (1u << bit) : 0, v;
        if (kind == 0x30) v = X | mask;
        else if (kind == 0x31) v = X & ~mask;
        else if (kind == 0x32) v = X ^ mask;
        else {
            sh_flags(p, (X & mask) ? 0 : 1, 0, live);
            if (live & (1u << SZ)) p->astat = (p->astat & ~(1u << SZ)) | (((X & mask) ? 0u : 1u) << SZ);
            return;
        }
        sh_flags(p, v, 0, live); W(w, rn, v); return;
    }
    case 0x20: case 0x21: {
        int32_t sx = s32(X); int32_t v;
        if (sx == 0 || sx == -1) v = -31;
        else v = bitlen32(sx >= 0 ? (uint32_t)sx : (uint32_t)~sx) - 31;
        sh_flags(p, (uint32_t)v, 0, live); W(w, rn, (uint32_t)v); return;
    }
    case 0x22: case 0x23: {
        uint32_t src = kind == 0x22 ? X : ~X;
        uint32_t v = 32 - bitlen32(src), msb = X >> 31, sz = kind == 0x22 ? msb : (msb ^ 1);
        setfl(p, SH_FLAGS, (sz << SZ) | (v == 32 ? 1u << SV : 0), live);
        W(w, rn, v); return;
    }
    case 0x24: {
        double x = py_maxd(-65504.0, py_mind(65504.0, (double)f32(X)));
        _Float16 h = (_Float16)x; uint16_t hb; memcpy(&hb, &h, 2);
        W(w, rn, hb); return;
    }
    case 0x25: {
        uint16_t hb = X & 0xFFFF; _Float16 h; memcpy(&h, &hb, 2);
        W(w, rn, b32((double)h)); return;
    }
    case 0x14: case 0x16: case 0x1C: case 0x1D: case 0x1F: {
        if (!hasimm) {
            if (kind == 0x14 || kind == 0x16) { bitfifo(st, pe, w, kind, rn, X, RN, 0, 0, live); return; }
            if (kind == 0x1F) { bitfifo(st, pe, w, kind, rn, X, RN, 1, RN, live); return; }
        }
        bitfifo(st, pe, w, kind, rn, X, RN, 1, Y, live);
        return;
    }
    }
    DN_ERR(st, 0x60000 | (uint32_t)op);
}

/* ------------------------------------------------------------ campo COMPUTE */
RT_INLINE void one(St *st, int pe, Wr *w, uint32_t comp, uint32_t live) {
    if (comp == 0) return;
    if (!(comp >> 22)) {
        int cu = (comp >> 20) & 3, op = (comp >> 12) & 0xFF;
        int rn = (comp >> 8) & 15, rx = (comp >> 4) & 15, ry = comp & 15;
        if (cu == 0) {
            if ((op >> 4) == 0x7 || (op >> 4) == 0xF) {
                int rs = (comp >> 12) & 15, ra = (comp >> 8) & 15;
                PE *p = &st->pe[pe];
                if ((op >> 4) == 0x7) {
                    uint32_t X = get_r(st, pe, rx), Y = get_r(st, pe, ry);
                    W(w, ra, add(st, p, X, Y, 0, 0, live));
                    W(w, rs, add(st, p, X, Y, 1, 1, live));
                } else {
                    float FX = f32(get_r(st, pe, rx)), FY = f32(get_r(st, pe, ry));
                    W(w, ra, alu_float_flags_a(p, FX + FY, live));
                    W(w, rs, alu_float_flags_a(p, FX - FY, live));
                }
                return;
            }
            alu(st, pe, w, op, rn, rx, ry, live);
            return;
        }
        if (cu == 1) { mul(st, pe, w, op, rn, rx, ry, live); return; }
        if (cu == 2) { shift(st, pe, w, op, rn, rx, ry, 0, 0, live); return; }
        DN_ERR(st, 0x70000);
        return;
    }
    int op6 = (comp >> 16) & 0x3F;
    PE *p = &st->pe[pe];
    if (op6 == 0 || op6 == 1) {                        /* MR <-> registro */
        int d = op6 & 1, mr = (comp >> 12) & 0xF, rn = (comp >> 8) & 0xF;
        int b = (mr & 4) != 0, part = mr & 3;
        u128 cur = mr_get(p, b);
        if (d) {
            uint32_t v = get_r(st, pe, rn);
            if (part == 2) v &= 0xFFFF;
            u128 fmask = (u128)(part < 2 ? M32 : 0xFFFFu) << (32 * part);
            mr_set(p, b, (s128)((cur & ~fmask) | ((u128)v << (32 * part))));
        } else {
            uint32_t v = (uint32_t)((cur >> (32 * part)) & (part < 2 ? M32 : 0xFFFFu));
            if (part == 2 && (v & 0x8000)) v |= 0xFFFF0000u;
            W(w, rn, v);
        }
        return;
    }
    int rm = (comp >> 12) & 15, ra = (comp >> 8) & 15;
    int rxm = (comp >> 6) & 3, rym = 4 + ((comp >> 4) & 3);
    int rxa = 8 + ((comp >> 2) & 3), rya = 12 + (comp & 3);
    if ((op6 >> 4) >= 2) {
        int rs = (comp >> 16) & 15;
        if ((op6 >> 4) == 2) {
            mul(st, pe, w, 0x79, rm, rxm, rym, live);
            uint32_t X = get_r(st, pe, rxa), Y = get_r(st, pe, rya);
            W(w, ra, add(st, p, X, Y, 0, 0, live));
            W(w, rs, add(st, p, X, Y, 1, 1, live));
        } else {
            mul(st, pe, w, 0x30, rm, rxm, rym, live);
            float FX = f32(get_r(st, pe, rxa)), FY = f32(get_r(st, pe, rya));
            W(w, ra, alu_float_flags_a(p, FX + FY, live));
            W(w, rs, alu_float_flags_a(p, FX - FY, live));
        }
        return;
    }
    if ((op6 >> 3) == 3) {
        static const int aops[8] = {0x81, 0x82, 0xDA, 0xD9, 0x89, 0xB0, 0xE2, 0xE1};
        mul(st, pe, w, 0x30, rm, rxm, rym, live);
        alu(st, pe, w, aops[op6 & 7], ra, rxa, rya, live);
        return;
    }
    int mode = op6 >> 2, aop = op6 & 3, mulop;
    switch (mode) {
    case 1: mulop = 0x79; break;
    case 2: mulop = 0xBC; break;
    case 3: mulop = 0xB9; break;
    case 4: mulop = 0xFC; break;
    case 5: mulop = 0xF9; break;
    default: DN_ERR(st, 0x80000 | (uint32_t)op6); return;
    }
    if (aop == 3) { DN_ERR(st, 0x80100 | (uint32_t)op6); return; }
    mul(st, pe, w, mulop, rm, rxm, rym, live);
    alu(st, pe, w, aop == 0 ? 0x01 : (aop == 1 ? 0x02 : 0x09), ra, rxa, rya, live);
}


/* compute.execute: pesm = maschera dei PE (-1 = tutti quelli attivi) */
RT_INLINE void execute(St *st, uint32_t comp, int pesm, uint32_t live) {
    comp &= 0x7FFFFF;
    if (!comp) return;
    if (pesm < 0) pesm = simd(st) ? 3 : 1;
    Wr w0, w1; w0.n = 0; w1.n = 0;
    if (pesm & 1) one(st, 0, &w0, comp, live);
    if (pesm & 2) one(st, 1, &w1, comp, live);
    apply(st, 0, &w0);
    apply(st, 1, &w1);
}

RT_INLINE void short_pe(St *st, int pe, uint32_t comp12, uint32_t live);
RT_INLINE void execute_short(St *st, uint32_t comp12, int pesm, uint32_t live) {
    if (pesm < 0) pesm = simd(st) ? 3 : 1;
    if (pesm & 1) short_pe(st, 0, comp12, live);
    if (pesm & 2) short_pe(st, 1, comp12, live);
}
RT_INLINE void short_pe(St *st, int pe, uint32_t comp12, uint32_t live) {
    int op = (comp12 >> 8) & 0xF, rn = (comp12 >> 4) & 0xF, rx = comp12 & 0xF;
    {
        Wr w; w.n = 0;
        switch (op) {
        case 0: alu(st, pe, &w, 0x01, rn, rn, rx, live); break;
        case 1: alu(st, pe, &w, 0x02, rn, rn, rx, live); break;
        case 2: alu(st, pe, &w, 0x21, rn, rx, 0, live); break;
        case 3: alu(st, pe, &w, 0x0A, rn, rn, rx, live); break;
        case 4: alu(st, pe, &w, 0x43, rn, rx, 0, live); break;
        case 5: alu(st, pe, &w, 0x29, rn, rx, 0, live); break;
        case 6: alu(st, pe, &w, 0x2A, rn, rx, 0, live); break;
        case 7: mul(st, pe, &w, 0x70, rn, rn, rx, live); break;
        case 8: alu(st, pe, &w, 0x81, rn, rn, rx, live); break;
        case 9: alu(st, pe, &w, 0x82, rn, rn, rx, live); break;
        case 10: alu(st, pe, &w, 0xCA, rn, rx, 0, live); break;
        case 11: alu(st, pe, &w, 0x8A, rn, rn, rx, live); break;
        case 12: alu(st, pe, &w, 0x40, rn, rn, rx, live); break;
        case 13: alu(st, pe, &w, 0x41, rn, rn, rx, live); break;
        case 14: alu(st, pe, &w, 0x42, rn, rn, rx, live); break;
        default: mul(st, pe, &w, 0x30, rn, rn, rx, live); break;
        }
        apply(st, pe, &w);
    }
}

/* forms._shiftimm (Type 6): shift immediato su ogni PE attivo, scritture alla fine */
RT_INLINE void shiftimm(St *st, uint32_t si, uint32_t ex, int pesm, uint32_t live) {
    int op6 = (si >> 16) & 0x3F, data8 = (si >> 8) & 0xFF, rn = (si >> 4) & 15, rx = si & 15;
    int op8 = op6 << 2, kind = op8 >> 2;
    int64_t imm;
    switch (kind) {
    case 0x10: case 0x11: case 0x12: case 0x13: case 0x19: case 0x1B:
    case 0x30: case 0x31: case 0x32: case 0x33: case 0x14: case 0x16:
        imm = ((int64_t)ex << 8) | data8; break;
    default: imm = (int8_t)data8;
    }
    Wr w0, w1; w0.n = 0; w1.n = 0;
    if (pesm & 1) shift(st, 0, &w0, op8, rn, rx, 0, 1, imm, live);
    if (pesm & 2) shift(st, 1, &w1, op8, rn, rx, 0, 1, imm, live);
    apply(st, 0, &w0);
    apply(st, 1, &w1);
}

/* ============================================================== forme (forms.py) */
RT_INLINE int simd_step(St *st, uint32_t addr) { int sp = spc(st, addr); return sp == SP_BYTE ? 4 : (sp == SP_SW ? 2 : 1); }
RT_INLINE uint32_t pair_get(St *st, int ureg) { return get_ureg(st, ((ureg >> 4) << 4) | ((ureg & 15) ^ 1)); }
RT_INLINE void pair_set(St *st, int ureg, uint32_t v) { set_ureg(st, ((ureg >> 4) << 4) | ((ureg & 15) ^ 1), v); }

/* forms.comp_ureg: registro complementare in SIMD, -1 se non ne ha */
RT_INLINE int comp_ureg(int u) {
    if (u < 16) return 0x50 + u;
    if (u >= 0x50 && u < 0x60) return u - 0x50;
    switch (u) {
    case 0x70: return 0x71; case 0x71: return 0x70; case 0x7e: return 0x7f; case 0x7f: return 0x7e;
    case 0x76: return 0x77; case 0x77: return 0x76; case 0x78: return 0x79; case 0x79: return 0x78;
    case 0x6c: return 0x6d; case 0x6d: return 0x6c;
    }
    return -1;
}

#if defined(KERNEL_MODE) && defined(KSPACE)
/* l'accesso che l'istruzione fara' (DAG idx, modifica mod, post/pre, size) cade in memoria
   diretta dello spazio KSPACE? (stessa aritmetica di dag_access_seg) */
RT_INLINE int kacc(St *st, int idx, uint32_t mod, int post, int size) {
    uint32_t ia = get_dag(st, KI, idx);
    const Seg *g = seg_of(st, ia);
    if (g->space != KSPACE || !g->base) return 0;
    uint32_t addr = ia;
    if (!post) {
        int64_t scale = (KSPACE == 2 && (size == 1 || size == 2)) ? size : (KSPACE == 2 ? 4 : 1);
        addr = (uint32_t)((int64_t)ia + (int64_t)(int32_t)mod * scale);
        g = seg_of(st, addr);
        if (g->space != KSPACE || !g->base) return 0;
    }
    if (size == 8) {
        g = seg_of(st, addr + (KSPACE == 2 ? 4 : 1));
        if (g->space != KSPACE || !g->base) return 0;
    }
    return 1;
}
#endif
/* forms._mem_move; pesm = -1 per "pes is None" */
/* d34: intervalli dello spazio a byte con puntatore host diretto (numerazione di vp_range):
   2 = 0x240000 su l1, 5 = sistema su l1, 6 = sistema su l2, 7 = DDR. Ogni istruzione il cui
   profilo (out/rec/vprof.json, REC_VSPEC) mostra sempre lo stesso intervallo passa da qui:
   guardia sull'indirizzo (di partenza e di accesso), altrimenti la generica fuori linea. */
RT_INLINE uint32_t vr_lo(int rk) { return rk == 2 ? 0x00240000u : rk == 5 ? 0x28000000u : rk == 6 ? 0x20000000u : 0x80000000u; }
RT_INLINE uint32_t vr_len(St *st, int rk) { return rk == 2 ? 0x001C0000u : rk == 5 ? 0x00400000u : rk == 6 ? 0x00100000u : st->ddr_size; }
RT_INLINE uint8_t *vr_base(St *st, int rk) { return rk == 2 ? st->l1 + 0x00240000u : rk == 5 ? st->l1 : rk == 6 ? st->l2 : st->ddr; }
void mem_move_gen(St *st, int ureg, int idx, uint32_t mod, int post, int write, int size, int sgn, int pesm);
RT_INLINE void mem_move(St *st, int ureg, int idx, uint32_t mod, int post, int write, int size, int sgn, int pesm, int rk) {
    const Seg *sg;
#ifdef EXP_NOSIMD
    if (rk >= 0) {
        uint32_t ia = get_dag(st, KI, idx);
        int64_t scale = (size == 1 || size == 2) ? size : 4;
        int64_t nw = (int64_t)ia + (int64_t)(int32_t)mod * scale;
        uint32_t addr = post ? ia : (uint32_t)nw;
        uint32_t lo = vr_lo(rk), len = vr_len(st, rk);
        if (UNLIKELY((addr - lo) > len - 8u || (ia - lo) > len - 8u)) { mem_move_gen(st, ureg, idx, mod, post, write, size, sgn, pesm); return; }
        if (post) set_dag(st, KI, idx, circ(st, idx, nw, scale));
        uint8_t *q = vr_base(st, rk) + (addr - lo); SEP(q, st);
        if (write) {
            uint32_t v = get_ureg(st, ureg);
            if (size == 8) { memcpy(q, &v, 4); uint32_t h = pair_get(st, ureg); memcpy(q + 4, &h, 4); }
            else if (size == 4) memcpy(q, &v, 4);
            else if (size == 2) { uint16_t h = (uint16_t)v; memcpy(q, &h, 2); }
            else *q = (uint8_t)v;
        } else if (size == 8) {
            uint32_t v, h; memcpy(&v, q, 4); memcpy(&h, q + 4, 4);
            set_ureg(st, ureg, v); pair_set(st, ureg, h);
        } else {
            uint32_t v;
            if (size == 4) memcpy(&v, q, 4);
            else if (size == 2) { uint16_t h; memcpy(&h, q, 2); v = h; }
            else v = *q;
            if (sgn && size < 4) v = (uint32_t)sext(v, 8 * size);
            set_ureg(st, ureg, v);
        }
        return;
    }
#else
    (void)rk;
#endif
    uint32_t addr = dag_access_seg(st, idx, mod, post, size != 8 ? size : 4, &sg);
    int cu = comp_ureg(ureg);
    int sm = simd(st) && (write || cu >= 0);
    int dx = 1, dy = 1;
    if (pesm >= 0) { dx = pesm & 1; dy = (pesm >> 1) & 1; }
    if (!dx && !(sm && dy)) return;
    int sp = SEG_SPACE(sg);
    int k = sp == SP_BYTE ? (size == 1 ? 1 : (size == 2 ? 2 : 4)) : (sp == SP_SW ? 2 : 1);
    if (write) {
        uint32_t v = get_ureg(st, ureg);
        if (dx) {
            if (size == 8) { sto(st, addr, v, 4); sto(st, addr + (sp == SP_BYTE ? 4 : 1), pair_get(st, ureg), 4); }
            else sto_seg(st, sg, addr, v, size);
        }
        if (sm && dy) sto(st, addr + k, cu >= 0 ? get_ureg(st, cu) : v, size);
    } else {
        if (dx) {
            if (size == 8) {
                uint32_t lo = ld(st, addr, 4, 0), hi = ld(st, addr + (sp == SP_BYTE ? 4 : 1), 4, 0);
                set_ureg(st, ureg, lo); pair_set(st, ureg, hi);
            } else set_ureg(st, ureg, ld_seg(st, sg, addr, size, sgn));
        }
        if (sm && dy) set_ureg(st, cu, ld(st, addr + k, size, sgn));
    }
}

RT_INLINE void cmp_(St *st, uint32_t comp, int pesm, uint32_t live) { if (comp && pesm) execute(st, comp, pesm, live); }

/* Type 1a/1b */
RT_INLINE void op_1a(St *st, int dmi, int dmm, int pmi, int pmm, int dmd, int pmd, int dreg, int preg, uint32_t comp, uint32_t live) {
    int sm = simd(st);
    uint32_t dv0 = 0, dv1 = 0, pv0 = 0, pv1 = 0, dl0 = 0, dl1 = 0, pl0 = 0, pl1 = 0;
    if (dmd) { dv0 = get_r(st, 0, dreg); dv1 = get_r(st, 1, dreg); }
    if (pmd) { pv0 = get_r(st, 0, preg); pv1 = get_r(st, 1, preg); }
    uint32_t daddr = dag_access(st, dmi, get_dag(st, KM, dmm), 1, 4);
    uint32_t paddr = dag_access(st, pmi, get_dag(st, KM, pmm), 1, 4);
    if (!dmd) { dl0 = ld(st, daddr, 4, 0); if (sm) dl1 = ld(st, daddr + simd_step(st, daddr), 4, 0); }
    if (!pmd) { pl0 = ld(st, paddr, 4, 0); if (sm) pl1 = ld(st, paddr + simd_step(st, paddr), 4, 0); }
    if (dmd) { sto(st, daddr, dv0, 4); if (sm) sto(st, daddr + simd_step(st, daddr), dv1, 4); }
    if (pmd) { sto(st, paddr, pv0, 4); if (sm) sto(st, paddr + simd_step(st, paddr), pv1, 4); }
    if (comp) execute(st, comp, -1, live);
    if (!dmd) { set_r(st, 0, dreg, dl0); if (sm) set_r(st, 1, dreg, dl1); }
    if (!pmd) { set_r(st, 0, preg, pl0); if (sm) set_r(st, 1, preg, pl1); }
}

/* Type 3a/3b/3d: forms._type3 */
RT_INLINE void op_3(St *st, int c, int u, int idx, int midx, int d, int ureg, int size, int sgn, uint32_t comp, uint32_t live, int rk) {
    int pm = pes(st, c);
    if (!pm) return;
    uint32_t mod = get_dag(st, KM, midx);
    if (d) { mem_move(st, ureg, idx, mod, u, 1, size, 0, pm, rk); cmp_(st, comp, pm, live); }
    else { cmp_(st, comp, pm, live); mem_move(st, ureg, idx, mod, u, 0, size, sgn, pm, rk); }
}
/* Type 4a/4b/4d: forms._type4 (modifica immediata) */
RT_INLINE void op_4(St *st, int c, int u, int idx, int32_t data, int d, int dreg, int size, int sgn, uint32_t comp, uint32_t live, int rk) {
    int pm = pes(st, c);
    if (!pm) return;
    if (d) { mem_move(st, dreg, idx, (uint32_t)data, u, 1, size, 0, pm, rk); cmp_(st, comp, pm, live); }
    else { cmp_(st, comp, pm, live); mem_move(st, dreg, idx, (uint32_t)data, u, 0, size, sgn, pm, rk); }
}
/* Type 5a/5b move */
RT_INLINE void op_5move(St *st, int c, int src, int dst, uint32_t comp, uint32_t live) {
    int pm = pes(st, c);
    if (!pm) return;
    uint32_t v = get_ureg(st, src);
    int sm = simd(st);
    int cs = comp_ureg(src), cd = comp_ureg(dst);
    uint32_t vy = (sm && cs >= 0) ? get_ureg(st, cs) : v;
    cmp_(st, comp, pm, live);
    if (pm & 1) set_ureg(st, dst, v);
    if (sm && (pm & 2) && cd >= 0) set_ureg(st, cd, vy);
}
RT_INLINE void op_5swap(St *st, int c, int d, int cd, uint32_t comp, uint32_t live) {
    if (!pes(st, c)) return;
    uint32_t a = get_r(st, 0, d), b = get_r(st, 1, cd);
    if (comp) execute(st, comp, -1, live);
    set_r(st, 0, d, b);
    set_r(st, 1, cd, a);
}
RT_INLINE void op_6nomem(St *st, int c, uint32_t si, uint32_t ex, uint32_t live) { shiftimm(st, si, ex, pes(st, c), live); }
RT_INLINE void op_6mem(St *st, int c, int idx, int midx, int d, int dreg, uint32_t si, uint32_t ex, uint32_t live, int rk) {
    int pm = pes(st, c);
    if (!pm) return;
    if (d) { mem_move(st, dreg, idx, get_dag(st, KM, midx), 1, 1, 4, 0, pm, rk); shiftimm(st, si, ex, pm, live); }
    else { shiftimm(st, si, ex, pm, live); mem_move(st, dreg, idx, get_dag(st, KM, midx), 1, 0, 4, 0, pm, rk); }
}
RT_INLINE void op_7a(St *st, int c, int src, int dst, int midx, int kind, uint32_t comp, uint32_t live) {
    int pm = pes(st, c);
    if (!pm) return;
    cmp_(st, comp, pm, live);
    modify(st, src, dst, get_dag(st, KM, midx), kind);
}
RT_INLINE void op_7b(St *st, int c, int src, int dst, int midx) {
    if (!pes(st, c)) return;
    modify(st, src, dst, get_dag(st, KM, midx), 0);
}
RT_INLINE void op_7d(St *st, int c, int breg, int toby, int src, int dst, uint32_t comp, uint32_t live) {
    if (!pes(st, c)) return;
    int kind = breg ? KB : KI;
    uint32_t v = get_dag(st, kind, src);
    int sp; uint32_t b = translate(v, &sp);
    if (toby) {
        if (sp != SP_BYTE) v = b;
        if (v - 0x28000000u < 0x400000u) v -= 0x28000000u;
    } else if (sp == SP_BYTE) {
        uint32_t local = v >= 0x28000000u ? v - 0x28000000u : v;
        v = local >> 2;
    }
    if (comp) execute(st, comp, -1, live);
    set_dag(st, kind, dst, v);
}

/* ------------------------------------------------------------------ salti */
RT_INLINE void rti_finish(St *st) {
    if (st->nsts) {
        st->nsts--;
        set_mode1(st, st->sts[st->nsts][0]);
        st->pe[0].astat = st->sts[st->nsts][1];
        st->pe[1].astat = st->sts[st->nsts][2];
    }
    if (st->imaskp) st->imaskp &= st->imaskp - 1;
    irq_recheck(st);
}
RT_INLINE void push_pc(St *st, uint32_t v) {
    if (st->npc < PCSTK_MAX) st->pcstk[st->npc++] = v; else DN_ERR(st, 0x90001);
}
/* core.branch; ret_db = l'istruzione dopo i due slot */
RT_INLINE void branch(St *st, uint32_t pc, uint32_t *nxt, uint32_t target, int db, int call, uint32_t ret_db) {
    target &= 0xFFFFFF;
    uint32_t ret;
    if (db) {
        ret = ret_db;
        st->dly_active = 1; st->dly_target = target; st->dly_left = 2; st->dly_from = pc; st->dly_rti = 0;
    } else {
        ret = *nxt;
        *nxt = target;
    }
    if (call) push_pc(st, ret);
}
RT_INLINE void branch_common(St *st, uint32_t pc, uint32_t *nxt, uint32_t target, int call, int la, int db, int ci, uint32_t ret_db) {
    if (la && st->nloops) { st->nloops--; if (st->npc) st->npc--; }
    branch(st, pc, nxt, target, db, call, ret_db);
    if (ci) {
        if (db) st->dly_rti = 1;
        else rti_finish(st);
    }
}
/* forms._branch_compute: con ELSE nei PE con condizione falsa */
RT_INLINE void branch_compute(St *st, int c, int e, uint32_t comp, uint32_t live) {
    if (!comp) return;
    int t = pes(st, c);
    if (e) { int all = simd(st) ? 3 : 1; cmp_(st, comp, all & ~t, live); }
    else cmp_(st, comp, t, live);
}
RT_INLINE void do_return(St *st, uint32_t pc, uint32_t *nxt, int x, int db) {
    if (!st->npc) { DN_ERR(st, 0x90002); return; }
    uint32_t target = st->pcstk[--st->npc];
    if (db) {
        st->dly_active = 1; st->dly_target = target; st->dly_left = 2; st->dly_from = pc; st->dly_rti = x;
    } else {
        if (x) rti_finish(st);
        *nxt = target;
    }
}
RT_INLINE void do_loop(St *st, uint32_t fall, uint32_t end, int has_counter, uint32_t counter, int term) {
    if (!has_counter && term == 15) { counter = st->lcntr; has_counter = 1; }
    push_pc(st, fall);
    if (st->nloops >= LOOP_MAX) { DN_ERR(st, 0x90003); return; }
    st->loops[st->nloops][0] = (end & 0xFFFFFF) | ((uint32_t)(has_counter ? 15 : term) << 24);
    st->loops[st->nloops][1] = has_counter ? counter : 0;
    st->nloops++;
}

/* ---------------------------------------------------------------- gruppo III */
RT_INLINE void op_14a(St *st, int d, int l, int ureg, uint32_t addr, int k) {
    int size = l ? 8 : 4, sm = simd(st), cu = comp_ureg(ureg);
    if (d) {
        uint32_t v = get_ureg(st, ureg);
        sto(st, addr, v, 4);
        if (size == 8) sto(st, addr + k, pair_get(st, ureg), 4);
        else if (sm) sto(st, addr + k, cu >= 0 ? get_ureg(st, cu) : v, 4);
    } else {
        set_ureg(st, ureg, ld(st, addr, 4, 0));
        if (size == 8) pair_set(st, ureg, ld(st, addr + k, 4, 0));
        else if (sm && cu >= 0) set_ureg(st, cu, ld(st, addr + k, 4, 0));
    }
}
RT_INLINE void op_14d(St *st, int d, int dreg, uint32_t addr, int size, int sgn, int k) {
    int sm = simd(st);
    if (d) {
        sto(st, addr, get_r(st, 0, dreg), size);
        if (sm) sto(st, addr + k, get_r(st, 1, dreg), size);
    } else {
        set_r(st, 0, dreg, ld(st, addr, size, sgn));
        if (sm) set_r(st, 1, dreg, ld(st, addr + k, size, sgn));
    }
}
void op_16_gen(St *st, int idx, int midx, int size, uint32_t data);
RT_INLINE void op_16(St *st, int idx, int midx, int size, uint32_t data, int rk) {
#ifdef EXP_NOSIMD
    if (rk >= 0) {                                    /* d34: come mem_move, scrittura post-modify */
        uint32_t ia = get_dag(st, KI, idx);
        int64_t scale = (size == 1 || size == 2) ? size : 4;
        uint32_t lo = vr_lo(rk), len = vr_len(st, rk);
        if (UNLIKELY((ia - lo) > len - 8u)) { op_16_gen(st, idx, midx, size, data); return; }
        set_dag(st, KI, idx, circ(st, idx, (int64_t)ia + (int64_t)(int32_t)get_dag(st, KM, midx) * scale, scale));
        uint8_t *q = vr_base(st, rk) + (ia - lo); SEP(q, st);
        if (size == 4) memcpy(q, &data, 4);
        else if (size == 2) { uint16_t h = (uint16_t)data; memcpy(q, &h, 2); }
        else *q = (uint8_t)data;
        return;
    }
#else
    (void)rk;
#endif
    uint32_t addr = dag_access(st, idx, get_dag(st, KM, midx), 1, size);
    sto(st, addr, data, size);
    if (simd(st)) {
        int k = spc(st, addr) == SP_BYTE ? size : simd_step(st, addr);
        sto(st, addr + k, data, size);
    }
}

/* ---------------------------------------------------------------- gruppo IV */
RT_INLINE int sreg_pair(int code) {
    switch (code) { case 0x70: return 0x71; case 0x76: return 0x77; case 0x78: return 0x79; case 0x7e: return 0x7f; }
    return -1;
}
RT_INLINE void bitop1(St *st, int code, int pe, int bop, uint32_t data) {
    uint32_t v = get_ureg(st, code);
    switch (bop) {
    case 0: set_ureg(st, code, v | data); break;
    case 1: set_ureg(st, code, v & ~data); break;
    case 2: set_ureg(st, code, v ^ data); break;
    case 4: case 5: {
        int t = bop == 4 ? ((v & data) == data) : (v == data);
        st->pe[pe].astat = (st->pe[pe].astat & ~(1u << BTF)) | ((uint32_t)t << BTF);
        break;
    }
    default: DN_ERR(st, 0x90004);
    }
}
RT_INLINE void op_18a(St *st, int bop, int sreg, uint32_t data) {
    int code = 0x70 + sreg;
    int sm = simd(st), pr = sreg_pair(code);
    bitop1(st, code, 0, bop, data);
    if (sm && pr >= 0) bitop1(st, pr, 1, bop, data);
}
RT_INLINE void op_19bitrev(St *st, int src, int dst, uint32_t data) {
    uint32_t v = get_dag(st, KI, src) + data;
    v = __builtin_bitreverse32(v);
    set_dag(st, KI, dst, v);
}
RT_INLINE void op_20a(St *st, int lpu, int lpo, int spu, int spo, int ppu, int ppo) {
    if (lpu) {
        if (st->nloops < LOOP_MAX) { st->loops[st->nloops][0] = 0; st->loops[st->nloops][1] = 0; st->nloops++; }
        else DN_ERR(st, 0x90005);
    }
    if (lpo && st->nloops) st->nloops--;
    if (spu) {
        if (st->nsts < STS_MAX) {
            st->sts[st->nsts][0] = st->mode1; st->sts[st->nsts][1] = st->pe[0].astat;
            st->sts[st->nsts][2] = st->pe[1].astat; st->nsts++;
        } else DN_ERR(st, 0x90006);
    }
    if (spo && st->nsts) {
        st->nsts--;
        set_mode1(st, st->sts[st->nsts][0]); st->pe[0].astat = st->sts[st->nsts][1]; st->pe[1].astat = st->sts[st->nsts][2];
        irq_recheck(st);
    }
    if (ppu) push_pc(st, st->npc ? st->pcstk[st->npc - 1] : 0);
    if (ppo && st->npc) st->npc--;
}
RT_INLINE void op_25direct(St *st, uint32_t pc, uint32_t *nxt, uint32_t target, uint32_t ret_db) {
    set_r(st, 0, 2, get_dag(st, KI, 6));
    if (simd(st)) set_r(st, 1, 2, get_dag(st, KI, 6));
    set_dag(st, KI, 6, get_dag(st, KI, 7));
    branch(st, pc, nxt, target, 1, 0, ret_db);
}
RT_INLINE void op_25pcrel(St *st, uint32_t pc, uint32_t *nxt, uint32_t target, uint32_t ret_db) {
    set_r(st, 0, 2, get_dag(st, KI, 6));
    set_dag(st, KI, 6, get_dag(st, KI, 7));
    branch(st, pc, nxt, target, 1, 0, ret_db);
}
/* Type 17: Ureg = immediato; in SIMD un registro dati carica anche il complementare */
RT_INLINE void op_17(St *st, int ureg, uint32_t v) {
    set_ureg(st, ureg, v);
    int cu = comp_ureg(ureg);
    if (cu >= 0 && simd(st)) set_ureg(st, cu, v);
}
RT_INLINE void op_rframe(St *st) {
    uint32_t i6 = get_dag(st, KI, 6);
    set_dag(st, KI, 7, i6);
    set_dag(st, KI, 6, ld(st, i6, 4, 0));
}

/* ------------------------------------------------ sequencer: epilogo di un'istruzione */
/* core.step dopo l'handler: fine loop (solo dove un DO puo' finire) e salto ritardato */
RT_INLINE uint32_t loop_end(St *st, uint32_t pc, uint32_t nxt, uint32_t fall) {
    if (st->nloops && pc == (st->loops[st->nloops - 1][0] & 0xFFFFFF) && nxt == fall) {
        uint32_t *lp = st->loops[st->nloops - 1];
        int term = (lp[0] >> 24) & 31, done;
        if (term == 15) { lp[1] = lp[1] - 1; done = lp[1] == 0; }
        else done = term != 31 && cond(st, term, 0);
        if (done) { st->nloops--; if (st->npc) st->npc--; }
        else nxt = st->npc ? st->pcstk[st->npc - 1] : nxt;
    }
    return nxt;
}
RT_INLINE uint32_t delayed_step(St *st, uint32_t pc, uint32_t nxt) {
    if (UNLIKELY(st->dly_active) && st->dly_from != pc) {
        if (--st->dly_left <= 0) {
            nxt = st->dly_target;
            int rti = st->dly_rti;
            st->dly_active = 0;
            if (rti) rti_finish(st);
            /* un interrupt rimasto in attesa del salto puo' ora essere servito */
            if (st->irptl & st->imask) irq_recheck(st);
        }
    }
    return nxt;
}

/* ------------------------------------------------------------- interrupt e timer */
#define IVT_BASE 0x90000u
enum { TMZHI = 11, SECI = 15, TMZLI = 22 };
RT_INLINE void timer_fire(St *st) {
    st->irptl |= (1u << TMZHI) | (1u << TMZLI);
    st->tcount = st->tperiod;
    st->timer_due = st->cycles + (st->tperiod > 1 ? st->tperiod : 1);
}
RT_INLINE int take_irq(St *st) {
    uint32_t pend = st->irptl & st->imask;
    int n = __builtin_ctz(pend);
    if (st->imaskp) {
        int cur = __builtin_ctz(st->imaskp);
        if (!((st->mode1 >> NESTM) & 1) || n >= cur) return 0;
    }
    push_pc(st, st->pc);
    if (st->nsts < STS_MAX) {
        st->sts[st->nsts][0] = st->mode1; st->sts[st->nsts][1] = st->pe[0].astat;
        st->sts[st->nsts][2] = st->pe[1].astat; st->nsts++;
    } else DN_ERR(st, 0x90007);
    set_mode1(st, st->mode1 & ~st->mmask);
    st->irptl &= ~(1u << n);
    st->imaskp |= 1u << n;
    st->pc = IVT_BASE + 4 * (uint32_t)n;
    st->idle = 0;
    st->irq_count[n]++;
    return 1;
}

/* ------------------------------------------------------ task idle (speed.py) */
#ifdef IDLE_WAITING                      /* PORT: nell'interprete sono variabili (dn_interp.c); con DN_FIXED_IDLE
                                            (libreria costruita dal plugin, vst/NativeBuild.cpp) restano le costanti dei sorgenti */
#ifndef DN_FIXED_IDLE
extern uint32_t dn_idle_waiting, dn_idle_ready0;
#undef IDLE_WAITING
#undef IDLE_READY0
#define IDLE_WAITING dn_idle_waiting
#define IDLE_READY0 dn_idle_ready0
#endif
RT_INLINE int idle_hook(St *st) {
    if (st->dly_active || (st->irptl & st->imask)) return 0;
    if (ld(st, IDLE_WAITING, 4, 0) == 0 && ld(st, IDLE_READY0, 4, 0) <= 1) {
        st->idle = 1;
        st->idle_skips++;
        st->deadline = st->cycles;
        return 1;
    }
    return 0;
}
#endif

int rt_exec(St *st);

#ifdef RT_MAIN
/* d34: versioni generiche fuori linea, chiamate dal percorso veloce quando la guardia fallisce */
void mem_move_gen(St *st, int ureg, int idx, uint32_t mod, int post, int write, int size, int sgn, int pesm) {
    mem_move(st, ureg, idx, mod, post, write, size, sgn, pesm, -1);
}
void op_16_gen(St *st, int idx, int midx, int size, uint32_t data) { op_16(st, idx, midx, size, data, -1); }
#ifdef RT_VPROF
uint32_t rt_vp_cnt[1u << 24];
uint8_t rt_vp_sp[1u << 24];
uint8_t rt_vp_rng[1u << 24];
uint32_t *rt_vp_cnt_ptr(void) { return rt_vp_cnt; }
uint8_t *rt_vp_sp_ptr(void) { return rt_vp_sp; }
uint8_t *rt_vp_rng_ptr(void) { return rt_vp_rng; }
#endif
#include "periph.h"

/* core.step(), parte prima dell'istruzione, e ciclo fino a `limit` (in cicli).
 * Ritorna 1 se ci sono eventi del Python da eseguire, 2 al limite, 3 per un PC
 * non compilato (lo esegue il Python), 4 per un errore (err_code). */
static int rt_run_body(St *st, int64_t limit);
static uint64_t rt_idle_cyc;                          /* cicli saltati in idle (diagnostica) */
uint64_t rt_idle_cycles(void) { return rt_idle_cyc; }
int rt_run(St *st, int64_t limit) {
    uint64_t fpcr = dn_fpcr_get();
    dn_fpcr_set(fpcr | (1ull << 24));      /* FZ: denormali a zero */
    int r = rt_run_body(st, limit);
    dn_fpcr_set(fpcr);
    return r;
}

static Seg segtab[65536];
/* costruisce la tabella dei segmenti dalle finestre di memoria (dopo l1/l2/ddr) */
void rt_init_seg(St *st) {
    for (uint32_t k = 0; k < 65536; k++) {
        uint32_t a = k << 16;
        Seg *g = &segtab[k];
        int sp; uint32_t b = translate(a, &sp);
        g->space = (uint8_t)sp; g->base = 0; g->shift = 0;
        if (a >= 0x00090000u && a < 0x00100000u) { g->base = st->l1; g->shift = 2; }
        else if (a >= 0x00120000u && a < 0x00200000u) { g->base = st->l1; g->shift = 1; }
        else if (a >= 0x00240000u && a < 0x00400000u) { g->base = st->l1; g->shift = 0; }
        else if (a >= 0x00B80000u && a < 0x00C00000u) { g->base = (uint8_t *)((uintptr_t)st->l2 - 2 * 0x00B80000u); g->shift = 1; }
        else if (a >= 0x005C0000u && a < 0x00600000u) { g->base = (uint8_t *)((uintptr_t)st->l2 - 4 * 0x005C0000u); g->shift = 2; }
        else if (sp == SP_BYTE && b - 0x28000000u < 0x400000u) g->base = (uint8_t *)((uintptr_t)st->l1 - 0x28000000u);
        else if (sp == SP_BYTE && b - 0x20000000u < 0x100000u) g->base = (uint8_t *)((uintptr_t)st->l2 - 0x20000000u);
        else if (sp == SP_BYTE && b - 0x80000000u < st->ddr_size) g->base = (uint8_t *)((uintptr_t)st->ddr - 0x80000000u);
    }
    st->seg = segtab;
}

static int rt_run_body(St *st, int64_t limit) {
    st->limit = limit;
    for (;;) {
        if (st->err_code) return 4;
        if (st->timer_due >= 0 && st->cycles >= st->timer_due) timer_fire(st);
        if (st->next_event >= 0 && st->cycles >= st->next_event) {
            if (!st->per) return 1;                   /* eventi del Python */
            if (!run_events(st)) return 5;            /* serve un frame dal ColdFire */
        }
        if ((st->irptl & st->imask) && !st->dly_active && ((st->mode1 >> IRPTEN) & 1)) take_irq(st);
        if (st->idle) {
            int64_t due = -1;
            if (st->timer_due > st->cycles) due = st->timer_due;
            if (st->next_event > st->cycles && (due < 0 || st->next_event < due)) due = st->next_event;
            {
                int64_t nc = due >= 0 ? due : st->cycles + 1;
                rt_idle_cyc += (uint64_t)(nc - st->cycles);
                st->cycles = nc;
            }
            if (st->cycles >= limit) return 2;
            continue;
        }
        if (st->cycles >= limit) return 2;
        int64_t dl = limit;
        if (st->timer_due >= 0 && st->timer_due < dl) dl = st->timer_due;
        if (st->next_event >= 0 && st->next_event < dl) dl = st->next_event;
        /* interrupt servibile ma bloccato dal salto ritardato: un'istruzione alla volta */
        if ((st->irptl & st->imask) && ((st->mode1 >> IRPTEN) & 1) && st->dly_active && st->cycles + 1 < dl)
            dl = st->cycles + 1;
        st->deadline = dl;
        int r = rt_exec(st);
        if (r) return r;
    }
}

/* per il controllo del layout dal Python */
long rt_sizeof(void) { return (long)sizeof(St); }
long rt_offset(int k) {
    switch (k) {
    case 0: return (long)__builtin_offsetof(St, dag);
    case 1: return (long)__builtin_offsetof(St, px);
    case 2: return (long)__builtin_offsetof(St, timer_due);
    case 3: return (long)__builtin_offsetof(St, pcstk);
    case 4: return (long)__builtin_offsetof(St, dly_active);
    case 5: return (long)__builtin_offsetof(St, cycles);
    case 6: return (long)__builtin_offsetof(St, l1);
    case 7: return (long)__builtin_offsetof(St, mmio_calls);
    case 8: return (long)sizeof(PE);
    }
    return -1;
}
#endif

#endif
