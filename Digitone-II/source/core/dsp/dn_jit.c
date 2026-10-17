/* L'esecutore a blocchi dell'interprete (docs/DSP-INTERPRETE.md, "Esecutore a blocchi"): ogni
 * tratto di istruzioni in sequenza senza effetti sul flusso diventa un breve pezzo di codice
 * macchina emesso a runtime che chiama in fila i gestori (chiamate dirette predette: ~0,5 ns
 * contro ~1,6 ns del salto indiretto per istruzione); il resto (salti, fine dei loop, slot,
 * cio' che tocca interrupt e MODE1) lo esegue questo driver in C, come l'epilogo completo
 * dell'esecutore a goto calcolato. Niente nel codice emesso dipende dal firmware: sono solo
 * chiamate ai gestori del plugin con la Ins decodificata come argomento. arm64 (Mac) e x86-64
 * (Windows, ABI Microsoft); altrove, o senza memoria eseguibile, resta l'altro esecutore. */
#include "dn_interp_names.h"
#define EXP_NOSIMD
#define IDLE_WAITING 0u
#define IDLE_READY0 0u
#include "rt/rt.h"
#include "dn_interp_int.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
#else
#include <sys/mman.h>
#include <pthread.h>
#if defined(__APPLE__)
#include <libkern/OSCacheControl.h>
#endif
#endif

#if defined(__aarch64__)
#define JIT_ARCH 1
#elif (defined(__x86_64__) || defined(_M_X64)) && defined(_WIN32)
#define JIT_ARCH 2
#else
#define JIT_ARCH 0
#endif

/* ------------------------------------------------------------------- gestori */
typedef void (*hplain_fn)(St *, const Ins *);
typedef uint32_t (*hflow_fn)(St *, const Ins *, uint32_t);
#define HSUF(x) x##_n
#include "dn_jit_h.inc"
extern const hplain_fn *dni_tplain_s(void);
extern const hflow_fn *dni_tflow_s(void);

/* ----------------------------------------------------------- memoria eseguibile */
#define JARENA_BYTES (48u << 20)
#define JMAX 16                                   /* chiamate per blocco: la scadenza si controlla a ogni blocco */
#define JPOOL 4096                                /* JBlk per pezzo */

typedef struct JBlk {
    void (*code)(St *);
    uint32_t ncalls;
    const Ins *term;                              /* l'istruzione che chiude il blocco (NULL: continua) */
    uint32_t term_pc;                             /* suo indirizzo, o dove continua */
    uint8_t kind;                                 /* 0 continua; 1 l'ultima chiamata e' una plain con epilogo (fine loop/slot); 2 terminatore semplice; 3 speciale/PC cattivo */
    uint32_t fall;                                /* kind 1: l'indirizzo dopo l'ultima istruzione */
    uint32_t c_pc;                                /* cache a un posto: il blocco dopo questo (per l'ultimo pc visto) */
    struct JBlk *c_blk;
} JBlk;

typedef struct JPoolChunk { struct JPoolChunk *next; JBlk b[JPOOL]; } JPoolChunk;

typedef struct JitState {
    uint8_t *base;
    size_t used;
    JBlk **blk[2];                                /* per indice di ins_index; [0] senza SIMD, [1] con SIMD */
    JPoolChunk *pool;
    int pool_n;
    uint64_t st_blocks, st_calls, st_terms, st_miss, st_built;
    uint64_t st_termop[OPS_TOTAL], st_termfl[256];
} JitState;

static uint8_t *jit_map(size_t n)
{
#if defined(_WIN32)
    return (uint8_t *)VirtualAlloc(NULL, n, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
#else
    int flags = MAP_PRIVATE | MAP_ANON;
#if defined(__APPLE__)
    flags |= MAP_JIT;
#endif
    void *p = mmap(NULL, n, PROT_READ | PROT_WRITE | PROT_EXEC, flags, -1, 0);
    return p == MAP_FAILED ? NULL : (uint8_t *)p;
#endif
}

static void jit_wr(int on)
{
#if defined(__APPLE__) && defined(__aarch64__)
    pthread_jit_write_protect_np(!on);
#else
    (void)on;
#endif
}

static void jit_flush(void *p, size_t n)
{
#if defined(_WIN32)
    FlushInstructionCache(GetCurrentProcess(), p, n);
#elif defined(__APPLE__)
    sys_icache_invalidate(p, n);
#else
    __builtin___clear_cache((char *)p, (char *)p + n);
#endif
}

static JitState *jit_state(dni *I)
{
    JitState *J = (JitState *)I->jit;
    if (J) return J;
    if (!JIT_ARCH) return NULL;
    J = (JitState *)calloc(1, sizeof *J);
    if (!J) return NULL;
    J->base = jit_map(JARENA_BYTES);
    J->blk[0] = (JBlk **)calloc(INS_REGIONS + 16, sizeof(JBlk *));
    if (!J->base || !J->blk[0]) {
        if (J->base) {
#if defined(_WIN32)
            VirtualFree(J->base, 0, MEM_RELEASE);
#else
            munmap(J->base, JARENA_BYTES);
#endif
        }
        free(J->blk[0]); free(J);
        I->jit_off = 1;
        return NULL;
    }
    J->pool_n = JPOOL;
    I->jit = J;
    return J;
}

static void pool_free(JitState *J)
{
    while (J->pool) { JPoolChunk *n = J->pool->next; free(J->pool); J->pool = n; }
    J->pool_n = JPOOL;
}

void dni_jit_free(dni *I)
{
    JitState *J = (JitState *)I->jit;
    if (!J) return;
    if (getenv("DNI_VERBOSE")) fprintf(stderr, "jit: blocchi eseguiti %llu, chiamate %llu (%.1f per blocco), terminatori %llu, mancati nella cache a un posto %llu, memoria emessa %zu KB\n", (unsigned long long)J->st_blocks, (unsigned long long)J->st_calls, (double)J->st_calls / (J->st_blocks ? J->st_blocks : 1), (unsigned long long)J->st_terms, (unsigned long long)J->st_miss, J->used >> 10);
    if (getenv("DNI_VERBOSE")) {
        for (int n = 0; n < 14; n++) {
            int b = -1;
            for (int i = 0; i < OPS_TOTAL; i++) if (J->st_termop[i] && (b < 0 || J->st_termop[i] > J->st_termop[b])) b = i;
            if (b < 0) break;
            fprintf(stderr, "  terminatore xop %d: %llu\n", b, (unsigned long long)J->st_termop[b]);
            J->st_termop[b] = 0;
        }
        for (int n = 0; n < 8; n++) {
            int b = -1;
            for (int i = 0; i < 256; i++) if (J->st_termfl[i] && (b < 0 || J->st_termfl[i] > J->st_termfl[b])) b = i;
            if (b < 0) break;
            fprintf(stderr, "  terminatore flags %#x: %llu\n", b, (unsigned long long)J->st_termfl[b]);
            J->st_termfl[b] = 0;
        }
    }
#if defined(_WIN32)
    VirtualFree(J->base, 0, MEM_RELEASE);
#else
    munmap(J->base, JARENA_BYTES);
#endif
    pool_free(J);
    free(J->blk[0]); free(J->blk[1]); free(J);
    I->jit = NULL;
}

/* tutti i blocchi da rifare: un'istruzione gia' in un blocco ha cambiato i flag (fine loop, slot) */
static void jit_reset(dni *I)
{
    JitState *J = (JitState *)I->jit;
    if (!J) return;
    memset(J->blk[0], 0, (INS_REGIONS + 16) * sizeof(JBlk *));
    if (J->blk[1]) memset(J->blk[1], 0, (INS_REGIONS + 16) * sizeof(JBlk *));
    J->used = 0;
    pool_free(J);
    for (uint32_t k = 0; k < INS_REGIONS; k++) if (I->tab[k].rsvd) I->tab[k].rsvd = 0;
}

static JBlk *pool_get(JitState *J)
{
    if (J->pool_n >= JPOOL) {
        JPoolChunk *c = (JPoolChunk *)calloc(1, sizeof *c);
        if (!c) return NULL;
        c->next = J->pool; J->pool = c; J->pool_n = 0;
    }
    return &J->pool->b[J->pool_n++];
}

/* ------------------------------------------------------------------ emissione */
static size_t emit_block(uint8_t *mem, const Ins *const *ins, const uint32_t *fn_idx, int n, void **fns)
{
    size_t p = 0;
    (void)fn_idx;
#if JIT_ARCH == 1
    size_t ncode = 4u * (2 + 4 * (size_t)n + 3);
    size_t lit0 = (ncode + 7) & ~(size_t)7;
    size_t dat0 = (lit0 + 8 * (size_t)n + 15) & ~(size_t)15;
    uint32_t *c = (uint32_t *)mem;
    c[p++] = 0xA9BF7BF3u;                 /* stp x19, x30, [sp, #-16]! */
    c[p++] = 0xAA0003F3u;                 /* mov x19, x0 */
    for (int i = 0; i < n; i++) {
        size_t at;
        int64_t off;
        memcpy(mem + lit0 + 8 * (size_t)i, &fns[i], 8);
        memcpy(mem + dat0 + sizeof(Ins) * (size_t)i, ins[i], sizeof(Ins));
        c[p++] = 0xAA1303E0u;             /* mov x0, x19 */
        at = p * 4;
        off = (int64_t)(dat0 + sizeof(Ins) * (size_t)i) - (int64_t)at;       /* adr x1, ins_i */
        c[p++] = 0x10000001u | (uint32_t)((off & 3) << 29) | (uint32_t)(((off >> 2) & 0x7FFFF) << 5);
        at = p * 4;
        off = (int64_t)(lit0 + 8 * (size_t)i) - (int64_t)at;                  /* ldr x16, lit_i */
        c[p++] = 0x58000010u | (uint32_t)(((off >> 2) & 0x7FFFF) << 5);
        c[p++] = 0xD63F0200u;             /* blr x16 */
    }
    c[p++] = 0xA8C17BF3u;                 /* ldp x19, x30, [sp], #16 */
    c[p++] = 0xD65F03C0u;                 /* ret */
    c[p++] = 0xD503201Fu;                 /* nop (riempimento) */
    return dat0 + sizeof(Ins) * (size_t)n;
#elif JIT_ARCH == 2
    size_t ncode = 8 + 16 * (size_t)n + 6;
    size_t lit0 = (ncode + 7) & ~(size_t)7;
    size_t dat0 = (lit0 + 8 * (size_t)n + 15) & ~(size_t)15;
    static const uint8_t pro[8] = {0x53, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x89, 0xCB};   /* push rbx; sub rsp,32; mov rbx,rcx */
    memcpy(mem, pro, 8); p = 8;
    for (int i = 0; i < n; i++) {
        int32_t rel;
        memcpy(mem + lit0 + 8 * (size_t)i, &fns[i], 8);
        memcpy(mem + dat0 + sizeof(Ins) * (size_t)i, ins[i], sizeof(Ins));
        mem[p++] = 0x48; mem[p++] = 0x89; mem[p++] = 0xD9;                           /* mov rcx, rbx */
        mem[p++] = 0x48; mem[p++] = 0x8D; mem[p++] = 0x15;                           /* lea rdx, [rip+rel32] */
        rel = (int32_t)((int64_t)(dat0 + sizeof(Ins) * (size_t)i) - (int64_t)(p + 4));
        memcpy(mem + p, &rel, 4); p += 4;
        mem[p++] = 0xFF; mem[p++] = 0x15;                                            /* call [rip+rel32] */
        rel = (int32_t)((int64_t)(lit0 + 8 * (size_t)i) - (int64_t)(p + 4));
        memcpy(mem + p, &rel, 4); p += 4;
    }
    mem[p++] = 0x48; mem[p++] = 0x83; mem[p++] = 0xC4; mem[p++] = 0x20;              /* add rsp, 32 */
    mem[p++] = 0x5B; mem[p++] = 0xC3;                                                /* pop rbx; ret */
    return dat0 + sizeof(Ins) * (size_t)n;
#else
    (void)mem; (void)ins; (void)n; (void)fns;
    return p;
#endif
}

static JBlk *jit_build(dni *I, JitState *J, uint32_t pc, const hplain_fn *tp, const hflow_fn *tf)
{
    const Ins *list[JMAX];
    void *fns[JMAX];
    uint32_t cur = pc, k, fall = 0;
    int n = 0, kind = 0;
    const Ins *term = NULL;
    JBlk *b;
    size_t need, bytes;
    uint8_t *mem;
    for (;;) {
        Ins *in = ins_get(I, cur);
        if (!in || in->op == OP_BAD) { kind = 3; term = NULL; break; }
        if (in->op == OP_HALT) { kind = 3; term = in; break; }
        if (in->flags == IF_EPI && tp[in->xop] && n < JMAX) {       /* plain con solo l'epilogo: ultima chiamata del blocco */
            list[n] = in; fns[n] = (void *)tp[in->xop]; n++;
            if (!ins_index(cur, &k)) I->tab[k].rsvd = 1;
            fall = cur + in->words;
            kind = 1;
            break;
        }
        if (in->flags) { term = in; kind = (in->flags == IF_EPI && (tf[in->xop] || tp[in->xop])) ? 2 : 3; break; }
        if (n == JMAX) break;
        if (!tp[in->xop]) { term = in; kind = (tf[in->xop]) ? 2 : 3; break; }
        list[n] = in; fns[n] = (void *)tp[in->xop]; n++;
        if (!ins_index(cur, &k)) I->tab[k].rsvd = 1;
        cur += in->words;
    }
    need = 64 + (size_t)n * (16 + 8 + sizeof(Ins)) + 128;
    if (J->used + need + sizeof(JBlk) > JARENA_BYTES) { jit_reset(I); return NULL; }
    b = pool_get(J);
    if (!b) return NULL;
    b->ncalls = (uint32_t)n; b->term = term; b->term_pc = cur; b->kind = (uint8_t)kind; b->fall = fall; b->code = NULL;
    b->c_pc = 0xFFFFFFFFu; b->c_blk = NULL;
    if (n) {
        mem = J->base + J->used;
        mem = (uint8_t *)(((uintptr_t)mem + 15) & ~(uintptr_t)15);
        jit_wr(1);
        bytes = emit_block(mem, list, NULL, n, fns);
        jit_wr(0);
        jit_flush(mem, bytes);
        J->used = (size_t)(mem - J->base) + bytes;
        b->code = (void (*)(St *))(void *)mem;
    }
    return b;
}

/* ------------------------------------------------------------------- il driver */
static inline JBlk *jit_get(dni *I, JitState *J, JBlk **tab, uint32_t pc, const hplain_fn *tp, const hflow_fn *tf)
{
    uint32_t k;
    JBlk *b;
    if (UNLIKELY(ins_index(pc, &k))) return NULL;
    b = tab[k];
    if (UNLIKELY(!b)) {
        b = jit_build(I, J, pc, tp, tf);
        if (b) tab[k] = b;
    }
    return b;
}

int dni_exec_jit(St *st)
{
    dni *I = dni_cur;
    JitState *J = jit_state(I);
    int64_t cyc = st->cycles;
    uint32_t pc;
    int mode = (st->mode1 >> PEYEN) & 1;
    const hplain_fn *tp = mode ? dni_tplain_s() : tplain_n;
    const hflow_fn *tf = mode ? dni_tflow_s() : tflow_n;
    JBlk **tab, *b;
    if (!J) return mode ? dni_exec_simd(st) : dni_exec_nosimd(st);
    if (mode && !J->blk[1]) {
        J->blk[1] = (JBlk **)calloc(INS_REGIONS + 16, sizeof(JBlk *));
        if (!J->blk[1]) return dni_exec_simd(st);
    }
    tab = J->blk[mode];
    if (UNLIKELY(cyc >= st->deadline)) return 0;
    pc = st->pc;
    for (;;) {
        if (UNLIKELY(I->jflush)) { I->jflush = 0; jit_reset(I); }
        b = jit_get(I, J, tab, pc, tp, tf);
        if (UNLIKELY(!b)) {
            if (I->jflush) continue;
            st->cycles = cyc; st->pc = pc;
            return mode ? dni_exec_simd(st) : dni_exec_nosimd(st);
        }
        if (UNLIKELY(I->jflush)) continue;
        for (;;) {
            JBlk *nb;
            if (LIKELY(b->ncalls != 0)) {
                st->cycles = cyc;
                b->code(st);
                cyc += b->ncalls;
            }
            pc = b->term_pc;
            if (b->kind == 1) {                                  /* l'ultima chiamata era una plain di fine loop o slot */
                uint32_t nxt = b->fall;
                st->cycles = cyc;
                if (st->nloops) nxt = loop_end(st, pc, nxt, b->fall);
                if (UNLIKELY(st->dly_active)) nxt = delayed_step(st, pc, nxt);
                pc = nxt;
            } else if (b->kind) {
                const Ins *in = b->term;
                uint32_t nxt, fall;
                if (UNLIKELY(b->kind == 3)) {
                    if (!in) { I->bad_pc = pc; st->cycles = cyc; st->pc = pc; st->stop_reason = 3; return 3; }
                    if (in->op == OP_HALT) { st->cycles = cyc; st->pc = pc; st->stop_reason = 6; return 6; }
                }
                nxt = pc + in->words; fall = nxt;
                st->cycles = cyc; st->pc = pc;
                if (UNLIKELY(b->kind == 3) && (in->flags & (IF_IDLELOOP | IF_DELAYSKIP | IF_BACKJUMP))) {
                    if (in->flags & IF_IDLELOOP) { if (idle_hook(st)) return 0; }
                    if ((in->flags & IF_DELAYSKIP) && I->fast_delays) {
                        const int32_t *a = in->a;
                        uint32_t addr = dag_access(st, a[1], (uint32_t)a[2], 0, 4);
                        if (ld(st, addr, 4, 0) < (uint32_t)a[8]) { sto(st, addr, (uint32_t)a[8], 4); I->skipped_delay++; }
                    }
                    if ((in->flags & IF_BACKJUMP) && I->fast_delays) dni_check_backjump(I, pc, (Ins *)in);
                }
                if (tf[in->xop]) nxt = tf[in->xop](st, in, pc);
                else if (tp[in->xop]) tp[in->xop](st, in);
                else { I->bad_pc = pc; st->stop_reason = 3; return 3; }
                cyc++;
                I->last_pc = pc;
                st->cycles = cyc;
                if (UNLIKELY(in->flags & IF_LOOPSTK)) dni_mark_loop_end(I);
                if (st->nloops) nxt = loop_end(st, pc, nxt, fall);
                if (UNLIKELY(st->dly_active)) nxt = delayed_step(st, pc, nxt);
                pc = nxt;
            }
            if (UNLIKELY(cyc >= st->deadline) || UNLIKELY(I->jflush)) { st->cycles = cyc; st->pc = pc; return 0; }
            if (LIKELY(pc == b->c_pc)) nb = b->c_blk;
            else {
                nb = jit_get(I, J, tab, pc, tp, tf);
                if (UNLIKELY(!nb)) { st->cycles = cyc; st->pc = pc; return 0; }
                if (UNLIKELY(I->jflush)) { st->cycles = cyc; st->pc = pc; return 0; }
                b->c_pc = pc; b->c_blk = nb;
            }
            b = nb;
        }
    }
}
