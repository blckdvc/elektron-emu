/* Il traduttore SHARC+ -> C nel plugin (incluso in fondo a dn_interp.c): porta in C di
 * emu/sharc/rec/gen.py (generate, discover, body, liveness, nuclei dei loop, blocchi e
 * dispatcher) con la decodifica dell'interprete. Produce gli STESSI sorgenti di
 * tools/dsp_gensrc.py (dsp_main.c, dsp_001.c ...): ogni riga di testo qui sotto e' la
 * stringa del generatore Python, e il confronto e' byte per byte (dsptool gensrc).
 *
 * Parte da uno stato DNDS (la memoria dopo il boot dell'interprete), legge il programma dalle
 * regioni di codice, e consegna i file a una funzione (`sink`). Il risultato e' un dato derivato
 * dal firmware: il plugin lo scrive solo nella cartella dell'utente e lo compila li'
 * (vst/NativeBuild.cpp). Configurazione fissa: quella predefinita del Python (niente REC_FAST,
 * REC_VPROF, REC_VSPEC; REC_SEPSTORE, REC_SIMD2, REC_KERNELS e REC_KCOUNTER attivi).
 */

#include "dn_gen_seeds.inc"

#define G_WINDOW 0x200u
#define G_GAP 0x4000u
#define G_CHECK_EVERY 8

/* ------------------------------------------------------------------ buffer di testo */
typedef struct { char *p; size_t n, cap; } SB;

static void sb_grow(SB *s, size_t more)
{
    if (s->n + more + 1 > s->cap) {
        size_t c = s->cap ? s->cap : 256;
        while (c < s->n + more + 1) c *= 2;
        s->p = (char *)realloc(s->p, c);
        s->cap = c;
    }
}
static void sb_puts(SB *s, const char *t)
{
    size_t l = strlen(t);
    sb_grow(s, l);
    memcpy(s->p + s->n, t, l + 1);
    s->n += l;
}
static void sb_printf(SB *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void sb_printf(SB *s, const char *fmt, ...)
{
    va_list ap;
    int l;
    va_start(ap, fmt);
    l = vsnprintf(NULL, 0, fmt, ap);
    va_end(ap);
    sb_grow(s, (size_t)l);
    va_start(ap, fmt);
    vsnprintf(s->p + s->n, (size_t)l + 1, fmt, ap);
    va_end(ap);
    s->n += (size_t)l;
}
/* una riga di un elenco che Python unisce con '\n' (la '\n' si mette dopo, in sb_line_end) */
#define LN(sb, ...) do { sb_printf(sb, __VA_ARGS__); sb_puts(sb, "\n"); } while (0)

/* ------------------------------------------------------------------ istruzioni scoperte */
typedef struct {
    uint32_t pc;
    Dec d;
    char *body;                      /* NULL: forma non supportata (gen.body -> None) */
    uint32_t live;                   /* flag vivi dopo l'istruzione */
    uint32_t reads, kills, term;     /* flag_effects; term: flag letti dalla condizione della fine di un loop */
    uint32_t live_in;
    int32_t ls_head;                 /* inizi di loop che finiscono qui (lista in G->ls) */
    int32_t slot2_owner;             /* indice del salto ritardato di cui e' il secondo slot, +1 */
    int32_t nsucc;                   /* -1 = successori sconosciuti */
    uint32_t *succ;
    uint8_t slot, loopend, check;
    uint8_t in_chunk_flag;
} GI;

typedef struct { uint32_t start; int32_t next; } GLs;

typedef struct {
    St *st;
    int err;
    char *errmsg;
    GI *code;
    uint32_t ncode, cap;             /* in ordine di scoperta */
    uint32_t *idx;                   /* ins_index -> indice in code + 1 */
    GLs *ls;
    uint32_t nls, lscap;
    uint32_t *bl;                    /* indici (in code) delle istruzioni con corpo, per pc crescente */
    uint32_t nbl;
    uint32_t idle_loop;              /* 0 = nessuno */
} Gen;

static GI *g_at(Gen *G, uint32_t pc)
{
    uint32_t k;
    if (ins_index(pc, &k)) return NULL;
    return G->idx[k] ? &G->code[G->idx[k] - 1] : NULL;
}

static int g_has_handler(int t)
{
    switch (t) {
    case T_1a: case T_1b: case T_2a: case T_2b: case T_2c: case T_3a: case T_3b: case T_3c: case T_3d:
    case T_4a: case T_4b: case T_4d: case T_5a_move: case T_5a_swap: case T_5b_move: case T_5b_swap:
    case T_6a_mem: case T_6a_nomem: case T_7a: case T_7b: case T_7d: case T_8a_abs: case T_8a_rel:
    case T_9a_abs: case T_9a_rel: case T_9b_abs: case T_9b_rel: case T_11a: case T_11c:
    case T_12a_imm: case T_12a_ureg: case T_13a: case T_14a: case T_14d: case T_15a: case T_15b:
    case T_16a: case T_16b: case T_17a: case T_17b: case T_18a: case T_19a: case T_19a_bitrev:
    case T_20a: case T_21a: case T_21c: case T_22a: case T_22c: case T_25a_direct: case T_25a_pcrel:
    case T_25a_rframe: case T_25c_rframe: case T_26a: case T_23p_undoc16: case T_8p_undoc48:
    case T_19p_undoc48:
        return 1;
    }
    return 0;
}

static int g_in_regions(uint32_t pc)
{
    return (pc - 0x90000u < 0x400u) || (pc - 0x120000u < 0xE0000u) || (pc - 0xB80000u < 0x80000u);
}

#define GFV(g, n) ((g)->d.f[FN_##n])
#define GHAS(g, n) (((g)->d.has >> FN_##n) & 1)
#define D_(x) ((long long)(x))

static uint32_t g_comp(const GI *g) { return GHAS(g, compute) ? GFV(g, compute) : 0; }
static int g_cond(const GI *g) { return GHAS(g, cond) ? (int)GFV(g, cond) : 31; }

/* ------------------------------------------------------------------ scoperta del codice */
static int g_discover(Gen *G, const uint32_t *seeds, uint32_t nseeds)
{
    uint32_t *q = NULL, qh = 0, qn = 0, qcap = 0;
    uint32_t i;
#define QPUSH(v) do { if (qn == qcap) { qcap = qcap ? qcap * 2 : 4096; q = (uint32_t *)realloc(q, qcap * 4); } q[qn++] = (v); } while (0)
    for (i = 0; i < nseeds; i++) QPUSH(seeds[i]);
    while (qh < qn) {
        uint32_t pc = q[qh++];
        Dec d;
        GI *g;
        uint32_t t;
        if (!g_in_regions(pc) || g_at(G, pc)) continue;
        if (sh_decode(G->st, pc, &d) || !g_has_handler(d.type)) continue;
        /* riempimenti a zero: dopo 8 NOP a zero di fila ci si ferma */
        if (d.type == T_21a && d.raw == 0) {
            int run = 0;
            uint32_t a = pc;
            while (run < 8) {
                GI *prev = g_at(G, a - (uint32_t)d.words);
                if (!prev || prev->d.type != T_21a || prev->d.raw != 0) break;
                run++;
                a -= (uint32_t)d.words;
            }
            if (run >= 8) continue;
        }
        if (G->ncode == G->cap) {
            G->cap = G->cap ? G->cap * 2 : 65536;
            G->code = (GI *)realloc(G->code, (size_t)G->cap * sizeof(GI));
        }
        g = &G->code[G->ncode];
        memset(g, 0, sizeof *g);
        g->pc = pc;
        g->d = d;
        g->ls_head = -1;
        {
            uint32_t k;
            ins_index(pc, &k);
            G->idx[k] = ++G->ncode;
        }
        if (targets_of(pc, &d, &t)) QPUSH(t);
        QPUSH(pc + (uint32_t)d.words);
    }
    free(q);
#undef QPUSH
    return 0;
}

/* ------------------------------------------------------------------ vivezza dei flag */
static int g_successors(Gen *G, GI *g, uint32_t *out, int *n)
{
    int t = g->d.type, c = g_cond(g);
    uint32_t fall = g->pc + (uint32_t)g->d.words, tg;
    *n = 0;
    if (is_flow(t)) {
        if (t == T_11a || t == T_11c || t == T_23p_undoc16 || t == T_9a_abs || t == T_9b_abs) return -1;
        if (GFV(g, b)) return -1;
        if (!GFV(g, j)) {
            if (!targets_of(g->pc, &g->d, &tg)) return -1;      /* Python: IndexError; non succede */
            out[(*n)++] = tg;
            if (c != 31) out[(*n)++] = fall;
            return 0;
        }
        out[(*n)++] = fall;
        return 0;
    }
    out[(*n)++] = fall;
    return 0;                        /* gli inizi dei loop che finiscono qui li aggiunge g_liveness */
}

static void g_liveness(Gen *G)
{
    uint32_t i, ncode = G->ncode;
    uint32_t *work, wh = 0, wn = 0, wcap;
    uint8_t *queued;
    uint32_t *pstart, *pnext_cnt, *plist;
    GI *code = G->code;

    /* fine loop -> inizi del corpo, e flag letti dalla condizione di fine */
    for (i = 0; i < ncode; i++) {
        uint32_t e;
        GI *g = &code[i];
        if (loop_end_of(g->pc, &g->d, &e)) {
            GI *eg = g_at(G, e);
            if (eg) {
                if (G->nls == G->lscap) {
                    G->lscap = G->lscap ? G->lscap * 2 : 1024;
                    G->ls = (GLs *)realloc(G->ls, G->lscap * sizeof(GLs));
                }
                G->ls[G->nls].start = g->pc + (uint32_t)g->d.words;
                G->ls[G->nls].next = eg->ls_head;
                eg->ls_head = (int32_t)G->nls++;
                if (g->d.type == T_13a) {
                    uint32_t term = GFV(g, term);
                    if (term != 15 && term != 31) eg->term |= cond_reads((int)term);
                }
            }
        }
    }
    /* effetti e successori */
    for (i = 0; i < ncode; i++) {
        GI *g = &code[i];
        uint32_t out[64];
        int n, r;
        flag_effects(&g->d, &g->reads, &g->kills);
        r = g_successors(G, g, out, &n);
        if (r < 0) g->nsucc = -1;
        else {
            if (!is_flow(g->d.type)) {
                int32_t l;
                for (l = g->ls_head; l >= 0 && n < 64; l = G->ls[l].next) out[n++] = G->ls[l].start;
            }
            g->nsucc = n;
            g->succ = (uint32_t *)malloc((size_t)(n ? n : 1) * 4);
            memcpy(g->succ, out, (size_t)n * 4);
        }
    }
    /* secondo slot di un salto ritardato: i suoi successori sono quelli del salto */
    for (i = 0; i < ncode; i++) {
        GI *g = &code[i];
        if (is_db(&g->d)) {
            uint32_t a1 = g->pc + (uint32_t)g->d.words;
            GI *s1 = g_at(G, a1);
            if (s1) {
                GI *s2 = g_at(G, a1 + (uint32_t)s1->d.words);
                if (s2) s2->slot2_owner = (int32_t)i + 1;
            }
        }
    }
    for (i = 0; i < ncode; i++) {
        GI *s2 = &code[i], *b;
        int t;
        uint32_t tg;
        if (!s2->slot2_owner) continue;
        b = &code[s2->slot2_owner - 1];
        t = b->d.type;
        free(s2->succ);
        s2->succ = NULL;
        if (t == T_25a_direct || t == T_25a_pcrel) {
            if (targets_of(b->pc, &b->d, &tg)) { s2->nsucc = 1; s2->succ = (uint32_t *)malloc(4); s2->succ[0] = tg; }
            else s2->nsucc = -1;
        } else if (t == T_11a || t == T_11c || t == T_23p_undoc16 || t == T_9a_abs || t == T_9b_abs
                   || (is_flow(t) && GFV(b, b))) {
            s2->nsucc = -1;
        } else if (targets_of(b->pc, &b->d, &tg)) {
            uint32_t fall2 = s2->pc + (uint32_t)s2->d.words;
            s2->nsucc = g_cond(b) != 31 ? 2 : 1;
            s2->succ = (uint32_t *)malloc(8);
            s2->succ[0] = tg;
            if (s2->nsucc == 2) s2->succ[1] = fall2;
        } else s2->nsucc = -1;
    }
    /* predecessori (solo fra istruzioni scoperte) in forma CSR */
    pstart = (uint32_t *)calloc(ncode + 2, 4);
    for (i = 0; i < ncode; i++) {
        int32_t k;
        for (k = 0; k < code[i].nsucc; k++) {
            GI *x = g_at(G, code[i].succ[k]);
            if (x) pstart[(x - code) + 1]++;
        }
    }
    for (i = 0; i < ncode; i++) pstart[i + 1] += pstart[i];
    plist = (uint32_t *)malloc((size_t)(pstart[ncode] + 1) * 4);
    pnext_cnt = (uint32_t *)calloc(ncode + 1, 4);
    for (i = 0; i < ncode; i++) {
        int32_t k;
        for (k = 0; k < code[i].nsucc; k++) {
            GI *x = g_at(G, code[i].succ[k]);
            if (x) {
                uint32_t xi = (uint32_t)(x - code);
                plist[pstart[xi] + pnext_cnt[xi]++] = i;
            }
        }
    }
    /* punto fisso all'indietro */
    wcap = ncode + 16;
    work = (uint32_t *)malloc((size_t)wcap * 4);
    queued = (uint8_t *)malloc(ncode + 1);
    for (i = 0; i < ncode; i++) { work[i] = i; queued[i] = 1; code[i].live_in = 0; code[i].live = 0; }
    wn = ncode;
    while (wh != wn) {
        uint32_t ci = work[wh % wcap], lo, li;
        GI *g = &code[ci];
        wh++;
        queued[ci] = 0;
        if (g->nsucc < 0) lo = 0xFFFFFFFFu;
        else {
            int32_t k;
            lo = 0;
            for (k = 0; k < g->nsucc; k++) {
                GI *x = g_at(G, g->succ[k]);
                lo |= x ? x->live_in : 0xFFFFFFFFu;
            }
        }
        lo |= g->term;
        g->live = lo;
        li = g->reads | (lo & ~g->kills);
        if (li != g->live_in) {
            uint32_t p;
            g->live_in = li;
            for (p = pstart[ci]; p < pstart[ci + 1]; p++) {
                uint32_t pi = plist[p];
                if (!queued[pi]) {
                    queued[pi] = 1;
                    work[wn % wcap] = pi;
                    wn++;
                }
            }
        }
    }
    free(work); free(queued); free(pstart); free(pnext_cnt); free(plist);
}

/* ------------------------------------------------------------------ gen.body() */
static int g_after_slots(Gen *G, const GI *g, uint32_t *out)
{
    if (after_slots(G->st, g->pc, g->d.words, out)) {
        G->err = 1;
        return -1;
    }
    return 0;
}

static void g_branch(Gen *G, const GI *g, SB *b, const char *ok, const char *target)
{
    long long call = D_(GFV(g, b)), la = D_(GFV(g, a)), db = D_(GFV(g, j));
    long long ci = GHAS(g, ci) ? D_(GFV(g, ci)) : 0;
    uint32_t ret_db = 0;
    if (db) g_after_slots(G, g, &ret_db);
    sb_printf(b, "if (%s) branch_common(st, 0x%x, &nxt, %s, %lld, %lld, %lld, %lld, 0x%x);", ok, g->pc, target, call, la, db, ci, ret_db);
}

/* -> testo (malloc'd; "" se vuoto) o NULL per una forma non supportata */
static char *g_body(Gen *G, const GI *g)
{
    SB b = {0};
    const int c = g_cond(g);
    const uint32_t comp = g_comp(g), live = g->live, pc = g->pc;
    const int t = g->d.type;
    char tgs[96];
    sb_puts(&b, "");
    switch (t) {
    case T_1a: case T_1b:
        sb_printf(&b, "op_1a(st, %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld, 0x%x, 0x%x);",
                  D_(GFV(g, dmi)), D_(GFV(g, dmm)), 8 + D_(GFV(g, pmi)), 8 + D_(GFV(g, pmm)),
                  D_(GFV(g, dmd)), D_(GFV(g, pmd)), D_(GFV(g, dmdreg)), D_(GFV(g, pmdreg)), comp, live);
        return b.p;
    case T_2a:
        if (comp) sb_printf(&b, "cmp_(st, 0x%x, pes(st, %d), 0x%x);", comp, c, live);
        return b.p;
    case T_2b:
        if (comp) sb_printf(&b, "execute(st, 0x%x, -1, 0x%x);", comp, live);
        return b.p;
    case T_2c:
        sb_printf(&b, "execute_short(st, 0x%x, -1, 0x%x);", GFV(g, compute), live);
        return b.p;
    case T_3a: case T_3b: case T_3d: {
        long long u = D_(GFV(g, u)), i = D_(GFV(g, i)), m = D_(GFV(g, m)), gg = D_(GFV(g, g)), dd = D_(GFV(g, d));
        long long l = D_(GFV(g, l));
        int size, sgn;
        if (t != T_3a && GHAS(g, x)) size_of(t, &g->d, &size, &sgn);
        else { size = l ? 8 : 4; sgn = 0; }
        sb_printf(&b, "op_3(st, %d, %lld, %lld, %lld, %lld, 0x%x, %d, %d, 0x%x, 0x%x, %d);",
                  c, u, i + 8 * gg, m + 8 * gg, dd, GFV(g, ureg), size, sgn, comp, live, -1);
        return b.p;
    }
    case T_3c:
        sb_printf(&b, "mem_move(st, %lld, %lld, get_dag(st, KM, %lld), 1, %d, 4, 0, -1, %d);",
                  D_(GFV(g, dreg)), D_(GFV(g, dmi)), D_(GFV(g, dmm)), GFV(g, d) ? 1 : 0, -1);
        return b.p;
    case T_4a: case T_4b: case T_4d: {
        long long i = D_(GFV(g, i)), gg = D_(GFV(g, g)), dd = D_(GFV(g, d)), u = D_(GFV(g, u));
        int size = 4, sgn = 0;
        if (t != T_4a) size_of(t, &g->d, &size, &sgn);
        sb_printf(&b, "op_4(st, %d, %lld, %lld, %d, %lld, %lld, %d, %d, 0x%x, 0x%x, %d);",
                  c, u, i + 8 * gg, sext_i(GFV(g, data), 6), dd, D_(GFV(g, dreg)), size, sgn, comp, live, -1);
        return b.p;
    }
    case T_5a_move: case T_5b_move: {
        uint32_t src = (GFV(g, srcureghigh) << 2) | GFV(g, srcureglow);
        sb_printf(&b, "op_5move(st, %d, 0x%x, 0x%x, 0x%x, 0x%x);", c, src, GFV(g, dstureg), comp, live);
        return b.p;
    }
    case T_5a_swap: case T_5b_swap:
        sb_printf(&b, "op_5swap(st, %d, %lld, %lld, 0x%x, 0x%x);", c, D_(GFV(g, dreg)), D_(GFV(g, cdreg)), comp, live);
        return b.p;
    case T_6a_nomem: {
        uint32_t ex = GHAS(g, dataex) ? GFV(g, dataex) : 0;
        sb_printf(&b, "op_6nomem(st, %d, 0x%x, 0x%x, 0x%x);", c, GFV(g, shiftimm), ex, live);
        return b.p;
    }
    case T_6a_mem: {
        long long i = D_(GFV(g, i)), m = D_(GFV(g, m)), gg = D_(GFV(g, g)), dd = D_(GFV(g, d));
        uint32_t ex = GHAS(g, dataex) ? GFV(g, dataex) : 0;
        sb_printf(&b, "op_6mem(st, %d, %lld, %lld, %lld, %lld, 0x%x, 0x%x, 0x%x, %d);",
                  c, i + 8 * gg, m + 8 * gg, dd, D_(GFV(g, dreg)), GFV(g, shiftimm), ex, live, -1);
        return b.p;
    }
    case T_7a: {
        int src, dst, gg = (int)GFV(g, g);
        int m = (int)((g->d.raw >> 27) & 7) + 8 * gg, w = (int)((g->d.raw >> 39) & 1), l = (int)((g->d.raw >> 23) & 1);
        int kind = w ? 2 : (l ? 1 : 0);
        regs19(&g->d, &src, &dst);
        sb_printf(&b, "op_7a(st, %d, %d, %d, %d, %d, 0x%x, 0x%x);", c, src, dst, m, kind, comp, live);
        return b.p;
    }
    case T_7b: {
        int src, dst;
        regs19(&g->d, &src, &dst);
        sb_printf(&b, "op_7b(st, %d, %d, %d, %lld);", c, src, dst, D_(GFV(g, m)) + 8 * D_(GFV(g, g)));
        return b.p;
    }
    case T_7d: {
        int src, dst;
        regs19(&g->d, &src, &dst);
        sb_printf(&b, "op_7d(st, %d, %lld, %lld, %d, %d, 0x%x, 0x%x);", c, D_(GFV(g, breg)), D_(GFV(g, toby)), src, dst, comp, live);
        return b.p;
    }
    case T_8a_abs: case T_8a_rel: case T_8p_undoc48: {
        uint32_t target;
        char okb[64];
        if (t == T_8a_abs) target = GFV(g, addr);
        else if (t == T_8a_rel) target = pc + (uint32_t)sext_i(GFV(g, reladdr), 24);
        else target = GFV(g, r) ? pc + (uint32_t)sext_i(GFV(g, imm), 24) : GFV(g, imm);
        snprintf(okb, sizeof okb, "bcond(st, %d)", c);
        snprintf(tgs, sizeof tgs, "0x%x", target & 0xFFFFFFu);
        g_branch(G, g, &b, okb, tgs);
        return b.p;
    }
    case T_9a_abs: case T_9a_rel: {
        char okb[64], tgx[160];
        if (t == T_9a_abs)
            snprintf(tgx, sizeof tgx, "(get_dag(st, KI, %lld) + get_dag(st, KM, %lld)) & 0xFFFFFF", 8 + D_(GFV(g, pmi)), 8 + D_(GFV(g, pmm)));
        else
            snprintf(tgx, sizeof tgx, "0x%x", (pc + (uint32_t)sext_i(GFV(g, reladdr), 6)) & 0xFFFFFFu);
        sb_printf(&b, "{ int ok = bcond(st, %d); uint32_t tg = %s; branch_compute(st, %d, %lld, 0x%x, 0x%x); ", c, tgx, c, D_(GFV(g, e)), comp, live);
        snprintf(okb, sizeof okb, "ok");
        g_branch(G, g, &b, okb, "tg");
        sb_puts(&b, " }");
        return b.p;
    }
    case T_9b_abs: case T_9b_rel: {
        char okb[64], tgx[160];
        if (t == T_9b_abs)
            snprintf(tgx, sizeof tgx, "(get_dag(st, KI, %lld) + get_dag(st, KM, %lld)) & 0xFFFFFF", 8 + D_(GFV(g, pmi)), 8 + D_(GFV(g, pmm)));
        else
            snprintf(tgx, sizeof tgx, "0x%x", (pc + (uint32_t)sext_i(GFV(g, reladdr), 6)) & 0xFFFFFFu);
        snprintf(okb, sizeof okb, "bcond(st, %d)", c);
        g_branch(G, g, &b, okb, tgx);
        return b.p;
    }
    case T_11a: {
        long long e = GHAS(g, e) ? D_(GFV(g, e)) : 0;
        sb_printf(&b, "{ int ok = bcond(st, %d); branch_compute(st, %d, %lld, 0x%x, 0x%x); if (ok) do_return(st, 0x%x, &nxt, %lld, %lld); }",
                  c, c, e, comp, live, pc, D_(GFV(g, x)), D_(GFV(g, j)));
        return b.p;
    }
    case T_11c:
        sb_printf(&b, "if (bcond(st, %d)) do_return(st, 0x%x, &nxt, %lld, %lld);", c, pc, D_(GFV(g, x)), D_(GFV(g, j)));
        return b.p;
    case T_23p_undoc16: {
        uint32_t op = GFV(g, operand);
        sb_printf(&b, "if (bcond(st, %d)) do_return(st, 0x%x, &nxt, %d, %d);", (int)((op >> 1) & 31), pc, (int)((op >> 8) & 1), (int)((op >> 6) & 1));
        return b.p;
    }
    case T_12a_imm: {
        uint32_t end = (pc + (uint32_t)sext_i(GFV(g, reladdr), 23)) & 0xFFFFFFu, data = GFV(g, data);
        sb_printf(&b, "st->lcntr = 0x%x; do_loop(st, 0x%x, 0x%x, 1, 0x%x, 0);", data, pc + (uint32_t)g->d.words, end, data);
        return b.p;
    }
    case T_12a_ureg: {
        uint32_t end = (pc + (uint32_t)sext_i(GFV(g, reladdr), 23)) & 0xFFFFFFu;
        sb_printf(&b, "st->lcntr = get_ureg(st, 0x%x); do_loop(st, 0x%x, 0x%x, 1, st->lcntr, 0);", GFV(g, ureg), pc + (uint32_t)g->d.words, end);
        return b.p;
    }
    case T_13a: {
        uint32_t end = (pc + (uint32_t)sext_i(GFV(g, reladdr), 23)) & 0xFFFFFFu;
        sb_printf(&b, "do_loop(st, 0x%x, 0x%x, 0, 0, %lld);", pc + (uint32_t)g->d.words, end, D_(GFV(g, term)));
        return b.p;
    }
    case T_14a: {
        uint32_t addr = GFV(g, addr);
        sb_printf(&b, "op_14a(st, %lld, %lld, 0x%x, 0x%x, %d);", D_(GFV(g, d)), D_(GFV(g, l)), GFV(g, ureg), addr, simd_step_of(addr));
        return b.p;
    }
    case T_14d: {
        uint32_t addr = GFV(g, addr);
        int size, sgn, k;
        size_of(t, &g->d, &size, &sgn);
        k = space_of(addr) == SP_BYTE ? size : simd_step_of(addr);
        sb_printf(&b, "op_14d(st, %lld, %lld, 0x%x, %d, %d, %d);", D_(GFV(g, d)), D_(GFV(g, dreg)), addr, size, sgn, k);
        return b.p;
    }
    case T_15a: case T_15b: {
        long long gg = D_(GFV(g, g)), i = D_(GFV(g, i));
        uint32_t mod = t == T_15a ? GFV(g, addr) : (uint32_t)sext_i(GFV(g, data), 7);
        sb_printf(&b, "mem_move(st, 0x%x, %lld, 0x%x, 0, %d, %d, 0, -1, %d);",
                  GFV(g, ureg), i + 8 * gg, mod, GFV(g, d) ? 1 : 0, GFV(g, l) ? 8 : 4, -1);
        return b.p;
    }
    case T_16a: {
        int size = GFV(g, sl) ? 2 : (GFV(g, by) ? 1 : 4);
        long long gg = D_(GFV(g, g));
        sb_printf(&b, "op_16(st, %lld, %lld, %d, 0x%x, %d);", D_(GFV(g, i)) + 8 * gg, D_(GFV(g, m)) + 8 * gg, size, GFV(g, data), -1);
        return b.p;
    }
    case T_16b: {
        long long gg = D_(GFV(g, g));
        sb_printf(&b, "op_16(st, %lld, %lld, 4, 0x%x, %d);", D_(GFV(g, i)) + 8 * gg, D_(GFV(g, m)) + 8 * gg,
                  (uint32_t)sext_i(GFV(g, data), 16), -1);
        return b.p;
    }
    case T_17a:
        sb_printf(&b, "op_17(st, 0x%x, 0x%x);", GFV(g, ureg), GFV(g, data));
        return b.p;
    case T_17b:
        sb_printf(&b, "op_17(st, 0x%x, 0x%x);", GFV(g, ureg), (uint32_t)sext_i(GFV(g, data), 16));
        return b.p;
    case T_18a:
        sb_printf(&b, "op_18a(st, %lld, %lld, 0x%x);", D_(GFV(g, bop)), D_(GFV(g, sreg)), GFV(g, data));
        return b.p;
    case T_19a: {
        int src, dst;
        regs19(&g->d, &src, &dst);
        sb_printf(&b, "modify(st, %d, %d, 0x%x, 0);", src, dst, GFV(g, data));
        return b.p;
    }
    case T_19p_undoc48: {
        int src, dst;
        regs19(&g->d, &src, &dst);
        sb_printf(&b, "modify(st, %d, %d, 0x%x, %d);", src, dst, GFV(g, data), GFV(g, u) ? 2 : 1);
        return b.p;
    }
    case T_19a_bitrev: {
        int src, dst;
        regs19(&g->d, &src, &dst);
        sb_printf(&b, "op_19bitrev(st, %d, %d, 0x%x);", src, dst, GFV(g, data));
        return b.p;
    }
    case T_20a:
        sb_printf(&b, "op_20a(st, %lld, %lld, %lld, %lld, %lld, %lld);", D_(GFV(g, lpu)), D_(GFV(g, lpo)), D_(GFV(g, spu)),
                  D_(GFV(g, spo)), D_(GFV(g, ppu)), D_(GFV(g, ppo)));
        return b.p;
    case T_21a: case T_21c: case T_26a:
        return b.p;
    case T_22a: case T_22c:
        sb_puts(&b, "st->idle = 1; st->deadline = st->cycles;");
        return b.p;
    case T_25a_direct: {
        uint32_t aft;
        if (g_after_slots(G, g, &aft)) { free(b.p); return NULL; }
        sb_printf(&b, "op_25direct(st, 0x%x, &nxt, 0x%x, 0x%x);", pc, GFV(g, addr) & 0xFFFFFFu, aft);
        return b.p;
    }
    case T_25a_pcrel: {
        uint32_t aft;
        if (g_after_slots(G, g, &aft)) { free(b.p); return NULL; }
        sb_printf(&b, "op_25pcrel(st, 0x%x, &nxt, 0x%x, 0x%x);", pc, (pc + (uint32_t)sext_i(GFV(g, reladdr), 24)) & 0xFFFFFFu, aft);
        return b.p;
    }
    case T_25c_rframe: case T_25a_rframe:
        sb_puts(&b, "op_rframe(st);");
        return b.p;
    }
    free(b.p);
    return NULL;
}

/* ------------------------------------------------------------------ predicati sul testo */
static const char *const IRQ_TOUCH[] = {"set_ureg(st, 0x72", "set_ureg(st, 0x7a", "set_ureg(st, 0x7b", "set_ureg(st, 0x7c",
    "set_ureg(st, 0x6f", "set_ureg(st, 0x74", "op_17(st, 0x72", "op_17(st, 0x7a", "op_17(st, 0x7b",
    "op_17(st, 0x7c", "op_17(st, 0x6f", "op_17(st, 0x74", "op_18a", "op_20a", "st->idle", NULL};

static int g_touch(const GI *g, const char *b)
{
    int i;
    for (i = 0; IRQ_TOUCH[i]; i++)
        if (strstr(b, IRQ_TOUCH[i])) return 1;
    if (GHAS(g, ureg)) {
        uint32_t u = GFV(g, ureg);
        return u == 0x72 || u == 0x7a || u == 0x7b || u == 0x7c || u == 0x6f || u == 0x74;
    }
    return 0;
}

/* _kjump: salto diretto ammesso in un nucleo -> 1 con destinazione e ritardo */
static int g_kjump(const GI *g, const char *b, uint32_t *target, int *db)
{
    const char *p;
    unsigned long pcx, tg;
    int call, la, dbv, ci;
    char *end;
    if (g->d.type != T_8a_abs && g->d.type != T_8a_rel) return 0;
    while (*b == ' ') b++;
    if (strncmp(b, "if (bcond(st, ", 14)) return 0;
    p = b + 14;
    while (*p >= '0' && *p <= '9') p++;
    if (p == b + 14 || strncmp(p, ")) branch_common(st, 0x", 23)) return 0;
    p += 23;
    pcx = strtoul(p, &end, 16);
    (void)pcx;
    if (end == p || strncmp(end, ", &nxt, 0x", 10)) return 0;
    p = end + 10;
    tg = strtoul(p, &end, 16);
    if (end == p || strncmp(end, ", ", 2)) return 0;
    p = end + 2;
    if (sscanf(p, "%d, %d, %d, %d, 0x", &call, &la, &dbv, &ci) != 4) return 0;
    if (call != 0 || ci != 0 || (la != 0 && la != 1) || (dbv != 0 && dbv != 1)) return 0;
    *target = (uint32_t)tg;
    *db = dbv;
    return 1;
}

/* _split_args: gli argomenti di primo livello della chiamata in `b` (copiati in buf) */
static int g_split_args(const char *b, char *buf, size_t bufn, char *args[], int maxargs)
{
    const char *i = strchr(b, '(');
    int depth = 0, na = 0;
    size_t n = 0;
    if (!i) return -1;
    args[0] = buf;
    for (i++; *i; i++) {
        char ch = *i;
        if (ch == '(') depth++;
        else if (ch == ')') {
            if (depth == 0) {
                char *s;
                buf[n] = 0;
                for (s = args[na]; *s == ' '; s++) {}
                args[na] = s;
                for (s = args[na] + strlen(args[na]); s > args[na] && s[-1] == ' '; ) *--s = 0;
                return na + 1;
            }
            depth--;
        }
        if (ch == ',' && depth == 0) {
            char *s;
            buf[n++] = 0;
            for (s = args[na]; *s == ' '; s++) {}
            args[na] = s;
            for (s = args[na] + strlen(args[na]); s > args[na] && s[-1] == ' '; ) *--s = 0;
            if (++na >= maxargs) return -1;
            args[na] = buf + n;
        } else {
            if (n + 2 >= bufn) return -1;
            buf[n++] = ch;
        }
    }
    return -1;
}

static const char *const KFORMS[] = {"op_3", "op_4", "mem_move", "op_6mem", "op_16", "cmp_", "execute", "execute_short",
    "op_5move", "op_6nomem", "op_7a", "op_7b", "modify", "op_17", NULL};

/* _kacc: il controllo dell'accesso in un nucleo. 0 = forma non ammessa (None), 1 = ammessa
   (out vuoto se non accede alla memoria) */
static int g_kacc(const char *b0, char *out, size_t outn)
{
    char buf[512], *a[24], bb[512];
    const char *b = b0;
    size_t l, nfn = 0, k;
    int i, na, semis = 0;
    out[0] = 0;
    while (*b == ' ') b++;
    l = strlen(b);
    while (l && b[l - 1] == ' ') l--;
    if (l >= sizeof bb) return 0;
    memcpy(bb, b, l);
    bb[l] = 0;
    b = bb;
    while ((b[nfn] >= 'a' && b[nfn] <= 'z') || (b[nfn] >= 'A' && b[nfn] <= 'Z') || (b[nfn] >= '0' && b[nfn] <= '9') || b[nfn] == '_') nfn++;
    if (!nfn || strncmp(b + nfn, "(st", 3)) return 0;
    {
        char c = b[nfn + 3];
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') return 0;   /* \b */
    }
    for (i = 0; KFORMS[i]; i++)
        if (strlen(KFORMS[i]) == nfn && !strncmp(KFORMS[i], b, nfn)) break;
    if (!KFORMS[i]) return 0;
    for (k = 0; k < l; k++) if (b[k] == ';') semis++;
    if (semis != 1 || l < 2 || strcmp(b + l - 2, ");")) return 0;
    na = g_split_args(b, buf, sizeof buf, a, 24);
    if (na < 0) return 0;
    if (!strncmp(b, "op_3(", 5) && nfn == 4) {
        if (na < 8) return 0;
        snprintf(out, outn, "kacc(st, %s, get_dag(st, KM, %s), %s, %s)", a[3], a[4], a[2], a[7]);
    } else if (nfn == 4 && !strncmp(b, "op_4", 4)) {
        if (na < 8) return 0;
        snprintf(out, outn, "kacc(st, %s, (uint32_t)(%s), %s, %s)", a[3], a[4], a[2], a[7]);
    } else if (nfn == 8 && !strncmp(b, "mem_move", 8)) {
        if (na < 7) return 0;
        snprintf(out, outn, "kacc(st, %s, %s, %s, %s)", a[2], a[3], a[4], a[6]);
    } else if (nfn == 7 && !strncmp(b, "op_6mem", 7)) {
        if (na < 4) return 0;
        snprintf(out, outn, "kacc(st, %s, get_dag(st, KM, %s), 1, 4)", a[2], a[3]);
    } else if (nfn == 5 && !strncmp(b, "op_16", 5)) {
        if (na < 4) return 0;
        snprintf(out, outn, "kacc(st, %s, get_dag(st, KM, %s), 1, %s)", a[1], a[2], a[3]);
    }
    return 1;
}

/* ------------------------------------------------------------------ blocchi e nuclei */
static GI *g_blk(Gen *G, uint32_t pc)           /* blocks.get(pc) */
{
    GI *g = g_at(G, pc);
    return g && g->body ? g : NULL;
}

static int g_in_chunk(const GI *a, const GI *b) { return a && b && a->pc / G_WINDOW == b->pc / G_WINDOW; }

static const char *const G_HEADER = "/* Generato da emu/sharc/rec/gen.py: non modificare. */\n";

typedef struct { uint32_t k0, k1; } GKern;      /* non usato: i nuclei si cercano per inizio */

typedef struct {
    uint32_t s0, e, fall;
    uint32_t b0, b1;                             /* corpo = bl[b0..b1) */
    int valid;
} GKL;

/* posizione in bl del primo pc >= x */
static uint32_t g_lower(Gen *G, uint32_t x)
{
    uint32_t lo = 0, hi = G->nbl;
    while (lo < hi) {
        uint32_t mid = (lo + hi) / 2;
        if (G->code[G->bl[mid]].pc < x) lo = mid + 1; else hi = mid;
    }
    return lo;
}

static int g_lcntr_hazard(const GI *g)
{
    int t = g->d.type;
    if (g_cond(g) == 15) return 1;
    if (is_flow(t) && GHAS(g, a) && GFV(g, a)) return 1;
    if ((t == T_3a || t == T_3b || t == T_3d || t == T_15a || t == T_15b) && GHAS(g, ureg)
        && GFV(g, ureg) >= 0x64 && GFV(g, ureg) <= 0x67) return 1;
    if ((t == T_17a || t == T_17b) && GFV(g, ureg) >= 0x64 && GFV(g, ureg) <= 0x67) return 1;
    if (t == T_5a_move || t == T_5b_move) {
        uint32_t src = (GFV(g, srcureghigh) << 2) | GFV(g, srcureglow), dst = GFV(g, dstureg);
        if ((src >= 0x64 && src <= 0x67) || (dst >= 0x64 && dst <= 0x67)) return 1;
    }
    return 0;
}

/* _kernel_loops per il loop DO a `pc`: riempie kl e ritorna 1 se ammesso */
static int g_kernel_loop(Gen *G, const GI *loop, GKL *kl)
{
    uint32_t s0 = loop->pc + (uint32_t)loop->d.words, e;
    uint32_t b0, b1, p, a;
    uint32_t jslots[256];
    int njs = 0, ok;
    if (!loop_end_of(loop->pc, &loop->d, &e)) return 0;
    if (e < s0 || !g_blk(G, e)) return 0;
    b0 = g_lower(G, s0);
    b1 = g_lower(G, e + 1);
    ok = b1 > b0 && G->code[G->bl[b0]].pc == s0;
    a = s0;
    for (p = b0; p < b1 && ok; p++) {
        GI *x = &G->code[G->bl[p]];
        const char *b = x->body;
        uint32_t jt;
        int jdb, bad, j, isj = 0;
        char tmp[512];
        if (b[0]) isj = g_kjump(x, b, &jt, &jdb);
        if (isj) {
            if (jdb) {
                uint32_t a1 = x->pc + (uint32_t)x->d.words;
                GI *s1 = g_blk(G, a1);
                if (njs + 2 > 256) return 0;
                jslots[njs++] = a1;
                if (s1) jslots[njs++] = a1 + (uint32_t)s1->d.words;
            }
            {
                int in_js = 0;
                for (j = 0; j < njs; j++) if (jslots[j] == x->pc) in_js = 1;
                bad = x->pc != a || (x->slot && !in_js) || (G->idle_loop && x->pc == G->idle_loop) || x->pc == e || x->loopend;
            }
        } else {
            uint32_t tg;
            int in_js = 0;
            for (j = 0; j < njs; j++) if (jslots[j] == x->pc) in_js = 1;
            bad = x->pc != a || (x->slot && !in_js) || (G->idle_loop && x->pc == G->idle_loop) || is_flow(x->d.type)
                  || targets_of(x->pc, &x->d, &tg) || strstr(b, "0x63") || g_touch(x, b)
                  || (x->loopend && x->pc != e) || (b[0] && !g_kacc(b, tmp, sizeof tmp));
        }
        if (bad) ok = 0;
        a = x->pc + (uint32_t)x->d.words;
    }
    if (!ok) return 0;
    {
        GI *last = &G->code[G->bl[b1 - 1]];
        uint32_t jj;
        if (last->pc != e) return 0;
        for (jj = 0; jj < (uint32_t)njs; jj++) {
            uint32_t q = g_lower(G, jslots[jj]);
            if (q >= b1 || G->code[G->bl[q]].pc != jslots[jj]) return 0;      /* jslots - set(body) */
            if (jslots[jj] == e) return 0;                                    /* e in jslots */
        }
        kl->s0 = s0; kl->e = e; kl->fall = e + (uint32_t)last->d.words; kl->b0 = b0; kl->b1 = b1; kl->valid = 1;
    }
    return 1;
}

static int g_in_body(Gen *G, const GKL *k, uint32_t pc)
{
    uint32_t q = g_lower(G, pc);
    return q >= k->b0 && q < k->b1 && G->code[G->bl[q]].pc == pc;
}

static int g_kernel_c(Gen *G, const GKL *k, int use_kcounter, SB *out)
{
    uint32_t s0 = k->s0, e = k->e, fall = k->fall, p;
    int safe_lcntr = use_kcounter;
    uint32_t nb = k->b1 - k->b0;
    uint32_t *jt = (uint32_t *)calloc(nb, 4);          /* destinazione del salto in questo pc */
    uint8_t *isj = (uint8_t *)calloc(nb, 1), *jdb = (uint8_t *)calloc(nb, 1);
    uint32_t *dest = (uint32_t *)calloc(nb, 4);
    uint8_t *hasdest = (uint8_t *)calloc(nb, 1), *islabel = (uint8_t *)calloc(nb, 1);
    char kacc[512];
    int rc = 0;
    for (p = 0; p < nb; p++) {
        GI *x = &G->code[G->bl[k->b0 + p]];
        if (g_lcntr_hazard(x)) safe_lcntr = 0;
        if (x->body[0]) {
            uint32_t t; int db;
            if (g_kjump(x, x->body, &t, &db)) { isj[p] = 1; jt[p] = t; jdb[p] = (uint8_t)db; }
        }
    }
    /* dests: salti non ritardati, poi gli slot2 dei ritardati (sovrascrivono) */
    for (p = 0; p < nb; p++)
        if (isj[p] && !jdb[p]) { dest[p] = jt[p]; hasdest[p] = 1; }
    for (p = 0; p < nb; p++) {
        if (isj[p] && jdb[p]) {
            GI *x = &G->code[G->bl[k->b0 + p]];
            uint32_t a1 = x->pc + (uint32_t)x->d.words;
            GI *s1 = g_blk(G, a1);
            uint32_t s2pc, q;
            if (!s1) goto done;
            s2pc = a1 + (uint32_t)s1->d.words;
            q = g_lower(G, s2pc);
            if (q < k->b1 && q >= k->b0 && G->code[G->bl[q]].pc == s2pc) {
                dest[q - k->b0] = jt[p]; hasdest[q - k->b0] = 1;
            } else goto done;
        }
    }
    for (p = 0; p < nb; p++)
        if (hasdest[p]) {
            uint32_t t = dest[p];
            if (t != s0 && g_in_body(G, k, t)) islabel[g_lower(G, t) - k->b0] = 1;
        }
    LN(out, "int kl_%x(St *restrict sto, int64_t *cycp, uint32_t *nxtp) {", s0);
    LN(out, "    if ((sto->mode1 >> PEYEN) & 1) return 0;          /* nuclei solo senza SIMD */");
    LN(out, "#ifdef KERNEL_NOCOPY");
    LN(out, "    St *restrict st = sto; k_real = sto;");
    LN(out, "#else");
    LN(out, "    St Lk; memcpy(&Lk, sto, sizeof(St)); k_real = sto;");
    LN(out, "    St *restrict st = &Lk;");
    LN(out, "#endif");
    LN(out, "    int64_t cyc = *cycp, cyc0 = cyc; uint32_t nxt;");
    sb_puts(out, "    k_calls++;\n");
    if (safe_lcntr) {
        LN(out, "    int32_t lc_nl = st->nloops;");
        LN(out, "    int lc_active = lc_nl > 0 && (st->loops[lc_nl - 1][0] & 0xFFFFFFu) == 0x%xu"
                " && ((st->loops[lc_nl - 1][0] >> 24) & 31u) == 15u;", e);
        LN(out, "    uint32_t lc_val = lc_active ? st->loops[lc_nl - 1][1] : 0;");
        LN(out, "    int lc_hasret = st->npc != 0;");
        LN(out, "    uint32_t lc_ret = lc_hasret ? st->pcstk[st->npc - 1] : 0;");
    }
    LN(out, "L_top:");
    LN(out, "    k_iters++;");
    for (p = 0; p < nb; p++) {
        GI *x = &G->code[G->bl[k->b0 + p]];
        uint32_t f = x->pc + (uint32_t)x->d.words, pc = x->pc;
        const char *b = x->body;
        if (islabel[p]) LN(out, "K_%x:", pc);
        if (b[0] && !isj[p]) {
            if (g_kacc(b, kacc, sizeof kacc) && kacc[0])
                LN(out, "    if (UNLIKELY(!%s)) { nxt = 0x%x; goto L_out; }", kacc, pc);
        }
        LN(out, "    nxt = 0x%x;", f);
        if (b[0]) LN(out, "    %s", b);
        LN(out, "    cyc++;");
        if (pc == e) {
            if (safe_lcntr) {
                LN(out, "    if (lc_active) {");
                LN(out, "        if (nxt == 0x%x) {", fall);
                LN(out, "            if (--lc_val == 0) {                  /* come loop_end: CURLCNTR = 0, poi i pop */");
                LN(out, "                st->loops[lc_nl - 1][1] = 0; st->nloops--; if (st->npc) st->npc--; lc_active = 0;");
                LN(out, "            }");
                LN(out, "            else if (lc_hasret) nxt = lc_ret;");
                LN(out, "        }");
                LN(out, "    } else nxt = loop_end(st, 0x%x, nxt, 0x%x);", e, fall);
            } else
                LN(out, "    nxt = loop_end(st, 0x%x, nxt, 0x%x);", e, fall);
        }
        if (x->slot) LN(out, "    nxt = delayed_step(st, 0x%x, nxt);", pc);
        if (x->check || pc == e) LN(out, "    if (UNLIKELY(cyc >= st->deadline)) goto L_out;");
        if (hasdest[p] && pc != e) {
            uint32_t t = dest[p];
            if (t == s0) LN(out, "    if (nxt == 0x%x) goto L_top;", t);
            else if (g_in_body(G, k, t)) LN(out, "    if (nxt == 0x%x) goto K_%x;", t, t);
            LN(out, "    if (nxt != 0x%x) goto L_out;", f);
        } else if (x->slot && pc != e)
            LN(out, "    if (nxt != 0x%x) goto L_out;", f);
    }
    LN(out, "    if (nxt == 0x%x) goto L_top;", s0);
    LN(out, "L_out:");
    LN(out, "    if (cyc == cyc0) return 0;                     /* nessuna istruzione: codice generico */");
    LN(out, "    k_instrs += (uint64_t)(cyc - cyc0);");
    if (safe_lcntr) LN(out, "    if (lc_active) st->loops[lc_nl - 1][1] = lc_val;");
    LN(out, "#ifndef KERNEL_NOCOPY");
    LN(out, "    memcpy(sto, &Lk, sizeof(St));");
    LN(out, "#endif");
    LN(out, "    *cycp = cyc; *nxtp = nxt; return 1;");
    sb_puts(out, "}");                                    /* l'elenco Python non ha '\n' in coda */
    rc = 1;
done:
    free(jt); free(isj); free(jdb); free(dest); free(hasdest); free(islabel);
    return rc;
}

/* _chunk_c (non veloce): una funzione C per un blocco di istruzioni consecutive della finestra */
static void g_chunk_c(Gen *G, uint32_t b0, uint32_t b1, const GKL *kern, int nk, const char *pfx, SB *out)
{
    GI *first = &G->code[G->bl[b0]], *lastg = &G->code[G->bl[b1 - 1]];
    uint32_t lo = first->pc, hi = lastg->pc + (uint32_t)lastg->d.words, p, a;
    uint32_t win = lo / G_WINDOW;
    int i;
    LN(out, "int rt_next(St *st);");
    for (i = 0; i < nk; i++)
        LN(out, "int kl_%x(St *restrict sto, int64_t *cycp, uint32_t *nxtp);", kern[i].s0);
    LN(out, "int %s_%x(St *restrict st) {", pfx, win);
    sb_printf(out, "    static void *const tab[%u] = {", hi - lo);
    for (a = lo, p = b0; a < hi; a++) {
        int inside = 0;
        if (p < b1 && G->code[G->bl[p]].pc == a) { inside = 1; p++; }
        if (a != lo) sb_puts(out, ", ");
        if (inside) sb_printf(out, "&&L_%x", a); else sb_puts(out, "&&L_miss");
    }
    sb_puts(out, "};\n");
    LN(out, "    uint32_t nxt;");
    LN(out, "    if (st->pc - 0x%xu >= %uu) return 0;", lo, hi - lo);
    LN(out, "    int64_t cyc = st->cycles;");
    LN(out, "    nxt = st->pc;");
    LN(out, "    goto *tab[st->pc - 0x%xu];", lo);
    LN(out, "L_miss:");
    LN(out, "    st->cycles = cyc; st->pc = nxt; st->stop_reason = 3; return 3;");
    for (p = b0; p < b1; p++) {
        GI *x = &G->code[G->bl[p]];
        uint32_t pc = x->pc, fall = pc + (uint32_t)x->d.words;
        const char *b = x->body;
        int dynamic, fall_inside;
        GI *fg = g_blk(G, fall);
        fall_inside = fg && fg->pc / G_WINDOW == win;
        LN(out, "L_%x:", pc);
        for (i = 0; i < nk; i++)
            if (kern[i].s0 == pc) {
                uint32_t kf = kern[i].fall;
                GI *kg = g_blk(G, kf);
                LN(out, "    if (kl_%x(st, &cyc, &nxt)) {", pc);
                LN(out, "        if (UNLIKELY(cyc >= st->deadline)) { st->cycles = cyc; st->pc = nxt; return 0; }");
                if (kg && kg->pc / G_WINDOW == win) LN(out, "        if (nxt == 0x%x) goto L_%x;", kf, kf);
                LN(out, "        if (nxt - 0x%xu < %uu) goto *tab[nxt - 0x%xu];", lo, hi - lo, lo);
                LN(out, "        st->cycles = cyc; st->pc = nxt; __attribute__((musttail)) return rt_next(st);");
                LN(out, "    }");
            }
        if (G->idle_loop && pc == G->idle_loop)
            LN(out, "    st->cycles = cyc; st->pc = 0x%x; if (idle_hook(st)) return 0;", pc);
        if (strstr(b, "0x63")) LN(out, "    st->pc = 0x%x;", pc);
        LN(out, "    nxt = 0x%x;", fall);
        if (b[0]) LN(out, "    %s", b);
        LN(out, "    cyc++;");
        dynamic = is_flow(x->d.type);
        if (x->loopend) { LN(out, "    nxt = loop_end(st, 0x%x, nxt, 0x%x);", pc, fall); dynamic = 1; }
        if (x->slot) { LN(out, "    nxt = delayed_step(st, 0x%x, nxt);", pc); dynamic = 1; }
        if (x->check) LN(out, "    if (UNLIKELY(cyc >= st->deadline)) { st->cycles = cyc; st->pc = nxt; return 0; }");
        if (dynamic) {
            if (fall_inside) LN(out, "    if (nxt == 0x%x) goto L_%x;", fall, fall);
            LN(out, "    if (nxt - 0x%xu < %uu) goto *tab[nxt - 0x%xu];", lo, hi - lo, lo);
            LN(out, "    st->cycles = cyc; st->pc = nxt; __attribute__((musttail)) return rt_next(st);");
        } else if (fall_inside)
            LN(out, "    goto L_%x;", fall);
        else
            LN(out, "    st->cycles = cyc; st->pc = 0x%x; __attribute__((musttail)) return rt_next(st);", fall);
    }
    sb_puts(out, "}");
}

/* _check_pcs: i pc dopo cui si controlla la scadenza, nella finestra [b0, b1) */
static void g_check_pcs(Gen *G, uint32_t b0, uint32_t b1)
{
    uint32_t p;
    int run = 0;
    uint32_t win = G->code[G->bl[b0]].pc / G_WINDOW;
    for (p = b0; p < b1; p++) {
        GI *x = &G->code[G->bl[p]];
        uint32_t fall = x->pc + (uint32_t)x->d.words;
        GI *fg = g_blk(G, fall);
        int dynamic = is_flow(x->d.type) || x->loopend || x->slot, exits;
        run++;
        exits = !dynamic && !(fg && fg->pc / G_WINDOW == win);
        if (dynamic || exits || run >= G_CHECK_EVERY || g_touch(x, x->body)) {
            run = 0;
            x->check = 1;
        }
    }
}

typedef struct { uint32_t b0, b1; } GChunk;

static int g_cmp_pcidx(const void *a, const void *b)
{
    const uint32_t *x = (const uint32_t *)a, *y = (const uint32_t *)b;
    return x[0] < y[0] ? -1 : (x[0] > y[0] ? 1 : 0);
}

static int g_cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : (x > y ? 1 : 0);
}

static int g_generate(Gen *G, const uint32_t *seeds, uint32_t nseeds, dn_gen_sink_fn sink, void *ctx, char *err, size_t errlen)
{
    SB defs = {0}, head = {0}, headn = {0};
    SB *srcs = NULL;
    uint32_t nsrcs = 0, scap = 0;
    uint32_t i, nch, c;
    GChunk *chunks;
    GKL *kls = NULL;
    uint32_t nkl = 0, klcap = 0;
    uint32_t (*pi)[2];
    SB ksrc = {0};
    int nks = 0, rc = -1;

    g_discover(G, seeds, nseeds);
    if (!G->ncode) { seterr(err, errlen, "no code found in the DSP program"); return -1; }
    for (i = 0; i < G->ncode; i++) {
        GI *g = &G->code[i];
        uint32_t e;
        if (loop_end_of(g->pc, &g->d, &e)) {
            GI *eg = g_at(G, e);
            if (eg) eg->loopend = 1;
        }
        if (is_db(&g->d)) {
            uint32_t a1 = g->pc + (uint32_t)g->d.words;
            GI *s1 = g_at(G, a1);
            if (s1) {
                GI *s2 = g_at(G, a1 + (uint32_t)s1->d.words);
                s1->slot = 1;
                if (s2) s2->slot = 1;
            } else {
                GI *s1b = g_at(G, a1);     /* a1 non e' codice: niente da segnare */
                (void)s1b;
            }
        }
    }
    g_liveness(G);
    for (i = 0; i < G->ncode; i++) {
        G->code[i].body = g_body(G, &G->code[i]);
        if (G->err) { seterr(err, errlen, "the DSP program ends inside a branch"); return -1; }
    }
    pi = (uint32_t (*)[2])malloc((size_t)G->ncode * sizeof *pi);
    G->bl = (uint32_t *)malloc((size_t)G->ncode * 4);
    for (i = 0, G->nbl = 0; i < G->ncode; i++)
        if (G->code[i].body) { pi[G->nbl][0] = G->code[i].pc; pi[G->nbl][1] = i; G->nbl++; }
    qsort(pi, G->nbl, sizeof *pi, g_cmp_pcidx);
    for (i = 0; i < G->nbl; i++) G->bl[i] = pi[i][1];
    free(pi);
    if (!G->nbl) { seterr(err, errlen, "no supported instructions"); return -1; }

    /* difetto della porta: i flag slot/loopend per le istruzioni senza corpo restano, ma servono solo ai blocchi */
    sb_printf(&defs, "#define RT_SEPSTORE\n");
    if (G->idle_loop) {
        sb_printf(&defs, "#define IDLE_WAITING 0x%xu\n#define IDLE_READY0 0x%xu\n", 0x2d0704u, 0x2d0608u);
    }
    sb_printf(&head, "%s%s#include \"rt.h\"\n", G_HEADER, defs.p);
    sb_printf(&headn, "%s%s#define EXP_NOSIMD\n#include \"rt.h\"\n", G_HEADER, defs.p);

    /* finestre */
    chunks = (GChunk *)malloc((size_t)G->nbl * sizeof *chunks);
    nch = 0;
    for (i = 0; i < G->nbl;) {
        uint32_t j = i, w = G->code[G->bl[i]].pc / G_WINDOW;
        while (j < G->nbl && G->code[G->bl[j]].pc / G_WINDOW == w) j++;
        chunks[nch].b0 = i; chunks[nch].b1 = j; nch++;
        i = j;
    }
    for (c = 0; c < nch; c++) g_check_pcs(G, chunks[c].b0, chunks[c].b1);

    /* nuclei dei loop */
    for (i = 0; i < G->ncode; i++) {
        GI *g = &G->code[i];
        GKL kl;
        if (g->d.type != T_12a_imm && g->d.type != T_12a_ureg) continue;
        if (g_kernel_loop(G, g, &kl)) {
            uint32_t q;
            for (q = 0; q < nkl; q++) if (kls[q].s0 == kl.s0) break;       /* stesso inizio: l'ultimo vince */
            if (q == nkl) {
                if (nkl == klcap) { klcap = klcap ? klcap * 2 : 256; kls = (GKL *)realloc(kls, klcap * sizeof(GKL)); }
                nkl++;
            }
            kls[q] = kl;
        }
    }

    /* sorgenti: [0] = dispatcher (dopo), poi per ogni finestra ch_ e chn_, poi i nuclei */
    scap = 2 * nch + 4;
    srcs = (SB *)calloc(scap, sizeof(SB));
    nsrcs = 1;                                       /* posto del dispatcher */
    for (c = 0; c < nch; c++) {
        GKL kin[64];
        int nk = 0, a, b;
        uint32_t lo = G->code[G->bl[chunks[c].b0]].pc;
        uint32_t hi = G->code[G->bl[chunks[c].b1 - 1]].pc + (uint32_t)G->code[G->bl[chunks[c].b1 - 1]].d.words;
        (void)lo; (void)hi;
        for (i = 0; i < nkl; i++) {
            GI *kg = g_blk(G, kls[i].s0);
            if (kg && kg->pc / G_WINDOW == G->code[G->bl[chunks[c].b0]].pc / G_WINDOW) {
                if (nk >= 64) { seterr(err, errlen, "too many loop kernels in a window"); goto out; }
                kin[nk++] = kls[i];
            }
        }
        for (a = 0; a < nk; a++)                       /* sorted(kin) */
            for (b = a + 1; b < nk; b++)
                if (kin[b].s0 < kin[a].s0) { GKL t = kin[a]; kin[a] = kin[b]; kin[b] = t; }
        for (a = 0; a < nk; a++) {
            if (nks++) sb_puts(&ksrc, "\n\n");
            if (!g_kernel_c(G, &kin[a], 1, &ksrc)) { seterr(err, errlen, "kernel with a branch outside the body"); goto out; }
        }
        sb_puts(&srcs[nsrcs], head.p);
        g_chunk_c(G, chunks[c].b0, chunks[c].b1, kin, nk, "ch", &srcs[nsrcs]);
        sb_puts(&srcs[nsrcs], "\n");
        nsrcs++;
        sb_puts(&srcs[nsrcs], headn.p);
        g_chunk_c(G, chunks[c].b0, chunks[c].b1, kin, nk, "chn", &srcs[nsrcs]);
        sb_puts(&srcs[nsrcs], "\n");
        nsrcs++;
    }
    if (nks) {
        sb_puts(&srcs[nsrcs], "#define KERNEL_MODE\n#define EXP_NOSIMD\n#define KSPACE 2\n#define KERNEL_NOCOPY\n");
        sb_puts(&srcs[nsrcs], head.p);
        sb_puts(&srcs[nsrcs], "St *k_real;\nuint64_t k_calls, k_iters, k_instrs;\n"
                              "uint64_t rt_kstat(int k) { return k == 2 ? k_instrs : k ? k_iters : k_calls; }\n");
        sb_puts(&srcs[nsrcs], ksrc.p);
        sb_puts(&srcs[nsrcs], "\n");
        nsrcs++;
    }

    /* dispatcher */
    {
        SB *m = &srcs[0];
        uint32_t (*reg)[3] = (uint32_t (*)[3])malloc((size_t)nch * sizeof *reg);     /* lo, hi, primo chunk */
        uint32_t *rend = (uint32_t *)malloc((size_t)nch * 4);                         /* ultimo chunk + 1 */
        uint32_t nreg = 0, r;
        for (c = 0; c < nch; c++) {
            GI *f = &G->code[G->bl[chunks[c].b0]], *l = &G->code[G->bl[chunks[c].b1 - 1]];
            uint32_t lo = f->pc, hi = l->pc + (uint32_t)l->d.words;
            if (nreg && lo - reg[nreg - 1][1] <= G_GAP) { reg[nreg - 1][1] = hi; rend[nreg - 1] = c + 1; }
            else { reg[nreg][0] = lo; reg[nreg][1] = hi; reg[nreg][2] = c; rend[nreg] = c + 1; nreg++; }
        }
        sb_printf(m, "#define RT_MAIN\n%s\n", head.p);
        sb_puts(m, "typedef int (*chunk_fn)(St *);\n");
        for (c = 0; c < nch; c++) {
            if (c) sb_puts(m, "\n");
            sb_printf(m, "int ch_%x(St *st);", G->code[G->bl[chunks[c].b0]].pc / G_WINDOW);
        }
        sb_puts(m, "\n");
        sb_printf(m, "static chunk_fn const chunks[%u] = {", nch);
        for (c = 0; c < nch; c++) sb_printf(m, "%sch_%x", c ? ", " : "", G->code[G->bl[chunks[c].b0]].pc / G_WINDOW);
        sb_puts(m, "};\n");
        for (c = 0; c < nch; c++) {
            if (c) sb_puts(m, "\n");
            sb_printf(m, "int chn_%x(St *st);", G->code[G->bl[chunks[c].b0]].pc / G_WINDOW);
        }
        sb_puts(m, "\n");
        sb_printf(m, "static chunk_fn const chunks_n[%u] = {", nch);
        for (c = 0; c < nch; c++) sb_printf(m, "%schn_%x", c ? ", " : "", G->code[G->bl[chunks[c].b0]].pc / G_WINDOW);
        sb_puts(m, "};\n");
        sb_puts(m, "#define CHUNK_TAB(st) (((st)->mode1 >> PEYEN) & 1 ? chunks : chunks_n)\n");
        for (r = 0; r < nreg; r++) {
            uint32_t lo = reg[r][0], hi = reg[r][1], x;
            uint16_t *idx = (uint16_t *)calloc(hi - lo, 2);
            for (c = reg[r][2]; c < rend[r]; c++) {
                GI *f = &G->code[G->bl[chunks[c].b0]], *l = &G->code[G->bl[chunks[c].b1 - 1]];
                uint32_t a = f->pc, b = l->pc + (uint32_t)l->d.words;
                for (x = a; x < b; x++) idx[x - lo] = (uint16_t)(c + 1);
            }
            sb_printf(m, "static const uint16_t reg%u[%u] = {", r, hi - lo);
            for (x = 0; x < hi - lo; x++) sb_printf(m, "%s%u", x ? "," : "", idx[x]);
            sb_puts(m, "};\n");
            free(idx);
        }
        sb_puts(m, "static uint64_t n_dispatch;\n");
        sb_puts(m, "uint64_t rt_ndispatch(void) { return n_dispatch; }\n");
        sb_puts(m, "int rt_next(St *st) {\n");
        sb_puts(m, "    n_dispatch++;\n");
        sb_puts(m, "    if (UNLIKELY((st->cycles >= st->deadline) | (st->err_code != 0))) return st->err_code ? 4 : 0;\n");
        sb_puts(m, "    uint32_t pc = st->pc; int k = 0;\n");
        for (r = 0; r < nreg; r++)
            sb_printf(m, "    %sif (pc - 0x%xu < %uu) k = reg%u[pc - 0x%xu];\n", r == 0 ? "" : "else ", reg[r][0], reg[r][1] - reg[r][0], r, reg[r][0]);
        sb_puts(m, "    if (!k) { st->stop_reason = 3; return 3; }\n");
        sb_puts(m, "    __attribute__((musttail)) return CHUNK_TAB(st)[k - 1](st);\n");
        sb_puts(m, "}\n");
        sb_puts(m, "int rt_exec(St *st) {\n");
        sb_puts(m, "    for (;;) {\n");
        sb_puts(m, "        if (UNLIKELY(st->cycles >= st->deadline)) return 0;\n");
        sb_puts(m, "        uint32_t pc = st->pc; int k = 0;\n");
        for (r = 0; r < nreg; r++)
            sb_printf(m, "        %sif (pc - 0x%xu < %uu) k = reg%u[pc - 0x%xu];\n", r == 0 ? "" : "else ", reg[r][0], reg[r][1] - reg[r][0], r, reg[r][0]);
        sb_puts(m, "        if (!k) { st->stop_reason = 3; return 3; }\n");
        sb_puts(m, "        int r = CHUNK_TAB(st)[k - 1](st);\n");
        sb_puts(m, "        if (UNLIKELY(st->err_code)) return 4;\n");
        sb_puts(m, "        if (r) return r;\n");
        sb_puts(m, "    }\n");
        sb_puts(m, "}\n");
        free(reg); free(rend);
    }

    /* consegna */
    for (i = 0; i < nsrcs; i++) {
        char name[32];
        if (i == 0) snprintf(name, sizeof name, "dsp_main.c"); else snprintf(name, sizeof name, "dsp_%03u.c", i);
        if (sink(ctx, name, srcs[i].p, srcs[i].n)) { seterr(err, errlen, "cannot write the generated sources"); goto out; }
    }
    rc = 0;
out:
    for (i = 0; i < nsrcs; i++) free(srcs[i].p);
    free(srcs); free(kls); free(chunks);
    free(defs.p); free(head.p); free(headn.p); free(ksrc.p);
    (void)g_cmp_u32;
    return rc;
}

/* Il punto d'ingresso: dallo stato DNDS di un boot dell'interprete ai sorgenti C. */
int dn_dsp_gensrc(const uint8_t *dnds, size_t len, dn_gen_sink_fn sink, void *ctx, char *err, size_t errlen)
{
    dni *I = dni_load(dnds, len, err, errlen);
    Gen G;
    uint32_t *seeds, ns = 0, i;
    int r;
    static const int shape[8] = {T_25a_direct, T_3c, T_16a, T_3c, T_2b, T_8a_rel, T_18a, T_8a_rel};
    if (!I) return -1;
    memset(&G, 0, sizeof G);
    G.st = &I->st;
    G.idx = (uint32_t *)calloc(INS_REGIONS, 4);
    seeds = (uint32_t *)malloc((size_t)(1 + 64 + 64 + DN_GEN_NSEEDS + 8) * 4);
    seeds[ns++] = I->st.pc;
    for (i = 0; i < 64; i++) seeds[ns++] = 0x90000u + 4 * i;
    for (i = 0; i < (uint32_t)I->st.npc && i < 64; i++) seeds[ns++] = I->st.pcstk[i];
    for (i = 0; i < DN_GEN_NSEEDS; i++) seeds[ns++] = dn_gen_seeds[i];
    {   /* il task idle, solo se il codice ha la forma attesa (come idle_install) */
        uint32_t at = 0xb88aabu;
        Dec t;
        int ok = 1;
        for (i = 0; i < 8 && ok; i++) {
            if (sh_decode(&I->st, at, &t) || t.type != shape[i]) ok = 0; else at += (uint32_t)t.words;
        }
        G.idle_loop = ok ? 0xb88aabu : 0;
    }
    r = g_generate(&G, seeds, ns, sink, ctx, err, errlen);
    for (i = 0; i < G.ncode; i++) { free(G.code[i].body); free(G.code[i].succ); }
    free(G.code); free(G.idx); free(G.ls); free(G.bl); free(seeds);
    dni_release(I);
    return r;
}
