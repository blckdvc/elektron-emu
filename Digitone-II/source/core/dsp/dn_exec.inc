/* L'esecutore dell'interprete (incluso da dn_exec_simd.c e dn_exec_nosimd.c, compilati con e
   senza EXP_NOSIMD): una istruzione per giro, controllo della scadenza dopo ognuna come
   Sharc.step. Dopo l'inclusione di rt/rt.h e dn_interp_int.h. */
#ifdef DNI_PROF
extern uint64_t dni_prof_ticks[OPS_TOTAL], dni_prof_cnt[OPS_TOTAL];
static inline uint64_t dni_rdtick(void) { uint64_t v; __asm__ volatile("mrs %0, cntvct_el0" : "=r"(v)); return v; }
extern uint64_t dni_h[12][8192];
extern uint64_t dni_prof_mem[2][64], dni_prof_mv[2], dni_prof_cmpc[2], dni_prof_op[OP_COUNT], dni_prof_comp[4096], dni_prof_simd[2];
#endif

/* Una istruzione per giro; il dispatch e' replicato in ogni gestore (goto calcolato): niente
   ricerca per le istruzioni in sequenza (la Ins successiva sta a `in + words`), controllo della
   scadenza dopo ognuna come Sharc.step; gli errori (DN_ERR) abbassano la scadenza. */
int EXEC_NAME(St *st)
{
    dni *I = dni_cur;
#ifdef DNI_PROF
    uint64_t tlast = dni_rdtick(); int prev_x = 0;
#endif
    uint32_t pc, nxt = 0, w;
    int64_t cyc;
    int full = 0;
    Ins *in;
    const int32_t *a;
#ifdef EXP_NOSIMD
#define XOP(in) ((in)->xop)
#else
#define XOP(in) ((in)->op)
#endif
    static const void *const tab[OPS_TOTAL] = {
        [OP_UNDEC] = &&L_UNDEC, [OP_BAD] = &&L_BAD, [OP_NOP] = &&L_NOP, [OP_1A] = &&L_1A, [OP_CMP] = &&L_CMP,
        [OP_EXEC] = &&L_EXEC, [OP_EXECS] = &&L_EXECS, [OP_3] = &&L_3, [OP_3C] = &&L_3C, [OP_4] = &&L_4,
        [OP_5MOVE] = &&L_5MOVE, [OP_5SWAP] = &&L_5SWAP, [OP_6NOMEM] = &&L_6NOMEM, [OP_6MEM] = &&L_6MEM,
        [OP_7A] = &&L_7A, [OP_7B] = &&L_7B, [OP_7D] = &&L_7D, [OP_BR] = &&L_BR, [OP_BR9A] = &&L_BAD,
        [OP_BR9B] = &&L_BAD, [OP_RET] = &&L_RET, [OP_LOOPI] = &&L_LOOPI, [OP_LOOPU] = &&L_LOOPU,
        [OP_LOOPT] = &&L_LOOPT, [OP_14A] = &&L_14A, [OP_14D] = &&L_14D, [OP_MEM15] = &&L_MEM15, [OP_16] = &&L_16,
        [OP_17] = &&L_17, [OP_18A] = &&L_18A, [OP_MODIFY] = &&L_MODIFY, [OP_BITREV] = &&L_BITREV,
        [OP_20A] = &&L_20A, [OP_IDLE] = &&L_IDLE, [OP_25D] = &&L_25D, [OP_25P] = &&L_25P,
        [OP_RFRAME] = &&L_RFRAME, [OP_HALT] = &&L_HALT,
#ifdef EXP_NOSIMD
#include "dn_spec_tab.inc"
#endif
    };

/* dopo l'istruzione: i cicli (in un registro: st->cycles si aggiorna ogni 8 istruzioni e agli
   epiloghi completi, come i blocchi generati), poi la Ins successiva. Le istruzioni senza IF_EPI
   non cambiano il flusso: avanzano di `words` e basta. */
#define NEXT() do { \
        cyc++; \
        if (UNLIKELY(full)) { \
            uint32_t fl_ = pc + w; \
            full = 0; st->cycles = cyc; \
            if (UNLIKELY(in->flags & IF_LOOPSTK)) dni_mark_loop_end(I); \
            if (st->nloops) nxt = loop_end(st, pc, nxt, fl_); \
            if (UNLIKELY(st->dly_active)) nxt = delayed_step(st, pc, nxt); \
            st->pc = nxt; \
            I->last_pc = pc; \
            if (UNLIKELY(cyc >= st->deadline)) return 0; \
            if (nxt == fl_) { uint32_t nw2_ = in->nw; in += w; w = nw2_; } else { in = ins_get(I, nxt); if (UNLIKELY(!in)) goto L_NOCODE; w = in->words; } \
            pc = nxt; \
        } else { \
            uint32_t nw_ = in->nw; \
            pc += w; in += w; w = nw_; \
            if (UNLIKELY(!(cyc & 7))) { \
                st->cycles = cyc; st->pc = pc; \
                if (UNLIKELY(cyc >= st->deadline)) return 0; \
            } \
        } \
        if (UNLIKELY(in->flags)) goto L_FLAGS; \
        a = in->a; \
        goto *tab[XOP(in)]; \
    } while (0)

    cyc = st->cycles;
    if (UNLIKELY(cyc >= st->deadline)) return 0;
    pc = st->pc;
    in = ins_get(I, pc);
    if (UNLIKELY(!in)) goto L_NOCODE;
    w = in->words;
    if (UNLIKELY(in->flags)) goto L_FLAGS;
    a = in->a;
    goto *tab[XOP(in)];

L_NOCODE:
    st->cycles = cyc;
    I->bad_pc = nxt; st->stop_reason = 3; st->pc = nxt; return 3;
L_UNDEC:
    dni_decode_slow(I, pc, in);
    w = in->words;
    if (UNLIKELY(in->flags)) goto L_FLAGS;
    a = in->a;
    goto *tab[XOP(in)];
L_BAD:
    st->cycles = cyc;
    I->bad_pc = pc; st->stop_reason = 3; st->pc = pc; return 3;
L_FLAGS:
    st->cycles = cyc; st->pc = pc;
    nxt = pc + w;
    a = in->a;
    if (in->flags & (IF_EPI | IF_IDLELOOP | IF_DELAYSKIP | IF_BACKJUMP)) full = 1;
    if (in->flags & IF_IDLELOOP) { if (idle_hook(st)) return 0; }
    if ((in->flags & IF_DELAYSKIP) && I->fast_delays) {
        uint32_t addr = dag_access(st, a[1], (uint32_t)a[2], 0, 4);
        if (ld(st, addr, 4, 0) < (uint32_t)a[8]) { sto(st, addr, (uint32_t)a[8], 4); I->skipped_delay++; }
    }
    if ((in->flags & IF_BACKJUMP) && I->fast_delays) dni_check_backjump(I, pc, in);
    goto *tab[XOP(in)];

#ifdef DNI_PROF
#define PROF_HOOK() do { \
        dni_prof_op[in->op]++; dni_prof_simd[(st->mode1 >> PEYEN) & 1]++; \
        { uint64_t tn_ = dni_rdtick(); dni_prof_ticks[prev_x] += tn_ - tlast; dni_prof_cnt[prev_x]++; tlast = tn_; prev_x = XOP(in); } \
        { uint32_t cp_ = 0; int ck_ = 0; \
          switch (in->op) { \
          case OP_1A: cp_ = (uint32_t)a[8]; ck_ = 1; break; case OP_3: case OP_4: cp_ = (uint32_t)a[8]; ck_ = 3; break; \
          case OP_5MOVE: cp_ = (uint32_t)a[3]; ck_ = 5; break; case OP_7A: cp_ = (uint32_t)a[5]; ck_ = 7; break; \
          case OP_EXEC: case OP_CMP: cp_ = (uint32_t)a[0]; ck_ = 2; break; default: break; } \
          if (ck_ && cp_) dni_prof_comp[((cp_ >> 22) ? 0x800 | ((cp_ >> 16) & 0x3F) : ((cp_ >> 20) & 3) << 8 | ((cp_ >> 12) & 0xFF)) & 0xFFF]++; \
          if (in->op == OP_3 || in->op == OP_4) dni_prof_mem[in->op == OP_3][(a[0] == 31) | (a[1] << 1) | (a[4] << 2) | ((a[6] == 4) << 3) | (a[7] << 4)]++; \
          if (in->op == OP_5MOVE) dni_prof_mv[(a[0] == 31)]++; \
          if (in->op == OP_CMP) dni_prof_cmpc[a[1] == 31]++; \
          if (XOP(in) < OP_COUNT) { \
            switch (in->op) { \
            case OP_6NOMEM: dni_h[0][((a[1] >> 16) & 0x3F) | ((a[0] == 31) << 6)]++; break; \
            case OP_BR: dni_h[1][(a[0] == 31) | (a[5] << 1) | (a[7] << 2) | (a[8] << 3) | (a[4] << 4) | (a[10] << 5)]++; break; \
            case OP_17: dni_h[2][a[0] & 0x7F]++; break; \
            case OP_MEM15: dni_h[3][(a[0] & 0x7F) | (a[4] << 7) | (a[3] << 11)]++; break; \
            case OP_3: dni_h[4][((a[0] != 31)) | ((a[5] >= 16) << 1) | ((a[6] != 4) << 2) | (a[7] << 3) | (((uint32_t)a[8] ? 1u : 0u) << 4)]++; \
                       if ((uint32_t)a[8]) { uint32_t cp3 = (uint32_t)a[8]; dni_h[8][(cp3 >> 22) ? 0x800 | ((cp3 >> 16) & 0x3F) : (cp3 >> 12) & 0x3FF]++; } break; \
            case OP_4: dni_h[7][((a[0] != 31)) | ((a[5] >= 16) << 1) | ((a[6] != 4) << 2) | (a[7] << 3) | (((uint32_t)a[8] ? 1u : 0u) << 4)]++; break; \
            case OP_CMP: dni_h[5][a[1] & 31]++; { uint32_t cp3 = (uint32_t)a[0]; dni_h[9][(cp3 >> 22) ? 0x800 | ((cp3 >> 16) & 0x3F) : (cp3 >> 12) & 0x3FF]++; } break; \
            case OP_5MOVE: dni_h[6][((a[1] >> 4) & 7) | (((a[2] >> 4) & 7) << 3) | ((a[0] == 31) << 6) | (((uint32_t)a[3] ? 1u : 0u) << 7)]++; break; \
            case OP_7A: dni_h[10][(a[0] == 31) | (a[4] << 1) | (((uint32_t)a[5] ? 1u : 0u) << 3)]++; break; \
            case OP_MODIFY: dni_h[11][a[3]]++; break; \
            default: break; } } \
        } \
    } while (0)
#else
#define PROF_HOOK() ((void)0)
#endif
#define H(L, body) L: PROF_HOOK(); body; NEXT();

#ifdef EXP_NOSIMD
#define OP_PLAIN(L, body) H(L, body)
#define OP_FLOW(L, body) L: PROF_HOOK(); { body; } NEXT();
#include "dn_spec_bodies.inc"
#undef OP_PLAIN
#undef OP_FLOW
#endif
#define OP_PLAIN(L, body) H(L, body)
#define OP_FLOW(L, body) L: PROF_HOOK(); { body; } NEXT();
#include "dn_ops.inc"
#undef OP_PLAIN
#undef OP_FLOW
    L_HALT: st->cycles = cyc; st->stop_reason = 6; st->pc = pc; return 6;
#undef H
#undef NEXT
}
