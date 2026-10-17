/* I gestori dell'esecutore a blocchi e le loro tabelle: incluso da dn_jit.c (senza SIMD) e da
   dn_jit_simd.c (con SIMD): stesso testo, rt.h compilato con e senza EXP_NOSIMD. Definisce
   HSUF(tplain)/HSUF(tflow). */
/* ------------------------------------------------------------------- gestori */
#define OP_PLAIN(L, body) static void h_##L(St *st, const Ins *in) { const int32_t *a = in->a; (void)a; (void)in; body; }
#define OP_FLOW(L, body) static uint32_t h_##L(St *st, const Ins *in, uint32_t pc) { \
        const int32_t *a = in->a; uint32_t nxt = pc + in->words; (void)a; body; return nxt; }
#include "dn_ops.inc"
#include "dn_spec_bodies.inc"
#undef OP_PLAIN
#undef OP_FLOW

#define OPT_PLAIN(idx, f) idx = f,
#define OPT_FLOW(idx, f)
static const hplain_fn HSUF(tplain)[OPS_TOTAL] = {
    [OP_NOP] = h_L_NOP, [OP_1A] = h_L_1A, [OP_CMP] = h_L_CMP, [OP_EXEC] = h_L_EXEC, [OP_EXECS] = h_L_EXECS,
    [OP_3] = h_L_3, [OP_3C] = h_L_3C, [OP_4] = h_L_4, [OP_5MOVE] = h_L_5MOVE, [OP_5SWAP] = h_L_5SWAP,
    [OP_6NOMEM] = h_L_6NOMEM, [OP_6MEM] = h_L_6MEM, [OP_7A] = h_L_7A, [OP_7B] = h_L_7B, [OP_7D] = h_L_7D,
    [OP_LOOPI] = h_L_LOOPI, [OP_LOOPU] = h_L_LOOPU, [OP_LOOPT] = h_L_LOOPT, [OP_14A] = h_L_14A, [OP_14D] = h_L_14D,
    [OP_MEM15] = h_L_MEM15, [OP_16] = h_L_16, [OP_17] = h_L_17, [OP_18A] = h_L_18A, [OP_MODIFY] = h_L_MODIFY,
    [OP_BITREV] = h_L_BITREV, [OP_20A] = h_L_20A, [OP_IDLE] = h_L_IDLE, [OP_RFRAME] = h_L_RFRAME,
#include "dn_spec_jit.inc"
};
#undef OPT_PLAIN
#undef OPT_FLOW
#define OPT_PLAIN(idx, f)
#define OPT_FLOW(idx, f) idx = f,
static const hflow_fn HSUF(tflow)[OPS_TOTAL] = {
    [OP_BR] = h_L_BR, [OP_RET] = h_L_RET, [OP_25D] = h_L_25D, [OP_25P] = h_L_25P,
#include "dn_spec_jit.inc"
};
#undef OPT_PLAIN
#undef OPT_FLOW

