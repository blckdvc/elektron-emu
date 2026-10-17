/* Parti comuni dell'interprete SHARC+ (dn_interp.c, dn_exec_*.c): istruzioni decodificate e istanza.
   Va incluso DOPO rt/rt.h (usa St e Seg). */
#ifndef DN_INTERP_INT_H
#define DN_INTERP_INT_H

enum {
    OP_UNDEC = 0, OP_BAD, OP_NOP, OP_1A, OP_CMP, OP_EXEC, OP_EXECS, OP_3, OP_3C, OP_4, OP_5MOVE, OP_5SWAP,
    OP_6NOMEM, OP_6MEM, OP_7A, OP_7B, OP_7D, OP_BR, OP_BR9A, OP_BR9B, OP_RET, OP_LOOPI, OP_LOOPU, OP_LOOPT,
    OP_14A, OP_14D, OP_MEM15, OP_16, OP_17, OP_18A, OP_MODIFY, OP_BITREV, OP_20A, OP_IDLE, OP_25D, OP_25P,
    OP_RFRAME, OP_HALT, OP_COUNT
};
enum { IF_IDLELOOP = 1, IF_DELAYSKIP = 2, IF_BACKJUMP = 4,
       IF_EPI = 8,       /* epilogo completo: salto, fine loop, slot, o tocca interrupt/MODE1 */
       IF_PCREAD = 16, /* legge il registro PC */
       IF_LOOPSTK = 32 }; /* scrive gli stack dei loop o del PC: la fine del loop in cima va segnata IF_EPI */ /* legge il registro PC */

typedef struct {
    uint8_t op, words, flags, type;     /* type: T_* della forma decodificata */
    uint16_t xop;                       /* gestore dell'esecutore senza SIMD (specializzato o = op) */
    uint8_t nw, rsvd;                   /* nw: parole dell'istruzione successiva in sequenza; rsvd: in un blocco dell'esecutore a blocchi */
    uint32_t live;                      /* flag di ASTAT vivi dopo l'istruzione (0xFFFFFFFF = tutti) */
    int32_t a[13];
} Ins;                                  /* 64 byte */

#define INS_REGIONS 0x1D0000u           /* L1 SW (0xE0000) + L2 SW (0x80000) + ISA (0x70000) */

static inline int ins_index(uint32_t pc, uint32_t *k)
{
    if (pc - 0x120000u < 0xE0000u) *k = pc - 0x120000u;
    else if (pc - 0xB80000u < 0x80000u) *k = 0xE0000u + (pc - 0xB80000u);
    else if (pc - 0x90000u < 0x70000u) *k = 0x160000u + (pc - 0x90000u);
    else return -1;
    return 0;
}


#include "dn_spec.h"
#include "dn_spec_gen.h"
#define OPS_TOTAL (OP_COUNT + SPX_COUNT)

typedef struct dni dni;
struct dni {
    St st;
    void *per;                       /* Periph (periph.h), malloc */
    uint8_t *l1, *l2, *ddr;
    void *seg;                       /* Seg[65536] di questa istanza */
    uint8_t *mark;                   /* IF_EPI da applicare alle Ins ancora da decodificare (fine loop, slot) */
    void *jit;                       /* JitState dell'esecutore a blocchi (dn_jit.c) */
    int jit_off;                     /* 1: non disponibile o spento */
    volatile int jflush;             /* blocchi da rifare (un'istruzione in un blocco ha cambiato flag) */
    void *tab_raw;                   /* l'allocazione di tab (allineata a 64 byte dentro) */
    Ins *tab;                        /* INS_REGIONS + coda: una Ins per parola di codice, calloc'd (pigra) */
    int fast_delays;                 /* cicli di ritardo software saltati (solo nell'INIT del boot) */
    uint64_t skipped_delay;
    uint32_t bad_pc;                 /* l'ultimo PC non decodificabile */
    uint32_t last_pc;                /* l'ultima istruzione eseguita (diagnostica degli errori) */
    uint64_t decoded;                /* istruzioni decodificate (diagnostica) */
    char err[200];
};

#ifdef _MSC_VER
#define DN_TLS __declspec(thread)
#else
#define DN_TLS __thread
#endif
extern DN_TLS dni *dni_cur;

int dni_decode_slow(dni *I, uint32_t pc, Ins *in);
void dni_check_backjump(dni *I, uint32_t pc, Ins *jmp);
void dni_mark_loop_end(dni *I);
int dni_exec_jit(St *st);
int dni_exec_nosimd(St *st);
int dni_exec_simd(St *st);
void dni_jit_free(dni *I);

static inline Ins *ins_get(dni *I, uint32_t pc)
{
    uint32_t k;
    Ins *in;
    if (UNLIKELY(ins_index(pc, &k))) return NULL;
    in = &I->tab[k];
    if (UNLIKELY(in->op == OP_UNDEC)) dni_decode_slow(I, pc, in);
    return in;
}

#endif
