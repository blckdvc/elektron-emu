/* L'interprete SHARC+ del DSP del Digitone II (docs/DSP-INTERPRETE.md).
 *
 * Esegue il programma DSP leggendolo dalla memoria emulata (decodifica a richiesta delle
 * istruzioni VISA/ISA, cache per indirizzo) con la STESSA semantica del DSP ricompilato: le
 * funzioni di rt/rt.h (copia di emu/sharc/rec/rt.h) sono chiamate con i campi decodificati
 * come argomenti variabili invece che costanti. Niente nel plugin deriva dal firmware: il
 * programma arriva a runtime dalla sezione 7 del .syx dell'utente (dni_boot) o da uno stato
 * DNDS gia' fatto da dni_boot (cache dell'utente).
 *
 *   - decodifica: dn_sharc_tab.inc.c (tabelle dai manuali pubblici) + gli stessi casi
 *     speciali di Sharc.fetch (emu/sharc/core.py);
 *   - traduzione: translate() e' gen.body() (emu/sharc/rec/gen.py) con live = tutti i flag;
 *   - esecuzione: rt_exec() e' il dispatcher generato (rt_exec / ch_*) con un controllo
 *     della scadenza a ogni istruzione, come Sharc.step;
 *   - le periferiche, gli anelli dei frame e dell'audio sono quelli di rt/periph.h.
 */
#include "dn_dsp.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>
#include <time.h>

#include "dn_interp_names.h"

#define RT_MAIN
#define IDLE_WAITING 0u            /* il task idle: variabili dn_idle_* (rt.h, PORT) */
#define IDLE_READY0 0u
#define segtab (*dn_segp)          /* piu' istanze: una tabella dei segmenti per istanza */
#include "rt/rt.h"
#undef segtab

#include "dn_sharc_tab.inc.c"
#include "dn_interp_int.h"

uint32_t dn_idle_waiting, dn_idle_ready0;

#define DN_L1_SIZE 0x400000u
#define DN_L2_SIZE 0x100000u
#define DN_DDR_SIZE 0x4000000u
#define DN_SPARSE 64
#define DN_PAGE 0x10000u
#define LIVE 0xFFFFFFFFu

/* ============================================================== decodifica */
typedef struct {
    int type;                       /* T_* */
    int words;                      /* parole da 16 bit */
    uint64_t raw;                   /* l'istruzione nella sua larghezza */
    uint64_t has;                   /* bit FN_* presenti */
    uint32_t f[FN_COUNT];
} Dec;

#define FV(n) (d->f[FN_##n])
#define HAS(n) ((d->has >> FN_##n) & 1)

/* finestra di 6 byte del codice a `pc`; -1 se fuori dalle regioni di codice. nw: ISA (NW) */
/* ws: le parole per i controlli del prefisso (ISA: coppie big endian); wd: le parole della decodifica
   generica, sempre little endian (sharc_disasm.disassemble legge la finestra con "<H") */
static int fetch_window(St *st, uint32_t pc, uint16_t ws[3], uint16_t wd[3], int *nw)
{
    const uint8_t *p;
    uint8_t buf[6] = {0};
    *nw = 0;
    if (pc - 0x120000u < 0xE0000u) {                       /* L1, parole corte */
        uint32_t o = 2 * pc;
        if (o + 6 <= DN_L1_SIZE) p = st->l1 + o;
        else { size_t n = o < DN_L1_SIZE ? DN_L1_SIZE - o : 0; if (n) memcpy(buf, st->l1 + o, n); p = buf; }
    } else if (pc - 0xB80000u < 0x80000u) {                // L2, parole corte
        uint32_t o = 2 * (pc - 0xB80000u);
        if (o + 6 <= DN_L2_SIZE) p = st->l2 + o;
        else { size_t n = o < DN_L2_SIZE ? DN_L2_SIZE - o : 0; if (n) memcpy(buf, st->l2 + o, n); p = buf; }
    } else if (pc - 0x90000u < 0x70000u) {                 // L1, ISA a 48 bit (tabella dei vettori)
        uint32_t b0;
        if (pc >= 0xE0000u) b0 = 0x380000u + (pc - 0xE0000u) * 6;
        else if (pc >= 0xC0000u) b0 = 0x300000u + (pc - 0xC0000u) * 6;
        else if (pc >= 0xB0000u) b0 = 0x2C0000u + (pc - 0xB0000u) * 6;
        else b0 = 0x240000u + (pc - 0x90000u) * 6;
        if (b0 + 6 > DN_L1_SIZE) return -1;
        p = st->l1 + b0;
        *nw = 1;
    } else return -1;
    if (*nw) {                                             /* MSW per primo */
        ws[0] = (uint16_t)((p[4] << 8) | p[5]);
        ws[1] = (uint16_t)((p[2] << 8) | p[3]);
        ws[2] = (uint16_t)((p[0] << 8) | p[1]);
        wd[0] = (uint16_t)(p[4] | (p[5] << 8));
        wd[1] = (uint16_t)(p[2] | (p[3] << 8));
        wd[2] = (uint16_t)(p[0] | (p[1] << 8));
    } else {
        for (int i = 0; i < 3; i++) wd[i] = ws[i] = (uint16_t)(p[2 * i] | (p[2 * i + 1] << 8));
    }
    return 0;
}

static void dec_fields(Dec *d, int form)
{
    const sh_form *f = &sh_forms[form];
    uint64_t insn = ((d->raw));
    for (int i = 0; i < f->nf; i++) {
        const sh_field *fl = &sh_fields[f->f0 + i];
        uint32_t v = (uint32_t)((insn >> fl->lo) & ((1ull << (fl->hi - fl->lo + 1)) - 1));
        if (!((d->has >> fl->id) & 1)) { d->f[fl->id] = 0; d->has |= 1ull << fl->id; }
        d->f[fl->id] |= v << fl->lblo;
    }
}

/* Sharc.fetch: 0 = ok, -1 = non decodificabile */
static int sh_decode(St *st, uint32_t pc, Dec *d)
{
    uint16_t ws[3], wd[3];
    int nw;
    uint32_t w0;
    memset(d, 0, sizeof *d);
    if (fetch_window(st, pc, ws, wd, &nw)) return -1;
    w0 = ws[0];
    uint64_t frame = ((uint64_t)ws[0] << 32) | ((uint64_t)ws[1] << 16) | ws[2];       /* per i casi speciali */
    uint64_t gframe = ((uint64_t)wd[0] << 32) | ((uint64_t)wd[1] << 16) | wd[2];      /* decodifica generica */
    if ((w0 & 0xFE00) == 0x0200) {                         /* Type 6a senza memoria (FINDINGS d8) */
        d->type = T_6a_nomem; d->words = 3; d->raw = frame;
        d->f[FN_cond] = (uint32_t)((frame >> 33) & 31); d->f[FN_dataex] = (uint32_t)((frame >> 27) & 15);
        d->f[FN_shiftimm] = (uint32_t)(frame & 0x7FFFFF);
        d->has = (1ull << FN_cond) | (1ull << FN_dataex) | (1ull << FN_shiftimm);
    } else if ((w0 & 0xFF80) == 0x0180) {                  /* Type 2b (32 bit) */
        d->type = T_2b; d->words = 2; d->raw = ((uint64_t)w0 << 16) | ws[1];
        d->f[FN_compute] = ((w0 & 0x7F) << 16) | ws[1]; d->has = 1ull << FN_compute;
    } else if ((w0 & 0xFF80) == 0x0100) {                  /* Type 2a (48 bit) */
        d->type = T_2a; d->words = 3; d->raw = frame;
        d->f[FN_cond] = (uint32_t)((frame >> 33) & 31); d->f[FN_compute] = (uint32_t)(frame & 0x7FFFFF);
        d->has = (1ull << FN_cond) | (1ull << FN_compute);
    } else if ((w0 & 0xF000) == 0xC000 && !nw) {           /* Type 2c (16 bit) */
        d->type = T_2c; d->words = 1; d->raw = w0;
        d->f[FN_compute] = w0 & 0xFFF; d->has = 1ull << FN_compute;
    } else if (w0 == 0x0081) {                             /* IDLE a 16 bit */
        d->type = T_22c; d->words = 1; d->raw = w0; d->f[FN_emu] = 0; d->has = 1ull << FN_emu;
    } else {
        int best = -1, bl = -1, bf = -1, tie = 0;
        for (int i = 0; i < SH_NFORMS; i++)
            if ((gframe & sh_forms[i].mask) == sh_forms[i].value) {
                const sh_form *f = &sh_forms[i];
                if (f->lead > bl || (f->lead == bl && f->fixed > bf)) { best = i; bl = f->lead; bf = f->fixed; tie = 0; }
                else if (f->lead == bl && f->fixed == bf) tie = 1;
            }
        if (best < 0 || tie) return -1;
        d->type = sh_forms[best].type;
        d->words = sh_forms[best].bits / 16;
        d->raw = gframe >> (48 - sh_forms[best].bits);
        dec_fields(d, best);
    }
    if (nw) {
        if (d->words != 3) return -1;                       /* "istruzione ISA non a 48 bit" */
        d->words = 1;                                       /* words_per = 1 */
    }
    return 0;
}

/* ============================================================= traduzione */
#include "dn_interp_int.h"



static int sext_i(uint32_t v, int bits)
{
    v &= (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1);
    return (bits < 32 && ((v >> (bits - 1)) & 1)) ? (int)(v - (1u << bits)) : (int)v;
}
static int sext_s(uint32_t v, int bits) { return sext_i(v, bits); }

/* forms._size_of: tabelle BHSE del PRM */
static void size_of(int type, const Dec *d, int *size, int *sgn)
{
    int l = (int)FV(l), x = (int)FV(x);
    *size = 4; *sgn = 0;
    if (type == T_3b || type == T_4b) {
        int w = (int)FV(w);
        if (type == T_3b) {
            if (l == 0 && x == 1 && w == 1) { *size = 4; *sgn = 0; }
            else if (l == 0 && x == 0 && w == 0) { *size = 1; *sgn = 0; }
            else if (l == 0 && x == 1 && w == 0) { *size = 1; *sgn = 1; }
            else if (l == 1 && x == 0 && w == 0) { *size = 2; *sgn = 0; }
            else if (l == 1 && x == 1 && w == 0) { *size = 2; *sgn = 1; }
            else if (l == 1 && x == 1 && w == 1) { *size = 8; *sgn = 0; }
        } else {
            if (l == 1 && x == 1 && w == 1) { *size = 4; *sgn = 0; }
            else if (l == 0 && x == 0 && w == 0) { *size = 1; *sgn = 0; }
            else if (l == 1 && x == 0 && w == 0) { *size = 2; *sgn = 0; }
            else if (l == 0 && x == 1 && w == 0) { *size = 1; *sgn = 1; }
            else if (l == 1 && x == 1 && w == 0) { *size = 2; *sgn = 1; }
        }
    } else {
        if (l == 0 && x == 0) { *size = 1; *sgn = 0; }
        else if (l == 1 && x == 0) { *size = 2; *sgn = 0; }
        else if (l == 0 && x == 1) { *size = 1; *sgn = 1; }
        else if (l == 1 && x == 1) { *size = 2; *sgn = 1; }
    }
}

static void regs19(const Dec *d, int *src, int *dst)
{
    int g = (int)FV(g), s = (int)FV(is);
    *src = s + 8 * g;
    *dst = (s ^ (int)FV(idis)) + 8 * g;
}

/* l'indirizzo dopo i due slot di un salto (DB): il ritorno di una CALL (DB) */
static int after_slots(St *st, uint32_t pc, int words, uint32_t *out)
{
    Dec t;
    uint32_t a = pc + (uint32_t)words;
    if (sh_decode(st, a, &t)) return -1;
    a += (uint32_t)t.words;
    if (sh_decode(st, a, &t)) return -1;
    a += (uint32_t)t.words;
    *out = a;
    return 0;
}

static int simd_step_of(uint32_t addr)
{
    int sp = space_of(addr);
    return sp == SP_BYTE ? 4 : (sp == SP_SW ? 2 : 1);
}

/* gen.body(): l'istruzione decodificata -> operazione e argomenti. 0 = ok, -1 = forma non
   supportata (OP_BAD). */
static int ins_translate(St *st, uint32_t pc, const Dec *d, Ins *in)
{
    const int c = HAS(cond) ? (int)FV(cond) : 31;
    const uint32_t comp = HAS(compute) ? FV(compute) : 0;
    int32_t *a = in->a;
    memset(in, 0, sizeof *in);
    in->words = (uint8_t)d->words;
    switch (d->type) {
    case T_1a: case T_1b:
        in->op = OP_1A;
        a[0] = (int)FV(dmi); a[1] = (int)FV(dmm); a[2] = 8 + (int)FV(pmi); a[3] = 8 + (int)FV(pmm);
        a[4] = (int)FV(dmd); a[5] = (int)FV(pmd); a[6] = (int)FV(dmdreg); a[7] = (int)FV(pmdreg);
        a[8] = (int32_t)comp;
        return 0;
    case T_2a:
        if (comp) { in->op = OP_CMP; a[0] = (int32_t)comp; a[1] = c; } else in->op = OP_NOP;
        return 0;
    case T_2b:
        if (comp) { in->op = OP_EXEC; a[0] = (int32_t)comp; } else in->op = OP_NOP;
        return 0;
    case T_2c:
        in->op = OP_EXECS; a[0] = (int32_t)FV(compute);
        return 0;
    case T_3a: case T_3b: case T_3d: {
        int size, sgn, g = (int)FV(g);
        if (d->type != T_3a && HAS(x)) size_of(d->type, d, &size, &sgn);
        else { size = FV(l) ? 8 : 4; sgn = 0; }
        in->op = OP_3;
        a[0] = c; a[1] = (int)FV(u); a[2] = (int)FV(i) + 8 * g; a[3] = (int)FV(m) + 8 * g; a[4] = (int)FV(d);
        a[5] = (int)FV(ureg); a[6] = size; a[7] = sgn; a[8] = (int32_t)comp;
        return 0;
    }
    case T_3c:
        in->op = OP_3C; a[0] = (int)FV(dreg); a[1] = (int)FV(dmi); a[2] = (int)FV(dmm); a[3] = FV(d) ? 1 : 0;
        return 0;
    case T_4a: case T_4b: case T_4d: {
        int size = 4, sgn = 0, g = (int)FV(g);
        if (d->type != T_4a) size_of(d->type, d, &size, &sgn);
        in->op = OP_4;
        a[0] = c; a[1] = (int)FV(u); a[2] = (int)FV(i) + 8 * g; a[3] = sext_s(FV(data), 6); a[4] = (int)FV(d);
        a[5] = (int)FV(dreg); a[6] = size; a[7] = sgn; a[8] = (int32_t)comp;
        return 0;
    }
    case T_5a_move: case T_5b_move:
        in->op = OP_5MOVE; a[0] = c; a[1] = (int)((FV(srcureghigh) << 2) | FV(srcureglow)); a[2] = (int)FV(dstureg);
        a[3] = (int32_t)comp;
        return 0;
    case T_5a_swap: case T_5b_swap:
        in->op = OP_5SWAP; a[0] = c; a[1] = (int)FV(dreg); a[2] = (int)FV(cdreg); a[3] = (int32_t)comp;
        return 0;
    case T_6a_nomem:
        in->op = OP_6NOMEM; a[0] = c; a[1] = (int32_t)FV(shiftimm); a[2] = HAS(dataex) ? (int)FV(dataex) : 0;
        return 0;
    case T_6a_mem: {
        int g = (int)FV(g);
        in->op = OP_6MEM; a[0] = c; a[1] = (int)FV(i) + 8 * g; a[2] = (int)FV(m) + 8 * g; a[3] = (int)FV(d);
        a[4] = (int)FV(dreg); a[5] = (int32_t)FV(shiftimm); a[6] = HAS(dataex) ? (int)FV(dataex) : 0;
        return 0;
    }
    case T_7a: {
        int src, dst, g = (int)FV(g);
        int m = (int)((d->raw >> 27) & 7) + 8 * g, w = (int)((d->raw >> 39) & 1), l = (int)((d->raw >> 23) & 1);
        regs19(d, &src, &dst);
        in->op = OP_7A; a[0] = c; a[1] = src; a[2] = dst; a[3] = m; a[4] = w ? 2 : (l ? 1 : 0); a[5] = (int32_t)comp;
        return 0;
    }
    case T_7b: {
        int src, dst;
        regs19(d, &src, &dst);
        in->op = OP_7B; a[0] = c; a[1] = src; a[2] = dst; a[3] = (int)FV(m) + 8 * (int)FV(g);
        return 0;
    }
    case T_7d: {
        int src, dst;
        regs19(d, &src, &dst);
        in->op = OP_7D; a[0] = c; a[1] = (int)FV(breg); a[2] = (int)FV(toby); a[3] = src; a[4] = dst; a[5] = (int32_t)comp;
        return 0;
    }
    case T_8a_abs: case T_8a_rel: case T_8p_undoc48: case T_9a_abs: case T_9a_rel: case T_9b_abs: case T_9b_rel: {
        uint32_t target = 0, ret_db = 0;
        int db = (int)FV(j), absmode = 0;
        int ty = d->type;
        if (ty == T_8a_abs) target = FV(addr);
        else if (ty == T_8a_rel) target = pc + (uint32_t)sext_i(FV(reladdr), 24);
        else if (ty == T_8p_undoc48) target = FV(r) ? pc + (uint32_t)sext_i(FV(imm), 24) : FV(imm);
        else if (ty == T_9a_abs || ty == T_9b_abs) absmode = 1;
        else target = pc + (uint32_t)sext_i(FV(reladdr), 6);
        if (db && after_slots(st, pc, d->words, &ret_db)) return -1;
        in->op = OP_BR;
        a[0] = c; a[1] = (int32_t)(target & 0xFFFFFF);
        if (absmode) { a[2] = 8 + (int)FV(pmi); a[3] = 8 + (int)FV(pmm); }
        a[4] = absmode; a[5] = (int)FV(b); a[6] = (int)FV(a); a[7] = db; a[8] = HAS(ci) ? (int)FV(ci) : 0;
        a[9] = (int32_t)ret_db;
        if (ty == T_9a_abs || ty == T_9a_rel) { a[10] = 1; a[11] = (int)FV(e); a[12] = (int32_t)comp; }
        if (ty == T_8a_rel && c == 31 && !FV(b) && target < pc) in->flags |= IF_BACKJUMP;
        return 0;
    }
    case T_11a:
        in->op = OP_RET; a[0] = c; a[1] = (int)FV(x); a[2] = (int)FV(j); a[3] = 1; a[4] = HAS(e) ? (int)FV(e) : 0;
        a[5] = (int32_t)comp;
        return 0;
    case T_11c:
        in->op = OP_RET; a[0] = c; a[1] = (int)FV(x); a[2] = (int)FV(j);
        return 0;
    case T_23p_undoc16: {
        int op = (int)FV(operand);
        in->op = OP_RET; a[0] = (op >> 1) & 31; a[1] = (op >> 8) & 1; a[2] = (op >> 6) & 1;
        return 0;
    }
    case T_12a_imm:
        in->op = OP_LOOPI; a[0] = (int32_t)FV(data); a[1] = (int32_t)(pc + (uint32_t)d->words);
        a[2] = (int32_t)((pc + (uint32_t)sext_i(FV(reladdr), 23)) & 0xFFFFFF);
        return 0;
    case T_12a_ureg:
        in->op = OP_LOOPU; a[0] = (int)FV(ureg); a[1] = (int32_t)(pc + (uint32_t)d->words);
        a[2] = (int32_t)((pc + (uint32_t)sext_i(FV(reladdr), 23)) & 0xFFFFFF);
        return 0;
    case T_13a:
        in->op = OP_LOOPT; a[0] = (int32_t)(pc + (uint32_t)d->words);
        a[1] = (int32_t)((pc + (uint32_t)sext_i(FV(reladdr), 23)) & 0xFFFFFF); a[2] = (int)FV(term);
        return 0;
    case T_14a:
        in->op = OP_14A; a[0] = (int)FV(d); a[1] = (int)FV(l); a[2] = (int)FV(ureg); a[3] = (int32_t)FV(addr);
        a[4] = simd_step_of(FV(addr));
        return 0;
    case T_14d: {
        int size, sgn;
        size_of(d->type, d, &size, &sgn);
        in->op = OP_14D; a[0] = (int)FV(d); a[1] = (int)FV(dreg); a[2] = (int32_t)FV(addr); a[3] = size; a[4] = sgn;
        a[5] = space_of(FV(addr)) == SP_BYTE ? size : simd_step_of(FV(addr));
        return 0;
    }
    case T_15a: case T_15b: {
        int g = (int)FV(g);
        uint32_t mod = d->type == T_15a ? FV(addr) : (uint32_t)sext_i(FV(data), 7);
        in->op = OP_MEM15; a[0] = (int)FV(ureg); a[1] = (int)FV(i) + 8 * g; a[2] = (int32_t)mod; a[3] = FV(d) ? 1 : 0;
        a[4] = FV(l) ? 8 : 4;
        return 0;
    }
    case T_16a: {
        int size = FV(sl) ? 2 : (FV(by) ? 1 : 4);
        in->op = OP_16; a[0] = (int)FV(i) + 8 * (int)FV(g); a[1] = (int)FV(m) + 8 * (int)FV(g); a[2] = size;
        a[3] = (int32_t)FV(data);
        return 0;
    }
    case T_16b:
        in->op = OP_16; a[0] = (int)FV(i) + 8 * (int)FV(g); a[1] = (int)FV(m) + 8 * (int)FV(g); a[2] = 4;
        a[3] = sext_i(FV(data), 16);
        return 0;
    case T_17a:
        in->op = OP_17; a[0] = (int)FV(ureg); a[1] = (int32_t)FV(data);
        return 0;
    case T_17b:
        in->op = OP_17; a[0] = (int)FV(ureg); a[1] = sext_i(FV(data), 16);
        return 0;
    case T_18a:
        in->op = OP_18A; a[0] = (int)FV(bop); a[1] = (int)FV(sreg); a[2] = (int32_t)FV(data);
        return 0;
    case T_19a: {
        int src, dst;
        regs19(d, &src, &dst);
        in->op = OP_MODIFY; a[0] = src; a[1] = dst; a[2] = (int32_t)FV(data); a[3] = 0;
        return 0;
    }
    case T_19p_undoc48: {
        int src, dst;
        regs19(d, &src, &dst);
        in->op = OP_MODIFY; a[0] = src; a[1] = dst; a[2] = (int32_t)FV(data); a[3] = FV(u) ? 2 : 1;
        return 0;
    }
    case T_19a_bitrev: {
        int src, dst;
        regs19(d, &src, &dst);
        in->op = OP_BITREV; a[0] = src; a[1] = dst; a[2] = (int32_t)FV(data);
        return 0;
    }
    case T_20a:
        in->op = OP_20A; a[0] = (int)FV(lpu); a[1] = (int)FV(lpo); a[2] = (int)FV(spu); a[3] = (int)FV(spo);
        a[4] = (int)FV(ppu); a[5] = (int)FV(ppo);
        return 0;
    case T_21a: case T_21c: case T_26a:
        in->op = OP_NOP;
        return 0;
    case T_22a: case T_22c:
        in->op = OP_IDLE;
        return 0;
    case T_25a_direct: case T_25a_pcrel: {
        uint32_t ret_db, target;
        if (after_slots(st, pc, d->words, &ret_db)) return -1;
        target = d->type == T_25a_direct ? (FV(addr) & 0xFFFFFF) : ((pc + (uint32_t)sext_i(FV(reladdr), 24)) & 0xFFFFFF);
        in->op = d->type == T_25a_direct ? OP_25D : OP_25P; a[0] = (int32_t)target; a[1] = (int32_t)ret_db;
        return 0;
    }
    case T_25c_rframe: case T_25a_rframe:
        in->op = OP_RFRAME;
        return 0;
    }
    return -1;
}

void dni_analyze(dni *I, const uint32_t *seeds, uint32_t nseeds);

/* =============================================================== istanza */
/* i flag di sequenziamento (IF_EPI, IF_PCREAD): l'esecutore salta l'epilogo completo
   (fine loop, salto ritardato, scadenza) per le istruzioni che non possono servirlo, come i
   blocchi generati (gen._check_pcs). Servono: salti e chiamate e ritorni, i loop (e la loro
   ultima istruzione), i due slot di un salto ritardato, e cio' che tocca MODE1, gli interrupt, il
   timer o IDLE. Il controllo della scadenza avviene comunque ogni 8 istruzioni. */
static int ureg_loopstk(int u) { return u >= 0x64 && u <= 0x67; }

static int ureg_touches(int u)
{
    return u == 0x72 || u == 0x7a || u == 0x7b || u == 0x7c || u == 0x6f || u == 0x74;
}

static void mark_at(dni *I, uint32_t pc, int flag)
{
    uint32_t k;
    if (ins_index(pc, &k)) return;
    I->mark[k] |= (uint8_t)flag;
    if (I->tab[k].op) {
        if ((I->tab[k].flags & flag) != flag && I->tab[k].rsvd) I->jflush = 1;      /* era gia' in un blocco */
        I->tab[k].flags |= (uint8_t)flag;
    }
}

static void mark_flags(dni *I, uint32_t pc, Ins *in)
{
    uint32_t k;
    const int32_t *a = in->a;
    int epi = 0, pcr = 0, lstk = 0;
    if (!ins_index(pc, &k)) in->flags |= I->mark[k];
    switch (in->op) {
    case OP_BR: case OP_RET: case OP_25D: case OP_25P: case OP_IDLE: case OP_HALT: case OP_18A: epi = 1; break;
    case OP_20A: epi = 1; lstk = 1; break;
    case OP_LOOPI: case OP_LOOPU: case OP_LOOPT: epi = 1; break;
    case OP_3: epi = ureg_touches(a[5]); pcr = a[5] == 0x63; lstk = ureg_loopstk(a[5]); break;
    case OP_MEM15: epi = ureg_touches(a[0]); pcr = a[0] == 0x63; lstk = ureg_loopstk(a[0]); break;
    case OP_5MOVE: epi = ureg_touches(a[2]); pcr = a[1] == 0x63; lstk = ureg_loopstk(a[2]); break;
    case OP_17: epi = ureg_touches(a[0]); lstk = ureg_loopstk(a[0]); break;
    case OP_14A: epi = ureg_touches(a[2]); pcr = a[2] == 0x63; lstk = ureg_loopstk(a[2]); break;
    default: break;
    }
    if (epi || lstk) in->flags |= IF_EPI;
    if (lstk) in->flags |= IF_LOOPSTK;
    if (pcr) in->flags |= IF_PCREAD;
    /* inizio e fine dei loop, slot dei salti ritardati */
    if (in->op == OP_LOOPI || in->op == OP_LOOPU) mark_at(I, (uint32_t)a[2], IF_EPI);
    else if (in->op == OP_LOOPT) mark_at(I, (uint32_t)a[1], IF_EPI);
    if (in->op == OP_BR || in->op == OP_RET || in->op == OP_25D || in->op == OP_25P) {
        int db = in->op == OP_BR ? a[7] : in->op == OP_RET ? a[2] : 1;
        if (db) {
            uint32_t s1 = pc + in->words;
            Ins *i1;
            mark_at(I, s1, IF_EPI);
            i1 = ins_get(I, s1);
            if (i1 && i1->op != OP_BAD) mark_at(I, s1 + i1->words, IF_EPI);
        }
    }
}

/* dopo un'istruzione che ha scritto gli stack dei loop: la fine del loop in cima e' IF_EPI */
void dni_mark_loop_end(dni *I)
{
    St *st = &I->st;
    if (st->nloops > 0) mark_at(I, st->loops[st->nloops - 1][0] & 0xFFFFFF, IF_EPI);
}

/* decodifica a richiesta di un'istruzione non ancora in cache (chiamata da ins_get) */
int dni_decode_slow(dni *I, uint32_t pc, Ins *in)
{
    Dec d;
    if (sh_decode(&I->st, pc, &d) || ins_translate(&I->st, pc, &d, in)) {
        memset(in, 0, sizeof *in);
        in->op = OP_BAD; in->words = 1;
        I->bad_pc = pc;
    }
    in->type = (uint8_t)d.type;
    in->live = 0xFFFFFFFFu;
    {   /* parole dell'istruzione che segue */
        Dec nd;
        in->nw = (in->op != OP_BAD && !sh_decode(&I->st, pc + in->words, &nd)) ? (uint8_t)nd.words : 1;
    }
    in->xop = (uint16_t)spx_find(in->op, in->a);        /* gestore specializzato (senza SIMD) */
    if (!in->xop) in->xop = in->op;
    mark_flags(I, pc, in);
    I->decoded++;
    return 0;
}

DN_TLS dni *dni_cur;
#ifdef DNI_PROF
#include "dn_prof.inc"
#endif

static void ins_clear(dni *I)
{
    if (I->tab)                      /* tutto da rifare (core.cache.clear); solo le Ins usate, per non toccare pagine vuote */
        for (uint32_t k = 0; k < INS_REGIONS; k++)
            if (I->tab[k].op) memset(&I->tab[k], 0, sizeof(Ins));
}

/* speed.check_backjump: il ciclo di ritardo software `for (volatile i = 0; i < LIMITE; i++);`
   chiuso dal salto a `pc` (backward, incondizionato) diventa un salto del contatore al limite
   (hook sul primo istruzione del ciclo, IF_DELAYSKIP; dati nei campi a[8..12] dell'istruzione) */
void dni_check_backjump(dni *I, uint32_t pc, Ins *jmp)
{
    static const int shape1[7] = {T_15b, T_17a, T_2b, T_8a_rel, T_2c, T_15b, T_8a_rel};
    uint32_t target = (uint32_t)jmp->a[1], at = target;
    Dec b[7];
    Ins *t;
    int i, rc, rl;
    jmp->flags &= (uint8_t)~IF_BACKJUMP;
    for (i = 0; i < 7; i++) {
        if (sh_decode(&I->st, at, &b[i])) return;
        at += (uint32_t)b[i].words;
    }
    for (i = 0; i < 7; i++) {
        int want = shape1[i];
        if (i == 1 && b[1].type == T_17b) want = T_17b;
        if (b[i].type != want) return;
    }
    {
        const Dec *ld = &b[0], *lim = &b[1], *cmp = &b[2], *br = &b[3], *inc = &b[4], *sto = &b[5];
        const Dec *d;
        d = ld;  rc = (int)FV(ureg);
        {
            int ld_d = (int)ld->f[FN_d], st_d = (int)sto->f[FN_d];
            int ld_i = (int)ld->f[FN_i], st_i = (int)sto->f[FN_i], ld_g = (int)ld->f[FN_g], st_g = (int)sto->f[FN_g];
            if (ld_d || !st_d || (int)sto->f[FN_ureg] != rc || rc >= 16 || ld_i != st_i || ld_g != st_g
                || ld->f[FN_data] != sto->f[FN_data]) return;
        }
        {
            uint32_t endpc = target;
            for (i = 0; i < 6; i++) endpc += (uint32_t)b[i].words;
            if (endpc != pc) return;
        }
        if (br->f[FN_cond] != 17) return;
        if (inc->f[FN_compute] != (uint32_t)(0x500 | (rc << 4) | rc)) return;
        rl = (int)lim->f[FN_ureg];
        if (cmp->f[FN_compute] != (uint32_t)((0x0B << 12) | (rc << 4) | rl) || rl >= 16) return;
        {
            uint32_t limit = lim->f[FN_data];
            if (lim->type == T_17b) limit = (uint32_t)sext_i(limit, 16);
            t = ins_get(I, target);
            if (!t || t->op != OP_MEM15) return;
            t->flags |= IF_DELAYSKIP;
            t->a[8] = (int32_t)limit;
        }
    }
}

/* ============================================================== esecuzione */
/* il dispatcher: la variante dell'esecutore per PEYEN (senza SIMD i rami per PEy spariscono: il
   96% delle istruzioni del PLAY gira cosi'), scelta a ogni rientro. Ogni scrittura di MODE1
   mette deadline = cycles, quindi l'esecutore esce prima dell'istruzione seguente. */
int dni_exec_simd(St *st);
int dni_exec_nosimd(St *st);
int rt_exec(St *st)
{
    if (!dni_cur->jit_off) return dni_exec_jit(st);
    return ((st->mode1 >> PEYEN) & 1) ? dni_exec_simd(st) : dni_exec_nosimd(st);
}

/* ======================================================= memoria e periferiche */
static Seg dn_seg_default[65536];
static Seg (*dn_segp)[65536] = &dn_seg_default;
static volatile int dn_seg_lock;
#define DNI_SEGBYTES (sizeof(Seg) * 65536)

static void init_seg(dni *I)
{
    while (__atomic_exchange_n(&dn_seg_lock, 1, __ATOMIC_ACQUIRE)) { }
    dn_segp = (Seg (*)[65536])I->seg;
    rt_init_seg(&I->st);
    dn_segp = &dn_seg_default;
    __atomic_store_n(&dn_seg_lock, 0, __ATOMIC_RELEASE);
}

static void seterr(char *err, size_t n, const char *fmt, ...)
{
    va_list ap;
    if (!err || !n) return;
    va_start(ap, fmt);
    vsnprintf(err, n, fmt, ap);
    va_end(ap);
}

static dni *dni_alloc(char *err, size_t errlen)
{
    dni *I = (dni *)calloc(1, sizeof *I);
    if (!I) { seterr(err, errlen, "memoria"); return NULL; }
    I->per = rt_periph_new();
    I->l1 = (uint8_t *)calloc(1, DN_L1_SIZE);
    I->l2 = (uint8_t *)calloc(1, DN_L2_SIZE);
    I->ddr = (uint8_t *)calloc(1, DN_DDR_SIZE);
    I->seg = malloc(DNI_SEGBYTES);
    I->tab_raw = calloc(((size_t)INS_REGIONS + 16) * sizeof(Ins) + 64, 1);
    I->tab = (Ins *)(((uintptr_t)I->tab_raw + 63) & ~(uintptr_t)63);
    I->mark = (uint8_t *)calloc(INS_REGIONS + 16, 1);
    I->jit_off = (getenv("DN_JIT") && !atoi(getenv("DN_JIT"))) ? 1 : 0;
    if (!I->tab_raw || !I->mark || !I->per || !I->l1 || !I->l2 || !I->ddr || !I->seg) {
        seterr(err, errlen, "memoria");
        free(I->per); free(I->l1); free(I->l2); free(I->ddr); free(I->seg); free(I->tab_raw); free(I->mark); free(I);
        return NULL;
    }
    return I;
}

static void dni_release(dni *I)
{
    if (!I) return;
    if (I->per) {
        int ns = ((Periph *)I->per)->nsparse;
        if (ns < 0 || ns > DN_SPARSE) ns = DN_SPARSE;
        for (int k = 0; k < ns; k++) free(((Periph *)I->per)->sparse[k]);
        free(I->per);
    }
    dni_jit_free(I);
    free(I->tab_raw); free(I->mark);
    free(I->l1); free(I->l2); free(I->ddr); free(I->seg);
    free(I);
}

static void st_link(dni *I)
{
    I->st.l1 = I->l1; I->st.l2 = I->l2; I->st.ddr = I->ddr; I->st.ddr_size = DN_DDR_SIZE;
    I->st.mmio_rd = 0; I->st.mmio_wr = 0;
    I->st.per = I->per;
    init_seg(I);
}

/* Periph e St all'accensione, come le crea il Python (periph.attach_all + Engine.from_py +
   go_native): i sei canali DMA, CGU acceso, SEC senza servizio, tutto il resto a zero */
static void blank_state(dni *I)
{
    static const struct { uint32_t base, sid, extra, kind, sport; int64_t wc; } dma[NDMA] = {
        {0x31022000u, 37, 0, DK_AUDIO, 0x31002000u, 1000}, {0x31022080u, 39, 0, DK_RX_ZERO, 0x31002080u, 1000},
        {0x31023000u, 53, 0, DK_DISCARD, 0x31002400u, 1000}, {0x31023080u, 55, 191, DK_RX_ZERO, 0x31002480u, 1000},
        {0x3102D200u, 69, 0, DK_REPLY, 0, 485}, {0x3102D280u, 70, 0, DK_FRAMES, 0, 485}};
    Periph *P = (Periph *)I->per;
    memset(&I->st, 0, sizeof I->st);
    I->st.timer_due = -1;
    I->st.next_event = -1;
    for (int ch = 0; ch < NDMA; ch++) {
        P->dma[ch].base = dma[ch].base; P->dma[ch].sec_id = dma[ch].sid; P->dma[ch].extra_sid = dma[ch].extra;
        P->dma[ch].kind = dma[ch].kind; P->dma[ch].sport = dma[ch].sport; P->dma[ch].word_cycles = dma[ch].wc;
    }
    P->sec_active = -1;
    P->cgu_pllen[0] = P->cgu_pllen[1] = 1;
    P->out_ch[0] = 0; P->out_ch[1] = 1;
    st_link(I);
}

/* scrive nella memoria emulata (Memory.write): L1/L2/DDR direttamente, il resto in pagine sparse */
static void mem_put(dni *I, uint32_t addr, const uint8_t *src, uint32_t n, int fill, uint32_t pattern)
{
    St *st = &I->st;
    while (n) {
        uint32_t chunk = n;
        uint8_t *p = host(st, addr, 1);
        if (p) {                                         /* fino alla fine della finestra */
            uint32_t room;
            if (addr - 0x28000000u < DN_L1_SIZE) room = DN_L1_SIZE - (addr - 0x28000000u);
            else if (addr - 0x20000000u < DN_L2_SIZE) room = DN_L2_SIZE - (addr - 0x20000000u);
            else room = DN_DDR_SIZE - (addr - 0x80000000u);
            if (chunk > room) chunk = room;
        } else {
            uint8_t *pg = sparse_page((Periph *)I->per, addr, 1);
            if (!pg) return;
            p = pg + (addr & 0xFFFF);
            if (chunk > 0x10000u - (addr & 0xFFFF)) chunk = 0x10000u - (addr & 0xFFFF);
        }
        if (fill) {
            if (pattern == 0) memset(p, 0, chunk);
            else for (uint32_t k = 0; k < chunk; k++) p[k] = (uint8_t)(pattern >> (8 * ((addr + k) & 3)));
        } else memcpy(p, src, chunk);
        addr += chunk; n -= chunk; if (src) src += chunk;
    }
}

/* ===================================================================== boot */
#define SENTINEL 0x1FFFFEu
#define INIT_STACK 0x0026FFF8u

typedef struct { uint32_t code, target, count, arg, payload_off, payload_len; int fill; } Blk;

static int parse_blocks(const uint8_t *d, size_t len, Blk **out, int *nout)
{
    int cap = 128, n = 0;
    Blk *b = (Blk *)malloc(cap * sizeof *b);
    size_t off = 0;
    if (!b) return -1;
    while (off + 16 <= len) {
        const uint8_t *h = d + off;
        uint8_t x = 0;
        uint32_t w[4];
        if (h[3] != 0xAD && h[3] != 0xAC && h[3] != 0xAB) break;
        for (int i = 0; i < 16; i++) x ^= h[i];
        if (x) break;
        for (int i = 0; i < 4; i++) w[i] = (uint32_t)h[4 * i] | ((uint32_t)h[4 * i + 1] << 8) | ((uint32_t)h[4 * i + 2] << 16) | ((uint32_t)h[4 * i + 3] << 24);
        int fill = (w[0] >> 8) & 1;
        size_t plen = fill ? 0 : w[2];
        if (off + 16 + plen > len) break;
        if (n == cap) { cap *= 2; b = (Blk *)realloc(b, cap * sizeof *b); if (!b) return -1; }
        b[n].code = w[0]; b[n].target = w[1]; b[n].count = w[2]; b[n].arg = w[3];
        b[n].payload_off = (uint32_t)(off + 16); b[n].payload_len = (uint32_t)plen; b[n].fill = fill;
        n++;
        off += 16 + plen;
    }
    *out = b; *nout = n;
    return 0;
}

/* il task idle di FreeRTOS (speed.IDLE_TASK, Digitone II 1.11): il ciclo a 0xb88aab col suo
   controllo della forma; senza questa forma (altra versione) niente salto del tempo */
static int idle_install(dni *I)
{
    static const int shape[8] = {T_25a_direct, T_3c, T_16a, T_3c, T_2b, T_8a_rel, T_18a, T_8a_rel};
    uint32_t at = 0xb88aabu;
    Dec t;
    for (int i = 0; i < 8; i++) {
        if (sh_decode(&I->st, at, &t) || t.type != shape[i]) return 0;
        at += (uint32_t)t.words;
    }
    {
        Ins *in = ins_get(I, 0xb88aabu);
        if (!in || in->op == OP_BAD) return 0;
        in->flags |= IF_IDLELOOP;
    }
    dn_idle_waiting = 0x2d0704u;
    dn_idle_ready0 = 0x2d0608u;
    return 1;
}

/* diagnostica: la forma e la parola grezza a `pc` */
static void describe(dni *I, uint32_t pc, char *out, size_t n)
{
    Dec d;
    uint16_t ws[3] = {0}, wd[3] = {0};
    int nw;
    if (fetch_window(&I->st, pc, ws, wd, &nw)) { snprintf(out, n, "pc %#x fuori dal codice", pc); return; }
    if (sh_decode(&I->st, pc, &d)) { snprintf(out, n, "pc %#x non decodificabile (%04x %04x %04x)", pc, ws[0], ws[1], ws[2]); return; }
    snprintf(out, n, "pc %#x forma %s raw %012llx compute %#x", pc, sh_type_names[d.type], (unsigned long long)d.raw, d.f[FN_compute]);
}

static int run_cycles(dni *I, int64_t target, volatile int *cancel, void (*progress)(void *, int, const char *), void *ctx, char *err, size_t errlen)
{
    St *st = &I->st;
    static uint8_t zero_frame[FRAME_BYTES];
    int64_t start = st->cycles, step = 20000000;
    dni_cur = I;
    while (st->cycles < target) {
        int64_t lim = st->cycles + step < target ? st->cycles + step : target;
        int r = rt_run(st, lim);
        if (r == 5) { rt_push_frame(st, zero_frame, FRAME_BYTES); continue; }
        if (r == 2) {
            if (cancel && *cancel) { seterr(err, errlen, "interrotto"); return -1; }
            if (progress) progress(ctx, (int)(100 * (st->cycles - start) / (target - start > 0 ? target - start : 1)), "dsp");
            continue;
        }
        if (r == 4) {
            char dsc[160];
            describe(I, I->last_pc, dsc, sizeof dsc);
            seterr(err, errlen, "errore del DSP: err_code %#x dopo %s; cicli %lld irptl %#x imask %#x imaskp %#x mode1 %#x pcstk %d",
                   st->err_code, dsc, (long long)st->cycles, st->irptl, st->imask, st->imaskp, st->mode1, st->npc);
            for (int q = 0; q < 64; q++) if (st->irq_count[q]) fprintf(stderr, "\nirq %d x%u", q, st->irq_count[q]);
            return -1;
        }
        if (r == 3) { seterr(err, errlen, "istruzione del DSP non decodificabile al pc %#x", I->bad_pc); return -1; }
        seterr(err, errlen, "rt_run inatteso: %d al pc %#x", r, st->pc);
        return -1;
    }
    return 0;
}

/* Come bootsnap.make + DspSynth.__init__ + go_native: dallo stream di boot della sezione 7
   fino a `target` cicli con frame nulli, poi lo stato "pulito" (campi di sola uscita e di
   diagnostica a zero, SPI2 agganciata ai blocchi della SPORT4) */
static dni *boot_state(const uint8_t *blob, size_t len, int64_t target, volatile int *cancel,
                       void (*progress)(void *, int, const char *), void *ctx, char *err, size_t errlen)
{
    dni *I = dni_alloc(err, errlen);
    Blk *blk = NULL;
    int nb = 0, i;
    uint32_t entry = 0;
    St *st;
    if (!I) return NULL;
    blank_state(I);
    st = &I->st;
    if (parse_blocks(blob, len, &blk, &nb) || nb < 2) {
        seterr(err, errlen, "sezione 7: non e' uno stream di boot del DSP");
        goto fail;
    }
    I->fast_delays = 1;
    dni_cur = I;
    {   /* l'HALT del ritorno dell'INIT */
        uint32_t k;
        ins_index(SENTINEL, &k);
        I->tab[k].op = OP_HALT; I->tab[k].xop = OP_HALT; I->tab[k].words = 1;
    }
    for (i = 0; i < nb; i++) {
        Blk *b = &blk[i];
        int flag_final = (b->code >> 15) & 1, flag_init = (b->code >> 11) & 1, flag_ignore = (b->code >> 12) & 1;
        if (flag_final) { entry = b->target; break; }
        if (flag_init) {
            static const int mv[6][2] = {{5, 0}, {6, 1}, {7, -1}, {13, 0}, {14, 1}, {15, -1}};
            for (int k = 0; k < 6; k++) st->dag[0][KM][mv[k][0]] = (uint32_t)mv[k][1];
            st->dag[0][KI][6] = INIT_STACK;
            sto(st, INIT_STACK, INIT_STACK, 4);
            sto(st, INIT_STACK - 4, SENTINEL - 1, 4);
            st->dag[0][KI][7] = INIT_STACK - 8;
            st->pc = b->target;
            {
                int r = rt_run(st, st->cycles + 50000000);
                if (r != 6) {
                    seterr(err, errlen, "boot del DSP: l'INIT a %#x non e' tornato (rt_run %d, pc %#x)", b->target, r, st->pc);
                    goto fail;
                }
            }
            continue;
        }
        if (flag_ignore) continue;
        if (b->fill) mem_put(I, b->target, NULL, b->count, 1, b->arg);
        else mem_put(I, b->target, blob + b->payload_off, b->payload_len, 0, 0);
        if (cancel && *cancel) { seterr(err, errlen, "interrotto"); goto fail; }
    }
    if (!entry) { seterr(err, errlen, "boot del DSP: manca il blocco FINAL"); goto fail; }
    ins_clear(I);
    st->pc = entry;
    st->npc = 0;
    I->fast_delays = 0;
    idle_install(I);
    if (run_cycles(I, target, cancel, progress, ctx, err, errlen)) goto fail;
    free(blk);
    return I;
fail:
    free(blk);
    dni_release(I);
    return NULL;
}

/* ============================================ stato "pulito" e file DNDS */
/* i soli campi che DspSynth porta nel C (Engine.from_py + go_native): il resto (anelli,
   contatori, diagnostica) riparte da zero. Le pagine sparse passano alla Periph nuova. */
static void rebase(dni *I)
{
    St *o = &I->st, n;
    Periph *po = (Periph *)I->per, *pn = (Periph *)rt_periph_new();
    int idx[NEV], k = 0;
    memset(&n, 0, sizeof n);
    memcpy(n.pe, o->pe, sizeof n.pe); memcpy(n.dag, o->dag, sizeof n.dag);
    n.mode1 = o->mode1; n.mode2 = o->mode2; n.mmask = o->mmask; n.irptl = o->irptl; n.imask = o->imask;
    n.imaskp = o->imaskp; n.lirptl = o->lirptl; n.flags = o->flags;
    memcpy(n.ustat, o->ustat, sizeof n.ustat);
    n.px = o->px; n.faddr = o->faddr; n.daddr = o->daddr; n.tperiod = o->tperiod; n.tcount = o->tcount;
    n.timer_due = o->timer_due; n.lcntr = o->lcntr; n.pc = o->pc;
    memcpy(n.pcstk, o->pcstk, sizeof n.pcstk); n.npc = o->npc;
    memcpy(n.loops, o->loops, sizeof n.loops); n.nloops = o->nloops;
    memcpy(n.sts, o->sts, sizeof n.sts); n.nsts = o->nsts;
    n.dly_active = o->dly_active; n.dly_target = o->dly_target; n.dly_left = o->dly_left; n.dly_from = o->dly_from;
    n.dly_rti = o->dly_rti;
    n.idle = o->idle; n.cycles = o->cycles;
    memcpy(n.irq_count, o->irq_count, sizeof n.irq_count);
    n.ddr_size = o->ddr_size;
    for (int ch = 0; ch < NDMA; ch++) {
        Dma *a = &po->dma[ch], *b = &pn->dma[ch];
        b->base = a->base; b->sec_id = a->sec_id; b->extra_sid = a->extra_sid; b->kind = a->kind; b->sport = a->sport;
        memcpy(b->regs, a->regs, sizeof b->regs); b->stat = a->stat; b->word_cycles = a->word_cycles;
    }
    memcpy(pn->sec_regs, po->sec_regs, sizeof pn->sec_regs); memcpy(pn->sec_pending, po->sec_pending, sizeof pn->sec_pending);
    pn->sec_active = po->sec_active;
    memcpy(pn->cgu_regs, po->cgu_regs, sizeof pn->cgu_regs);
    memcpy(pn->cgu_pllen, po->cgu_pllen, sizeof pn->cgu_pllen); memcpy(pn->cgu_bypass, po->cgu_bypass, sizeof pn->cgu_bypass);
    memcpy(pn->dmc_regs, po->dmc_regs, sizeof pn->dmc_regs); memcpy(pn->dmc_phy, po->dmc_phy, sizeof pn->dmc_phy);
    memcpy(pn->cdu_regs, po->cdu_regs, sizeof pn->cdu_regs);
    memcpy(pn->spi_regs, po->spi_regs, sizeof pn->spi_regs); pn->spi_stat = po->spi_stat; pn->spi_imsk = po->spi_imsk;
    pn->nsparse = po->nsparse;
    for (int s = 0; s < po->nsparse; s++) { pn->sparse_base[s] = po->sparse_base[s]; pn->sparse[s] = po->sparse[s]; po->sparse[s] = NULL; }
    pn->out_ch[0] = po->out_ch[0]; pn->out_ch[1] = po->out_ch[1];
    /* rt_spi_lock(1): niente eventi a tempo dei canali SPI2; gli altri nello stesso ordine */
    for (int e = 0; e < po->nev; e++)
        if (po->ev[e].ch != CH_SPI_TX && po->ev[e].ch != CH_SPI_RX) idx[k++] = e;
    for (int x = 1; x < k; x++) {                           /* ordine (ciclo, sequenza) */
        int v = idx[x], y = x - 1;
        while (y >= 0 && (po->ev[idx[y]].due > po->ev[v].due ||
                          (po->ev[idx[y]].due == po->ev[v].due && po->ev[idx[y]].seq > po->ev[v].seq))) { idx[y + 1] = idx[y]; y--; }
        idx[y + 1] = v;
    }
    pn->nev = k;
    for (int x = 0; x < k; x++) { pn->ev[x].due = po->ev[idx[x]].due; pn->ev[x].seq = (uint64_t)x + 1; pn->ev[x].ch = po->ev[idx[x]].ch; }
    pn->evseq = (uint64_t)k;
    pn->spi_lock = 1;
    n.next_event = -1;
    for (int x = 0; x < k; x++) if (n.next_event < 0 || pn->ev[x].due < n.next_event) n.next_event = pn->ev[x].due;
    free(po);
    I->per = pn;
    I->st = n;
    st_link(I);
}

static int wchunk(uint8_t **buf, size_t *len, size_t *cap, const char *tag, const void *data, size_t raw, int z)
{
    size_t slen = raw;
    uint8_t *tmp = NULL;
    uint8_t h[16];
    if (z) {
        uLongf zn = compressBound((uLong)raw);
        tmp = (uint8_t *)malloc(zn);
        if (!tmp || compress2(tmp, &zn, (const Bytef *)data, (uLong)raw, 6) != Z_OK) { free(tmp); return -1; }
        slen = zn; data = tmp;
    }
    memcpy(h, tag, 4);
    for (int i = 0; i < 4; i++) { h[4 + i] = (uint8_t)((z ? 1u : 0u) >> (8 * i)); h[8 + i] = (uint8_t)((uint32_t)raw >> (8 * i)); h[12 + i] = (uint8_t)((uint32_t)slen >> (8 * i)); }
    if (*len + 16 + slen > *cap) {
        size_t nc = *cap ? *cap : 1 << 20;
        uint8_t *q;
        while (nc < *len + 16 + slen) nc *= 2;
        q = (uint8_t *)realloc(*buf, nc);
        if (!q) { free(tmp); return -1; }
        *buf = q; *cap = nc;
    }
    memcpy(*buf + *len, h, 16); memcpy(*buf + *len + 16, data, slen);
    *len += 16 + slen;
    free(tmp);
    return 0;
}

/* DNDS v1 (tools/dsp_export.py): la chiave e' a zero (l'interprete accetta qualunque stato) */
static int dni_export(dni *I, uint8_t **out, size_t *outlen)
{
    uint8_t *buf = NULL, head[48];
    size_t len = 48, cap = 0;
    Periph *P = (Periph *)I->per;
    St st = I->st;
    Periph *pc = (Periph *)malloc(sizeof(Periph));
    int ns = P->nsparse;
    if (!pc) return -1;
    memcpy(pc, P, sizeof(Periph));
    st.l1 = st.l2 = st.ddr = NULL; st.mmio_rd = NULL; st.mmio_wr = NULL; st.seg = NULL; st.per = NULL;
    for (int k = 0; k < DN_SPARSE; k++) pc->sparse[k] = NULL;
    buf = (uint8_t *)malloc(48);
    if (!buf) { free(pc); return -1; }
    cap = 48;
    if (wchunk(&buf, &len, &cap, "ST  ", &st, sizeof st, 0) || wchunk(&buf, &len, &cap, "PER ", pc, sizeof(Periph), 1)) goto fail;
    for (int k = 0; k < ns; k++) {
        uint8_t *tmp = (uint8_t *)malloc(4 + DN_PAGE);
        if (!tmp) goto fail;
        for (int i = 0; i < 4; i++) tmp[i] = (uint8_t)(P->sparse_base[k] >> (8 * i));
        memcpy(tmp + 4, P->sparse[k], DN_PAGE);
        if (wchunk(&buf, &len, &cap, "SPG ", tmp, 4 + DN_PAGE, 1)) { free(tmp); goto fail; }
        free(tmp);
    }
    if (wchunk(&buf, &len, &cap, "L1  ", I->l1, DN_L1_SIZE, 1) || wchunk(&buf, &len, &cap, "L2  ", I->l2, DN_L2_SIZE, 1)
        || wchunk(&buf, &len, &cap, "DDR ", I->ddr, DN_DDR_SIZE, 1)) goto fail;
    memset(head, 0, sizeof head);
    memcpy(head, "DNDS", 4);
    head[4] = 1;
    {
        uint32_t v[6] = {(uint32_t)sizeof(St), (uint32_t)sizeof(Periph), DN_L1_SIZE, DN_L2_SIZE, DN_DDR_SIZE, (uint32_t)(5 + ns)};
        for (int k = 0; k < 6; k++) for (int i = 0; i < 4; i++) head[24 + 4 * k + i] = (uint8_t)(v[k] >> (8 * i));
    }
    memcpy(buf, head, 48);
    free(pc);
    *out = buf; *outlen = len;
    return 0;
fail:
    free(buf); free(pc);
    return -1;
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
typedef struct { const uint8_t *p, *end; } Rd;
static int rchunk(Rd *r, const char *tag, uint8_t *dst, size_t raw, char *err, size_t errlen)
{
    const uint8_t *h = r->p, *src;
    uint32_t flags, rlen, slen;
    if (r->end - r->p < 16) { seterr(err, errlen, "dnds: manca il blocco %.4s", tag); return -1; }
    flags = rd32(h + 4); rlen = rd32(h + 8); slen = rd32(h + 12);
    if (memcmp(h, tag, 4)) { seterr(err, errlen, "dnds: blocco %.4s al posto di %.4s", (const char *)h, tag); return -1; }
    if ((size_t)(r->end - r->p - 16) < slen) { seterr(err, errlen, "dnds: blocco %.4s troncato", tag); return -1; }
    if (rlen != raw) { seterr(err, errlen, "dnds: blocco %.4s di %u byte, attesi %zu", tag, rlen, raw); return -1; }
    src = h + 16;
    if (flags & 1) {
        uLongf got = (uLongf)raw;
        if (uncompress(dst, &got, src, slen) != Z_OK || got != raw) { seterr(err, errlen, "dnds: blocco %.4s: zlib", tag); return -1; }
    } else {
        if (slen != raw) { seterr(err, errlen, "dnds: blocco %.4s: lunghezze", tag); return -1; }
        memcpy(dst, src, raw);
    }
    r->p = src + slen;
    return 0;
}

static dni *dni_load(const uint8_t *buf, size_t len, char *err, size_t errlen)
{
    dni *I;
    Rd r;
    int ns;
    if (len < 48 || memcmp(buf, "DNDS", 4)) { seterr(err, errlen, "dnds: non e' un file DNDS"); return NULL; }
    if (rd32(buf + 4) != 1) { seterr(err, errlen, "dnds: versione %u, attesa 1", rd32(buf + 4)); return NULL; }
    if (rd32(buf + 24) != sizeof(St) || rd32(buf + 28) != sizeof(Periph)) {
        seterr(err, errlen, "dnds: St %u / Periph %u byte, l'interprete %zu / %zu", rd32(buf + 24), rd32(buf + 28), sizeof(St), sizeof(Periph));
        return NULL;
    }
    if (rd32(buf + 32) != DN_L1_SIZE || rd32(buf + 36) != DN_L2_SIZE || rd32(buf + 40) != DN_DDR_SIZE) {
        seterr(err, errlen, "dnds: dimensioni della memoria diverse");
        return NULL;
    }
    I = dni_alloc(err, errlen);
    if (!I) return NULL;
    r.p = buf + 48; r.end = buf + len;
    if (rchunk(&r, "ST  ", (uint8_t *)&I->st, sizeof(St), err, errlen)) goto fail;
    if (rchunk(&r, "PER ", (uint8_t *)I->per, sizeof(Periph), err, errlen)) goto fail;
    for (int k = 0; k < DN_SPARSE; k++) ((Periph *)I->per)->sparse[k] = NULL;
    ns = ((Periph *)I->per)->nsparse;
    if (ns < 0 || ns > DN_SPARSE) { seterr(err, errlen, "dnds: %d pagine sparse", ns); goto fail; }
    ((Periph *)I->per)->nsparse = 0;                         /* cresce con le pagine caricate (free in sicurezza) */
    for (int k = 0; k < ns; k++) {
        uint8_t *tmp = (uint8_t *)malloc(4 + DN_PAGE);
        if (!tmp) { seterr(err, errlen, "memoria"); goto fail; }
        if (rchunk(&r, "SPG ", tmp, 4 + DN_PAGE, err, errlen)) { free(tmp); goto fail; }
        memmove(tmp, tmp + 4, DN_PAGE);
        ((Periph *)I->per)->sparse[k] = tmp;
        ((Periph *)I->per)->nsparse = k + 1;
    }
    if (rchunk(&r, "L1  ", I->l1, DN_L1_SIZE, err, errlen) || rchunk(&r, "L2  ", I->l2, DN_L2_SIZE, err, errlen)
        || rchunk(&r, "DDR ", I->ddr, DN_DDR_SIZE, err, errlen)) goto fail;
    if (r.p != r.end) { seterr(err, errlen, "dnds: %td byte in piu' in fondo", r.end - r.p); goto fail; }
    if (I->st.ddr_size != DN_DDR_SIZE) { seterr(err, errlen, "dnds: ddr_size %u", I->st.ddr_size); goto fail; }
    st_link(I);
    for (int q = 0; q < I->st.nloops && q < LOOP_MAX; q++) mark_at(I, I->st.loops[q][0] & 0xFFFFFF, IF_EPI);
    if (I->st.dly_active) {                                   /* salto ritardato in corso: gli slot */
        Ins *i1;
        mark_at(I, I->st.pc, IF_EPI);
        i1 = ins_get(I, I->st.pc);
        if (i1 && i1->op != OP_BAD) mark_at(I, I->st.pc + i1->words, IF_EPI);
    }
    return I;
fail:
    dni_release(I);
    return NULL;
}

/* ============================================================ API (dn_dsp_ops) */
static void *ops_create(const uint8_t *buf, size_t len, char *err, size_t errlen)
{
    dni *I = dni_load(buf, len, err, errlen);
    if (!I) return NULL;
    dni_cur = I;
    idle_install(I);
    return I;
}
static void ops_destroy(void *d) { dni_release((dni *)d); }

static int ops_frame(void *dd, const uint8_t *frame, size_t n)
{
    dni *I = (dni *)dd;
    St *st = &I->st;
    int pending = 1;
    int64_t target = st->cycles + ((int64_t)1 << 40);
    dni_cur = I;
    while (st->cycles < target) {
        int r = rt_run(st, target);
        if (r == 5) {                                       /* il DSP vuole un frame */
            if (!pending) return 0;
            pending = 0;
            rt_push_frame(st, frame, (uint32_t)n);
        } else if (r == 2) {
            break;
        } else if (r == 4) {
            snprintf(I->err, sizeof I->err, "errore dell'interprete: err_code %#x a pc %#x", st->err_code, st->pc);
            return -4;
        } else if (r == 3) {
            snprintf(I->err, sizeof I->err, "istruzione non decodificabile a pc %#x", I->bad_pc);
            return -3;
        } else {
            snprintf(I->err, sizeof I->err, "rt_run inatteso: %d (pc %#x)", r, st->pc);
            return -1;
        }
    }
    return 1;
}
static int ops_pull(void *dd, int32_t *out, int max) { return rt_pull_audio(&((dni *)dd)->st, out, max); }
static uint32_t ops_reply(void *dd, uint8_t *out, uint32_t n) { return rt_reply(&((dni *)dd)->st, out, n); }
static const char *ops_error(const void *dd) { return ((const dni *)dd)->err; }
static int64_t ops_cycles(const void *dd) { return ((const dni *)dd)->st.cycles; }
static const uint8_t *ops_memory(const void *dd, int which, size_t *len)
{
    const dni *I = (const dni *)dd;
    switch (which) {
    case 0: if (len) *len = DN_L1_SIZE; return I->l1;
    case 1: if (len) *len = DN_L2_SIZE; return I->l2;
    case 2: if (len) *len = DN_DDR_SIZE; return I->ddr;
    }
    if (len) *len = 0;
    return 0;
}

static void ops_analyze(void *dd)
{
    dni_cur = (dni *)dd;
    dni_analyze((dni *)dd, NULL, 0);
}

static const dn_dsp_ops g_ops = {"interprete", "", ops_create, ops_destroy, ops_frame, ops_pull, ops_reply,
                                 ops_error, ops_cycles, ops_memory, ops_analyze};
const dn_dsp_ops *dni_dsp_get_ops(void) { return &g_ops; }

/* Il boot del DSP dalla sezione 7 del firmware (stream di boot) fino a `cycles` cicli del DSP:
   -> lo stato DNDS (malloc'd, free) per dn_dsp_new / dn_set_dsp_state */
int dn_dsp_boot(const uint8_t *sec7, size_t len, int64_t cycles, volatile int *cancel,
                void (*progress)(void *, int, const char *), void *ctx, uint8_t **dnds, size_t *dnds_len,
                char *err, size_t errlen)
{
    dni *I = boot_state(sec7, len, cycles, cancel, progress, ctx, err, errlen);
    int r;
    if (!I) return -1;
    rebase(I);
    r = dni_export(I, dnds, dnds_len);
    if (r) seterr(err, errlen, "memoria");
    dni_release(I);
    return r;
}

/* ======================================================== analisi dei flag (gen.liveness) */
/* Quali flag di ASTAT sono vivi dopo ogni istruzione: porta di emu/sharc/rec/gen.py (discover,
   flag_effects, successors, liveness). Un flag morto non si calcola (setfl con maschera
   `live`). Un PC non scoperto, un salto indiretto, una chiamata e un ritorno hanno tutti i flag
   vivi: l'analisi e' conservativa, quindi non cambia mai il risultato. */
#define F_AZ (1u << AZ)
#define F_ALU ALU_FLAGS
#define F_MUL MUL_FLAGS
#define F_SH SH_FLAGS

static uint32_t cond_reads(int c)
{
    if (c == 31) return 0;
    switch (c & 15) {
    case 0: return 1u << AZ;
    case 1: case 2: return (1u << AN) | (1u << AZ);
    case 3: return 1u << AC;
    case 4: return 1u << AV;
    case 5: return 1u << MV;
    case 6: return 1u << MN;
    case 7: return 1u << SV;
    case 8: return 1u << SZ;
    case 13: return 1u << BTF;
    case 14: return 1u << SF;
    }
    return 0;
}

static uint32_t compute_kills(uint32_t comp)
{
    comp &= 0x7FFFFF;
    if (!comp) return 0;
    if (!(comp >> 22)) {
        int cu = (comp >> 20) & 3, op = (comp >> 12) & 0xFF;
        if (cu == 0) return F_ALU;
        if (cu == 1) return (op >> 6 == 0 && op != 0x30) ? 0 : F_MUL;
        if (cu == 2) return ((op >> 2) == 0x24 || (op >> 2) == 0x25 || (op >> 2) == 0x1C) ? 0 : F_SH;
        return 0;
    }
    {
        int op6 = (comp >> 16) & 0x3F;
        if (op6 == 0 || op6 == 1) return 0;
        return F_MUL | F_ALU;
    }
}

static uint32_t compute_reads(uint32_t comp)
{
    comp &= 0x7FFFFF;
    if (comp && !(comp >> 22) && ((comp >> 20) & 3) == 0) {
        int op = (comp >> 12) & 0xFF;
        if (op == 0x05 || op == 0x06 || op == 0x25 || op == 0x26) return 1u << AC;
    }
    return 0;
}

static int is_flow(int t)
{
    return t == T_8a_abs || t == T_8a_rel || t == T_8p_undoc48 || t == T_9a_abs || t == T_9a_rel || t == T_9b_abs
        || t == T_9b_rel || t == T_11a || t == T_11c || t == T_23p_undoc16;
}

static int is_db(const Dec *d)
{
    int t = d->type;
    const Dec *x = d;
    if (t == T_25a_direct || t == T_25a_pcrel) return 1;
    if (t == T_23p_undoc16) return (int)((d->f[FN_operand] >> 6) & 1);
    if (is_flow(t)) return (int)x->f[FN_j];
    return 0;
}

/* targets(): destinazioni statiche (salti, chiamate, fine dei loop). -> numero (0..1) */
static int targets_of(uint32_t pc, const Dec *d, uint32_t *out)
{
    switch (d->type) {
    case T_8a_abs: case T_25a_direct: *out = d->f[FN_addr] & 0xFFFFFF; return 1;
    case T_8a_rel: case T_25a_pcrel: *out = (pc + (uint32_t)sext_i(d->f[FN_reladdr], 24)) & 0xFFFFFF; return 1;
    case T_9a_rel: case T_9b_rel: *out = (pc + (uint32_t)sext_i(d->f[FN_reladdr], 6)) & 0xFFFFFF; return 1;
    case T_8p_undoc48:
        *out = (d->f[FN_r] ? pc + (uint32_t)sext_i(d->f[FN_imm], 24) : d->f[FN_imm]) & 0xFFFFFF; return 1;
    case T_12a_imm: case T_12a_ureg: case T_13a: *out = (pc + (uint32_t)sext_i(d->f[FN_reladdr], 23)) & 0xFFFFFF; return 1;
    }
    return 0;
}

static int loop_end_of(uint32_t pc, const Dec *d, uint32_t *e)
{
    if (d->type == T_12a_imm || d->type == T_12a_ureg || d->type == T_13a) {
        *e = (pc + (uint32_t)sext_i(d->f[FN_reladdr], 23)) & 0xFFFFFF;
        return 1;
    }
    return 0;
}

static void flag_effects(const Dec *d, uint32_t *reads, uint32_t *kills)
{
    int t = d->type;
    int c = (d->has >> FN_cond) & 1 ? (int)d->f[FN_cond] : 31;
    uint32_t comp = (d->has >> FN_compute) & 1 ? d->f[FN_compute] : 0, r, k = 0;
    if (t == T_23p_undoc16) c = (int)((d->f[FN_operand] >> 1) & 31);
    r = cond_reads(c);
    if (t == T_2c) {
        int op = (d->f[FN_compute] >> 8) & 0xF;
        k = (op == 7 || op == 15) ? F_MUL : F_ALU;
    } else if (t == T_6a_nomem || t == T_6a_mem) {
        int kind = (d->f[FN_shiftimm] >> 16) & 0x3F;
        k = (kind == 0x24 || kind == 0x25 || kind == 0x1C) ? 0 : F_SH;
    } else if (comp) {
        r |= compute_reads(comp);
        if (t != T_9a_abs && t != T_9a_rel && t != T_11a) k = compute_kills(comp);
    }
    if (c != 31 && t != T_1a && t != T_1b && t != T_2b && t != T_2c) k = 0;
    if (t == T_20a && d->f[FN_spu]) r = 0xFFFFFFFFu;
    if (t == T_18a && (d->f[FN_sreg] == 6 || d->f[FN_sreg] == 7)) { r = 0xFFFFFFFFu; k = 0; }
    if (t == T_5a_move || t == T_5b_move) {
        uint32_t src = (d->f[FN_srcureghigh] << 2) | d->f[FN_srcureglow];
        if (src == 0x76 || src == 0x77) r = 0xFFFFFFFFu;
    }
    if ((t == T_3a || t == T_3b || t == T_3d || t == T_15a || t == T_15b || t == T_14a) && ((d->has >> FN_ureg) & 1)
        && (d->f[FN_ureg] == 0x76 || d->f[FN_ureg] == 0x77) && d->f[FN_d]) r = 0xFFFFFFFFu;
    if (t == T_12a_ureg && (d->f[FN_ureg] == 0x76 || d->f[FN_ureg] == 0x77)) r = 0xFFFFFFFFu;
    *reads = r; *kills = k;
}

typedef struct {
    uint32_t pc, reads, kills, term;
    int32_t succ[2];
    int nsucc;
    int unknown;                    /* successori sconosciuti: tutti i flag vivi */
    int32_t ls_head;                /* lista (in ls_nodes) degli inizi di loop che finiscono qui */
} ANode;

typedef struct {
    ANode *n;
    uint32_t cnt, cap;
    int32_t *map;                    /* indice di ins_index -> nodo, -1 non visto, -2 non codice */
} Analysis;

static int an_find(Analysis *A, uint32_t pc)
{
    uint32_t k;
    if (ins_index(pc, &k)) return -1;
    return A->map[k] >= 0 ? A->map[k] : -1;
}

/* aggiunge `pc` all'insieme (decodificandolo) se vale; -1 se non e' codice supportato */
static int an_add(dni *I, Analysis *A, uint32_t pc, uint32_t *stack, uint32_t *sp)
{
    uint32_t k;
    Ins *in;
    if (ins_index(pc, &k)) return -1;
    if (A->map[k] >= 0) return A->map[k];
    if (A->map[k] == -2) return -1;
    in = ins_get(I, pc);
    if (!in || in->op == OP_BAD) { A->map[k] = -2; return -1; }
    if (A->cnt == A->cap) {
        A->cap = A->cap ? A->cap * 2 : 1 << 16;
        A->n = (ANode *)realloc(A->n, A->cap * sizeof(ANode));
    }
    A->n[A->cnt].pc = pc;
    A->map[k] = (int32_t)A->cnt;
    stack[(*sp)++] = pc;
    return (int)A->cnt++;
}

void dni_analyze(dni *I, const uint32_t *seeds, uint32_t nseeds)
{
    Analysis A = {0};
    uint32_t *stack, sp = 0, i;
    uint32_t *live_in = NULL;
    Dec *decs = NULL;
    int32_t *slot2 = NULL;
    uint32_t *ls_start = NULL, nls = 0;
    int32_t *ls_next = NULL;
    A.map = (int32_t *)malloc(INS_REGIONS * sizeof(int32_t));
    stack = (uint32_t *)malloc(INS_REGIONS * sizeof(uint32_t));
    if (!A.map || !stack) { free(A.map); free(stack); return; }
    for (i = 0; i < INS_REGIONS; i++) A.map[i] = -1;
    /* semi: quelli dati, i vettori, il pc, la pila dei pc, le istruzioni gia' decodificate */
    for (i = 0; i < nseeds; i++) an_add(I, &A, seeds[i], stack, &sp);
    for (i = 0; i < 64; i++) an_add(I, &A, 0x90000u + 4 * i, stack, &sp);
    an_add(I, &A, I->st.pc, stack, &sp);
    for (int q = 0; q < I->st.npc; q++) an_add(I, &A, I->st.pcstk[q], stack, &sp);
    for (uint32_t k = 0; k < INS_REGIONS; k++) {
        uint32_t pc;
        if (I->tab[k].op <= OP_BAD) continue;
        pc = k < 0xE0000u ? 0x120000u + k : k < 0x160000u ? 0xB80000u + (k - 0xE0000u) : 0x90000u + (k - 0x160000u);
        an_add(I, &A, pc, stack, &sp);
    }
    /* discesa ricorsiva con fall-through (gen.discover); i riempimenti a zero si fermano */
    while (sp) {
        uint32_t pc = stack[--sp], t1;
        Dec d;
        Ins *in = ins_get(I, pc);
        if (!in || in->op == OP_BAD || sh_decode(&I->st, pc, &d)) continue;
        if (targets_of(pc, &d, &t1)) an_add(I, &A, t1, stack, &sp);
        if (d.type == T_21a && d.raw == 0) {                 /* NOP a zero: dopo 8 di fila ci si ferma */
            int run = 0;
            uint32_t a = pc;
            while (run < 8) {
                Dec pd;
                if (an_find(&A, a - 3) < 0 || sh_decode(&I->st, a - 3, &pd) || pd.type != T_21a || pd.raw != 0) break;
                run++; a -= 3;
            }
            if (run >= 8) continue;
        }
        an_add(I, &A, pc + (uint32_t)d.words, stack, &sp);
    }
    free(stack);
    live_in = (uint32_t *)calloc(A.cnt ? A.cnt : 1, 4);
    slot2 = (int32_t *)malloc((A.cnt + 1) * sizeof(int32_t));
    decs = (Dec *)malloc((A.cnt + 1) * sizeof(Dec));
    ls_start = (uint32_t *)malloc((A.cnt + 1) * 4);
    ls_next = (int32_t *)malloc((A.cnt + 1) * sizeof(int32_t));
    if (!live_in || !slot2 || !decs || !ls_start || !ls_next) goto out;
    for (i = 0; i < A.cnt; i++) {
        Dec *d = &decs[i];
        uint32_t e;
        sh_decode(&I->st, A.n[i].pc, d);
        slot2[i] = -1;
        A.n[i].ls_head = -1;
        A.n[i].term = 0;
    }
    for (i = 0; i < A.cnt; i++) {                           /* loop: inizi del corpo e condizione di fine */
        const Dec *d = &decs[i];
        uint32_t e;
        if (loop_end_of(A.n[i].pc, d, &e)) {
            int en = an_find(&A, e);
            if (en >= 0) {
                ls_start[nls] = A.n[i].pc + (uint32_t)d->words;
                ls_next[nls] = A.n[en].ls_head;
                A.n[en].ls_head = (int32_t)nls++;
                if (d->type == T_13a && d->f[FN_term] != 15 && d->f[FN_term] != 31)
                    A.n[en].term |= cond_reads((int)d->f[FN_term]);
            }
        }
    }
    for (i = 0; i < A.cnt; i++) {
        ANode *nd = &A.n[i];
        const Dec *d = &decs[i];
        uint32_t pc = nd->pc, fall = pc + (uint32_t)d->words, t1;
        int c = (d->has >> FN_cond) & 1 ? (int)d->f[FN_cond] : 31;
        flag_effects(d, &nd->reads, &nd->kills);
        nd->nsucc = 0; nd->unknown = 0;
        if (is_flow(d->type)) {
            if (d->type == T_11a || d->type == T_11c || d->type == T_23p_undoc16 || d->type == T_9a_abs
                || d->type == T_9b_abs || d->f[FN_b]) {
                nd->unknown = 1;
            } else if (!d->f[FN_j]) {
                targets_of(pc, d, &t1);
                nd->succ[nd->nsucc++] = an_find(&A, t1);
                if (c != 31) nd->succ[nd->nsucc++] = an_find(&A, fall);
            } else {
                nd->succ[nd->nsucc++] = an_find(&A, fall);
            }
            nd->ls_head = -1;                               /* i salti non ereditano gli inizi di loop */
        } else {
            nd->succ[nd->nsucc++] = an_find(&A, fall);
        }
        if (is_db(d)) {
            int n1 = an_find(&A, fall);
            if (n1 >= 0) {
                int n2 = an_find(&A, fall + (uint32_t)decs[n1].words);
                if (n2 >= 0) slot2[n2] = (int32_t)i;
            }
        }
    }
    for (i = 0; i < A.cnt; i++) {       /* secondo slot di un salto ritardato: i successori del salto */
        ANode *nd;
        const Dec *bd;
        uint32_t t1, fall2;
        int bc;
        if (slot2[i] < 0) continue;
        nd = &A.n[i]; bd = &decs[slot2[i]];
        bc = (bd->has >> FN_cond) & 1 ? (int)bd->f[FN_cond] : 31;
        fall2 = nd->pc + (uint32_t)decs[i].words;
        nd->nsucc = 0; nd->unknown = 0;
        if (bd->type == T_11a || bd->type == T_11c || bd->type == T_23p_undoc16 || bd->type == T_9a_abs || bd->type == T_9b_abs
            || (is_flow(bd->type) && bd->f[FN_b])) {
            nd->unknown = 1;
        } else if (targets_of(A.n[slot2[i]].pc, bd, &t1)) {
            nd->succ[nd->nsucc++] = an_find(&A, t1);
            if (bc != 31) nd->succ[nd->nsucc++] = an_find(&A, fall2);
        } else nd->unknown = 1;
    }
    {   /* punto fisso, all'indietro (i nodi sono in ordine di scoperta) */
        int changed = 1, guard = 0;
        uint32_t *live_out = (uint32_t *)calloc(A.cnt + 1, 4);
        if (!live_out) goto out;
        while (changed && guard++ < 10000) {
            changed = 0;
            for (int32_t q = (int32_t)A.cnt - 1; q >= 0; q--) {
                ANode *nd = &A.n[q];
                uint32_t lo = nd->term, li;
                if (nd->unknown) lo = 0xFFFFFFFFu;
                else {
                    for (int sx = 0; sx < nd->nsucc; sx++) lo |= nd->succ[sx] >= 0 ? live_in[nd->succ[sx]] : 0xFFFFFFFFu;
                    for (int32_t x = nd->ls_head; x >= 0; x = ls_next[x]) {
                        int sn = an_find(&A, ls_start[x]);
                        lo |= sn >= 0 ? live_in[sn] : 0xFFFFFFFFu;
                    }
                }
                live_out[q] = lo;
                li = nd->reads | (lo & ~nd->kills);
                if (li != live_in[q]) { live_in[q] = li; changed = 1; }
            }
        }
        {
            uint32_t dead = 0;
            for (i = 0; i < A.cnt; i++) {
                Ins *in = ins_get(I, A.n[i].pc);
                if (in) in->live = live_out[i];
                if ((live_out[i] & (ALU_FLAGS | MUL_FLAGS | SH_FLAGS)) == 0) dead++;
            }
            if (getenv("DNI_VERBOSE")) fprintf(stderr, "analisi: %u istruzioni, %u senza flag vivi\n", A.cnt, dead);
        }
        free(live_out);
    }
out:
    free(live_in); free(slot2); free(decs); free(ls_start); free(ls_next); free(A.n); free(A.map);
}

/* ===================================================== micro-benchmark (dsptool bench) */
/* Un loop DO di 16 istruzioni fabbricate di un solo tipo: ns per istruzione dell'esecutore.
   kind: 0 NOP, 1 EXEC Rn=Rx*Ry (float), 2 EXEC Rn=Rx+Ry, 3 OP_3 carica DM(I0,M0) post-modifica in R2,
   4 OP_3 carica + moltiplica, 5 5MOVE R3=R4, 6 OP_4 carica con modifica immediata, 7 MEM15 */
double dni_bench(int kind, int spec, int64_t iters)
{
    int mixed = kind == 100, body_n = mixed ? 200 : 16;
    unsigned rng = 12345;
    dni *I = dni_alloc(NULL, 0);
    St *st;
    const uint32_t base = 0x1c0000u, n = (uint32_t)body_n;
    struct timespec t0, t1;
    if (!I) return -1;
    blank_state(I);
    st = &I->st;
    dni_cur = I;
    for (uint32_t i = 0; i < n + 2; i++) {
        uint32_t k;
        Ins *in;
        ins_index(base + i, &k);
        in = &I->tab[k];
        memset(in, 0, sizeof *in);
        in->words = 1; in->live = 0xFFFFFFFFu;
        in->op = OP_NOP;
        if (i == 0) {                                          /* DO iters */
            in->op = OP_LOOPI; in->a[0] = (int32_t)iters; in->a[1] = (int32_t)(base + 1); in->a[2] = (int32_t)(base + n);
        } else if (i <= n) {
            int kk = kind;
            if (mixed) { rng = rng * 1103515245u + 12345u; kk = 1 + (int)((rng >> 16) % 7); }
            switch (kk) {
            case 1: in->op = OP_EXEC; in->a[0] = (int32_t)((0x130u << 12) | (1 << 8) | (2 << 4) | 3); break;
            case 2: in->op = OP_EXEC; in->a[0] = (int32_t)((0x081u << 12) | (1 << 8) | (2 << 4) | 3); break;
            case 3: in->op = OP_3; in->a[0] = 31; in->a[1] = 1; in->a[2] = 0; in->a[3] = 0; in->a[4] = 0; in->a[5] = 2; in->a[6] = 4; break;
            case 4: in->op = OP_3; in->a[0] = 31; in->a[1] = 1; in->a[2] = 0; in->a[3] = 0; in->a[4] = 0; in->a[5] = 2; in->a[6] = 4;
                    in->a[8] = (int32_t)((0x130u << 12) | (1 << 8) | (2 << 4) | 3); break;
            case 5: in->op = OP_5MOVE; in->a[0] = 31; in->a[1] = 4; in->a[2] = 3; break;
            case 6: in->op = OP_4; in->a[0] = 31; in->a[1] = 0; in->a[2] = 0; in->a[3] = 1; in->a[4] = 0; in->a[5] = 2; in->a[6] = 4; break;
            case 7: in->op = OP_MEM15; in->a[0] = 2; in->a[1] = 0; in->a[2] = 1; in->a[3] = 0; in->a[4] = 4; break;
            default: break;
            }
        }
        in->nw = 1;
        in->xop = in->op;
        if (spec) { int x = spx_find(in->op, in->a); if (x) in->xop = (uint16_t)x; }
        if (i == n) in->flags |= IF_EPI;
        if (i == 0) in->flags |= IF_EPI;
    }
    /* I0 = 0x300000 (byte, L1), M0 = 1; R2,R3 valori float */
    st->dag[0][KI][0] = 0x300000u; st->dag[0][KM][0] = 1;
    st->pe[0].r[0][2] = 0x3f800000u; st->pe[0].r[0][3] = 0x40000000u;
    st->pc = base;
    st->deadline = (int64_t)1 << 60;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    {
        int r = rt_exec(st);
        (void)r;
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    {
        double ns = (t1.tv_sec - t0.tv_sec) * 1e9 + (t1.tv_nsec - t0.tv_nsec);
        double instrs = (double)st->cycles;
        dni_release(I);
        return instrs > 0 ? ns / instrs : -1;
    }
}

#include "dn_gensrc.inc"
