
/* ================= dt2_spin (emu/nativespin.py) ================= */
typedef int (*emu_start_fn)(void *uc, uint64_t begin, uint64_t until, uint64_t timeout, size_t count);
typedef int (*py_raise_fn)(int32_t vec, int32_t level);   /* Machine.raise_vector: 1, 0, -1 errore */
typedef int64_t (*py_step_fn)(int64_t now);               /* evento Python .step: >=1, -1 None, -2 errore */
typedef int (*py_service_fn)(int64_t now);                /* evento Python .service: 0, -1 errore */

/* Cio' che serve per consegnare un interrupt: le funzioni di Unicorn e lo
   stato di dt2_raise_vector, e il Python di ripiego quando il C non puo'
   (pagina dello stack non mappata: Machine._ensure_frame la mappa). */
typedef struct irq_ctx {
    exc_state *exc;
    py_raise_fn py_raise;
    int64_t py_raises;
    int32_t err, pad;
} irq_ctx;

static const uint32_t INTC_BASE[3] = {0xFC048000u, 0xFC04C000u, 0xFC050000u};

static int irq_raise(void *uc, irq_ctx *q, int vec, int level)
{
    int r = dt2_raise_vector(uc, q->exc, vec, level, 0);
    if (r >= 0)
        return r;
    q->py_raises++;
    r = q->py_raise(vec, level);
    if (r < 0) { q->err = 20; return -1; }
    return r;
}

/* emu/pit.py interrupt_level: il livello programmato (ICR & 7), 0 = None
   (spento o mascherato), -1 errore. */
static int irq_level(void *uc, irq_ctx *q, int vec, int respect_mask)
{
    uint8_t b[8];
    int i, src, icr;
    uint32_t hi, lo, masked;
    if (vec < 64 || vec >= 256)
        return 0;
    i = (vec - 64) / 64;
    src = vec - 64 - 64 * i;
    if (q->exc->mem_read(uc, INTC_BASE[i] + 0x40u + (uint32_t)src, b, 1)) { q->err = 21; return -1; }
    icr = b[0] & 7;
    if (!icr || !respect_mask)
        return icr;
    if (q->exc->mem_read(uc, INTC_BASE[i] + 0x08u, b, 8)) { q->err = 21; return -1; }
    hi = be32(b); lo = be32(b + 4);
    masked = src < 32 ? (lo >> src) & 1u : (hi >> (src - 32)) & 1u;
    return masked ? 0 : icr;
}

static int irq_ipl(void *uc, irq_ctx *q)
{
    uint32_t sr = 0;
    if (q->exc->reg_read(uc, q->exc->r_sr, &sr)) { q->err = 22; return -1; }
    return (int)((sr >> 8) & 7u);
}

/* ---- Pits / Dtims: service intero (pit.Pits.service, dtim.Dtims.service,
   con e senza DT2_LEVEL_IRQ), riga per riga. ---- */
static int tim_try(void *uc, tim_state *t, int c)        /* _try_pending */
{
    int vec = t->vec0 + c, lvl, ipl, r;
    lvl = irq_level(uc, t->q, vec, 1);
    if (lvl < 0) return -1;
    ipl = irq_ipl(uc, t->q);
    if (ipl < 0) return -1;
    if (lvl == 0 || ipl >= lvl) { t->missed[c]++; return 0; }
    r = irq_raise(uc, t->q, vec, lvl);
    if (r < 0) return -1;
    if (r) { t->fired[c]++; t->pend[c] = 0; }
    return r;
}

int dt2_tim_service_full(void *uc, tim_state *t, int64_t done)
{
    double p, d = (double)done;
    int i, c, r;
    if (t->held)
        return 0;
    for (i = 0; i < t->nch; i++) {
        c = t->ch[i];
        r = tim_period(uc, t, c, &p);
        if (r < 0) return -1;
        if (r == 0) { t->has[c] = 0; continue; }
        if (!t->has[c]) { t->next[c] = d + p; t->has[c] = 1; continue; }
        if (d < t->next[c]) {
            if (t->level_irq && t->pend[c] && tim_try(uc, t, c) < 0) return -1;
            continue;
        }
        t->next[c] += p;
        if (t->next[c] <= d)
            t->next[c] = d + p;          /* drop the backlog, do not chase it */
        if (t->kind == 1) {              /* DTER |= REF prima dell'interrupt */
            uint8_t b;
            if (t->q->exc->mem_read(uc, t->base[c] + 3u, &b, 1)) { t->err = 2; return -1; }
            b |= 0x02;
            if (t->q->exc->mem_write(uc, t->base[c] + 3u, &b, 1)) { t->err = 2; return -1; }
        }
        if (t->level_irq) {
            if (t->pend[c]) t->merged[c]++;
            t->pend[c] = 1;
            if (tim_try(uc, t, c) < 0) return -1;
            continue;
        }
        {
            int vec = t->vec0 + c, lvl, ipl;
            lvl = irq_level(uc, t->q, vec, 1);
            if (lvl < 0) return -1;
            ipl = irq_ipl(uc, t->q);
            if (ipl < 0) return -1;
            if (lvl == 0 || ipl >= lvl)
                t->missed[c]++;
            else {
                r = irq_raise(uc, t->q, vec, lvl);
                if (r < 0) return -1;
                if (r) t->fired[c]++;
            }
        }
    }
    return 0;
}

/* ---- SSI0: _deliver_vector170 / _deliver_vector191 e i tre hook
   (_on_cint, _on_intfrch1, _on_force_rte) di emu/ssi.py. ---- */
static int sst_deliver(void *uc, sst_state *s)
{
    irq_ctx *q = s->q;
    int lvl, ipl, r;
    if (s->int50_asserted && !s->int50_delivered) {
        lvl = irq_level(uc, q, s->tx_vec, 1);
        if (lvl < 0) return -1;
        if (lvl) {
            ipl = irq_ipl(uc, q);
            if (ipl < 0) return -1;
            if (ipl < lvl) {
                r = irq_raise(uc, q, s->tx_vec, lvl);
                if (r < 0) return -1;
                if (r) { s->int50_delivered = 1; s->vector170++; }
            }
        }
    }
    if (s->force_asserted && !s->force_delivered) {
        lvl = irq_level(uc, q, s->force_vec, 0);
        if (lvl < 0) return -1;
        if (lvl) {
            ipl = irq_ipl(uc, q);
            if (ipl < 0) return -1;
            if (ipl < lvl) {
                r = irq_raise(uc, q, s->force_vec, lvl);
                if (r < 0) return -1;
                if (r) { s->force_delivered = 1; s->vector191++; }
            }
        }
    }
    return 0;
}

/* Ssi0Dma.service: le richieste scadute e poi le consegne. -> 0, -1 errore */
int dt2_sst_service_full(void *uc, sst_state *s, int64_t done)
{
    if (dt2_sst_service(uc, s, done) < 0)
        return -1;
    return sst_deliver(uc, s);
}

/* Ssi0Dma._on_cint: hook di scrittura su CINT (il gestore del vettore 170) */
void dt2_sst_on_cint(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    sst_state *s = (sst_state *)user_data;
    (void)uc; (void)type; (void)address;
    if (size == 1 && ((value & 0x40) || (value & 0x3F) == s->tx_ch)) {
        s->int50_asserted = 0;
        s->int50_delivered = 0;
    }
}

/* Ssi0Dma._on_intfrch1: hook di scrittura su INTFRCH1 (prima della scrittura) */
void dt2_sst_on_intfrch1(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    sst_state *s = (sst_state *)user_data;
    uint8_t b[4];
    int off = (int)(address - 0xFC04C010u), k, asserted;
    (void)type;
    if (s->mem_read(uc, 0xFC04C010u, b, 4)) { s->err = 10; return; }
    for (k = 0; k < size && off + k < 4; k++)
        if (off + k >= 0)
            b[off + k] = (uint8_t)((uint64_t)value >> (8 * (size - 1 - k)));
    asserted = (be32(b) & 0x80000000u) != 0;
    if (asserted && !s->force_asserted)
        s->force_delivered = 0;
    s->force_asserted = asserted;
    if (!asserted)
        s->force_delivered = 0;
}

/* Ssi0Dma._on_force_rte: all'RTE del gestore del canale 50 il vettore 191
   forzato entra se l'IPL che l'RTE ripristina lo consente (via dt2_gate cfn). */
void dt2_sst_force_rte(void *uc, uint64_t address, uint32_t size, void *data)
{
    sst_state *s = (sst_state *)data;
    irq_ctx *q = s->q;
    uint32_t sp = 0;
    uint8_t b[2];
    int lvl, r;
    (void)address; (void)size;
    if (!s->force_asserted || s->force_delivered)
        return;
    lvl = irq_level(uc, q, s->force_vec, 0);
    if (lvl <= 0)
        return;
    if (q->exc->reg_read(uc, q->exc->r_a7, &sp) || q->exc->mem_read(uc, sp + 2u, b, 2)) {
        q->err = 23;
        return;
    }
    if ((int)((rbe16(b) >> 8) & 7u) < lvl) {
        r = irq_raise(uc, q, s->force_vec, lvl);
        if (r == 1) { s->force_delivered = 1; s->vector191++; }
    }
}

/* ---- Interrupt forzati (emu/intfrc.py ForcedIrqs) in C ----
   inflight e skip: un bit per sorgente di ciascun INTC; delle sorgenti in
   attesa basta la massima (livello, vettore), l'unica che si consegna. */
typedef struct {
    irq_ctx *q;
    uint64_t inflight[3], skip[3];
    int32_t dirty, has_pend, best_lvl, best_vec;
    int64_t now, at_rte, retry;
    int64_t delivered[192];             /* indice: vettore - 64 */
    int32_t err, pad;
} frc_state;

static int frc_asserted(void *uc, frc_state *f)       /* ForcedIrqs._asserted */
{
    uint8_t b[8];
    int i, src;
    if (!f->dirty)
        return f->has_pend;
    f->dirty = 0;
    f->has_pend = 0; f->best_lvl = 0; f->best_vec = 0;
    for (i = 0; i < 3; i++) {
        uint32_t hi, lo;
        if (f->q->exc->mem_read(uc, INTC_BASE[i] + 0x10u, b, 8)) { f->err = 30; return -1; }
        hi = be32(b); lo = be32(b + 4);
        if (!(hi | lo))
            continue;
        for (src = 0; src < 64; src++) {
            uint32_t bit = src >= 32 ? (hi >> (src - 32)) & 1u : (lo >> src) & 1u;
            int vec = 64 + 64 * i + src, lvl;
            if (!bit || ((f->skip[i] >> src) & 1u) || ((f->inflight[i] >> src) & 1u))
                continue;
            if (f->q->exc->mem_read(uc, INTC_BASE[i] + 0x40u + (uint32_t)src, b, 1)) { f->err = 30; return -1; }
            lvl = b[0] & 7;
            if (!lvl)
                continue;
            if (!f->has_pend || lvl > f->best_lvl || (lvl == f->best_lvl && vec > f->best_vec)) {
                f->best_lvl = lvl;
                f->best_vec = vec;
            }
            f->has_pend = 1;
        }
    }
    return f->has_pend;
}

static void frc_took(frc_state *f, int vec, int at_rte)
{
    int i = (vec - 64) / 64, src = vec - 64 - 64 * i;
    f->inflight[i] |= 1ull << src;
    f->dirty = 1;
    f->delivered[vec - 64]++;
    if (at_rte)
        f->at_rte++;
}

/* ForcedIrqs.step: retry se c'e' qualcosa in attesa, -1 None, -2 errore */
int64_t dt2_frc_step(void *uc, frc_state *f)
{
    int a = frc_asserted(uc, f);
    if (a < 0) return -2;
    return a ? f->retry : -1;
}

/* ForcedIrqs.service -> 0, -1 errore */
int dt2_frc_service(void *uc, frc_state *f, int64_t done)
{
    int a, ipl, r, lvl, vec;
    f->now = done;
    a = frc_asserted(uc, f);
    if (a <= 0)
        return a;
    lvl = f->best_lvl; vec = f->best_vec;
    ipl = irq_ipl(uc, f->q);
    if (ipl < 0) return -1;
    if (lvl <= ipl && lvl != 7)
        return 0;
    r = irq_raise(uc, f->q, vec, lvl);
    if (r < 0) return -1;
    if (r) frc_took(f, vec, 0);
    return 0;
}

/* ForcedIrqs._on_write: hook di scrittura su INTFRCH/INTFRCL (prima della scrittura) */
void dt2_frc_on_write(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    frc_state *f = (frc_state *)user_data;
    uint8_t b[8];
    uint32_t hi, lo;
    uint64_t m;
    int i, off, k, src;
    (void)type;
    for (i = 0; i < 3; i++)
        if (address >= INTC_BASE[i] + 0x10u && address < INTC_BASE[i] + 0x18u)
            break;
    if (i == 3)
        return;
    if (f->q->exc->mem_read(uc, INTC_BASE[i] + 0x10u, b, 8)) { f->err = 30; return; }
    off = (int)(address - INTC_BASE[i] - 0x10u);
    for (k = 0; k < size && off + k < 8; k++)
        b[off + k] = (uint8_t)((uint64_t)value >> (8 * (size - 1 - k)));
    hi = be32(b); lo = be32(b + 4);
    f->dirty = 1;
    m = f->inflight[i];
    for (src = 0; src < 64; src++) {
        if (!((m >> src) & 1u))
            continue;
        if (!(src >= 32 ? (hi >> (src - 32)) & 1u : (lo >> src) & 1u))
            f->inflight[i] &= ~(1ull << src);
    }
}

/* ForcedIrqs._on_rte (via dt2_gate cfn, sugli RTE dell'immagine) */
void dt2_frc_on_rte(void *uc, uint64_t address, uint32_t size, void *data)
{
    frc_state *f = (frc_state *)data;
    irq_ctx *q = f->q;
    uint32_t pc = 0, sp = 0;
    uint8_t b[2];
    int a, lvl, vec, r;
    (void)size;
    a = frc_asserted(uc, f);
    if (a <= 0)
        return;
    if (q->exc->reg_read(uc, q->exc->r_pc, &pc) || pc != (uint32_t)address)
        return;
    if (q->exc->reg_read(uc, q->exc->r_a7, &sp) || q->exc->mem_read(uc, sp + 2u, b, 2)) {
        q->err = 23;
        return;
    }
    lvl = f->best_lvl; vec = f->best_vec;
    if (lvl <= (int)((rbe16(b) >> 8) & 7u))
        return;
    r = irq_raise(uc, q, vec, lvl);
    if (r == 1)
        frc_took(f, vec, 1);
}

/* ---- il ciclo ---- */
enum { SPIN_LIMIT = 0, SPIN_PC_ZERO = 1, SPIN_STOPPED = 2, SPIN_UC_ERROR = 3, SPIN_FLUSH = 4 };
enum { EV_SSI = 1, EV_FRC = 2, EV_PY = 3 };
#define SPIN_MAXEV 6

typedef struct {
    emu_start_fn emu_start;
    irq_ctx *q;
    step_state *stp;                    /* lo stepper nativo: {left, blocks} */
    double per_block;
    int32_t ntim, nev;
    tim_state *tim[4];                  /* Timers.sources, in ordine */
    int32_t ev[SPIN_MAXEV];             /* async_events, in ordine: EV_* */
    sst_state *sst;
    frc_state *frc;
    /* Eventi Python (MidiPort; sul Digitakt II anche Dspi1 e Transfer): il C
       chiama step/service solo se hint (o hint2, es. MidiPort.tx.pending) dice
       che possono avere qualcosa da fare; altrimenti step e' None e service
       non fa nulla. */
    const int32_t *hint[SPIN_MAXEV];
    const int32_t *hint2[SPIN_MAXEV];
    py_step_fn pstep[SPIN_MAXEV];
    py_service_fn pservice[SPIN_MAXEV];
    const int32_t *stop_flag;           /* 1: un hook Python ha chiamato uc.emu_stop() */
    int64_t base, done, instrs, now, executed, steps, tx_flush;
    uint32_t pc;
    int32_t uc_err, err, timing;
    int64_t emu_ns;                     /* timing != 0: tempo dentro emu_start (diagnostica) */
} spin_state;

/* orologio monotono per `timing` (scratch/mcsplitplay.py) */
#ifdef _WIN32
#include <windows.h>
static int64_t spin_clock_ns(void)
{
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (int64_t)((double)c.QuadPart * 1e9 / (double)f.QuadPart);
}
#else
#include <time.h>
static int64_t spin_clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}
#endif

/* CPython float_floor_div (Objects/floatobject.c, _float_div_mod): `a // b`
   tra float, esattamente come lo fa Python (non e' floor(a / b)). */
static double py_floordiv(double vx, double wx)
{
    double mod = fmod(vx, wx), div = (vx - mod) / wx, floordiv;
    if (mod) {
        if ((wx < 0) != (mod < 0)) { mod += wx; div -= 1.0; }
    }
    if (div) {
        floordiv = floor(div);
        if (div - floordiv > 0.5)
            floordiv += 1.0;
    } else
        floordiv = copysign(0.0, vx / wx);
    return floordiv;
}

/* NativeStepper.run: left = max(1, -int(-step // per_block)) */
int64_t dt2_spin_left(int64_t step, double per_block)
{
    int64_t left = -(int64_t)py_floordiv(-(double)step, per_block);
    return left < 1 ? 1 : left;
}

static int py_busy(spin_state *S, int i)
{
    return S->hint[i] && (*S->hint[i] || (S->hint2[i] && *S->hint2[i]));
}

/* spin: `for event in async_events: event.service(); pits.service()` */
static int spin_services(void *uc, spin_state *S, int64_t now)
{
    int i;
    for (i = 0; i < S->nev; i++) {
        switch (S->ev[i]) {
        case EV_SSI:
            if (dt2_sst_service_full(uc, S->sst, now) < 0) return -1;
            break;
        case EV_FRC:
            if (dt2_frc_service(uc, S->frc, now) < 0) return -1;
            break;
        case EV_PY:
            if (py_busy(S, i) && S->pservice[i](now) < 0) { S->err = 40; return -1; }
            break;
        }
    }
    for (i = 0; i < S->ntim; i++)
        if (dt2_tim_service_full(uc, S->tim[i], now) < 0) { S->err = 41; return -1; }
    return 0;
}

static int spin_hook_error(spin_state *S)
{
    return S->q->err || (S->sst && S->sst->err) || (S->frc && S->frc->err);
}

/* I servizi di un passo gia' eseguito (S->done aggiornato da chi chiama),
   poi il PC: la coda di un'iterazione che il Python ha ripreso (SPIN_STOPPED). */
int dt2_spin_service(void *uc, spin_state *S)
{
    S->now = S->base + S->done;
    if (spin_services(uc, S, S->now) < 0)
        return -1;
    if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &S->pc)) { S->err = 42; return -1; }
    return 0;
}

/* longrun.spin(m, pc, instrs, pits=timers, fast=True, async_events=...):
   lo stesso ciclo, gli stessi passi. Esce con SPIN_* (o -1, errore in
   S->err / q->err / sst->err / frc->err / tim->err). */
int dt2_spin(void *uc, spin_state *S)
{
    uint32_t pc = S->pc;
    while (S->done < S->instrs) {
        int64_t now, step = -1, r;
        int i, err;
        if (pc == 0) { S->pc = 0; return SPIN_PC_ZERO; }
        if (S->sst && S->sst->txbuf && S->sst->txlen > S->tx_flush) { S->pc = pc; return SPIN_FLUSH; }
        now = S->base + S->done;
        S->now = now;
        /* steps = [pits.step(now)] + [e.step(now) for e in async_events if not None] */
        for (i = 0; i < S->ntim; i++) {
            r = dt2_tim_step(uc, S->tim[i], now, -1);
            if (r < 0) { S->err = 43; return -1; }
            if (step < 0 || r < step) step = r;
        }
        for (i = 0; i < S->nev; i++) {
            r = -1;
            switch (S->ev[i]) {
            case EV_SSI:
                r = dt2_sst_step(uc, S->sst, now, -1);
                if (S->sst->err) return -1;
                if (!S->sst->en) r = -1;
                break;
            case EV_FRC:
                r = dt2_frc_step(uc, S->frc);
                if (r == -2) return -1;
                break;
            case EV_PY:
                if (py_busy(S, i)) {
                    r = S->pstep[i](now);
                    if (r == -2) { S->err = 40; return -1; }
                }
                break;
            }
            if (r >= 0 && r < step) step = r;
        }
        /* NativeStepper.run(pc, step) */
        S->stp->blocks = 0;
        S->stp->left = dt2_spin_left(step, S->per_block);
        if (S->timing) {
            int64_t t0 = spin_clock_ns();
            err = S->emu_start(uc, pc, 0, 0, 0);
            S->emu_ns += spin_clock_ns() - t0;
        } else
            err = S->emu_start(uc, pc, 0, 0, 0);
        S->steps++;
        S->executed = (int64_t)((double)S->stp->blocks * S->per_block);
        if (S->executed < 1) S->executed = 1;
        if (err) { S->uc_err = err; S->pc = pc; return SPIN_UC_ERROR; }
        if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &pc)) { S->err = 42; return -1; }
        if (*S->stop_flag) { S->pc = pc; return SPIN_STOPPED; }
        if (spin_hook_error(S)) { S->pc = pc; return -1; }
        if (pc == 0) { S->pc = 0; return SPIN_PC_ZERO; }
        S->done += S->executed;
        now = S->base + S->done;
        S->now = now;
        if (spin_services(uc, S, now) < 0) { S->pc = pc; return -1; }
        if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &pc)) { S->err = 42; return -1; }
    }
    S->pc = pc;
    return SPIN_LIMIT;
}
