
/* ================= dt2_spin (emu/nativespin.py) ================= */
typedef int (*emu_start_fn)(void *uc, uint64_t begin, uint64_t until, uint64_t timeout, size_t count);
typedef int (*py_raise_fn)(int32_t vec, int32_t level);   /* Machine.raise_vector: 1, 0, -1 error */
typedef int64_t (*py_step_fn)(int64_t now);               /* MidiPort.step: >=1, -1 None, -2 error */
typedef int (*py_service_fn)(int64_t now);                /* MidiPort.service: 0, -1 error */

/* What delivering an interrupt needs: Unicorn's functions and the state of
   dt2_raise_vector, and the Python fallback when C cannot (an unmapped stack
   page: Machine._ensure_frame maps it). */
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

/* emu/pit.py interrupt_level: the programmed level (ICR & 7), 0 = None
   (off or masked), -1 error. */
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

/* ---- Pits / Dtims: the whole service (pit.Pits.service,
   dtim.Dtims.service, with and without DT2_LEVEL_IRQ), line by line ---- */
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
        if (t->kind == 1) {              /* DTER |= REF before the interrupt */
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

/* ---- SSI: _deliver_vector170 (TX_VECTOR, 174 on the Rytm) /
   _deliver_vector191 and the three hooks (_on_cint, _on_intfrch1,
   _on_force_rte) of emu/ssi.py ---- */
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

/* Ssi0Dma.service: the requests due, then the deliveries. -> 0, -1 error */
int dt2_sst_service_full(void *uc, sst_state *s, int64_t done)
{
    if (dt2_sst_service(uc, s, done) < 0)
        return -1;
    return sst_deliver(uc, s);
}

/* Ssi0Dma._on_cint: write hook on CINT (the TX vector's handler) */
void dt2_sst_on_cint(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    sst_state *s = (sst_state *)user_data;
    (void)uc; (void)type; (void)address;
    if (size == 1 && ((value & 0x40) || (value & 0x3F) == s->tx_ch)) {
        s->int50_asserted = 0;
        s->int50_delivered = 0;
    }
}

/* Ssi0Dma._on_intfrch1: write hook on INTFRCH1 (before the write) */
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

/* Ssi0Dma._on_force_rte: at the RTE of the TX vector's handler the forced
   vector 191 comes in if the IPL the RTE restores allows it (via dt2_gate cfn). */
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

/* ---- Forced interrupts (emu/intfrc.py ForcedIrqs) in C ----
   inflight and skip: one bit per source of each INTC; of the sources
   pending only the highest (level, vector) matters, the one delivered. On
   the Rytm the RTE hook also takes the timer ticks pending (level IRQ) of
   the timer sources the loop in C holds (RTE_LISTENERS); outside the loop
   the timers are Python's, and the hook is the Python one (py_rte). */
typedef struct {
    irq_ctx *q;
    uint64_t inflight[3], skip[3];
    int32_t dirty, has_pend, best_lvl, best_vec;
    int64_t now, at_rte, retry;
    int64_t delivered[192];             /* index: vector - 64 */
    int32_t err, ntim;                  /* ntim: timer sources live in the C loop, 0 = Python's */
    tim_state *tim[4];
    dt2_gate_py py_rte;                 /* the Python RTE hook (ForcedIrqs._on_rte) */
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

/* for the Python of _OwnedForcedIrqs: -> 1 with the level and vector, 0 none, -1 error */
int dt2_frc_pending(void *uc, frc_state *f, int32_t *lvl, int32_t *vec)
{
    int a = frc_asserted(uc, f);
    if (a > 0) { *lvl = f->best_lvl; *vec = f->best_vec; }
    return a;
}

void dt2_frc_took(frc_state *f, int vec, int at_rte)
{
    int i = (vec - 64) / 64, src = vec - 64 - 64 * i;
    f->inflight[i] |= 1ull << src;
    f->dirty = 1;
    f->delivered[vec - 64]++;
    if (at_rte)
        f->at_rte++;
}

/* ForcedIrqs.step: retry if something is pending, -1 None, -2 error */
int64_t dt2_frc_step(void *uc, frc_state *f)
{
    int a = frc_asserted(uc, f);
    if (a < 0) return -2;
    return a ? f->retry : -1;
}

/* ForcedIrqs.service -> 0, -1 error */
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
    if (r) dt2_frc_took(f, vec, 0);
    return 0;
}

/* ForcedIrqs._on_write: write hook on INTFRCH/INTFRCL (before the write) */
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

/* ForcedIrqs._on_rte (via dt2_gate cfn, on the image's RTEs): the highest
   (level, vector) of the forced sources and of the timer ticks pending, a tie
   to the forced one (Python's max over [forced] + [timers]) */
void dt2_frc_on_rte(void *uc, uint64_t address, uint32_t size, void *data)
{
    frc_state *f = (frc_state *)data;
    irq_ctx *q = f->q;
    uint32_t pc = 0, sp = 0;
    uint8_t b[2];
    int a, lvl = 0, vec = 0, have, i, j, r, ti = -1, tc = -1;
    if (!f->ntim) {                     /* outside the loop in C: the timers are Python's */
        f->py_rte(uc, address, size);
        return;
    }
    a = frc_asserted(uc, f);
    if (a < 0)
        return;
    have = a;
    if (a) { lvl = f->best_lvl; vec = f->best_vec; }
    for (i = 0; i < f->ntim; i++) {                    /* Pits/Dtims.rte_candidates */
        tim_state *t = f->tim[i];
        if (t->held || !(t->pend[0] | t->pend[1] | t->pend[2] | t->pend[3]))
            continue;
        for (j = 0; j < t->nch; j++) {
            int c = t->ch[j], l, v;
            if (!t->pend[c])
                continue;
            v = t->vec0 + c;
            l = irq_level(uc, q, v, 1);
            if (l < 0) return;
            if (!l)
                continue;
            if (!have || l > lvl || (l == lvl && v > vec)) {
                lvl = l; vec = v; ti = i; tc = c; have = 1;
            }
        }
    }
    if (!have)
        return;
    if (q->exc->reg_read(uc, q->exc->r_pc, &pc) || pc != (uint32_t)address)
        return;
    if (q->exc->reg_read(uc, q->exc->r_a7, &sp) || q->exc->mem_read(uc, sp + 2u, b, 2)) {
        q->err = 23;
        return;
    }
    if (lvl <= (int)((rbe16(b) >> 8) & 7u))
        return;
    r = irq_raise(uc, q, vec, lvl);
    if (r == 1) {
        if (ti >= 0) {                                 /* rte_taken */
            f->tim[ti]->pend[tc] = 0;
            f->tim[ti]->fired[tc]++;
        } else
            dt2_frc_took(f, vec, 1);
    }
}

/* ---- the loop ---- */
enum { SPIN_LIMIT = 0, SPIN_PC_ZERO = 1, SPIN_STOPPED = 2, SPIN_UC_ERROR = 3, SPIN_FLUSH = 4 };
enum { EV_SSI = 1, EV_FRC = 2, EV_MIDI = 3, EV_NONE = 4 };

typedef struct {
    emu_start_fn emu_start;
    irq_ctx *q;
    step_state *stp;                    /* the native stepper: {left, blocks} */
    double per_block;
    int32_t ntim, nev;
    tim_state *tim[4];                  /* Timers.sources, in order */
    int32_t ev[4];                      /* async_events, in order: EV_* */
    sst_state *sst;
    frc_state *frc;
    const int32_t *midi_hint;           /* MidiPort: 1 = may have something to do */
    const int32_t *midi_txpend;         /* MidiPort.tx.pending */
    py_step_fn midi_step;
    py_service_fn midi_service;
    const int32_t *stop_flag;           /* 1: a Python hook called uc.emu_stop() */
    int64_t base, done, instrs, now, executed, steps, tx_flush;
    uint32_t pc;
    int32_t uc_err, err, timing;
    int64_t emu_ns;                     /* timing != 0: time inside emu_start (diagnostics) */
} spin_state;

/* monotonic clock for `timing` */
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
   between floats, exactly as Python does it (it is not floor(a / b)). */
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

static int midi_busy(spin_state *S)
{
    return S->midi_hint && (*S->midi_hint || (S->midi_txpend && *S->midi_txpend));
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
        case EV_MIDI:
            if (midi_busy(S) && S->midi_service(now) < 0) { S->err = 40; return -1; }
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

static int spin_flush_due(spin_state *S)
{
    sst_state *s = S->sst;
    return s && s->txbuf && (s->txlen > S->tx_flush || s->nchunk * 2 > s->chunkcap);
}

/* The services of a step already run (S->done updated by the caller), then
   the PC: the tail of an iteration the Python took over (SPIN_STOPPED). */
int dt2_spin_service(void *uc, spin_state *S)
{
    S->now = S->base + S->done;
    if (spin_services(uc, S, S->now) < 0)
        return -1;
    if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &S->pc)) { S->err = 42; return -1; }
    return 0;
}

/* longrun.spin(m, pc, instrs, pits=timers, fast=True, async_events=...):
   the same loop, the same steps. Exits with SPIN_* (or -1, error in
   S->err / q->err / sst->err / frc->err / tim->err). */
int dt2_spin(void *uc, spin_state *S)
{
    uint32_t pc = S->pc;
    while (S->done < S->instrs) {
        int64_t now, step = -1, r;
        int i, err;
        if (pc == 0) { S->pc = 0; return SPIN_PC_ZERO; }
        if (spin_flush_due(S)) { S->pc = pc; return SPIN_FLUSH; }
        now = S->base + S->done;
        S->now = now;
        /* steps = [pits.step(now)] + [e.step(now) for e in async_events if not None] */
        for (i = 0; i < S->ntim; i++) {
            r = dt2_tim_step(uc, S->tim[i], now, -1);
            if (r < 0) { S->err = 43; return -1; }
            if (step < 0 || r < step) step = r;
            S->tim[i]->armw = 0;          /* Dtims.step: arm -= channels */
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
            case EV_MIDI:
                if (midi_busy(S)) {
                    r = S->midi_step(now);
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
