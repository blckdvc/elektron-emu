
/* ================= dt2_cosim (emu/cosimspin.py) ================= */
/* La linea di sincronia (DspCompanion.use_main_sync): il MAIN alza e abbassa
   il pin A4 (0xEC094018 imposta, 0xEC094024 azzera); ogni cambio e' un fronte
   da consegnare al DTIN0 del DSP. Hook di scrittura, prima della scrittura. */
typedef struct {
    int32_t level, pad;
    int64_t edges;
} sync_state;

/* a write hook that sets a bit in an int (emu/dtim.py Dtims.arm) */
typedef struct { int32_t *bits; int32_t bit, pad; } setbit_state;

void dt2_set_bit(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    setbit_state *s = (setbit_state *)user_data;
    (void)uc; (void)type; (void)address; (void)size; (void)value;
    *s->bits |= s->bit;
}

void dt2_sync_set(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    sync_state *s = (sync_state *)user_data;
    (void)uc; (void)type; (void)address; (void)size;
    if ((value & 0x10) && !s->level) { s->level = 0x10; s->edges++; }
}

void dt2_sync_clr(void *uc, int type, uint64_t address, int size, int64_t value, void *user_data)
{
    sync_state *s = (sync_state *)user_data;
    (void)uc; (void)type; (void)address; (void)size;
    if (!(value & 0x10) && s->level) { s->level = 0; s->edges++; }
}

enum { CZ_DONE = 0, CZ_MAIN = 1, CZ_DSP = 2, CZ_LINK = 3 };
enum { PH_MAIN = 0, PH_MAIN_DONE = 1, PH_DSP_CHUNK = 2, PH_LINK = 3, PH_TAIL = 4 };

typedef struct {
    void *ucm, *ucd;                    /* MAIN e DSP */
    spin_state *Sm, *Sd;                /* i loro cicli dei passi (NativeSpin.S) */
    sync_state *sync;
    const int32_t *link_complete;       /* SpiLink.complete */
    int32_t *park;                      /* DSP parcheggiato (hook di dspcpu.prepare_timed) */
    int32_t *dstop;                     /* stop_flag del NativeSpin del DSP */
    int64_t slice, budget, executed;
    int64_t mnow, dnow;                 /* Timers.now del MAIN e del DSP */
    uint32_t pc, dpc, audio_idle, pad0;
    int32_t phase, stop, shortcut, moved;
    int32_t mcode, dcode, err, pad1;
    int64_t d_n, d_done, d_chunk, d_t0;
    int64_t dtin0_edges, last_edges, irq4, idle_skipped;
    int64_t slices, dsp_steps, shortcuts, parks;    /* diagnostica */
} cz_state;

/* il fronte di dspcpu.prepare_timed: edge() */
static int cz_edge(cz_state *Z)
{
    void *uc = Z->ucd;
    irq_ctx *q = Z->Sd->q;
    uint8_t b[2], c;
    int r;
    if (q->exc->mem_read(uc, 0xFC070000u, b, 2)) { Z->err = 50; return -1; }
    if (!(rbe16(b) & 0x00C0))
        return 0;
    if (q->exc->mem_read(uc, 0xFC070003u, &c, 1)) { Z->err = 50; return -1; }
    c |= 0x01;
    if (q->exc->mem_write(uc, 0xFC070003u, &c, 1)) { Z->err = 50; return -1; }
    r = irq_raise(uc, q, 96, -1);
    if (r < 0) { Z->err = 51; return -1; }
    if (r) Z->dtin0_edges++;
    return 0;
}

/* DspCompanion.deliver_irq4 */
static int cz_irq4(cz_state *Z)
{
    void *uc = Z->ucm;
    irq_ctx *q = Z->Sm->q;
    uint8_t b;
    int r;
    if (q->exc->mem_read(uc, 0xFC090003u, &b, 1)) { Z->err = 52; return -1; }
    if (!(b & 0x10))
        return 0;
    if (q->exc->mem_read(uc, 0xFC090006u, &b, 1)) { Z->err = 52; return -1; }
    b |= 0x10;
    if (q->exc->mem_write(uc, 0xFC090006u, &b, 1)) { Z->err = 52; return -1; }
    r = irq_raise(uc, q, 68, -1);
    if (r < 0) { Z->err = 53; return -1; }
    if (r) Z->irq4++;
    return 0;
}

/* il Timers.step del DSP: min(s.step(now, n) for s in sources) */
static int cz_dsp_tim_step(cz_state *Z, int64_t now, int64_t n, int64_t *out)
{
    int i;
    int64_t r, best = -1;
    for (i = 0; i < Z->Sd->ntim; i++) {
        r = dt2_tim_step(Z->ucd, Z->Sd->tim[i], now, n);
        if (r < 0) { Z->err = 54; return -1; }
        if (best < 0 || r < best) best = r;
    }
    *out = best;
    return 0;
}

/* il Timers.service del DSP */
static int cz_dsp_tim_service(cz_state *Z, int64_t now)
{
    int i;
    for (i = 0; i < Z->Sd->ntim; i++)
        if (dt2_tim_service_full(Z->ucd, Z->Sd->tim[i], now) < 0) { Z->err = 55; return -1; }
    return 0;
}

static void cz_reset(spin_state *S)
{
    S->err = 0; S->uc_err = 0; S->q->err = 0;
}

/* la coda di step_timed dopo un parcheggio: il resto della fetta accreditato,
   i timer serviti alla sua fine (anche dal Python, emu/cosimspin.py) */
int dt2_cz_park(cz_state *Z)
{
    Z->dnow = Z->d_t0 + Z->d_chunk;
    if (cz_dsp_tim_service(Z, Z->dnow) < 0)
        return -1;
    Z->idle_skipped++;
    Z->parks++;
    return 0;
}

/* Il ciclo del motore della GUI (emu/gui.py) con le due CPU:
       while executed < budget and stop == 'limit':
           pc, ex, stop = spin(main, pc, slice, ...)       # dt2_spin
           executed += ex
           if ex and dsp.step(ex): pc = main PC             # DspCompanion.step
   con DspCompanion.step e dspcpu.step_timed (sincronia esterna) riga per
   riga. Esce con CZ_DONE, o verso il Python per cio' che il C non fa:
   CZ_MAIN (la fetta del MAIN e' uscita da dt2_spin per un motivo diverso dal
   budget), CZ_DSP (idem per il DSP, ma non per il parcheggio), CZ_LINK (un
   trasferimento SPI concluso: SpiLink.deliver). Il Python sistema e
   richiama: la fase dice da dove si riprende. -1: errore (Z->err, ...). */
int dt2_cosim(cz_state *Z)
{
    spin_state *Sm = Z->Sm, *Sd = Z->Sd;
    exc_state *em = Sm->q->exc, *ed = Sd->q->exc;
    int c;
    int64_t ex, r, d, chunk;
    for (;;) {
        switch (Z->phase) {
        case PH_MAIN:
            if (Z->executed >= Z->budget || Z->stop)
                return CZ_DONE;
            Sm->base = Z->mnow; Sm->done = 0; Sm->instrs = Z->slice; Sm->pc = Z->pc;
            cz_reset(Sm);
            Z->slices++;
            c = dt2_spin(Z->ucm, Sm);
            if (c != SPIN_LIMIT) { Z->mcode = c; return CZ_MAIN; }
            Z->phase = PH_MAIN_DONE;
            /* fall through */
        case PH_MAIN_DONE:
            ex = Sm->done;
            Z->mnow = Sm->base + Sm->done;
            Z->pc = Sm->pc;
            Z->executed += ex;
            Z->phase = PH_MAIN;
            if (!ex)
                continue;
            Z->dsp_steps++;
            Z->shortcut = 0;
            Z->moved = 0;
            /* DspCompanion.step(ex): parcheggiato nel loop audio, nessun fronte
               e nessun timer in scadenza: si accredita la fetta (stessa
               valutazione pigra del Python: lo step dei timer ha effetti) */
            if (!Z->sync->edges && Z->dpc == Z->audio_idle) {
                if (cz_dsp_tim_step(Z, Z->dnow, ex, &r) < 0) return -1;
                if (r >= ex) {
                    Z->dnow += ex;
                    Z->idle_skipped++;
                    Z->shortcuts++;
                    Z->shortcut = 1;
                    Z->phase = PH_LINK;
                    continue;
                }
            }
            if (Z->sync->edges) {
                if (ed->reg_write(Z->ucd, ed->r_pc, &Z->dpc)) { Z->err = 56; return -1; }
                while (Z->sync->edges) {
                    Z->sync->edges--;
                    if (cz_edge(Z) < 0) return -1;
                }
                if (ed->reg_read(Z->ucd, ed->r_pc, &Z->dpc)) { Z->err = 56; return -1; }
            }
            Z->d_n = ex;
            Z->d_done = 0;
            Z->phase = PH_DSP_CHUNK;
            /* fall through */
        case PH_DSP_CHUNK:
            /* step_timed(n) con la sincronia esterna: every = n */
            while (Z->d_done < Z->d_n) {
                chunk = Z->d_n - Z->d_done;
                Z->d_t0 = Z->dnow;
                Z->d_chunk = chunk;
                *Z->park = 0;
                Sd->base = Z->dnow; Sd->done = 0; Sd->instrs = chunk; Sd->pc = Z->dpc;
                cz_reset(Sd);
                *Z->dstop = 0;
                c = dt2_spin(Z->ucd, Sd);
                if (c == SPIN_LIMIT) {
                    Z->dnow = Sd->base + Sd->done;
                    d = Sd->done;
                } else if (c == SPIN_STOPPED && *Z->park) {
                    if (dt2_cz_park(Z) < 0) return -1;
                    d = chunk;
                } else {
                    Z->dcode = c;
                    return CZ_DSP;
                }
                Z->d_done += d > 1 ? d : 1;
                if (ed->reg_read(Z->ucd, ed->r_pc, &Z->dpc)) { Z->err = 56; return -1; }
            }
            Z->phase = PH_LINK;
            /* fall through */
        case PH_LINK:
            Z->phase = PH_TAIL;
            if (*Z->link_complete)
                return CZ_LINK;          /* il Python: moved = link.deliver() */
            /* fall through */
        case PH_TAIL:
            if (!Z->shortcut && Z->dtin0_edges != Z->last_edges) {
                Z->last_edges = Z->dtin0_edges;
                if (cz_irq4(Z) < 0) return -1;
                Z->moved = 1;
            }
            if (Z->moved && em->reg_read(Z->ucm, em->r_pc, &Z->pc)) { Z->err = 57; return -1; }
            Z->phase = PH_MAIN;
            continue;
        default:
            Z->err = 59;
            return -1;
        }
    }
}
