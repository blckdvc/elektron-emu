/* dn_engine.c, parte 3b: le scadenze dei timer in Fraction e il ciclo dei passi di
 * longrun.spin quando il C nativo non lo fa (emu/nativespin.py torna al Python): Dtims
 * trattenuti (l'intro, ARM_STEP) e scadenze in Fraction (dopo un cambio di ips, fino al
 * primo tick di ogni canale). Dal Digitakt II (e_exact.inc.c), adattato ai nomi del C
 * nativo del Digitone (dt2_ssi_*).
 *
 * Il Python: Pits.ips / Dtims.ips (setter) riarmano ogni scadenza con
 *     g = Fraction(now) + (Fraction(f) - Fraction(now)) * Fraction(value, old)
 * e da li' in poi i confronti sono esatti, `ceil(d - done)` e' esatto, e il
 * primo tick fa `next += p` (Fraction + float = float(next) + p): la scadenza
 * torna float. Qui la Fraction e' una coppia di interi a 128 bit ridotta.
 * Serve al boot freddo (l'intro, il cambio di ips al passaggio all'OS). */

typedef __int128 e_i128;

static e_i128 i128_gcd(e_i128 a, e_i128 b)
{
    if (a < 0)
        a = -a;
    if (b < 0)
        b = -b;
    while (b) {
        e_i128 t = a % b;
        a = b;
        b = t;
    }
    return a;
}

/* Fraction(x) di un float finito: n / d esatto, d potenza di 2 */
static void dbl_rat(double x, e_i128 *n, e_i128 *d)
{
    int ex;
    double m = frexp(x, &ex);
    int64_t mi = (int64_t)ldexp(m, 53);
    ex -= 53;
    if (!mi) {
        *n = 0;
        *d = 1;
        return;
    }
    while (ex < 0 && !(mi & 1)) {
        mi /= 2;
        ex++;
    }
    if (ex >= 0) {
        *n = (e_i128)mi << ex;
        *d = 1;
    } else {
        *n = mi;
        *d = (e_i128)1 << (-ex);
    }
}

static int i128_bits(e_i128 v)
{
    int b = 0;
    if (v < 0)
        v = -v;
    while (v) {
        v >>= 1;
        b++;
    }
    return b;
}

/* float(Fraction(n, d)) di CPython (n / d fra interi, arrotondato
   correttamente, meta' al pari), per n >= 0, d > 0 */
static double rat_to_double(e_i128 n, e_i128 d)
{
    int s, extra;
    e_i128 q, r, low, half;
    if (!n)
        return 0.0;
    s = 55 - (i128_bits(n) - i128_bits(d));
    if (s >= 0) {
        n <<= s;
    } else {
        d <<= -s;
    }
    q = n / d;
    r = n % d;
    extra = i128_bits(q) - 53;
    low = q & (((e_i128)1 << extra) - 1);
    half = (e_i128)1 << (extra - 1);
    q >>= extra;
    if (low > half || (low == half && (r != 0 || (q & 1))))
        q++;
    return ldexp((double)(int64_t)q, extra - s);
}

/* confronto esatto fra a = an/ad e b = bn/bd (numeratori >= 0, ad, bd > 0):
   frazioni continue, senza prodotti che traboccano */
static int rat_cmp(e_i128 an, e_i128 ad, e_i128 bn, e_i128 bd)
{
    int sign = 1;
    for (;;) {
        e_i128 qa = an / ad, qb = bn / bd, ra, rb, t;
        if (qa != qb)
            return qa < qb ? -sign : sign;
        ra = an % ad;
        rb = bn % bd;
        if (!ra && !rb)
            return 0;
        if (!ra)
            return -sign;
        if (!rb)
            return sign;
        /* ra/ad < rb/bd  <=>  bd/rb < ad/ra */
        t = ad;
        an = bd;
        ad = rb;
        bn = t;
        bd = ra;
        sign = -sign;
    }
}

/* la scadenza del canale c come razionale esatto */
static void tsrc_rat(e_tsrc *s, int c, e_i128 *n, e_i128 *d)
{
    if (s->frac[c]) {
        *n = s->fn[c];
        *d = s->fd[c];
    } else
        dbl_rat(s->t.next[c], n, d);
}

static int tsrc_any_frac(e_tsrc *s)
{
    return s->frac[0] | s->frac[1] | s->frac[2] | s->frac[3];
}

/* Pits.ips / Dtims.ips setter: il riarmo esatto (anche con lo stesso valore
   la scadenza diventa una Fraction, e il Python passa al suo ciclo) */
static int tsrc_set_ips(dt_engine *e, e_tsrc *s, int64_t value)
{
    int64_t old = s->ips;
    int c;
    (void)e;
    s->ips = value;
    s->t.ips = (double)value;
    if (!old)
        return 0;
    for (c = 0; c < 4; c++) {
        e_i128 fn, fd, now = s->now, n, d, g;
        if (!s->t.has[c])
            continue;
        tsrc_rat(s, c, &fn, &fd);
        n = now * fd * old + (fn - now * fd) * value;
        d = fd * old;
        g = i128_gcd(n, d);
        if (g > 1) {
            n /= g;
            d /= g;
        }
        s->frac[c] = 1;
        s->fn[c] = n;
        s->fd[c] = d;
        s->t.next[c] = rat_to_double(n, d);          /* solo informativo */
    }
    return 0;
}

#define E_IDLE_STEP 1000000
#define E_ARM_STEP 256

/* Pits.step / Dtims.step (remaining=None), il caso Python e quello nativo */
static int64_t ex_tim_step(void *uc, e_tsrc *s, int64_t done)
{
    tim_state *t = &s->t;
    int i, c, r, best = -1;
    int64_t n;
    double p;
    if (s->kind == 1 && t->held && s->arm) {
        n = E_ARM_STEP;                              /* deadline() None; min(IDLE_STEP, ARM_STEP) */
    } else {
        if (!t->held) {
            for (i = 0; i < t->nch; i++) {
                c = t->ch[i];
                if (s->kind == 1)
                    s->arm &= ~(1u << c);           /* self.arm.discard(ch) */
                r = tim_period(uc, t, c, &p);
                if (r < 0)
                    return -1;
                if (r == 0) {
                    t->has[c] = 0;
                    s->frac[c] = 0;
                    continue;
                }
                if (!t->has[c]) {
                    t->next[c] = (double)done + p;
                    t->has[c] = 1;
                    s->frac[c] = 0;
                }
                if (best < 0)
                    best = c;
                else if (!s->frac[c] && !s->frac[best]) {
                    if (t->next[c] < t->next[best])
                        best = c;
                } else {
                    e_i128 an, ad, bn, bd;
                    tsrc_rat(s, c, &an, &ad);
                    tsrc_rat(s, best, &bn, &bd);
                    if (rat_cmp(an, ad, bn, bd) < 0)
                        best = c;
                }
            }
        }
        if (best < 0)
            n = E_IDLE_STEP;
        else if (!s->frac[best]) {
            double d = ceil(t->next[best] - (double)done);
            n = (int64_t)d;
        } else {
            e_i128 num = s->fn[best] - (e_i128)done * s->fd[best], q = num / s->fd[best];
            if (num % s->fd[best] != 0 && num > 0)
                q++;                                 /* math.ceil di una Fraction */
            n = (int64_t)q;
        }
        if (n < 1)
            n = 1;
    }
    if (t->level_irq && (t->pend[0] | t->pend[1] | t->pend[2] | t->pend[3]) && n > 2000)
        n = 2000;
    return n < 1 ? 1 : n;
}

/* Pits.service / Dtims.service: dt2_tim_service_full con le Fraction */
static int ex_tim_service(void *uc, e_tsrc *s, int64_t done)
{
    tim_state *t = &s->t;
    double p, d = (double)done;
    int i, c, r;
    if (t->held)
        return 0;
    for (i = 0; i < t->nch; i++) {
        c = t->ch[i];
        r = tim_period(uc, t, c, &p);
        if (r < 0) return -1;
        if (r == 0) { t->has[c] = 0; s->frac[c] = 0; continue; }
        if (!t->has[c]) { t->next[c] = d + p; t->has[c] = 1; s->frac[c] = 0; continue; }
        if (s->frac[c] ? rat_cmp((e_i128)done, 1, s->fn[c], s->fd[c]) < 0 : d < t->next[c]) {
            if (t->level_irq && t->pend[c] && tim_try(uc, t, c) < 0) return -1;
            continue;
        }
        if (s->frac[c]) {
            t->next[c] = rat_to_double(s->fn[c], s->fd[c]) + p;      /* Fraction + float */
            s->frac[c] = 0;
        } else
            t->next[c] += p;
        if (t->next[c] <= d)
            t->next[c] = d + p;
        if (t->kind == 1) {
            uint8_t b;
            if (t->store[c]) t->store[c][3] |= 0x02;
            else {
                if (hread(uc, t->q->exc, t->base[c] + 3u, &b, 1)) { t->err = 2; return -1; }
                b |= 0x02;
                if (hwrite(uc, t->q->exc, t->base[c] + 3u, &b, 1)) { t->err = 2; return -1; }
            }
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

/* il ciclo Python di longrun.spin serve? (NativeSpin.spin -> None) */
static int ex_needed(dt_engine *e)
{
    int k;
    for (k = 0; k < e->ntim; k++)
        if (tsrc_any_frac(&e->tim[k]) || (e->tim[k].kind == 1 && e->tim[k].t.held))
            return 1;
    return 0;
}

/* spin_services con i timer esatti */
static int ex_services(dt_engine *e, void *uc, spin_state *S, int64_t now)
{
    int i;
    for (i = 0; i < S->nev; i++) {
        switch (S->ev[i]) {
        case EV_SSI:
            if (dt2_ssi_service_full(uc, S->ssi, now) < 0) return -1;
            break;
        case EV_FRC:
            if (dt2_frc_service(uc, S->frc, now) < 0) return -1;
            break;
        case EV_PY:
            if (py_busy(S, i) && S->py_service[i](now) < 0) { S->err = 40; return -1; }
            break;
        }
    }
    for (i = 0; i < e->ntim; i++)
        if (ex_tim_service(uc, &e->tim[i], now) < 0) { S->err = 41; return -1; }
    return 0;
}

static int ex_spin_service(dt_engine *e, void *uc, spin_state *S)
{
    S->now = S->base + S->done;
    if (ex_services(e, uc, S, S->now) < 0)
        return -1;
    if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &S->pc)) { S->err = 42; return -1; }
    return 0;
}

/* dt2_spin con ex_tim_step / ex_tim_service: stessi passi del ciclo Python */
static int ex_spin(dt_engine *e, void *uc, spin_state *S)
{
    uint32_t pc = S->pc;
    while (S->done < S->instrs) {
        int64_t now, step = -1, r;
        int i, err;
        if (pc == 0) { S->pc = 0; return SPIN_PC_ZERO; }
        now = S->base + S->done;
        S->now = now;
        for (i = 0; i < e->ntim; i++) {
            r = ex_tim_step(uc, &e->tim[i], now);
            if (r < 0) { S->err = 43; return -1; }
            if (step < 0 || r < step) step = r;
        }
        for (i = 0; i < S->nev; i++) {
            r = -1;
            switch (S->ev[i]) {
            case EV_SSI:
                r = dt2_ssi_step(uc, S->ssi, now, -1);
                if (S->ssi->err) return -1;
                if (!S->ssi->en) r = -1;
                break;
            case EV_FRC:
                r = dt2_frc_step(uc, S->frc);
                if (r == -2) return -1;
                break;
            case EV_PY:
                if (py_busy(S, i)) {
                    r = S->py_step[i](now);
                    if (r == -2) { S->err = 40; return -1; }
                }
                break;
            }
            if (r >= 0 && r < step) step = r;
        }
        S->stp->blocks = 0;
        S->stp->left = dt2_spin_left(step, S->per_block);
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
        if (ex_services(e, uc, S, now) < 0) { S->pc = pc; return -1; }
        if (S->q->exc->reg_read(uc, S->q->exc->r_pc, &pc)) { S->err = 42; return -1; }
    }
    S->pc = pc;
    return SPIN_LIMIT;
}

/* emu/pit.py intro_running: il vettore 208 punta ancora all'ISR dell'intro
   e PIT3 e' acceso */
static int e_intro_running(dt_engine *e)
{
    uint8_t b[4];
    if (!e->p->intro_pit3_isr)
        return 0;
    if (e_read(e, 0x40000340u, b, 4))
        return 0;
    return e_be32(b) == e->p->intro_pit3_isr && (e_r16(e, PIT_BASES[3]) & 0x01);
}

/* GUI: handover all'uscita dell'intro (Emulator.run) */
static void h_handover(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    int k;
    (void)uc; (void)a; (void)s;
    for (k = 0; k < e->ntim; k++)
        e->tim[k].t.held = 0;                   /* pits.release() */
    e->use_panel = 1;
    if (e->post_intro_ips) {                    /* _pending_ips.append((instrs, ips)) */
        e->has_pending_ips = 1;
        e->pending_ips_at = e->instrs;
        e->pending_ips_n = e->post_intro_ips;
    }
}

/* Uno stato senza il componente Ssi0Dma: l'aggiornamento di emu/gui.py. 1 = "legacy"
   (Ssi0Dma.arm_legacy: i TCD 48/50 gia' programmati dal firmware con la forma attesa, acceso),
   2 = "cold" (arm_cold: TCD vuoti, spento, i canali li accendono le SERQ del firmware),
   0 = ne' l'uno ne' l'altro (errore) */
static int dn_ssi_upgrade(dt_engine *e)
{
    uint8_t tcd[2][0x20];
    int k, legacy = 1, b;
    if (e_read(e, 0xFC045000u + 48u * 0x20u, tcd[0], 0x20) || e_read(e, 0xFC045000u + 50u * 0x20u, tcd[1], 0x20))
        legacy = 0;
    for (k = 0; k < 2 && legacy; k++) {
        const uint8_t *t = tcd[k];
        uint32_t periph = k ? e_be32(t + 16) : e_be32(t);
        uint16_t attr = (uint16_t)(t[4] << 8 | t[5]), csr = (uint16_t)(t[30] << 8 | t[31]);
        int16_t soff = (int16_t)(t[6] << 8 | t[7]), doff = (int16_t)(t[22] << 8 | t[23]);
        uint32_t nbytes = e_be32(t + 8);
        uint16_t citer = (uint16_t)(t[20] << 8 | t[21]), biter = (uint16_t)(t[28] << 8 | t[29]);
        if (k == 0 ? !(periph == 0xFC0BC008u && attr == 0x0202 && soff == 0 && nbytes == 0x20
                       && doff == 4 && (csr & 0x12) == 0x10)
                   : !(periph == 0xFC0BC000u && attr == 0x0202 && soff == 4 && nbytes == 0x20
                       && doff == 0 && (csr & 0x12) == 0x12))
            legacy = 0;
        if (!citer || !biter)
            legacy = 0;
    }
    if (legacy) {
        e->sst.en = 3;                         /* enabled.update((48, 50)) */
        e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
        e->sst.has_next = 1;
        return 1;
    }
    for (k = 0; k < 2; k++)
        for (b = 0; b < 0x20; b++)
            if (tcd[k][b])
                return 0;
    e->sst.en = 0;
    e->sst.has_next = 0;
    return 2;
}
