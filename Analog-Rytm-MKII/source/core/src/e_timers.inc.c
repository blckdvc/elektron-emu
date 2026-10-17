/* engine.c, parte 3: Pits/Dtims (emu/pit.py, emu/dtim.py) intorno a
 * tim_state, l'orologio DTCN del Rytm (dtim.install_counters: la lettura in C,
 * native/dt2_hooks.c dt2_dtcn_on_read con `origin` condiviso; la scrittura
 * qui), Dtims.arm, il riarmo di DTIM3 (rearm_cleared), le callback del C
 * nativo (py_raise, il ripiego dell'INTR). */

static const uint32_t PIT_BASES[4] = {0xFC080000u, 0xFC084000u, 0xFC088000u, 0xFC08C000u};
static const uint32_t DTIM_BASES[4] = {0xFC070000u, 0xFC074000u, 0xFC078000u, 0xFC07C000u};
#define DTMR_OFF 0x00
#define DTRR_OFF 0x04
#define DTCN_OFF 0x0C
#define DTCN_RST 0x01
#define BOOT_DTMR0 0x0003
#define BOOT_DTMR2 0x0007

static void tsrc_init(dt_engine *e, e_tsrc *s, int kind, int nch, const uint32_t *ch, int64_t ips,
                      int held)
{
    int i;
    memset(s, 0, sizeof *s);
    s->kind = kind;
    s->ips = ips;
    s->t.mem_read = (mem_rw_fn)uc_mem_read;
    s->t.kind = kind;
    s->t.nch = nch;
    s->t.held = held;
    s->t.level_irq = 1;                      /* configure_rytm_audio: _LEVEL_IRQ = True */
    for (i = 0; i < nch; i++)
        s->t.ch[i] = (int32_t)ch[i];
    for (i = 0; i < 4; i++) {
        s->t.base[i] = kind ? DTIM_BASES[i] : PIT_BASES[i];
        s->t.store[i] = NULL;
    }
    s->t.ips = (double)ips;
    s->t.q = &e->q;
    s->t.vec0 = kind ? 96 : 205;
    s->t.pend_cap = 60000;                   /* LEVEL_RETRY_RTE: gli hook RTE di intfrc ci sono */
}

/* Pits/Dtims.ips = n (Emulator.run, _pending_ips): lo stesso valore lascia le
   scadenze come sono (il Python le porta in Fraction esatte e le stesse
   istanze; il primo tick le riporta a float); un valore diverso con un timer
   armato non e' portato (non succede: lo stato e' gia' a 60M). */
static int tsrc_set_ips(dt_engine *e, e_tsrc *s, int64_t value)
{
    int c;
    if (value == s->ips)
        return 0;
    for (c = 0; c < 4; c++)
        if (s->t.has[c]) {
            dt_errf(e->err, sizeof e->err, "ips change %lld -> %lld with an armed timer: "
                    "not supported (make the state with tools/arcore_ref.py --save)",
                    (long long)s->ips, (long long)value);
            return -1;
        }
    s->ips = value;
    s->t.ips = (double)value;
    return 0;
}

/* ------------------------------------------------------------ DTCN */
/* clock() di install_counters: l'istante del passo (S.now durante la chiamata
   C, altrimenti Dtims.now), piu' i blocchi gia' eseguiti dentro emu_start */
static int64_t dtcn_now(dt_engine *e)
{
    int64_t now = e->running ? e->S.now : e->tim[1].now;
    if (e->running)
        now += (int64_t)((double)e->stp.blocks * e->S.per_block);
    return now;
}

/* install_counters, la parte all'installazione: DTIM0 e DTIM2 del bootstrap;
   gli altri canali ricavano `origin` alla prima lettura (dt2_dtcn_on_read) */
static void dtcn_install(dt_engine *e)
{
    int ch;
    dtcn_state *d = &e->dtcn;
    memset(d, 0, sizeof *d);
    d->mem_read = (mem_rw_fn)uc_mem_read;
    d->mem_write = (mem_rw_fn)(void *)uc_mem_write;
    d->stp = &e->stp;
    for (ch = 0; ch < 4; ch++) {
        e->dtcn_sites[ch].s = d;
        e->dtcn_sites[ch].ch = ch;
    }
    if (e_r16(e, DTIM_BASES[0] + DTMR_OFF) == 0) {
        e_w16(e, DTIM_BASES[0] + DTMR_OFF, BOOT_DTMR0);
        d->origin[0] = 0;
        d->has_origin[0] = 1;
    }
    if (e_r16(e, DTIM_BASES[2] + DTMR_OFF) == 0)
        e_w16(e, DTIM_BASES[2] + DTMR_OFF, BOOT_DTMR2);
}

/* install_counters on_write: una scrittura di DTCN azzera; l'uscita dal reset
   (o un canale senza origine) riparte dall'istante attuale */
static void h_dtcn_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dtcn_site *t = (dtcn_site *)ud;
    dt_engine *e = e_cur;
    dtcn_state *d = t->s;
    int ch = t->ch;
    uint32_t b = DTIM_BASES[ch];
    (void)uc; (void)ty;
    if (a == b + DTCN_OFF) {
        d->origin[ch] = dtcn_now(e);
        d->has_origin[ch] = 1;
    } else if (a == b + DTMR_OFF && size >= 2) {
        int was = e_r16(e, b + DTMR_OFF) & DTCN_RST;
        if (((v & DTCN_RST) && !was) || !d->has_origin[ch]) {
            d->origin[ch] = dtcn_now(e);
            d->has_origin[ch] = 1;
        }
    }
}

/* fuori dal ciclo C la lettura di DTCN e' quella Python (py_read): qui non
   succede (il motore esegue solo dentro dt2_spin), ma il C nativo la chiede */
static void dtcn_py_read(void *uc, int type, uint64_t address, int size, int64_t value)
{
    (void)uc; (void)type; (void)address; (void)size; (void)value;
}

/* Dtims.__init__: l'hook `arm` (Dtims.arm.add(ch)) sui registri del canale 3 */
static void h_dtim_arm(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e->dtim_arm |= 1u << 3;
}

/* dtim.rearm_cleared(m, (3,)): DTIM3 azzerato da un vecchio clear_stale */
static void dtim_rearm_cleared(dt_engine *e)
{
    uint8_t b[4];
    uint16_t dtmr;
    uint32_t dtrr, vec;
    if (e_read(e, DTIM_BASES[3] + DTMR_OFF, b, 2))
        return;
    dtmr = e_be16(b);
    if (e_read(e, DTIM_BASES[3] + DTRR_OFF, b, 4))
        return;
    dtrr = e_be32(b);
    if (e_read(e, 0x40000000u + 4u * 99u, b, 4))
        return;
    vec = e_be32(b);
    if (dtmr == 0 && dtrr && 0x40000400u <= vec && vec < 0x48000000u)
        e_w16(e, DTIM_BASES[3] + DTMR_OFF, 0x001D);
}

/* ------------------------------------------------------- callback */
/* NativeSpin._py_raise: Machine.raise_vector(vec, level) */
static int py_raise_c(int32_t vec, int32_t level)
{
    return dtm_raise_vector(e_cur->m, (uint32_t)vec, 0, level < 0 ? -1 : level);
}

/* il ripiego di dt2_on_intr: Machine.install_exceptions on_intr (un'eccezione
   che non e' un RTE): raise_vector(vec, from_instruction=True), altrimenti
   halt_vec e stop */
static void intr_fallback_c(void *uc, uint32_t intno)
{
    dt_engine *e = e_cur;
    (void)uc;
    if (intno == 0x100u) {                    /* RTE con la pila illeggibile: UcError */
        e->m->cnt.halt_vec = (int32_t)intno;
        dtm_stop(e->m);
        return;
    }
    if (!dtm_raise_vector(e->m, intno, 1, -1)) {
        e->m->cnt.halt_vec = (int32_t)intno;
        dtm_stop(e->m);
    }
}

static int stop_c(void *uc)
{
    return (int)uc_emu_stop((uc_engine *)uc);
}
