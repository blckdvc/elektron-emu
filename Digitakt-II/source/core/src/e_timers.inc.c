/* engine.c, parte 3: Pits/Dtims (emu/pit.py, emu/dtim.py) intorno a
 * tim_state, l'orologio DTCN (dtim.install_counters), SSI0 e interrupt
 * forzati posseduti dal C (emu/nativespin.py _own_ssi / _own_frc). */

static const uint32_t PIT_BASES[4] = {0xFC080000u, 0xFC084000u, 0xFC088000u, 0xFC08C000u};
static const uint32_t DTIM_BASES[4] = {0xFC070000u, 0xFC074000u, 0xFC078000u, 0xFC07C000u};
#define E_F_BUS 132000000LL

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
    s->t.level_irq = 1;                      /* configure_audio: _LEVEL_IRQ = True */
    for (i = 0; i < nch; i++)
        s->t.ch[i] = (int32_t)ch[i];
    for (i = 0; i < 4; i++) {
        s->t.base[i] = kind ? DTIM_BASES[i] : PIT_BASES[i];
        s->t.store[i] = NULL;
    }
    s->t.ips = (double)ips;
    s->t.q = &e->q;
    s->t.vec0 = kind ? 96 : 205;
}

/* ------------------------------------------------------------ DTCN */
#define DTCN_RST 0x01
#define DTCN_FRR 0x08

static int64_t dtcn_now(dt_engine *e)
{
    /* clock(): l'istante del passo (S.now durante la chiamata C), piu' i
       blocchi gia' eseguiti dentro emu_start (stepper.running) */
    int64_t now = e->running ? e->S.now : e->tim[1].now;
    if (e->running)
        now += (int64_t)((double)e->stp.blocks * e->S.per_block);
    return now;
}

static int dtcn_value(dt_engine *e, int ch, uint32_t *out)
{
    uint32_t b = DTIM_BASES[ch];
    uint16_t dtmr = e_r16(e, b);
    uint32_t clk = (dtmr >> 1) & 3;
    __int128 now, div, ticks;
    if (!(dtmr & DTCN_RST) || clk == 0 || clk == 3)
        return 0;
    now = dtcn_now(e);
    div = (__int128)e->tim[1].ips * (((dtmr >> 8) & 0xFF) + 1) * (clk == 1 ? 1 : 16);
    if (!e->dtcn_has[ch]) {
        __int128 saved = e_r32(e, b + 0x0C);
        e->dtcn_origin[ch] = (int64_t)(now - e_floordiv128(saved * div, E_F_BUS));
        e->dtcn_has[ch] = 1;
    }
    ticks = e_floordiv128((now - e->dtcn_origin[ch]) * E_F_BUS, div);
    if (dtmr & DTCN_FRR) {
        uint32_t ref = e_r32(e, b + 0x04);
        if (ref) {
            __int128 m = (__int128)ref + 1;
            ticks = ticks - e_floordiv128(ticks, m) * m;
        }
    }
    *out = (uint32_t)(ticks & 0xFFFFFFFF);
    return 1;
}

typedef struct {
    dt_engine *e;
    int ch;
} e_dtcn;

static void h_dtcn_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_dtcn *d = (e_dtcn *)ud;
    uint32_t val;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    if (dtcn_value(d->e, d->ch, &val))
        e_w32(d->e, DTIM_BASES[d->ch] + 0x0C, val);
}

static void h_dtcn_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_dtcn *d = (e_dtcn *)ud;
    dt_engine *e = d->e;
    uint32_t b = DTIM_BASES[d->ch];
    (void)uc; (void)ty;
    if (a == b + 0x0C) {
        e->dtcn_origin[d->ch] = dtcn_now(e);
        e->dtcn_has[d->ch] = 1;
    } else if (a == b && size >= 2) {
        int was = e_r16(e, b) & DTCN_RST;
        if (((v & DTCN_RST) && !was) || !e->dtcn_has[d->ch]) {
            e->dtcn_origin[d->ch] = dtcn_now(e);
            e->dtcn_has[d->ch] = 1;
        }
    }
}

/* Dtims: hook `arm` (solo diagnostica quando i timer non sono trattenuti) */
static void h_dtim_arm(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_tsrc *s = (e_tsrc *)ud;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    s->arm |= 1u << 3;
}

/* ------------------------------------------------------- py_raise */
static int py_raise_c(int32_t vec, int32_t level)
{
    return dtm_raise_vector(e_cur->m, (uint32_t)vec, 0, level < 0 ? -1 : level);
}

static void intr_fallback_c(void *uc, uint32_t intno)
{
    (void)uc; (void)intno;                   /* non usato: l'INTR e' quello di machine.c */
}

static int stop_c(void *uc)
{
    return (int)uc_emu_stop((uc_engine *)uc);
}
