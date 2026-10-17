/* engine.c, parte 3: Pits/Dtims (emu/pit.py, emu/dtim.py) intorno a
 * tim_state, l'orologio DTCN del Model:Cycles (dtim.install_counters: DTIM0 e
 * DTIM2 armati dal bootstrap, DTIM2 sul pin esterno a 12,288 MHz, i contatori
 * che ripartono dal valore salvato), il riarmo di DTIM3 (rearm_cleared). */

static const uint32_t PIT_BASES[4] = {0xFC080000u, 0xFC084000u, 0xFC088000u, 0xFC08C000u};
static const uint32_t DTIM_BASES[4] = {0xFC070000u, 0xFC074000u, 0xFC078000u, 0xFC07C000u};
#define E_F_BUS 132000000LL
#define E_EXT_CLK_HZ 12288000LL
#define DTMR_OFF 0x00
#define DTRR_OFF 0x04
#define DTCN_OFF 0x0C
#define DTCN_RST 0x01
#define DTCN_FRR 0x08
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
    s->t.level_irq = 1;                      /* mcengine.configure: DT2_LEVEL_IRQ = 1 */
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
static int64_t dtcn_now(dt_engine *e)
{
    /* clock(): l'istante del passo (S.now durante la chiamata C), piu' i
       blocchi gia' eseguiti dentro emu_start (stepper.running) */
    int64_t now = e->running ? e->S.now : e->tim[1].now;
    if (e->running)
        now += (int64_t)((double)e->stp.blocks * e->S.per_block);
    return now;
}

/* -(-a // b) di Python fra interi positivi */
static __int128 e_ceildiv128(__int128 a, __int128 b)
{
    return -e_floordiv128(-a, b);
}

/* install_counters, la parte all'installazione (Dtims.__init__, owner.now = 0) */
static void dtcn_install(dt_engine *e, int64_t ips, int64_t owner_now)
{
    int ch;
    for (ch = 0; ch < 4; ch++)
        e->dtcn_has[ch] = 0;
    if (e_r16(e, DTIM_BASES[0] + DTMR_OFF) == 0) {
        e_w16(e, DTIM_BASES[0] + DTMR_OFF, BOOT_DTMR0);
        e->dtcn_has[0] = 1;
        e->dtcn_origin[0] = 0;
    }
    if (e_r16(e, DTIM_BASES[2] + DTMR_OFF) == 0)
        e_w16(e, DTIM_BASES[2] + DTMR_OFF, BOOT_DTMR2);
    for (ch = 0; ch < 4; ch++) {
        uint16_t dtmr;
        if (e->dtcn_has[ch])
            continue;
        dtmr = e_r16(e, DTIM_BASES[ch] + DTMR_OFF);
        if (dtmr & DTCN_RST) {
            uint32_t clk = (dtmr >> 1) & 3, saved = e_r32(e, DTIM_BASES[ch] + DTCN_OFF);
            __int128 o = owner_now;
            if (saved && !(dtmr & DTCN_FRR)) {
                if (clk == 1 || clk == 2) {
                    __int128 div = (__int128)(((dtmr >> 8) & 0xFF) + 1) * (clk == 1 ? 1 : 16);
                    o -= e_ceildiv128((__int128)saved * div * ips, E_F_BUS);
                } else if (clk == 3) {
                    __int128 ps = ((dtmr >> 8) & 0xFF) + 1;
                    o -= e_ceildiv128((__int128)saved * ps * ips, E_EXT_CLK_HZ);
                }
            }
            e->dtcn_origin[ch] = (int64_t)o;
            e->dtcn_has[ch] = 1;
        }
    }
}

static int dtcn_value(dt_engine *e, int ch, uint32_t *out)
{
    uint32_t b = DTIM_BASES[ch];
    uint16_t dtmr = e_r16(e, b + DTMR_OFF);
    uint32_t clk = (dtmr >> 1) & 3;
    __int128 now, div, hz, ticks;
    if (!(dtmr & DTCN_RST) || clk == 0)
        return 0;
    now = dtcn_now(e);
    if (clk == 3) {
        hz = E_EXT_CLK_HZ;
        div = (__int128)e->tim[1].ips * (((dtmr >> 8) & 0xFF) + 1);
    } else {
        hz = E_F_BUS;
        div = (__int128)e->tim[1].ips * (((dtmr >> 8) & 0xFF) + 1) * (clk == 1 ? 1 : 16);
    }
    if (!e->dtcn_has[ch]) {
        __int128 saved = e_r32(e, b + DTCN_OFF);
        e->dtcn_origin[ch] = (int64_t)(now - e_floordiv128(saved * div, hz));
        e->dtcn_has[ch] = 1;
    }
    ticks = e_floordiv128((now - e->dtcn_origin[ch]) * hz, div);
    if (dtmr & DTCN_FRR) {
        uint32_t ref = e_r32(e, b + DTRR_OFF);
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
        e_w32(d->e, DTIM_BASES[d->ch] + DTCN_OFF, val);
}

static void h_dtcn_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_dtcn *d = (e_dtcn *)ud;
    dt_engine *e = d->e;
    uint32_t b = DTIM_BASES[d->ch];
    (void)uc; (void)ty;
    if (a == b + DTCN_OFF) {
        e->dtcn_origin[d->ch] = dtcn_now(e);
        e->dtcn_has[d->ch] = 1;
    } else if (a == b + DTMR_OFF && size >= 2) {
        int was = e_r16(e, b + DTMR_OFF) & DTCN_RST;
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
