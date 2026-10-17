/* st_engine.c, parte 3: Pits/Dtims (emu/pit.py, emu/dtim.py) intorno a
 * tim_state, per ciascuna CPU; l'orologio DTCN (dtim.install_counters, la
 * versione della Syntakt: DTIM2 a pin esterno, origini al resume) e i
 * callback del ciclo C (py_raise, stop). Da digitakt/emulator/core/src/
 * e_timers.inc.c, con le differenze del dtim.py della Syntakt. */

static const uint32_t PIT_BASES[4] = {0xFC080000u, 0xFC084000u, 0xFC088000u, 0xFC08C000u};
static const uint32_t DTIM_BASES[4] = {0xFC070000u, 0xFC074000u, 0xFC078000u, 0xFC07C000u};
#define E_F_BUS 132000000LL
#define E_EXT_CLK_HZ 12288000LL
#define DTCN_RST 0x01
#define DTCN_FRR 0x08
#define BOOT_DTMR0 0x0003
#define BOOT_DTMR2 0x0007

static void tsrc_init(st_cpu *c, e_tsrc *s, int kind, int nch, const uint32_t *ch, int64_t ips, int held)
{
    int i;
    memset(s, 0, sizeof *s);
    s->kind = kind;
    s->ips = ips;
    s->t.mem_read = (mem_rw_fn)uc_mem_read;
    s->t.kind = kind;
    s->t.nch = nch;
    s->t.held = held;
    /* staudio.configure: _LEVEL_IRQ = True. Il motore freddo e' la CoSim senza ST_AUDIO
       (emu/pit.py: _LEVEL_IRQ = False, un tick che la CPU non puo' prendere e' perso):
       con i tick a livello il boot freddo cambia strada e il DSP non completa
       l'handshake col MAIN (docs/AU-PORT.md, "Boot freddo in C") */
    s->t.level_irq = c->e->cold ? 0 : 1;
    for (i = 0; i < nch; i++)
        s->t.ch[i] = (int32_t)ch[i];
    for (i = 0; i < 4; i++) {
        s->t.base[i] = kind ? DTIM_BASES[i] : PIT_BASES[i];
        s->t.store[i] = NULL;
    }
    s->t.ips = (double)ips;
    s->t.q = &c->q;
    s->t.vec0 = kind ? 96 : 205;
}

/* CTIM: Pits(...), Dtims(...) come le ha salvate il Python; per il DSP nello
   stesso ordine di dspcpu.prepare_timed (Pits (3,2,0), Dtims (0,1,2,3)) */
static int timers_restore(st_cpu *c, const uint8_t *d, uint64_t n, char *err, size_t errlen)
{
    uint64_t off = 4;
    uint32_t ns, i;
    if (n < 4)
        goto bad;
    ns = e_le32(d);
    if (ns != 2)
        goto bad;
    for (i = 0; i < ns; i++) {
        e_tsrc *s = &c->tim[i];
        uint32_t kind, nch, ch[4];
        int64_t ips, now;
        int held, k;
        const uint8_t *p = d + off;
        if (off + 149 > n)
            goto bad;
        kind = e_le32(p);
        nch = e_le32(p + 4);
        for (k = 0; k < 4; k++)
            ch[k] = e_le32(p + 8 + 4 * k);
        ips = (int64_t)e_le64(p + 24);
        now = (int64_t)e_le64(p + 32);
        held = p[40];
        if (kind != i || nch > 4 || nch == 0)
            goto bad;
        if (held) {
            dt_errf(err, errlen, "%s timers held (the intro): not supported", c->dsp ? "DSP" : "MAIN");
            return DT_ERR_UNSUPPORTED;
        }
        tsrc_init(c, s, (int)kind, (int)nch, ch, ips, held);
        if (i == 0)
            c->now = now;
        else if (now != c->now)
            goto bad;
        for (k = 0; k < 4; k++) {
            const uint8_t *q = p + 41 + 9 * k;
            s->t.has[k] = q[0] == 1;
            if (q[0] == 1)
                memcpy(&s->t.next[k], q + 1, 8);
        }
        for (k = 0; k < 4; k++) {
            s->t.fired[k] = (int64_t)e_le64(p + 77 + 8 * k);
            s->t.missed[k] = (int64_t)e_le64(p + 109 + 8 * k);
        }
        s->arm = e_le32(p + 141);
        s->stale = e_le32(p + 145);
        off += 149;
    }
    return 0;
bad:
    dt_errf(err, errlen, "bad %s timers component", c->dsp ? "DSP" : "MAIN");
    return DT_ERR_FORMAT;
}

/* ------------------------------------------------------------ DTCN */
/* clock(): l'istante del passo (S.now durante la chiamata C), piu' i blocchi
   gia' eseguiti dentro emu_start (stepper.running); fuori, Dtims.now */
static int64_t dtcn_now(st_cpu *c)
{
    int64_t now = c->running ? c->S.now : c->now;
    if (c->running)
        now += (int64_t)((double)c->stp.blocks * c->S.per_block);
    return now;
}

static int64_t dtcn_ips(st_cpu *c)
{
    return c->tim[1].ips;                     /* _dtcn_owner: il Dtims */
}

static int dtcn_value(st_cpu *c, int ch, uint32_t *out)
{
    uint32_t b = DTIM_BASES[ch];
    uint16_t dtmr = c_r16(c, b);
    uint32_t clk = (dtmr >> 1) & 3;
    __int128 now, div, hz, ticks;
    if (!(dtmr & DTCN_RST) || clk == 0)
        return 0;
    now = dtcn_now(c);
    if (clk == 3) {                           /* pin esterno: EXT_CLK_HZ */
        hz = E_EXT_CLK_HZ;
        div = (__int128)dtcn_ips(c) * (((dtmr >> 8) & 0xFF) + 1);
    } else {
        hz = E_F_BUS;
        div = (__int128)dtcn_ips(c) * (((dtmr >> 8) & 0xFF) + 1) * (clk == 1 ? 1 : 16);
    }
    if (!c->dtcn_has[ch]) {
        __int128 saved = c_r32(c, b + 0x0C);
        c->dtcn_origin[ch] = (int64_t)(now - e_floordiv128(saved * div, hz));
        c->dtcn_has[ch] = 1;
    }
    ticks = e_floordiv128((now - c->dtcn_origin[ch]) * hz, div);
    if (dtmr & DTCN_FRR) {
        uint32_t ref = c_r32(c, b + 0x04);
        if (ref) {
            __int128 m = (__int128)ref + 1;
            ticks = ticks - e_floordiv128(ticks, m) * m;
        }
    }
    *out = (uint32_t)(ticks & 0xFFFFFFFF);
    return 1;
}

static void h_dtcn_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_dtcn *d = (e_dtcn *)ud;
    uint32_t val;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    if (dtcn_value(d->c, d->ch, &val))
        c_w32(d->c, DTIM_BASES[d->ch] + 0x0C, val);
}

static void h_dtcn_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    e_dtcn *d = (e_dtcn *)ud;
    st_cpu *c = d->c;
    uint32_t b = DTIM_BASES[d->ch];
    (void)uc; (void)ty;
    if (a == b + 0x0C) {                      /* una scrittura azzera */
        c->dtcn_origin[d->ch] = dtcn_now(c);
        c->dtcn_has[d->ch] = 1;
    } else if (a == b && size >= 2) {         /* uscita dal reset: riparte da 0 */
        int was = c_r16(c, b) & DTCN_RST;
        if (((v & DTCN_RST) && !was) || !c->dtcn_has[d->ch]) {
            c->dtcn_origin[d->ch] = dtcn_now(c);
            c->dtcn_has[d->ch] = 1;
        }
    }
}

/* dtim.install_counters(m, owner): eseguita quando il Dtims e' costruito (il
   DSP: prima del ripristino, owner.now = 0; il MAIN: dopo, con la memoria
   ripristinata e owner.now = 0 perche' lo stato dei timer arriva dopo). Poi
   gli hook, in ordine: per canale lettura di DTCN, scrittura di DTMR e di DTCN. */
static void dtcn_install(st_cpu *c, int64_t owner_ips)
{
    int ch;
    for (ch = 0; ch < 4; ch++)
        c->dtcn_has[ch] = 0;
    if (c_r16(c, DTIM_BASES[0]) == 0) {
        c_w16(c, DTIM_BASES[0], BOOT_DTMR0);
        c->dtcn_has[0] = 1;
        c->dtcn_origin[0] = 0;
    }
    if (c_r16(c, DTIM_BASES[2]) == 0)
        c_w16(c, DTIM_BASES[2], BOOT_DTMR2);
    for (ch = 0; ch < 4; ch++) {
        uint16_t dtmr;
        uint32_t clk, saved;
        if (c->dtcn_has[ch])
            continue;
        dtmr = c_r16(c, DTIM_BASES[ch]);
        if (!(dtmr & DTCN_RST))
            continue;
        c->dtcn_has[ch] = 1;
        c->dtcn_origin[ch] = 0;                       /* int(owner.now) */
        clk = (dtmr >> 1) & 3;
        saved = c_r32(c, DTIM_BASES[ch] + 0x0C);
        if (saved && !(dtmr & DTCN_FRR)) {
            __int128 num;
            if (clk == 1 || clk == 2) {
                int64_t div = (((dtmr >> 8) & 0xFF) + 1) * (clk == 1 ? 1 : 16);
                num = (__int128)saved * div * owner_ips;
                c->dtcn_origin[ch] -= (int64_t)(-e_floordiv128(-num, E_F_BUS));
            } else if (clk == 3) {
                int64_t ps = ((dtmr >> 8) & 0xFF) + 1;
                num = (__int128)saved * ps * owner_ips;
                c->dtcn_origin[ch] -= (int64_t)(-e_floordiv128(-num, E_EXT_CLK_HZ));
            }
        }
    }
    for (ch = 0; ch < 4; ch++) {
        c->dtcn_ud[ch].c = c;
        c->dtcn_ud[ch].ch = ch;
        c_hook_mem(c, UC_HOOK_MEM_READ, (void *)h_dtcn_read, &c->dtcn_ud[ch], DTIM_BASES[ch] + 0x0C,
                   DTIM_BASES[ch] + 0x0F);
        c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_dtcn_write, &c->dtcn_ud[ch], DTIM_BASES[ch],
                   DTIM_BASES[ch]);
        c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)h_dtcn_write, &c->dtcn_ud[ch], DTIM_BASES[ch] + 0x0C,
                   DTIM_BASES[ch] + 0x0C);
    }
}

/* Dtims.__init__: l'hook `arm` (dt2_set_bit) per ogni canale */
static void dtim_arm_hooks(st_cpu *c, const e_tsrc *dt)
{
    int i;
    for (i = 0; i < dt->t.nch; i++) {
        int ch = dt->t.ch[i];
        c->arm[ch].bits = &c->arm_bits;
        c->arm[ch].bit = 1 << ch;
        c_hook_mem(c, UC_HOOK_MEM_WRITE, (void *)dt2_set_bit, &c->arm[ch], DTIM_BASES[ch],
                   DTIM_BASES[ch] + 0x0F);
    }
}

/* dtim.rearm_cleared (REARM = {3: 0x001D}) */
static void dtim_rearm(st_cpu *c, const e_tsrc *dt)
{
    int i;
    for (i = 0; i < dt->t.nch; i++) {
        uint32_t b, vec;
        uint16_t dtmr;
        uint32_t dtrr;
        if (dt->t.ch[i] != 3)
            continue;
        b = DTIM_BASES[3];
        dtmr = c_r16(c, b);
        dtrr = c_r32(c, b + 0x04);
        vec = c_r32(c, 0x40000000u + 4u * 99u);
        if (dtmr == 0 && dtrr && 0x40000400u <= vec && vec < 0x48000000u)
            c_w16(c, b, 0x001D);
    }
}

/* ------------------------------------------------------- callback del C */
static int py_raise_main(int32_t vec, int32_t level)
{
    return dtm_raise_vector(e_cur->cpu[0].m, (uint32_t)vec, 0, level < 0 ? -1 : level);
}

static int py_raise_dsp(int32_t vec, int32_t level)
{
    return dtm_raise_vector(e_cur->cpu[1].m, (uint32_t)vec, 0, level < 0 ? -1 : level);
}

static void intr_fallback_c(void *uc, uint32_t intno)
{
    (void)uc; (void)intno;                   /* non usato: l'INTR e' quello di machine.c */
}

static int stop_c(void *uc)
{
    return (int)uc_emu_stop((uc_engine *)uc);
}

/* lo stato nativo di una CPU: eccezioni, stepper, ciclo dei passi */
static void cpu_native_init(st_cpu *c)
{
    c->exc.reg_read = (reg_rw_fn)uc_reg_read;
    c->exc.reg_write = (reg_rw_fn)uc_reg_write;
    c->exc.mem_read = (mem_rw_fn)uc_mem_read;
    c->exc.mem_write = (mem_rw_fn)uc_mem_write;
    c->exc.vbr = DT_VBR;
    c->exc.r_a7 = UC_M68K_REG_A7;
    c->exc.r_sr = UC_M68K_REG_SR;
    c->exc.r_pc = UC_M68K_REG_PC;
    c->exc.fallback = intr_fallback_c;
    c->q.exc = &c->exc;
    c->q.py_raise = c->dsp ? py_raise_dsp : py_raise_main;
    c->stp.stop = stop_c;
    c->S.emu_start = (emu_start_fn)uc_emu_start;
    c->S.q = &c->q;
    c->S.stp = &c->stp;
    c->S.per_block = 3.87;                   /* _FastStepper.PER_BLOCK */
    c->S.tx_flush = 1 << 19;                 /* nativespin.TXFLUSH */
    c->S.stop_flag = &c->m->stop_flag;
}
