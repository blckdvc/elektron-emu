/* st_engine.c, parte 5: la CPU DSP (emu/dspcpu.py build + prepare_timed), la
 * mailbox condivisa (emu/cosim.py share), la linea di sincronia, e il giro
 * del motore con le due CPU (emu/cosimspin.py CosimNative.run: dt2_cosim e
 * le sue uscite verso cio' che nel Python resta Python, qui in C). */

#define DSP_LOAD 0x40000400u
#define MAILBOX_SIZE 0x100000u
#define MAIN_MAILBOX 0x10000000u
#define DSP_MAILBOX 0x00000000u
#define ST_SLICE 50000
#define DSP_DSPI1 0xFC03C000u
#define DSP_EDMA_SSRT 0xFC04401Eu

static const uint8_t SIG_BZERO[] = {0x20, 0x2f, 0x00, 0x08, 0x67, 0x0c, 0x20, 0x6f, 0x00, 0x04,
                                    0xd0, 0x88, 0x42, 0x18, 0xb0, 0x88, 0x66, 0xfa, 0x4e, 0x75};
static const uint8_t SIG_IDLE_WAIT[] = {0x20, 0x39, 0x44, 0x04, 0xf7, 0x50, 0x66, 0xf8};
static const uint8_t SIG_AUDIO_IDLE[] = {0x22, 0x3c, 0x40, 0x00, 0x09, 0x34, 0x21, 0x41, 0x01, 0x80,
                                         0x60, 0xfe};

static long find_bytes(const uint8_t *h, size_t hn, const uint8_t *n, size_t nn)
{
    size_t i;
    for (i = 0; i + nn <= hn; i++)
        if (h[i] == n[0] && !memcmp(h + i, n, nn))
            return (long)i;
    return -1;
}

/* ------------------------------------------------ hook del DSP (dspcpu) */
/* install_pit_delay: con PCSR.EN ogni lettura di PCSR trova PIF */
static void h_pit_delay_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    uint32_t base = (uint32_t)(uintptr_t)ud;
    uint8_t b[2];
    (void)ty; (void)a; (void)size; (void)v;
    if (uc_mem_read(uc, base, b, 2) != UC_ERR_OK)
        return;
    if (e_be16(b) & 0x0001) {
        uint16_t p = e_be16(b) | 0x0004;
        e_put16(b, p);
        uc_mem_write(uc, base, b, 2);
    }
}

static void h_pit_delay_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size; (void)v; (void)ud;      /* la scrittura arriva; PIF si alza in lettura */
}

/* install_dspi1_fpga */
static void h_dspi1_sr(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    uint8_t b[4] = {0x90, 0x00, 0x00, 0x00};                          /* TCF | EOQF */
    (void)ty; (void)a; (void)size; (void)v; (void)ud;
    uc_mem_write(uc, DSP_DSPI1 + 0x2C, b, 4);
}

static void h_dspi1_push(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a;
    if (size == 4) {
        uint8_t b[2] = {(uint8_t)((v >> 8) & 0xFF), (uint8_t)(v & 0xFF)};
        e_buf_put(&e->fpga_config, b, 2);
        e->fpga_config_total += 2;
        if (e->fpga_config.n > (1u << 20))       /* si tengono le ultime: se ne salvano 54666 */
            e_buf_drop(&e->fpga_config, e->fpga_config.n - 65536);
    }
}

/* prepare_timed hle_bzero */
static void h_bzero(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    st_cpu *c = DSPC(e);
    uint8_t b[12];
    uint32_t sp = c_rd(c, UC_M68K_REG_A7), ret, dst, n;
    (void)uc; (void)a; (void)s;
    c_read(c, sp, b, 12);
    ret = e_be32(b); dst = e_be32(b + 4); n = e_be32(b + 8);
    if (n) {
        uint64_t base;
        uint8_t *z = (uint8_t *)calloc(1, n);
        for (base = dst & ~0xFFFFFu; base < (uint64_t)dst + n; base += 0x100000u)
            dtm_ensure(c->m, base);
        if (z) {
            c_write(c, dst, z, n);
            free(z);
        }
    }
    c_wr(c, UC_M68K_REG_A7, sp + 4);
    c_wr(c, UC_M68K_REG_PC, ret);
    e->bzero_calls++;
}

/* prepare_timed park: fermo nel ciclo d'attesa del fronte, con il flag alzato */
static void h_park(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    st_cpu *c = DSPC(e);
    uint8_t b[4];
    (void)a; (void)s;
    if (c_read(c, e->idle_flag, b, 4) || (b[0] | b[1] | b[2] | b[3]) == 0)
        return;
    e->park = 1;
    c->m->stop_flag = 1;
    uc_emu_stop(uc);
}

/* prepare_timed park_audio: il `bra *` del ciclo audio */
static void h_park_audio(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)a; (void)s;
    e->park = 1;
    DSPC(e)->m->stop_flag = 1;
    uc_emu_stop(uc);
}

/* ------------------------------------------------- la mailbox condivisa */
/* cosim.share: la pagina che la macchina aveva (preservata) va nel buffer
   condiviso, poi il buffer si mappa al suo posto */
static int mailbox_share(st_cpu *c, uint32_t addr, uint8_t *buf)
{
    uint32_t i = addr >> 20;
    if (dtm_is_mapped(c->m, addr)) {
        if (uc_mem_read(c->uc, addr, buf, MAILBOX_SIZE) == UC_ERR_OK)
            uc_mem_unmap(c->uc, addr, MAILBOX_SIZE);
    }
    if (uc_mem_map_ptr(c->uc, addr, MAILBOX_SIZE, UC_PROT_ALL, buf) != UC_ERR_OK)
        return -1;
    c->m->mapped[i >> 3] |= (uint8_t)(1u << (i & 7));
    return 0;
}

/* --------------------------------------------- NativeSpin.drive (C) */
/* -> 0 'limit', 1 fermata (e->status), -1 errore */
static void tx_take(dt_engine *e);

static int cpu_drive(dt_engine *e, st_cpu *c, int code)
{
    spin_state *S = &c->S;
    for (;;) {
        if (code == SPIN_LIMIT)
            return 0;
        if (code == SPIN_FLUSH) {
            tx_take(e);
        } else if (code == SPIN_PC_ZERO) {
            snprintf(e->status, sizeof e->status, "halted: %s pc zero", c->dsp ? "DSP" : "MAIN");
            return 1;
        } else if (code == SPIN_UC_ERROR) {
            snprintf(e->status, sizeof e->status, "halted: %s %s", c->dsp ? "DSP" : "MAIN",
                     uc_strerror((uc_err)S->uc_err));
            return 1;
        } else if (code == SPIN_STOPPED) {
            if (c->dsp && e->park)
                return 1;                     /* il parcheggio (halt_vec = -1 nel Python) */
            if (c->m->cnt.halt_vec >= 0) {
                snprintf(e->status, sizeof e->status, "halted: %s unhandled vector %d at %#010x",
                         c->dsp ? "DSP" : "MAIN", c->m->cnt.halt_vec, S->pc);
                return 1;
            }
            if (S->pc == 0) {
                snprintf(e->status, sizeof e->status, "halted: %s pc zero", c->dsp ? "DSP" : "MAIN");
                return 1;
            }
            c->m->stop_flag = 0;
            S->done += S->executed;
            if (dt2_spin_service(c->uc, S) < 0) {
                snprintf(e->status, sizeof e->status, "halted: %s native error %d/%d",
                         c->dsp ? "DSP" : "MAIN", S->err, c->q.err);
                return -1;
            }
        } else {
            snprintf(e->status, sizeof e->status, "halted: %s native error %d/%d/%d/%d",
                     c->dsp ? "DSP" : "MAIN", S->err, c->q.err, e->sst.err, e->frc.err);
            return -1;
        }
        code = dt2_spin(c->uc, S);
    }
}

/* NativeSpin.enter (i controlli sono gia' stati fatti alla costruzione) */
static void cpu_enter(st_cpu *c, uint32_t pc)
{
    spin_state *S = &c->S;
    S->base = c->now;
    S->done = 0;
    S->instrs = 0;
    S->pc = pc;
    S->steps = 0;
    S->err = S->uc_err = 0;
    c->q.err = 0;
    c->m->stop_flag = 0;
    c->m->cnt.halt_vec = -1;
    c->running = 1;
}

/* CosimNative._dsp_rest: il passo del DSP uscito da dt2_spin non per il
   parcheggio: il resto di spin, poi la coda di step_timed */
static int cosim_dsp_rest(dt_engine *e, int code)
{
    st_cpu *d = DSPC(e);
    cz_state *Z = &e->Z;
    int64_t dd;
    int r = cpu_drive(e, d, code);           /* step_timed non guarda lo stop */
    if (r < 0)
        return -1;
    if (e->park) {
        if (dt2_cz_park(Z) < 0)
            return -1;
        dd = Z->d_chunk;
    } else {
        Z->dnow = d->S.base + d->S.done;
        dd = d->S.done;
    }
    Z->d_done += dd > 1 ? dd : 1;
    Z->dpc = c_rd(d, UC_M68K_REG_PC);
    Z->phase = PH_DSP_CHUNK;
    return 0;
}

/* CosimNative.run: un giro del motore della GUI con le due CPU.
   -> 0, 1 fermato (e->status) */
static int engine_cosim(dt_engine *e, int64_t budget, int64_t *executed)
{
    st_cpu *m = MAINC(e), *d = DSPC(e);
    cz_state *Z = &e->Z;
    int stop = 0, code;
    cpu_enter(m, e->pc);
    e->sst.err = 0;
    e->frc.err = 0;
    cpu_enter(d, e->dpc);
    e->cz_calls++;
    Z->budget = budget;
    Z->executed = 0;
    Z->pc = e->pc;
    Z->dpc = e->dpc;
    Z->mnow = m->S.base;
    Z->dnow = d->S.base;
    Z->phase = PH_MAIN;
    Z->stop = 0;
    Z->err = 0;
    Z->dtin0_edges = e->dtin0_edges;
    Z->last_edges = e->last_edges;
    Z->irq4 = e->irq4;
    Z->idle_skipped = e->idle_skipped;
    for (;;) {
        code = dt2_cosim(Z);
        if (code == CZ_DONE)
            break;
        if (code == CZ_MAIN) {
            int r;
            e->cz_main_exits++;
            r = cpu_drive(e, m, Z->mcode);
            Z->phase = PH_MAIN_DONE;
            if (r) {
                Z->stop = 1;
                stop = 1;
            }
        } else if (code == CZ_DSP) {
            e->cz_dsp_exits++;
            if (cosim_dsp_rest(e, Z->dcode) < 0) {
                stop = 1;
                break;
            }
        } else if (code == CZ_LINK) {
            e->cz_link_exits++;
            Z->moved = sl_deliver(e) ? 1 : 0;
        } else {
            snprintf(e->status, sizeof e->status, "halted: lock-step error %d (phase %d, MAIN %d/%d, DSP %d/%d)",
                     Z->err, Z->phase, m->S.err, m->q.err, d->S.err, d->q.err);
            stop = 1;
            break;
        }
    }
    /* leave */
    m->running = 0;
    d->running = 0;
    m->now = m->S.base + m->S.done;
    d->now = Z->dnow;
    tx_take(e);
    e->pc = Z->pc;
    e->dpc = Z->dpc;
    e->dtin0_edges = Z->dtin0_edges;
    e->last_edges = Z->last_edges;
    e->irq4 = Z->irq4;
    e->idle_skipped = Z->idle_skipped;
    *executed = Z->executed;
    return stop;
}
