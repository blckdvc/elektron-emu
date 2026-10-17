/* engine.c, parte 2: gli hook di emu/longrun.py build(), emu/edma.py,
 * emu/softfloat.py ed emu/hle.py (bitmap), riga per riga. */

#define E_PRINT 0x400054B4u
#define E_SWITCH_TO 0x4000044Au
#define E_USR8 0xEC070004u
#define E_UDR8 0xEC07000Cu
#define E_SERQ 0xFC044018u
#define E_TCD_BASE 0xFC045000u
#define E_DEPACK_LEN_CAP 0x10000u

/* ------------------------------------------------------ hook di build() */
static void h_flash_read(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t b[16];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7), ret, off, ln, dest;
    (void)uc; (void)a; (void)s;
    e_read(e, sp, b, 16);
    ret = e_be32(b); off = e_be32(b + 4); ln = e_be32(b + 8); dest = e_be32(b + 12);
    if (ln && dest && (uint64_t)off + ln <= e->flash_len) {
        uint64_t p;
        for (p = 0; p < (uint64_t)ln + 0x100000u; p += 0x100000u)
            dtm_ensure(e->m, (uint64_t)dest + p);
        e_write(e, dest, e->flash + off, ln);
    }
    e_wr(e, UC_M68K_REG_D0, 0);
    e_wr(e, UC_M68K_REG_A7, sp + 4);
    e_wr(e, UC_M68K_REG_PC, ret);
}

static void h_pend_call(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    e_w32(e, e->p->completion_sem, 1);
}

static void h_task_create(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    e->task_creates++;                       /* osservazione: ev['tasks'] */
}

static void h_print(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    e->prints++;                             /* osservazione: ev['prints'] */
}

static void h_setpixel_count(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s;
    ((dt_engine *)ud)->setpixel_calls++;
}

static void h_pxcopy_count(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s;
    ((dt_engine *)ud)->pxcopy_calls++;
}

static void h_depack_clamp(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    if (e_rd(e, UC_M68K_REG_D2) > E_DEPACK_LEN_CAP) {
        e_wr(e, UC_M68K_REG_D2, 1);
        e->depack_clamps++;
    }
}

/* ----------------------------------------------------- emu/edma.py */
static void tx_init(e_txch *t, uint32_t chan, uint32_t vector, e_buf *out, uint64_t *total, uint32_t *crc)
{
    memset(t, 0, sizeof *t);
    t->chan = chan;
    t->vector = vector;
    t->tcd = E_TCD_BASE + chan * 0x20u;
    t->out = out;
    t->out_total = total;
    t->out_crc = crc;
}

/* TxChannel.run */
static int64_t tx_run(dt_engine *e, e_txch *t)
{
    uint32_t citer = e_r16(e, t->tcd + 0x14) & 0x7FFF, biter = e_r16(e, t->tcd + 0x1C) & 0x7FFF;
    uint32_t src, nbytes, smod, mask, base;
    int32_t soff;
    uint64_t total = 0, i, k;
    uint8_t chunk[256];
    size_t nch = 0;
    if (!citer)
        return 0;
    src = e_r32(e, t->tcd + 0x00);
    soff = (int16_t)e_r16(e, t->tcd + 0x06);
    nbytes = e_r32(e, t->tcd + 0x08);
    smod = (e_r16(e, t->tcd + 0x04) >> 11) & 0x1F;
    mask = smod ? ((1u << smod) - 1u) : 0;
    base = mask ? (src & ~mask) : 0;
    /* src in Python e' un intero illimitato: senza modulo cresce oltre 32 bit,
       e mem_read di un indirizzo > 32 bit fallisce. Qui si tiene a 64 bit. */
    {
        int64_t s64 = src;
        for (i = 0; i < citer; i++) {
            for (k = 0; k < nbytes; k++) {
                uint8_t byte = 0;
                e_read(e, (uint64_t)s64, &byte, 1);
                chunk[nch++] = byte;
                if (nch == sizeof chunk) {
                    e_buf_put(t->out, chunk, nch);
                    *t->out_crc = e_crc(*t->out_crc, chunk, nch);
                    nch = 0;
                }
                total++;
                s64 += soff;
                if (mask)
                    s64 = (int64_t)(base | ((uint64_t)s64 & mask));
            }
        }
        if (nch) {
            e_buf_put(t->out, chunk, nch);
            *t->out_crc = e_crc(*t->out_crc, chunk, nch);
        }
        s64 += e_r32(e, t->tcd + 0x0C);
        if (mask)
            s64 = (int64_t)(base | ((uint64_t)s64 & mask));
        e_w32(e, t->tcd + 0x00, (uint32_t)s64);
    }
    e_w16(e, t->tcd + 0x14, (uint16_t)biter);
    *t->out_total += total;
    t->bytes += (int64_t)total;
    t->transfers++;
    t->pending++;
    return (int64_t)total;
}

/* TxChannel.deliver */
static int tx_deliver(dt_engine *e, e_txch *t)
{
    if (!t->pending)
        return 0;
    if ((e_rd(e, UC_M68K_REG_SR) & 0x0700) == 0x0700)
        return 0;
    t->pending--;
    return e_raise(e, (int)t->vector, -1);
}

static void h_edma_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t val, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a; (void)size;
    if ((val & 0x40) || (uint32_t)(val & 0x3F) == e->etx.chan)
        tx_run(e, &e->etx);
}

static void h_edma_wait(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    tx_deliver(e, &e->etx);
}

/* ------------------------------------------- DT2_SKIP_DELAYS (i177) */
static void h_skip_delay(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    e_delay *d = (e_delay *)ud;
    (void)uc; (void)a; (void)s;
    if (e_rd(d->e, d->reg) > 1) {
        e_wr(d->e, d->reg, 1);
        d->e->delay_loops_skipped++;
    }
}

/* ---------------------------------------------------- emu/softfloat.py */
#define SF_TINY 1.1754943508222875e-38           /* 2.0 ** -126 */
#define SF_MAX 3.4028235e38                       /* il letterale del Python */

static int sf_fast(double x)
{
    return -SF_MAX <= x && x <= SF_MAX && (x == 0.0 || fabs(x) >= SF_TINY);
}

static float sf_b2f(uint32_t bits)
{
    float f;
    memcpy(&f, &bits, 4);
    return f;
}

static void h_softfloat(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    e_sf *f = (e_sf *)ud;
    dt_engine *e = f->e;
    uint8_t b[12];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7), ret, out = 0;
    int arity = f->op == 4 || f->op == 5 ? 1 : 2;
    double x, y = 0, r;
    (void)uc; (void)a; (void)s;
    if (e_read(e, sp, b, arity == 1 ? 8 : 12))
        return;
    ret = e_be32(b);
    x = (double)sf_b2f(e_be32(b + 4));
    if (arity == 2)
        y = (double)sf_b2f(e_be32(b + 8));
    if (!sf_fast(x) || (arity == 2 && !sf_fast(y))) {
        e->sf_deferred++;
        return;
    }
    switch (f->op) {
    case 0: r = x * y; break;
    case 1: r = x - y; break;
    case 2: r = x + y; break;
    case 3:
        if (y == 0.0) {                          /* _div: ZeroDivisionError */
            if (x != x || x == 0.0)
                r = NAN;
            else
                r = copysign(INFINITY, x) * copysign(1.0, y);
        } else
            r = x / y;
        break;
    case 4: r = fabs(x); break;
    case 5:
        if (!(-2147483648.0 <= x && x < 2147483648.0)) {
            e->sf_deferred++;
            return;
        }
        out = (uint32_t)(int64_t)x;
        goto done;
    default:
        if (x != x || y != y)
            out = 1;
        else
            out = (uint32_t)((x > y) - (x < y));
        goto done;
    }
    if (!(sf_fast(r) && r != 0.0)) {
        e->sf_deferred++;
        return;
    }
    {
        float rf = (float)r;
        memcpy(&out, &rf, 4);
    }
done:
    e_wr(e, UC_M68K_REG_D0, out);
    e_wr(e, UC_M68K_REG_A7, sp + 4);
    e_wr(e, UC_M68K_REG_PC, ret);
    e->sf_calls++;
}

/* ------------------------------------------------------ emu/hle.py */
static int32_t s32(uint32_t v)
{
    return (int32_t)v;
}

static void h_bmp_set(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t b[20], f[16];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7), ret, bmp, val;
    int64_t x, y, w, h, stride;
    uint32_t base;
    (void)uc; (void)a; (void)s;
    e_read(e, sp, b, 20);
    ret = e_be32(b); bmp = e_be32(b + 4);
    x = s32(e_be32(b + 8)); y = s32(e_be32(b + 12)); val = e_be32(b + 16);
    e_read(e, (uint64_t)bmp + 4, f, 16);
    w = s32(e_be32(f)); h = s32(e_be32(f + 4)); stride = s32(e_be32(f + 8)); base = e_be32(f + 12);
    if (0 <= x && x < w && 0 <= y && y < h) {
        uint64_t off = (uint64_t)((int64_t)base + ((x * stride) + (y >> 5)) * 4);
        uint32_t word = e_r32(e, off), mask = 0x80000000u >> (y & 31);
        word = (val & 1) ? (word | mask) : (word & ~mask);
        e_w32(e, off, word);
        /* on_frame (l'intro): solo osservazione */
    }
    e_wr(e, UC_M68K_REG_A7, sp + 4);
    e_wr(e, UC_M68K_REG_PC, ret);
    e->bmp_set++;
}

static void h_bmp_get(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t b[16], f[16];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7), ret, bmp, out = 0;
    int64_t x, y, w, h, stride;
    uint32_t base;
    (void)uc; (void)a; (void)s;
    e_read(e, sp, b, 16);
    ret = e_be32(b); bmp = e_be32(b + 4);
    x = s32(e_be32(b + 8)); y = s32(e_be32(b + 12));
    e_read(e, (uint64_t)bmp + 4, f, 16);
    w = s32(e_be32(f)); h = s32(e_be32(f + 4)); stride = s32(e_be32(f + 8)); base = e_be32(f + 12);
    if (0 <= x && x < w && 0 <= y && y < h) {
        uint64_t off = (uint64_t)((int64_t)base + ((x * stride) + (y >> 5)) * 4);
        out = e_r32(e, off) & (0x80000000u >> (y & 31));
    }
    e_wr(e, UC_M68K_REG_D0, out);
    e_wr(e, UC_M68K_REG_A7, sp + 4);
    e_wr(e, UC_M68K_REG_PC, ret);
    e->bmp_get++;
}

/* -------------------------------------------------- unblock (satisfy) */
static int e_in(const uint32_t *v, int n, uint32_t x)
{
    int i;
    for (i = 0; i < n; i++)
        if (v[i] == x)
            return 1;
    return 0;
}

static void e_skip_add(dt_engine *e, uint32_t sem)
{
    if (!e_in(e->skip, e->nskip, sem) && e->nskip < (int)(sizeof e->skip / sizeof e->skip[0]))
        e->skip[e->nskip++] = sem;
}

static void h_satisfy(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t b[8];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7), ret, sem;
    (void)uc; (void)a; (void)s;
    if (e_read(e, sp, b, 8))
        return;
    ret = e_be32(b); sem = e_be32(b + 4);
    if (!sem || e_in(e->skip, e->nskip, sem) || e_in(e->skip_callers, e->nskip_callers, ret))
        return;
    if (e_read(e, sem, b, 4))
        return;                                /* except Exception: pass */
    if ((int32_t)e_be32(b) <= 0) {
        e_w32(e, sem, 1);
        e->satisfied++;
    }
}

static void h_intro_done_skip(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    e_skip_add(e, e->p->frame_sem);
}

/* --------------------------------------- rilascio del gate del frame */
static void e_zero_frame_gate(dt_engine *e)
{
    e_w32(e, e->p->frame_sem, 0);
    e->frame_gate_released++;
}

static void h_intro_done_zero(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s;
    e_zero_frame_gate((dt_engine *)ud);
}

/* dt2_gate_code py: _release_on_entry (senza contesto: e_cur) */
static void g_release_on_entry(void *uc, uint64_t a, uint32_t s)
{
    dt_engine *e = e_cur;
    (void)uc; (void)a; (void)s;
    if (!e->frame_gate_done) {
        e->frame_gate_done = 1;
        e_zero_frame_gate(e);
        e->ctx_gate.open = 0;
    }
}

/* ------------------------------------------------------------ UART8 */
static void h_uart8_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)size; (void)v;
    if (a == E_USR8)
        e_w8(e, E_USR8, (uint8_t)(0x04 | (e->uart_in.n ? 0x01 : 0)));
    else if (a == E_UDR8) {
        int c = e_fifo_pop(&e->uart_in);
        e_w8(e, E_UDR8, (uint8_t)(c < 0 ? 0 : c));
    }
}

static void h_uart8_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)size;
    if (a == E_UDR8) {
        uint8_t byte = (uint8_t)(v & 0xFF);
        e_buf_put(&e->uart_out, &byte, 1);
        e->uart_out_total++;
        e->uart_out_crc = e_crc(e->uart_out_crc, &byte, 1);
    }
}
