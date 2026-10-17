/* st_engine.c, parte 2: gli hook del MAIN di emu/longrun.py build(), emu/edma.py,
 * emu/softfloat.py, emu/hle.py (bitmap), la UART8, emu/board.py, la porta
 * 0x8C000000 (emu/dsp.py), la porta dei parametri verso l'FPGA
 * (emu/fpgaport.py), riga per riga. Da digitakt/emulator/core/src/e_build.inc.c
 * dove il Python e' lo stesso. */

#define E_PRINT 0x400054B4u
#define E_USR8 0xEC070004u
#define E_UDR8 0xEC07000Cu
#define E_SERQ 0xFC044018u
#define E_TCD_BASE 0xFC045000u
#define E_DEPACK_LEN_CAP 0x10000u

/* ------------------------------------------------------ hook di build() */
static void h_flash_read(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    st_cpu *c = MAINC(e);
    uint8_t b[16];
    uint32_t sp = c_rd(c, UC_M68K_REG_A7), ret, off, ln, dest;
    (void)uc; (void)a; (void)s;
    c_read(c, sp, b, 16);
    ret = e_be32(b); off = e_be32(b + 4); ln = e_be32(b + 8); dest = e_be32(b + 12);
    if (ln && dest && (uint64_t)off + ln <= e->flash_len) {
        uint64_t p;
        for (p = 0; p < (uint64_t)ln + 0x100000u; p += 0x100000u)
            dtm_ensure(c->m, (uint64_t)dest + p);
        c_write(c, dest, e->flash + off, ln);
    }
    c_wr(c, UC_M68K_REG_D0, 0);
    c_wr(c, UC_M68K_REG_A7, sp + 4);
    c_wr(c, UC_M68K_REG_PC, ret);
}

static void h_pend_call(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    c_w32(MAINC(e), e->p->completion_sem, 1);
}

static void h_task_create(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s;
    ((dt_engine *)ud)->task_creates++;      /* osservazione: ev['tasks'] */
}

static void h_print(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    (void)uc; (void)a; (void)s;
    ((dt_engine *)ud)->prints++;            /* osservazione: ev['prints'] */
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
    st_cpu *c = MAINC(e);
    (void)uc; (void)a; (void)s;
    if (c_rd(c, UC_M68K_REG_D2) > E_DEPACK_LEN_CAP) {
        c_wr(c, UC_M68K_REG_D2, 1);
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
static int64_t tx_run(st_cpu *c, e_txch *t)
{
    uint32_t citer = c_r16(c, t->tcd + 0x14) & 0x7FFF, biter = c_r16(c, t->tcd + 0x1C) & 0x7FFF;
    uint32_t src, nbytes, smod, mask, base;
    int32_t soff;
    uint64_t total = 0, i, k;
    uint8_t chunk[256];
    size_t nch = 0;
    if (!citer)
        return 0;
    src = c_r32(c, t->tcd + 0x00);
    soff = (int16_t)c_r16(c, t->tcd + 0x06);
    nbytes = c_r32(c, t->tcd + 0x08);
    smod = (c_r16(c, t->tcd + 0x04) >> 11) & 0x1F;
    mask = smod ? ((1u << smod) - 1u) : 0;
    base = mask ? (src & ~mask) : 0;
    {
        int64_t s64 = src;
        for (i = 0; i < citer; i++) {
            for (k = 0; k < nbytes; k++) {
                uint8_t byte = 0;
                c_read(c, (uint64_t)s64, &byte, 1);
                chunk[nch++] = byte;
                if (nch == sizeof chunk) {
                    e_buf_put(t->out, chunk, nch);
                    if (t->out_crc)
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
            if (t->out_crc)
                *t->out_crc = e_crc(*t->out_crc, chunk, nch);
        }
        s64 += c_r32(c, t->tcd + 0x0C);           /* SLAST: _u32, senza segno */
        if (mask)
            s64 = (int64_t)(base | ((uint64_t)s64 & mask));
        c_w32(c, t->tcd + 0x00, (uint32_t)s64);
    }
    c_w16(c, t->tcd + 0x14, (uint16_t)biter);
    if (t->out_total)
        *t->out_total += total;
    t->bytes += (int64_t)total;
    t->transfers++;
    t->pending++;
    return (int64_t)total;
}

/* TxChannel.deliver */
static int tx_deliver(st_cpu *c, e_txch *t)
{
    if (!t->pending)
        return 0;
    if ((c_rd(c, UC_M68K_REG_SR) & 0x0700) == 0x0700)
        return 0;
    t->pending--;
    return c_raise(c, (int)t->vector, -1);
}

static void h_edma_wait(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)a; (void)s;
    tx_deliver(MAINC(e), &e->etx);
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
    st_cpu *c = MAINC(e);
    uint8_t b[12];
    uint32_t sp = c_rd(c, UC_M68K_REG_A7), ret, out = 0;
    int arity = f->op == 4 || f->op == 5 ? 1 : 2;
    double x, y = 0, r;
    (void)uc; (void)a; (void)s;
    if (c_read(c, sp, b, arity == 1 ? 8 : 12))
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
    c_wr(c, UC_M68K_REG_D0, out);
    c_wr(c, UC_M68K_REG_A7, sp + 4);
    c_wr(c, UC_M68K_REG_PC, ret);
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
    st_cpu *c = MAINC(e);
    uint8_t b[20], f[16];
    uint32_t sp = c_rd(c, UC_M68K_REG_A7), ret, bmp, val;
    int64_t x, y, w, h, stride;
    uint32_t base;
    (void)uc; (void)a; (void)s;
    c_read(c, sp, b, 20);
    ret = e_be32(b); bmp = e_be32(b + 4);
    x = s32(e_be32(b + 8)); y = s32(e_be32(b + 12)); val = e_be32(b + 16);
    c_read(c, (uint64_t)bmp + 4, f, 16);
    w = s32(e_be32(f)); h = s32(e_be32(f + 4)); stride = s32(e_be32(f + 8)); base = e_be32(f + 12);
    if (0 <= x && x < w && 0 <= y && y < h) {
        uint64_t off = (uint64_t)((int64_t)base + ((x * stride) + (y >> 5)) * 4);
        uint32_t word = c_r32(c, off), mask = 0x80000000u >> (y & 31);
        word = (val & 1) ? (word | mask) : (word & ~mask);
        c_w32(c, off, word);
    }
    c_wr(c, UC_M68K_REG_A7, sp + 4);
    c_wr(c, UC_M68K_REG_PC, ret);
    e->bmp_set++;
}

static void h_bmp_get(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    st_cpu *c = MAINC(e);
    uint8_t b[16], f[16];
    uint32_t sp = c_rd(c, UC_M68K_REG_A7), ret, bmp, out = 0;
    int64_t x, y, w, h, stride;
    uint32_t base;
    (void)uc; (void)a; (void)s;
    c_read(c, sp, b, 16);
    ret = e_be32(b); bmp = e_be32(b + 4);
    x = s32(e_be32(b + 8)); y = s32(e_be32(b + 12));
    c_read(c, (uint64_t)bmp + 4, f, 16);
    w = s32(e_be32(f)); h = s32(e_be32(f + 4)); stride = s32(e_be32(f + 8)); base = e_be32(f + 12);
    if (0 <= x && x < w && 0 <= y && y < h) {
        uint64_t off = (uint64_t)((int64_t)base + ((x * stride) + (y >> 5)) * 4);
        out = c_r32(c, off) & (0x80000000u >> (y & 31));
    }
    c_wr(c, UC_M68K_REG_D0, out);
    c_wr(c, UC_M68K_REG_A7, sp + 4);
    c_wr(c, UC_M68K_REG_PC, ret);
    e->bmp_get++;
}

/* ------------------------------------------------------------ UART8 */
static void h_uart8_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    st_cpu *c = MAINC(e);
    (void)uc; (void)ty; (void)size; (void)v;
    if (a == E_USR8)
        c_w8(c, E_USR8, (uint8_t)(0x04 | (e->uart_in.n ? 0x01 : 0)));
    else if (a == E_UDR8) {
        int b = e_fifo_pop(&e->uart_in);
        c_w8(c, E_UDR8, (uint8_t)(b < 0 ? 0 : b));
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

/* ------------------------------------------- emu/dsp.py Fifo (fast=False) */
#define FIFO_STATUS 0x8C000002u
#define FIFO_LATCH 0x8C00000Au

static void h_fifo_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    static const uint8_t ready[2] = {0x00, 0x01};    /* polls > poll_delay (0): READY */
    (void)ty; (void)a; (void)size; (void)v;
    e->fifo_polls++;
    uc_mem_write(uc, FIFO_STATUS, ready, 2);
}

static void h_fifo_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e->fifo_words++;
    e->fifo_polls = 0;
}

static void h_fifo_latch(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    ((dt_engine *)ud)->fifo_bursts++;
}

/* --------------------------------------------------- emu/board.py */
#define BOARD_REVISION 0x04
#define BOARD_HALT_FLAG 0x43BDC4BCu
#define BOARD_HALT_SETTER 0x400A560Cu
static const uint8_t BOARD_HALT_SETTER_BYTES[] = {0x70, 0x01, 0x23, 0xc0, 0x43, 0xbd, 0xc4, 0xbc, 0x4e, 0x75};

static int board_install_repair(st_cpu *c)
{
    uint8_t old, rev = BOARD_REVISION, b[sizeof BOARD_HALT_SETTER_BYTES];
    dtm_ensure(c->m, 0);
    old = c_r8(c, 0);
    c_write(c, 0, &rev, 1);
    if (old != 0)
        return 0;
    if (c_read(c, BOARD_HALT_SETTER, b, sizeof b) || memcmp(b, BOARD_HALT_SETTER_BYTES, sizeof b))
        return 0;
    if (c_r32(c, BOARD_HALT_FLAG) != 1)
        return 0;
    c_w32(c, BOARD_HALT_FLAG, 0);
    return 1;
}

/* ------------------------------------------------- emu/fpgaport.py */
#define FP_DSPI1 0xFC03C000u
#define FP_TCD (E_TCD_BASE + 15u * 0x20u)
#define FP_TCF 0x80000000u
#define FP_EOQF 0x10000000u
#define FP_FRAME_WORDS 500

static void fpga_install(st_cpu *c)
{
    dtm_ensure(c->m, FP_DSPI1);
    dtm_ensure(c->m, FP_TCD);
    c_w32(c, FP_DSPI1 + 0x2C, c_r32(c, FP_DSPI1 + 0x2C) | FP_TCF | FP_EOQF);
}

/* FpgaPort._serq */
static void fpga_serq(dt_engine *e, int size, int64_t value)
{
    st_cpu *c = MAINC(e);
    uint8_t t[32];
    uint32_t saddr, nbytes, citer, biter, csr, n, k;
    int16_t soff;
    uint8_t *raw;
    if (size != 1 || (value & 0x40) || (value & 0x3F) != 15)
        return;
    c_read(c, FP_TCD, t, 32);
    saddr = e_be32(t); nbytes = e_be32(t + 8);
    citer = e_be16(t + 0x14) & 0x7FFF;
    biter = e_be16(t + 0x1C);
    soff = (int16_t)e_be16(t + 6);
    csr = e_be16(t + 0x1E);
    n = (citer ? citer : 1) * nbytes;
    n -= n % 4;
    raw = (uint8_t *)malloc(n ? n : 1);
    if (raw && !c_read(c, saddr, raw, n)) {
        /* hi = raw[2::4], lo = raw[3::4]; la cornice: sync 0xAAAA, 500 parole, EOQ 0x5555 */
        uint32_t words = n / 4, end = FP_FRAME_WORDS + 1, found = 0;
        if (words > end && raw[4 * end + 2] == 0x55 && raw[4 * end + 3] == 0x55)
            found = 1;
        else
            for (k = 1; k < words; k++)
                if (raw[4 * k + 2] == 0x55 && raw[4 * k + 3] == 0x55) {
                    end = k;
                    found = 1;
                    break;
                }
        if (words && raw[2] == 0xAA && raw[3] == 0xAA && found)
            e->fpga_sent++;
        else
            e->fpga_bad++;
    }
    free(raw);
    c_w32(c, FP_TCD + 0x00, (uint32_t)(saddr + (int64_t)soff * (citer ? citer : 1)));
    c_w16(c, FP_TCD + 0x14, (uint16_t)biter);
    c_w16(c, FP_TCD + 0x1E, (uint16_t)((csr | 0x0080) & ~0x0040));
    c_w32(c, FP_DSPI1 + 0x2C, c_r32(c, FP_DSPI1 + 0x2C) | FP_TCF | FP_EOQF);
}
