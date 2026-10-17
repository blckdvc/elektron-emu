/* engine.c, parte 5: la scheda del Model:Cycles, riga per riga dal Python:
 *   emu/mcpanel.py   il pannello a matrice su FlexBus 0x8C000000 (tasti,
 *                    encoder in quadratura, pad via ADC, LED), con MIN_READS
 *   emu/i2c.py       I2C0, il codec: ogni byte con ACK
 *   emu/pitpoll.py   PIT1 come ritardo a polling
 *   emu/dsp.py       la linea ready della porta 0x8C000000 (Fifo, poll_delay 0)
 *   emu/board.py     il registro di revisione della scheda a 0x0
 *   harness.mux_write_hook  un solo hook su SERQ con i gestori in ordine */

/* ------------------------------------------------------------ mcpanel */
#define MC_BASE 0x8C000000u
#define MC_COL (MC_BASE + 2)
#define MC_ADC1 0xFC094012u
#define MC_PAD_FULL 0x7FF8
#define MC_PAD_MIN 0x1200

/* KEYTABLE: codice -> (colonna, bit), dalla tabella del firmware 0x4010aef8 */
static const uint8_t MC_KEYTABLE[32] = {
    0x05, 0x06, 0x07, 0x08, 0x0e, 0x0b, 0x0d, 0x0a, 0x0c, 0x09, 0x01, 0x03, 0x04, 0x02, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x0f, 0x20};
static const int MC_GRAY[4] = {0, 1, 3, 2};

static int mc_wire(int code, int *c, int *b)
{
    int i;
    for (i = 0; i < 32; i++)
        if (MC_KEYTABLE[i] == code) {
            *c = i / 8;
            *b = i % 8;
            return 1;
        }
    return 0;
}

static void h_mc_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    mc_panel *p = &((dt_engine *)ud)->pn;
    (void)uc; (void)ty;
    if (a == MC_COL && size == 2) {
        p->col = (int)((v >> 5) & 7);
        if (v & 1) {
            if (p->col == 0)
                p->scans++;
        } else
            p->leds[p->col] = (uint8_t)((v >> 8) & 0xFF);
    }
}

static uint8_t mc_byte(mc_panel *p, int col)
{
    int k, pp;
    uint8_t v = 0;
    if (col < 4)
        return p->keys[col];
    for (pp = 0; pp < 4; pp++) {
        int s;
        k = (col - 4) * 4 + pp;
        if (p->pending[k]) {
            int d = p->pending[k] > 0 ? 1 : -1;
            p->enc[k] = ((p->enc[k] + d) % 4 + 4) % 4;      /* % di Python */
            p->pending[k] -= d;
        }
        s = MC_GRAY[p->enc[k]];
        v |= (uint8_t)(((s >> 1) & 1) << (2 * pp) | (s & 1) << (2 * pp + 1));
    }
    return v;
}

static void h_mc_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    mc_panel *p = &e->pn;
    int col = p->col, code;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e_w8(e, MC_COL, mc_byte(p, col));
    p->reads[col]++;
    for (code = 0; code < MC_NCODES; code++) {
        int c, b;
        if (!p->late[code] || !mc_wire(code, &c, &b))
            continue;
        if (c == col && p->reads[c] - p->down_at[code] >= MC_MIN_READS) {
            p->keys[c] &= (uint8_t)~(1u << b);
            p->late[code] = 0;
        }
    }
}

static void h_mc_read_adc(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    mc_panel *p = &e->pn;
    int col = p->col;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e_w16(e, MC_ADC1, p->adc[col]);
    p->adc_reads[col]++;
    if (p->pad_late[col] && p->adc_reads[col] - p->pad_at[col] >= MC_MIN_READS) {
        p->adc[col] = 0;
        p->pad_late[col] = 0;
    }
}

/* Panel.press / release / turn / pad (il lato dell'host) */
static void mc_press(mc_panel *p, int code)
{
    int c, b;
    if (code < 0 || code >= MC_NCODES || !mc_wire(code, &c, &b))
        return;
    p->keys[c] |= (uint8_t)(1u << b);
    p->down_at[code] = p->reads[c];
    p->has_down[code] = 1;
    p->late[code] = 0;
}

static void mc_release(mc_panel *p, int code)
{
    int c, b;
    if (code < 0 || code >= MC_NCODES || !mc_wire(code, &c, &b))
        return;
    if (p->has_down[code] && p->reads[c] - p->down_at[code] < MC_MIN_READS) {
        p->late[code] = 1;
        return;
    }
    p->keys[c] &= (uint8_t)~(1u << b);
}

static void mc_turn(mc_panel *p, int enc, int detents)
{
    if (enc >= 0 && enc < 16)
        p->pending[enc] += 2 * (int64_t)detents;
}

static void mc_pad(mc_panel *p, int n, int velocity)
{
    int k = 6 - n, v;
    if (n < 1 || n > 6)
        return;
    v = velocity < 0 ? 0 : velocity > 127 ? 127 : velocity;
    if (v == 0) {
        if (p->has_pad_at[k] && p->adc_reads[k] - p->pad_at[k] < MC_MIN_READS) {
            p->pad_late[k] = 1;
            return;
        }
        p->adc[k] = 0;
        return;
    }
    p->adc[k] = (uint16_t)(MC_PAD_MIN + (MC_PAD_FULL - MC_PAD_MIN) * v / 127);
    p->pad_at[k] = p->adc_reads[k];
    p->has_pad_at[k] = 1;
    p->pad_late[k] = 0;
}

/* ---------------------------------------------------------------- I2C0 */
#define I2C_BASE 0xFC058000u
#define I2C_CR (I2C_BASE + 0x08)
#define I2C_SR (I2C_BASE + 0x0C)
#define I2C_DR (I2C_BASE + 0x10)
#define I2C_MSTA 0x20
#define I2C_MTX 0x10
#define I2C_ICF 0x80
#define I2C_IBB 0x20
#define I2C_IIF 0x02

static void i2c_sync(dt_engine *e)
{
    e_w8(e, I2C_SR, e->i2c.sr);               /* try/except: senza pagina non scrive */
    e_w8(e, I2C_CR, e->i2c.cr);
}

static void h_i2c_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t value, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    mc_i2c *s = &e->i2c;
    uint8_t v = (uint8_t)(value & 0xFF);
    (void)uc; (void)ty; (void)size;
    if (a == I2C_CR) {
        if ((v & I2C_MSTA) && !(s->cr & I2C_MSTA)) {
            s->sr |= I2C_IBB;
            s->in_tx = 1;
        } else if ((s->cr & I2C_MSTA) && !(v & I2C_MSTA)) {
            s->sr &= (uint8_t)~I2C_IBB;
            s->in_tx = 0;
        }
        /* repeated start: solo il registro delle transazioni (osservazione) */
        s->cr = (uint8_t)(v & ~0x04);
    } else if (a == I2C_SR) {
        s->sr &= (uint8_t)~(I2C_IIF | 0x10);
    } else if (a == I2C_DR) {
        if (s->in_tx)
            s->bytes++;
        s->sr = (uint8_t)((s->sr | I2C_IIF | I2C_ICF) & ~0x01);
    }
}

static void h_i2c_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)size; (void)v;
    if (a == I2C_DR && !(e->i2c.cr & I2C_MTX)) {
        e->i2c.sr |= I2C_IIF | I2C_ICF;
        e_w8(e, I2C_DR, 0);
    }
    i2c_sync(e);
}

/* ------------------------------------------------------- PIT1 a polling */
#define PIT1_PCSR 0xFC084000u

static void h_pit1_poll(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t b[2] = {0, 0};
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e_read(e, PIT1_PCSR, b, 2);
    if ((b[1] & 0x01) && !(b[1] & 0x08)) {
        e->pit1_polls++;
        e_w8(e, PIT1_PCSR + 1, (uint8_t)(b[1] | 0x04));
    }
}

/* -------------------------------------------- dsp.Fifo (poll_delay = 0) */
#define DSP_STATUS (MC_BASE + 0x02)
#define DSP_LATCH (MC_BASE + 0x0A)

static void h_dsp_read(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e->dsp_polls++;
    e_w16(e, DSP_STATUS, 0x0001);             /* READY: polls > poll_delay */
}

static void h_dsp_write(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    e->dsp_words++;
    e->dsp_polls = 0;
}

static void h_dsp_latch(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    ((dt_engine *)ud)->dsp_bursts++;
}

/* ---------------------------------------------------------- board.py */
static void board_install(dt_engine *e)
{
    static const uint8_t setter[10] = {0x70, 0x01, 0x23, 0xc0, 0x43, 0xbd, 0xc4, 0xbc, 0x4e, 0x75};
    uint8_t old = 0, b[10];
    dtm_ensure(e->m, 0);
    e_read(e, 0, &old, 1);
    e_w8(e, 0, 0x04);                         /* REVISION */
    /* repair_halt: solo per un'immagine col setter del Syntakt */
    if (old == 0 && !e_read(e, 0x400A560Cu, b, 10) && !memcmp(b, setter, 10)
        && e_r32(e, 0x43BDC4BCu) == 1)
        e_w32(e, 0x43BDC4BCu, 0);
}

/* ------------------------------------------- SSI0: _on_serq e il Rytm */
static int ssi_on_serq(dt_engine *e, int size, int64_t v)
{
    if (size == 1 && !(v & 0x80)) {
        if (v & 0x40)
            e->sst.en |= 3;
        else if ((v & 0x3F) == 48)
            e->sst.en |= 1;
        else if ((v & 0x3F) == 50)
            e->sst.en |= 2;
        if (e->sst.en && !e->sst.has_next) {      /* next = now + Fraction(ips, rh) */
            e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
            e->sst.has_next = 1;
        }
    }
    /* ssi.install on_serq: il cablaggio del Rytm (52/54) solo se nulla e' abilitato */
    if (size == 1 && !(v & 0xC0) && ((v & 0x3F) == 52 || (v & 0x3F) == 54) && !e->sst.en)
        return -1;
    return 0;
}

/* harness.mux_write_hook(SERQ): i gestori nell'ordine in cui il Python li aggiunge */
static void h_serq_mux(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a;
    edma_on_serq(e, size, v);                     /* edma.install */
    if (ssi_on_serq(e, size, v) < 0) {            /* Ssi0Dma._on_serq + ssi.install */
        snprintf(e->status, sizeof e->status, "halted: Rytm SSI wiring requested");
        e->m->cnt.halt_vec = 0;
        dtm_stop(e->m);
    }
    sd_on_serq(e, size, v);                       /* Esdhc._on_serq */
    if (e->midi.installed)
        mi_on_serq(e, size, v);                   /* MidiPort._on_serq */
}

/* emu/mchw.py install: I2C0, PIT1 a polling, il pannello a matrice (il
 * motore del plugin e il boot freddo ne hanno bisogno uguale) */
static void mc_board_hooks(dt_engine *e)
{
    e->i2c.sr = I2C_ICF;
    i2c_sync(e);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_i2c_write, e, I2C_CR, I2C_DR + 3);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_i2c_read, e, I2C_CR, I2C_DR + 3);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_pit1_poll, e, PIT1_PCSR, PIT1_PCSR + 1);
    memset(e->pn.leds, 0xFF, sizeof e->pn.leds);
    dtm_ensure(e->m, MC_BASE);
    e_hook_mem(e, UC_HOOK_MEM_WRITE, (void *)h_mc_write, e, MC_BASE, MC_BASE + 0xF);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_mc_read, e, MC_COL, MC_COL + 1);
    e_hook_mem(e, UC_HOOK_MEM_READ, (void *)h_mc_read_adc, e, MC_ADC1, MC_ADC1 + 1);
}
