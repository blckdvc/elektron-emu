/* st_engine.c, parte 4: eSDHC + eMMC (emu/esdhc.py), MIDI DIN (emu/midi.py),
 * SpiLink (emu/spilink.py, lato MAIN e lato DSP) e l'hook di SERQ del MAIN con
 * i suoi sette ascoltatori nell'ordine di harness.mux_write_hook. eSDHC e MIDI
 * da digitakt/emulator/core/src/e_periph.inc.c (lo stesso Python, salvo
 * l'overlay salvato a byte e l'anello del MIDI trovato dal vettore 156). */

/* ================================================================ eSDHC */
#define SD_BASE 0xFC0CC000u
#define SD_BLKATTR 0x04
#define SD_CMDARG 0x08
#define SD_XFERTYP 0x0C
#define SD_CMDRSP0 0x10
#define SD_DATPORT 0x20
#define SD_PRSSTAT 0x24
#define SD_SYSCTL 0x2C
#define SD_IRQSTAT 0x30
#define SD_DMA_CHAN 59
#define SD_SECTOR 512

static void ov_free(e_overlay *o)
{
    size_t i;
    for (i = 0; i < o->cap; i++)
        if (o->tab[i].used) {
            free(o->tab[i].data);
            free(o->tab[i].mask);
        }
    free(o->tab);
    memset(o, 0, sizeof *o);
}

static size_t ov_slot(const e_overlay *o, uint64_t sec)
{
    return (size_t)((sec * 0x9E3779B97F4A7C15ull) >> 17) & (o->cap - 1);
}

static e_sector *ov_find(e_overlay *o, uint64_t sec)
{
    size_t i;
    if (!o->cap)
        return NULL;
    for (i = ov_slot(o, sec);; i = (i + 1) & (o->cap - 1)) {
        if (!o->tab[i].used)
            return NULL;
        if (o->tab[i].sec == sec)
            return &o->tab[i];
    }
}

static e_sector *ov_add(e_overlay *o, uint64_t sec)
{
    size_t i;
    if (2 * (o->n + 1) > o->cap) {
        e_overlay n2;
        size_t k;
        n2.cap = o->cap ? 2 * o->cap : 1024;
        n2.n = 0;
        n2.tab = (e_sector *)calloc(n2.cap, sizeof *n2.tab);
        if (!n2.tab)
            return NULL;
        for (k = 0; k < o->cap; k++)
            if (o->tab[k].used) {
                size_t j;
                for (j = ov_slot(&n2, o->tab[k].sec); n2.tab[j].used; j = (j + 1) & (n2.cap - 1))
                    ;
                n2.tab[j] = o->tab[k];
                n2.n++;
            }
        free(o->tab);
        *o = n2;
    }
    for (i = ov_slot(o, sec); o->tab[i].used; i = (i + 1) & (o->cap - 1))
        ;
    o->tab[i].used = 1;
    o->tab[i].sec = sec;
    o->tab[i].data = (uint8_t *)calloc(1, SD_SECTOR);
    o->tab[i].mask = NULL;
    o->tab[i].present = 0;
    o->n++;
    return &o->tab[i];
}

/* SectorOverlay.read_into */
static void ov_read_into(e_overlay *o, uint64_t start, uint8_t *buf, uint64_t n)
{
    uint64_t first, last, sec;
    if (!n)
        return;
    first = start / SD_SECTOR;
    last = (start + n - 1) / SD_SECTOR;
    for (sec = first; sec <= last; sec++) {
        e_sector *s = ov_find(o, sec);
        uint64_t base, lo, hi, a;
        if (!s)
            continue;
        base = sec * SD_SECTOR;
        lo = start > base ? start : base;
        hi = start + n < base + SD_SECTOR ? start + n : base + SD_SECTOR;
        if (!s->mask)
            memcpy(buf + (lo - start), s->data + (lo - base), (size_t)(hi - lo));
        else
            for (a = lo; a < hi; a++)
                if (s->mask[a - base])
                    buf[a - start] = s->data[a - base];
    }
}

/* SectorOverlay.__setitem__ */
static void ov_set_byte(e_overlay *o, uint64_t addr, uint8_t v)
{
    uint64_t sec = addr / SD_SECTOR, off = addr % SD_SECTOR;
    e_sector *s = ov_find(o, sec);
    if (!s) {
        s = ov_add(o, sec);
        if (!s)
            return;
        s->mask = (uint8_t *)calloc(1, SD_SECTOR);
        s->present = 0;
    }
    if (s->mask && !s->mask[off]) {
        s->mask[off] = 1;
        if (++s->present == SD_SECTOR) {
            free(s->mask);
            s->mask = NULL;
        }
    }
    s->data[off] = v;
}

/* SectorOverlay.write_bytes */
static void ov_write(e_overlay *o, uint64_t start, const uint8_t *p, uint64_t n)
{
    uint64_t i;
    if (start % SD_SECTOR == 0 && n % SD_SECTOR == 0) {
        for (i = 0; i < n / SD_SECTOR; i++) {
            uint64_t sec = start / SD_SECTOR + i;
            e_sector *s = ov_find(o, sec);
            if (!s)
                s = ov_add(o, sec);
            if (!s)
                return;
            free(s->mask);
            s->mask = NULL;
            memcpy(s->data, p + i * SD_SECTOR, SD_SECTOR);
        }
        return;
    }
    for (i = 0; i < n; i++)
        ov_set_byte(o, start + i, p[i]);
}

static void sd_put(dt_engine *e, uint32_t off, uint32_t v)
{
    c_w32(MAINC(e), SD_BASE + off, v);
}

static uint32_t sd_get(dt_engine *e, uint32_t off)
{
    return c_r32(MAINC(e), SD_BASE + off);
}

static void sd_post(dt_engine *e, uint32_t sem)
{
    uint8_t b[4];
    st_cpu *c = MAINC(e);
    if (!sem)
        return;
    dtm_ensure(c->m, sem);
    if (c_read(c, sem, b, 4))
        return;
    if ((int32_t)e_be32(b) <= 0)
        c_w32(c, sem, 1);
}

static void sd_init(dt_engine *e)
{
    static const uint32_t reset[][2] = {
        {0x00, 0}, {0x04, 0x00010000}, {0x08, 0}, {0x0C, 0}, {0x10, 0}, {0x14, 0}, {0x18, 0},
        {0x1C, 0}, {0x20, 0}, {0x24, 0xFF8800F8u | (1u << 16)}, {0x28, 0x00000020},
        {0x2C, 0x00008008}, {0x30, 0}, {0x34, 0x117F013F}, {0x38, 0}, {0x3C, 0},
        {0x40, 0x07F30000}, {0x44, 0x08100810}, {0x54, 0}, {0x58, 0}, {0xC0, 1},
        {0xFC, 0x00001201}};
    e_esdhc *sd = &e->sd;
    size_t i;
    memset(sd, 0, sizeof *sd);
    sd->blocks = 0x00800000;
    sd->ext_csd[0x98] = 1;                               /* SLC_OK */
    e_put32(sd->ext_csd + 0xD4, (uint32_t)sd->blocks);   /* SEC_COUNT */
    sd->ext_csd[0xAF] = 1;
    sd->ext_csd[0xB7] = 1;
    sd->ext_csd[0xB9] = 1;
    sd->ext_csd[0x9C] = 1;
    sd->ocr = 0xC0FF8080u;
    sd->cid[3] = 0x00110000u;
    sd->armed = -1;
    sd->drv_status = e->p->sd_status;
    sd->cmd_sem = e->p->sd_cmd_sem;
    sd->data_sem = e->p->sd_data_sem;
    sd->dma_sem = e->p->sd_dma_sem;
    dtm_ensure(MAINC(e)->m, SD_BASE);
    for (i = 0; i < sizeof reset / sizeof reset[0]; i++)
        sd_put(e, reset[i][0], reset[i][1]);
}

static void h_sd_sysctl(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint32_t cur = sd_get(e, SD_SYSCTL), bits = (1u << 24) | (1u << 25) | (1u << 26) | (1u << 27);
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    if (cur & bits)
        sd_put(e, SD_SYSCTL, cur & ~bits);
}

static void h_sd_datport(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    ((dt_engine *)ud)->sd.pattern = (uint32_t)v;
}

static void h_sd_xfertyp(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    st_cpu *c = MAINC(e);
    e_esdhc *sd = &e->sd;
    uint32_t xfer = size == 4 ? (uint32_t)v : sd_get(e, SD_XFERTYP);
    uint32_t idx = (xfer >> 24) & 0x3F, arg = sd_get(e, SD_CMDARG);
    uint32_t r[4] = {0x00000900u, 0, 0, 0};
    (void)uc; (void)ty; (void)a;
    sd->log_n++;
    switch (idx) {                                   /* Card.command */
    case 0: sd->selected = 0; r[0] = 0; break;
    case 1: r[0] = sd->ocr; break;
    case 2: case 9: case 10:
        memcpy(r, idx == 9 ? sd->csd : sd->cid, sizeof r);
        break;
    case 3: sd->rca = (arg >> 16) & 0xFFFF; break;
    case 7: sd->selected = ((arg >> 16) & 0xFFFF) == sd->rca; break;
    default: break;
    }
    sd_put(e, SD_CMDRSP0, r[0]);
    sd_put(e, SD_CMDRSP0 + 4, r[1]);
    sd_put(e, SD_CMDRSP0 + 8, r[2]);
    sd_put(e, SD_CMDRSP0 + 12, r[3]);
    sd_put(e, SD_PRSSTAT, sd_get(e, SD_PRSSTAT) & ~(1u | 2u | 4u));
    sd_put(e, SD_IRQSTAT, sd_get(e, SD_IRQSTAT) | 1u | 2u);
    if (idx == 6)
        sd_put(e, SD_IRQSTAT, sd_get(e, SD_IRQSTAT) | (1u << 24));
    if (xfer & (1u << 21)) {                                         /* DPSEL */
        uint64_t nbytes = (uint64_t)((sd_get(e, SD_BLKATTR) >> 16) & 0xFFFF) * 512;
        if (xfer & (1u << 4)) {                                      /* DTDSEL: card -> host */
            uint8_t *payload = NULL;
            uint64_t plen = 0;
            int have = 0;
            sd_put(e, SD_PRSSTAT, sd_get(e, SD_PRSSTAT) | (1u << 11));
            sd_put(e, SD_IRQSTAT, sd_get(e, SD_IRQSTAT) | (1u << 5));
            sd_put(e, SD_DATPORT, idx == 14 ? ~sd->pattern : 0);
            if (idx == 8) {
                payload = sd->ext_csd;
                plen = 512;
                have = 1;
            } else if (idx == 18) {
                payload = (uint8_t *)calloc(1, nbytes ? (size_t)nbytes : 1);
                plen = nbytes;
                have = payload != NULL;
                if (have)
                    ov_read_into(&sd->ov, (uint64_t)arg * 512, payload, plen);
            }
            if (have && sd->armed >= 0) {                            /* _dma_out */
                uint32_t tcd = E_TCD_BASE + (uint32_t)sd->armed * 0x20u;
                if (plen) {
                    uint32_t dst = c_r32(c, tcd + 0x10);
                    dtm_ensure(c->m, dst);
                    c_write(c, dst, payload, (size_t)plen);
                    c_w32(c, tcd + 0x10, (uint32_t)(dst + plen));
                    c_w16(c, tcd + 0x14, c_r16(c, tcd + 0x1C) & 0x7FFF);
                    c_w16(c, tcd + 0x1E, c_r16(c, tcd + 0x1E) | 0x80);
                    sd->dma_bytes += (int64_t)plen;
                }
                sd->armed = -1;
                sd_post(e, sd->dma_sem);
            }
            if (idx == 18)
                free(payload);
            sd_post(e, sd->data_sem);
        } else {                                                     /* host -> card */
            sd_put(e, SD_PRSSTAT, sd_get(e, SD_PRSSTAT) | (1u << 10));
            sd_put(e, SD_IRQSTAT, sd_get(e, SD_IRQSTAT) | (1u << 4));
            if (sd->armed >= 0) {
                uint8_t *payload = NULL;
                uint32_t tcd = E_TCD_BASE + (uint32_t)sd->armed * 0x20u;
                if (nbytes) {                                        /* _dma_in */
                    uint32_t src = c_r32(c, tcd + 0x00);
                    payload = (uint8_t *)malloc((size_t)nbytes);
                    if (payload) {
                        c_read(c, src, payload, (size_t)nbytes);
                        c_w32(c, tcd + 0x00, (uint32_t)(src + nbytes));
                        c_w16(c, tcd + 0x14, c_r16(c, tcd + 0x1C) & 0x7FFF);
                        c_w16(c, tcd + 0x1E, c_r16(c, tcd + 0x1E) | 0x80);
                        sd->dma_bytes += (int64_t)nbytes;
                    }
                }
                if (idx == 25 && payload)                            /* Card.write_data */
                    ov_write(&sd->ov, (uint64_t)arg * 512, payload, nbytes);
                free(payload);
                sd->armed = -1;
                sd_post(e, sd->dma_sem);
            }
            sd_post(e, sd->data_sem);
        }
    }
    dtm_ensure(c->m, sd->drv_status);
    c_w32(c, sd->drv_status, 0);
    sd_post(e, sd->cmd_sem);
}

/* ================================================================ MIDI */
#define MI_RX_VECTOR 156
#define MI_TCD36_DADDR 0xFC045490u
#define MI_RX_HANDLER 0x400021A0u
static const uint8_t MI_RX_HANDLER_VERIFY[] = {0x46, 0xfc, 0x27, 0x00, 0x4f, 0xef, 0xff, 0xf0, 0x48, 0xd7,
                                               0x03, 0x03, 0x70, 0x24, 0x13, 0xc0, 0xfc, 0x04, 0x40, 0x1c};
#define MI_RETRY 8000
#define MI_USR9 0xEC074004u
#define MI_UDR9 0xEC07400Cu
#define MI_UIMR9 0xEC074014u
#define MI_UART_VECTOR 181
#define MI_UART_SOURCE 53
#define MI_SIMR1 0xFC04C01Cu
#define MI_CIMR1 0xFC04C01Du

/* midi._find_rx_handler: -> 1 e il puntatore all'anello, 0 se non c'e' */
static int mi_find_rx_handler(st_cpu *c, uint32_t *ring_ptr)
{
    uint32_t cands[2], h;
    uint8_t b[sizeof MI_RX_HANDLER_VERIFY];
    int n = 0, i;
    if (!c_read(c, DT_VBR + MI_RX_VECTOR * 4u, b, 4))
        cands[n++] = e_be32(b);
    cands[n++] = MI_RX_HANDLER;
    for (i = 0; i < n; i++) {
        uint8_t op[6];
        h = cands[i];
        if (c_read(c, h, b, sizeof b) || memcmp(b, MI_RX_HANDLER_VERIFY, sizeof b))
            continue;
        if (c_read(c, h + 0x26, op, 6))
            continue;
        if (e_be16(op) == 0x2079) {                      /* movea.l (abs).l, a0 */
            *ring_ptr = e_be32(op + 2);
            return 1;
        }
    }
    return 0;
}

static double mi_per_byte(e_midi *mi)
{
    return mi->ips ? (double)mi->ips / 3125.0 : 0.0;
}

static void mi_update_hint(e_midi *mi)
{
    mi->hint = 0;
    if (mi->rx.n || mi->tx_out.n || (!mi->uart_masked && (mi->uimr & 0x01)))
        mi->hint = 1;
}

static int mi_deliver_rx(dt_engine *e)
{
    st_cpu *c = MAINC(e);
    e_midi *mi = &e->midi;
    int lvl;
    int64_t limit, n = 0;
    double per;
    uint32_t base, daddr;
    if (!mi->rx.n)
        return 0;
    lvl = c_irq_level(c, MI_RX_VECTOR, 1);
    if (!lvl)
        return 0;
    if (c_ipl(c) >= lvl)
        return 0;
    limit = (int64_t)mi->rx.n;
    per = mi_per_byte(mi);
    if (per != 0.0) {
        double now = (double)mi->now;
        int64_t k;
        if (!mi->has_rx_at || mi->rx_at < now - per) {
            mi->rx_at = now - per;               /* dopo una pausa: un byte subito */
            mi->has_rx_at = 1;
        }
        k = (int64_t)py_floordiv(now - mi->rx_at, per);
        if (k < limit)
            limit = k;
        if (limit <= 0)
            return 0;
        mi->rx_at += (double)limit * per;
    }
    base = c_r32(c, mi->ring_base_ptr);
    daddr = c_r32(c, MI_TCD36_DADDR);
    while (mi->rx.n && n < limit) {
        uint8_t byte = (uint8_t)e_fifo_pop(&mi->rx);
        c_w8(c, daddr, byte);
        daddr = base + (((daddr - base) + 1) & 0x7FFF);
        n++;
    }
    c_w32(c, MI_TCD36_DADDR, daddr);
    mi->rx_bytes += (int64_t)n;
    return c_raise(c, MI_RX_VECTOR, lvl);
}

static int mi_deliver_uart_irq(dt_engine *e)
{
    st_cpu *c = MAINC(e);
    e_midi *mi = &e->midi;
    int lvl;
    if (mi->uart_masked || !(mi->uimr & 0x01))
        return 0;
    lvl = c_irq_level(c, MI_UART_VECTOR, 0);
    if (!lvl)
        return 0;
    if (c_ipl(c) >= lvl)
        return 0;
    if (c_raise(c, MI_UART_VECTOR, lvl)) {
        mi->uart_irqs++;
        return 1;
    }
    return 0;
}

static void mi_flush_tx(e_midi *mi)
{
    if (!mi->tx_out.n)
        return;
    mi->tx_sent += (int64_t)mi->tx_out.n;
    e_buf_put(&mi->host_out, mi->tx_out.p, mi->tx_out.n);    /* verso l'host */
    mi->tx_out.n = 0;
}

/* NativeSpin._midi_step: MidiPort.step(now, None) */
static int64_t mi_step(int64_t done)
{
    e_midi *mi = &e_cur->midi;
    int busy = mi->rx.n || mi->tx.pending || (!mi->uart_masked && (mi->uimr & 0x01));
    double per;
    if (!busy)
        return -1;
    per = mi_per_byte(mi);
    if (mi->rx.n && per != 0.0 && mi->has_rx_at && !mi->tx.pending) {
        int64_t wait = (int64_t)(mi->rx_at + per - (double)done);
        if (wait > MI_RETRY)
            return wait;
    }
    return MI_RETRY;
}

/* NativeSpin._midi_service: MidiPort.service(now) e _update_hint */
static int mi_service(int64_t done)
{
    dt_engine *e = e_cur;
    e_midi *mi = &e->midi;
    mi->now = done;
    if (mi->tx.pending)
        tx_deliver(MAINC(e), &mi->tx);
    else if (!mi_deliver_rx(e))
        mi_deliver_uart_irq(e);
    mi_flush_tx(mi);
    mi_update_hint(mi);
    return 0;
}

static void h_mi_udr(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t byte = (uint8_t)(v & 0xFF);
    (void)uc; (void)ty; (void)a; (void)size;
    e_buf_put(&e->midi.tx_out, &byte, 1);
    e->midi.tx_out_total++;
    e->midi.hint = 1;
}

static void h_mi_uimr(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a; (void)size;
    e->midi.uimr = (uint8_t)(v & 0xFF);
    if (e->midi.uimr & 0x01)
        e->midi.hint = 1;
}

static void h_mi_imr(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    int off;
    (void)uc; (void)ty;
    for (off = 0; off < size; off++) {
        uint64_t ad = a + (uint64_t)off;
        uint32_t b = (uint32_t)(((uint64_t)v >> (8 * (size - 1 - off))) & 0xFF);
        if ((b & 0x40) || (b & 0x3F) == MI_UART_SOURCE) {
            if (ad == MI_SIMR1)
                e->midi.uart_masked = 1;
            else if (ad == MI_CIMR1) {
                e->midi.uart_masked = 0;
                e->midi.hint = 1;
            }
        }
    }
}

static void h_mi_usr(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    c_w8(MAINC((dt_engine *)ud), MI_USR9, 0x0C);
}

/* ============================================================ SpiLink */
#define SL_TCD29 (E_TCD_BASE + 29u * 0x20u)
#define SL_DSPI0 0xFC05C000u
#define SL_FIFO_DEPTH 16
#define SL_IDLE 0xFF

/* SpiLink._refill */
static void sl_refill(dt_engine *e)
{
    while (e->link_pending.n && e->link_main_tx.n < SL_FIFO_DEPTH) {
        uint8_t b = (uint8_t)e_fifo_pop(&e->link_pending);
        e_fifo_push(&e->link_main_tx, &b, 1);
    }
    if (e->link_dma_active && !e->link_pending.n) {
        e->link_dma_active = 0;
        e->link_complete = 1;
    }
}

/* SpiLink._serq (MAIN): (value & 0x7F) == 29, senza controllo della misura */
static void sl_serq(dt_engine *e, int64_t value)
{
    st_cpu *c = MAINC(e);
    uint32_t saddr, nbytes, citer, n;
    uint8_t *buf;
    if ((value & 0x7F) != 29)
        return;
    saddr = c_r32(c, SL_TCD29 + 0x00);
    nbytes = c_r32(c, SL_TCD29 + 0x08);
    citer = c_r16(c, SL_TCD29 + 0x16) & 0x7FFF;          /* r16(0x16): cosi' nel Python */
    n = nbytes * (citer ? citer : 1);
    buf = (uint8_t *)malloc(n ? n : 1);
    if (buf) {
        c_read(c, saddr, buf, n);
        e_fifo_push(&e->link_pending, buf, n);
        free(buf);
    }
    e->link_dma_active = 1;
    e->link_saddr = saddr;
    e->link_n = n;
    sl_refill(e);
}

/* SpiLink.deliver (fra le fette): -> 1 se c'era un trasferimento concluso */
static int sl_deliver(dt_engine *e)
{
    st_cpu *c = MAINC(e);
    uint16_t csr;
    if (!e->link_complete)
        return 0;
    e->link_complete = 0;
    csr = c_r16(c, SL_TCD29 + 0x1E);
    c_w16(c, SL_TCD29 + 0x1E, (uint16_t)((csr | 0x80) & ~0x40));
    c_w32(c, SL_TCD29 + 0x00, e->link_saddr + e->link_n);
    if (c_raise(c, 149, -1))
        e->link_dma_done++;
    return 1;
}

/* lato DSP: PUSHR (scrittura), SR e POPR (letture) del DSPI0 */
static void h_sl_push(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t r = (uint8_t)(v & 0xFF), b;
    (void)uc; (void)ty; (void)a; (void)size;
    e->link_received++;
    e->link_received_crc = e_crc(e->link_received_crc, &r, 1);
    if (e->link_main_tx.n) {
        b = (uint8_t)e_fifo_pop(&e->link_main_tx);
        e->link_sent++;
        e->link_sent_crc = e_crc(e->link_sent_crc, &b, 1);
        sl_refill(e);
    } else
        b = SL_IDLE;
    e_fifo_push(&e->link_dsp_rx, &b, 1);
}

static void h_sl_sr(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint32_t rxctr = e->link_dsp_rx.n < 15 ? (uint32_t)e->link_dsp_rx.n : 15;
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    c_w32(DSPC(e), SL_DSPI0 + 0x2C, 0x80000000u | 0x10000000u | (rxctr << 4));
}

static void h_sl_pop(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    int b = e_fifo_pop(&e->link_dsp_rx);
    (void)uc; (void)ty; (void)a; (void)size; (void)v;
    c_w32(DSPC(e), SL_DSPI0 + 0x38, b < 0 ? SL_IDLE : (uint32_t)b);
}

/* ============================================== SERQ del MAIN (mux) */
/* harness.mux_write_hook: un hook, gli ascoltatori nell'ordine in cui il
   Python li aggiunge: edma (35), Ssi0Dma._on_serq, ssi.install on_serq (il
   riconoscimento del Rytm: gia' fatto), esdhc, MIDI, FpgaPort, SpiLink */
static void h_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    st_cpu *c = MAINC(e);
    (void)uc; (void)ty; (void)a;
    /* edma.install on_serq */
    if ((v & 0x40) || (uint32_t)(v & 0x3F) == e->etx.chan)
        tx_run(c, &e->etx);
    /* Ssi0Dma._on_serq (non nel motore freddo: li' l'SSI non c'e') */
    if (!e->cold && size == 1 && !(v & 0x80)) {
        if (v & 0x40)
            e->sst.en |= 3;
        else if ((v & 0x3F) == e->sst.rx_ch)
            e->sst.en |= 1;
        else if ((v & 0x3F) == e->sst.tx_ch)
            e->sst.en |= 2;
        if (e->sst.en && !e->sst.has_next) {        /* next = now + Fraction(ips, rh) */
            e->sst.nextN = e->sst.now * e->sst.rh + e->sst.ips;
            e->sst.has_next = 1;
        }
    }
    /* esdhc */
    if (!(v & 0x40) && (v & 0x3F) == SD_DMA_CHAN)
        e->sd.armed = SD_DMA_CHAN;
    /* MIDI */
    if (e->midi.installed && ((v & 0x40) || (v & 0x3F) == 37)) {
        e->midi.hint = 1;
        tx_run(c, &e->midi.tx);
    }
    /* FpgaPort */
    if (!e->cold)
        fpga_serq(e, size, v);
    /* SpiLink */
    sl_serq(e, v);
}
