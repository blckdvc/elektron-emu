/* engine.c, parte 4: eSDHC + eMMC (emu/esdhc.py, la carta di emu/mchw.py con
 * la cancellazione CMD35/36/38) e MIDI (emu/midi.py). Riga per riga; gli
 * eventi asincroni come in emu/nativespin.py (step/service con `_hint`).
 * Dal motore del Digitakt II, con le differenze del Model:Cycles. */

/* ================================================================ eSDHC */
#define SD_BASE 0xFC0CC000u
#define SD_DSADDR 0x00
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
    e_w32(e, SD_BASE + off, v);
}

static uint32_t sd_get(dt_engine *e, uint32_t off)
{
    return e_r32(e, SD_BASE + off);
}

static void sd_post(dt_engine *e, uint32_t sem)
{
    uint8_t b[4];
    if (!sem)
        return;
    dtm_ensure(e->m, sem);
    if (e_read(e, sem, b, 4))
        return;
    if ((int32_t)e_be32(b) <= 0)
        e_w32(e, sem, 1);
}

/* emu/mchw.py card(): Toshiba (MID 0x11) "004GE0" in SLC, 0x3B0000 settori */
static void sd_init(dt_engine *e)
{
    static const uint32_t reset[][2] = {
        {0x00, 0}, {0x04, 0x00010000}, {0x08, 0}, {0x0C, 0}, {0x10, 0}, {0x14, 0}, {0x18, 0},
        {0x1C, 0}, {0x20, 0}, {0x24, 0xFF8800F8u | (1u << 16)}, {0x28, 0x00000020},
        {0x2C, 0x00008008}, {0x30, 0}, {0x34, 0x117F013F}, {0x38, 0}, {0x3C, 0},
        {0x40, 0x07F30000}, {0x44, 0x08100810}, {0x54, 0}, {0x58, 0}, {0xC0, 1},
        {0xFC, 0x00001201}};
    static const uint32_t cid[4] = {0x00000000u, 0x45300000u, 0x30303447u, 0x00110000u};
    static const uint8_t extra[][2] = {{0x9c, 0x00}, {0x9d, 0x01}, {0x9e, 0xd8}, {0xde, 0x01}, {0xe3, 0x08}};
    e_esdhc *sd = &e->sd;
    size_t i;
    memset(sd, 0, sizeof *sd);
    sd->blocks = 0x003B0000;
    sd->ext_csd[0x98] = 1;                               /* SLC_OK */
    e_put32(sd->ext_csd + 0xD4, (uint32_t)sd->blocks);   /* SEC_COUNT (big endian) */
    sd->ext_csd[0xAF] = 1;                               /* ERASE_GROUP_DEF */
    sd->ext_csd[0xB7] = 1;                               /* BUS_WIDTH */
    sd->ext_csd[0xB9] = 1;                               /* HS_TIMING */
    sd->ext_csd[0x9C] = 1;                               /* PARTITIONS_ATTRIBUTE */
    for (i = 0; i < sizeof extra / sizeof extra[0]; i++)  /* mchw.MMC_EXT */
        sd->ext_csd[extra[i][0]] = extra[i][1];
    sd->ocr = 0xC0FF8080u;
    memcpy(sd->cid, cid, sizeof cid);
    sd->armed = -1;
    sd->drv_status = e->p->sd_status;
    sd->cmd_sem = e->p->sd_cmd_sem;
    sd->data_sem = e->p->sd_data_sem;
    sd->dma_sem = e->p->sd_dma_sem;
    dtm_ensure(e->m, SD_BASE);
    for (i = 0; i < sizeof reset / sizeof reset[0]; i++)
        sd_put(e, reset[i][0], reset[i][1]);
}

/* SectorOverlay.erase_sectors(first, last): i settori escono dall'overlay */
static void ov_erase(e_overlay *o, uint64_t first, uint64_t last)
{
    e_overlay n2;
    size_t k;
    if (last < first || !o->n)
        return;
    n2.cap = o->cap;
    n2.n = 0;
    n2.tab = (e_sector *)calloc(n2.cap, sizeof *n2.tab);
    if (!n2.tab)
        return;
    for (k = 0; k < o->cap; k++) {
        e_sector *s = &o->tab[k];
        size_t j;
        if (!s->used)
            continue;
        if (first <= s->sec && s->sec <= last) {
            free(s->data);
            free(s->mask);
            continue;
        }
        for (j = ov_slot(&n2, s->sec); n2.tab[j].used; j = (j + 1) & (n2.cap - 1))
            ;
        n2.tab[j] = *s;
        n2.n++;
    }
    free(o->tab);
    *o = n2;
}

/* Card.erase: CMD38 sull'intervallo CMD35..CMD36 (senza immagine di fondo) */
static void sd_erase(e_esdhc *sd)
{
    uint64_t first, last;
    size_t before = sd->ov.n;
    if (!sd->has_erase_start || !sd->has_erase_end)
        return;
    first = sd->erase_start < sd->erase_end ? sd->erase_start : sd->erase_end;
    last = sd->erase_start < sd->erase_end ? sd->erase_end : sd->erase_start;
    if (last > (uint64_t)sd->blocks - 1)
        last = (uint64_t)sd->blocks - 1;
    ov_erase(&sd->ov, first, last);
    sd->erased += (int64_t)(before - sd->ov.n);
    sd->has_erase_start = sd->has_erase_end = 0;
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

/* Esdhc._on_serq (un gestore del multiplexer di SERQ) */
static void sd_on_serq(dt_engine *e, int size, int64_t v)
{
    (void)size;
    if (!(v & 0x40) && (v & 0x3F) == SD_DMA_CHAN)
        e->sd.armed = SD_DMA_CHAN;
}

static void h_sd_xfertyp(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
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
    case 35: sd->erase_start = arg; sd->has_erase_start = 1; break;     /* ERASE_GROUP_START */
    case 36: sd->erase_end = arg; sd->has_erase_end = 1; break;         /* ERASE_GROUP_END */
    case 38: sd_erase(sd); break;                                        /* ERASE */
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
                    uint32_t dst = e_r32(e, tcd + 0x10);
                    dtm_ensure(e->m, dst);
                    e_write(e, dst, payload, (size_t)plen);
                    e_w32(e, tcd + 0x10, (uint32_t)(dst + plen));
                    e_w16(e, tcd + 0x14, e_r16(e, tcd + 0x1C) & 0x7FFF);
                    e_w16(e, tcd + 0x1E, e_r16(e, tcd + 0x1E) | 0x80);
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
                    uint32_t src = e_r32(e, tcd + 0x00);
                    payload = (uint8_t *)malloc((size_t)nbytes);
                    if (payload) {
                        e_read(e, src, payload, (size_t)nbytes);
                        e_w32(e, tcd + 0x00, (uint32_t)(src + nbytes));
                        e_w16(e, tcd + 0x14, e_r16(e, tcd + 0x1C) & 0x7FFF);
                        e_w16(e, tcd + 0x1E, e_r16(e, tcd + 0x1E) | 0x80);
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
    dtm_ensure(e->m, sd->drv_status);
    e_w32(e, sd->drv_status, 0);
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
    e_midi *mi = &e->midi;
    int lvl;
    int64_t limit, n = 0;
    double per;
    uint32_t base, daddr;
    if (!mi->rx.n)
        return 0;
    lvl = e_irq_level(e, MI_RX_VECTOR, 1);
    if (!lvl)
        return 0;
    if (e_ipl(e) >= lvl)
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
    base = e_r32(e, mi->ring_base_ptr);
    daddr = e_r32(e, MI_TCD36_DADDR);
    while (mi->rx.n && n < limit) {
        uint8_t byte = (uint8_t)e_fifo_pop(&mi->rx);
        e_w8(e, daddr, byte);
        daddr = base + (((daddr - base) + 1) & 0x7FFF);
        n++;
    }
    e_w32(e, MI_TCD36_DADDR, daddr);
    mi->rx_bytes += (int64_t)n;
    return e_raise(e, MI_RX_VECTOR, lvl);
}

static int mi_deliver_uart_irq(dt_engine *e)
{
    e_midi *mi = &e->midi;
    int lvl;
    if (mi->uart_masked || !(mi->uimr & 0x01))
        return 0;
    lvl = e_irq_level(e, MI_UART_VECTOR, 0);
    if (!lvl)
        return 0;
    if (e_ipl(e) >= lvl)
        return 0;
    if (e_raise(e, MI_UART_VECTOR, lvl)) {
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

static int mi_service(int64_t done)
{
    dt_engine *e = e_cur;
    e_midi *mi = &e->midi;
    mi->now = done;
    if (mi->tx.pending)
        tx_deliver(e, &mi->tx);
    else if (!mi_deliver_rx(e))
        mi_deliver_uart_irq(e);
    mi_flush_tx(mi);
    mi_update_hint(mi);
    return 0;
}

/* MidiPort._on_serq (un gestore del multiplexer di SERQ) */
static void mi_on_serq(dt_engine *e, int size, int64_t v)
{
    (void)size;
    if ((v & 0x40) || (v & 0x3F) == 37) {
        e->midi.hint = 1;
        tx_run(e, &e->midi.tx);
    }
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
    e_w8((dt_engine *)ud, MI_USR9, 0x0C);
}


/* midi._find_rx_handler: il gestore RX dal vettore 156 (o l'indirizzo noto),
   riconosciuto dai byte d'apertura; l'operando di `movea.l abs,a0` a +0x26 e'
   il puntatore alla base dell'anello. -> 1 trovato */
static int mi_find_rx_handler(dt_engine *e, uint32_t *ring_base_ptr)
{
    uint32_t cands[2];
    int nc = 0, i;
    uint8_t b[4];
    if (!e_read(e, 0x40000000u + MI_RX_VECTOR * 4u, b, 4))
        cands[nc++] = e_be32(b);
    cands[nc++] = MI_RX_HANDLER;
    for (i = 0; i < nc; i++) {
        uint8_t v[sizeof MI_RX_HANDLER_VERIFY], op[6];
        if (e_read(e, cands[i], v, sizeof v) || memcmp(v, MI_RX_HANDLER_VERIFY, sizeof v))
            continue;
        if (e_read(e, cands[i] + 0x26, op, 6))
            continue;
        if (e_be16(op) == 0x2079) {
            *ring_base_ptr = e_be32(op + 2);
            return 1;
        }
    }
    return 0;
}
