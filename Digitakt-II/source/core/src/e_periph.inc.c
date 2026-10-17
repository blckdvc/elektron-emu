/* engine.c, parte 4 (comune): eSDHC + eMMC (emu/esdhc.py), MIDI DIN
 * (emu/midi.py), +Drive via RPC (emu/transfer.py, emu/guestcall.py). Riga per riga; gli eventi asincroni come in
 * emu/nativespin.py (step/service con `_hint`). */

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
    sd->drv_status = e->p->sd_status ? e->p->sd_status : 0x44E26F1Cu;
    sd->cmd_sem = e->p->sd_cmd_sem;
    sd->data_sem = e->p->sd_data_sem;
    sd->dma_sem = e->p->sd_dma_sem;
    dtm_ensure(e->m, SD_BASE);
    for (i = 0; i < sizeof reset / sizeof reset[0]; i++)
        sd_put(e, reset[i][0], reset[i][1]);
}

/* La eMMC del Digitakt II (FINDINGS i212). Al boot 0x400cc864 controlla la
   carta (0x4012dc80): cerca produttore (CID MID) e nome (CID PNM) nella tabella
   delle eMMC supportate del firmware (0x402b4a24, 7 voci da 0x20 byte), poi
   confronta MAX_ENH_SIZE_MULT, HC_WP_GRP_SIZE, HC_ERASE_GRP_SIZE e SEC_COUNT con
   la voce. Con la carta di sd_init (MID 0x11, nome vuoto) risponde -2, il codice
   di stato [0x42940a48] diventa 7 e il +Drive non si monta ne' si formatta mai.
   Qui la voce 0x402b4ae4 (MID 0x70 "TS0A32", la carta da 32 GB in modo SLC):
   i campi dell'EXT_CSD sono nell'ordine in cui il firmware li legge dal suo
   buffer (parole a 32 bit scambiate: 0x98 = PARTITION_SETTING_COMPLETED,
   0x9c..0x9e = MAX_ENH_SIZE_MULT, 0xde = HC_WP_GRP_SIZE, 0xe3 =
   HC_ERASE_GRP_SIZE, 0xd4 = SEC_COUNT big endian). */
#define SD_DT2_BLOCKS 0x02E1A000
static void sd_identity_dt2(e_esdhc *sd)
{
    sd->blocks = SD_DT2_BLOCKS;
    e_put32(sd->ext_csd + 0xD4, (uint32_t)sd->blocks);   /* SEC_COUNT (voce +0x1c) */
    sd->ext_csd[0x98] = 1;                               /* gia' configurata (SLC) */
    sd->ext_csd[0x9C] = 0x00;                            /* MAX_ENH_SIZE_MULT 0x0004db (voce +8) */
    sd->ext_csd[0x9D] = 0x04;
    sd->ext_csd[0x9E] = 0xDB;
    sd->ext_csd[0xDE] = 0x10;                            /* voce +0xc */
    sd->ext_csd[0xE3] = 0x01;                            /* voce +0x10 */
    sd->cid[3] = 0x00700000u;                            /* MID 0x70 */
    sd->cid[2] =((uint32_t)'T' << 24) | ((uint32_t)'S' << 16) | ((uint32_t)'0' << 8) | 'A';
    sd->cid[1] = ((uint32_t)'3' << 24) | ((uint32_t)'2' << 16);   /* "32", PRV 0, PSN 0 */
    sd->cid[0] = 0;
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

static void h_sd_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    (void)uc; (void)ty; (void)a; (void)size;
    if (!(v & 0x40) && (v & 0x3F) == SD_DMA_CHAN)
        ((dt_engine *)ud)->sd.armed = SD_DMA_CHAN;
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
#define MI_RING_BASE_PTR 0x40964D40u
#define MI_RX_HANDLER 0x400021A0u
static const uint8_t MI_RX_HANDLER_VERIFY[] = {0x46, 0xfc, 0x27, 0x00, 0x4f, 0xef, 0xff, 0xf0, 0x48, 0xd7,
                                               0x03, 0x03, 0x70, 0x24, 0x13, 0xc0, 0xfc, 0x04, 0x40, 0x1c};
#define MI_ROUTER_SEM 0x44E15BCCu
#define MI_ROUTER_PEND 0x4011EE1Au
static const uint8_t MI_ROUTER_VERIFY[] = {0x48, 0x79, 0x44, 0xe1, 0x5b, 0xcc, 0x28, 0x3c, 0x40, 0x11, 0xd5, 0x64};
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
    base = e_r32(e, MI_RING_BASE_PTR);
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

static void h_mi_serq(uc_engine *uc, uc_mem_type ty, uint64_t a, int size, int64_t v, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    (void)uc; (void)ty; (void)a; (void)size;
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

/* ========================================================== +Drive (RPC) */
#define TR_QUEUE 0x44E4AF40u
#define TR_ENQUEUE 0x40001896u
#define TR_USB_SEND 0x4000501Eu
static const uint8_t TR_USB_SEND_VERIFY[] = {0x4f, 0xef, 0xff, 0xec, 0x70, 0x01, 0x48, 0xd7, 0x04,
                                             0x3c, 0x26, 0x2f, 0x00, 0x18, 0x28, 0x2f, 0x00, 0x1c};
static const uint8_t TR_ENQUEUE_BYTES[] = {0x2f, 0x0a, 0x2f, 0x02, 0x20, 0x6f, 0x00, 0x0c, 0x40, 0xc2,
                                           0x46, 0xfc, 0x27, 0x00};
#define TR_PAGE 0x4F100000u
#define TR_RETRY 20000

static void h_tr_usb_send(uc_engine *uc, uint64_t a, uint32_t s, void *ud)
{
    dt_engine *e = (dt_engine *)ud;
    uint8_t b[12];
    uint32_t sp = e_rd(e, UC_M68K_REG_A7), ret, n, buf;
    (void)uc; (void)a; (void)s;
    e_read(e, sp, b, 12);
    ret = e_be32(b); n = e_be32(b + 4); buf = e_be32(b + 8);
    {                                     /* RpcLink.replies: [u32 n][raw] in coda (dt_engine_rpc_replies) */
        uint8_t *raw = (n > 0 && n <= 0x10000) ? (uint8_t *)malloc(n) : NULL;
        uint32_t len = 0;
        if (raw && !e_read(e, buf, raw, n))
            len = n;
        e_buf_put(&e->tr.last_reply, &len, 4);
        if (len)
            e_buf_put(&e->tr.last_reply, raw, len);
        free(raw);
    }
    e->tr.replies++;
    e_wr(e, UC_M68K_REG_A7, sp + 4);
    e_wr(e, UC_M68K_REG_PC, ret);
}

/* emu/guestcall.py make(...)(entry, args, count, mask_irq) */
static uint32_t guest_call(dt_engine *e, uint32_t entry, const uint32_t *args, int nargs, uint32_t count,
                           int mask_irq)
{
    static const int regs[17] = {UC_M68K_REG_SR, UC_M68K_REG_D0, UC_M68K_REG_D1, UC_M68K_REG_D2,
                                 UC_M68K_REG_D3, UC_M68K_REG_D4, UC_M68K_REG_D5, UC_M68K_REG_D6,
                                 UC_M68K_REG_D7, UC_M68K_REG_A0, UC_M68K_REG_A1, UC_M68K_REG_A2,
                                 UC_M68K_REG_A3, UC_M68K_REG_A4, UC_M68K_REG_A5, UC_M68K_REG_A6,
                                 UC_M68K_REG_A7};
    uint32_t saved[17], saved_pc, sentinel = TR_PAGE + 0x800, stack = TR_PAGE + 0x2000, sp, d0;
    uint32_t need = 4u * (1u + (uint32_t)nargs);
    uint8_t probe[64], frame[64];
    int i;
    int64_t left;
    for (i = 0; i < 17; i++)
        saved[i] = e_rd(e, regs[i]);
    saved_pc = e_rd(e, UC_M68K_REG_PC);
    if (mask_irq)
        e_wr(e, UC_M68K_REG_SR, saved[0] | 0x0700);
    sp = saved[16] - need;
    if (e_read(e, sp, probe, need))
        sp = stack - 0x100;
    e_put32(frame, sentinel);
    for (i = 0; i < nargs; i++)
        e_put32(frame + 4 + 4 * i, args[i]);
    e_write(e, sp, frame, need);
    e_wr(e, UC_M68K_REG_A7, sp);
    e->m->cnt.halt_vec = -1;
    left = (int64_t)(count / 4) + 10000;
    e->stp.left = left < 1 ? 1 : left;
    uc_emu_start(e->uc, entry, sentinel, 0, count);
    d0 = e_rd(e, UC_M68K_REG_D0);
    e->stp.left = 0;
    for (i = 0; i < 16; i++)
        e_wr(e, regs[i], saved[i]);
    e_wr(e, UC_M68K_REG_A7, saved[16]);
    e_wr(e, UC_M68K_REG_PC, saved_pc);
    return d0;
}

static int64_t tr_step(int64_t done)
{
    (void)done;
    return e_cur->tr.n ? TR_RETRY : -1;
}

static int tr_service(int64_t done)
{
    dt_engine *e = e_cur;
    e_transfer *t = &e->tr;
    e_msg msg;
    uint32_t pc, base, mbuf, mmsg, args[2];
    uint8_t *data, hdr[32];
    size_t dn;
    (void)done;
    if (t->n) {
        pc = e_rd(e, UC_M68K_REG_PC);
        if (e_in(e->idle_addrs, e->nidle, pc)) {
            msg = t->q[t->head];
            t->head = (t->head + 1) % t->cap;
            t->n--;
            dn = 0x1c + msg.n;                               /* transfer.frame */
            data = (uint8_t *)calloc(1, dn);
            if (data) {
                data[0] = 0xE2; data[1] = 0x02; data[0x0E] = 1; data[0x0F] = 0;
                e_put32(data + 0x12, 0x80000000u | (uint32_t)msg.n);
                e_put16(data + 0x16, (uint16_t)msg.n);
                memcpy(data + 0x1c, msg.data, msg.n);
                base = TR_PAGE + 0x4000 + (uint32_t)(t->slot % 8) * 0x4000;
                t->slot++;
                mmsg = base;
                mbuf = base + 0x100;
                e_write(e, mbuf, data, dn);
                memset(hdr, 0, sizeof hdr);
                hdr[0] = 0x25;
                e_put32(hdr + 4, (uint32_t)dn);
                e_put32(hdr + 8, mbuf);
                e_write(e, mmsg, hdr, 32);
                args[0] = TR_QUEUE;
                args[1] = mmsg;
                guest_call(e, TR_ENQUEUE, args, 2, 20000, 1);
                t->sent++;
                free(data);
            }
            free(msg.data);
        }
    }
    t->hint = t->n ? 1 : 0;
    return 0;
}
