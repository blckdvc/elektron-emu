/* engine.c, parte 7: salvare lo stato del motore (DTST v2, lo stesso formato di
 * tools/snap2dtst.py) -- come Emulator._save_session -> checkpoint.save_longrun
 * del Python: pagine, registri, EMAC, MOVEC, MMIO, contatori e i componenti
 * (uart_in, edma_tx, ssi0_dma, esdhc, timers) e quelli del Rytm che il Python
 * tiene fuori dallo snapshot: la flash SPI (out/rytm_nor.npz: CNOR) e i
 * registri dei LED (SESSION.leds.json: CLED). Da chiamare fra un giro e
 * l'altro, sul thread del motore. Dal Digitakt II e dal Model:Cycles. */

static int w_le32(e_buf *b, uint32_t v)
{
    uint8_t x[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    return e_buf_put(b, x, 4);
}

static int w_le64(e_buf *b, uint64_t v)
{
    return w_le32(b, (uint32_t)v) || w_le32(b, (uint32_t)(v >> 32));
}

static int w_u8(e_buf *b, uint8_t v)
{
    return e_buf_put(b, &v, 1);
}

static int w_f64(e_buf *b, double d)
{
    uint64_t v;
    memcpy(&v, &d, 8);
    return w_le64(b, v);
}

/* un record: tag, flag 0, lunghezza, dati */
static int w_rec(e_buf *out, uint32_t *nrec, const char *tag, const e_buf *body)
{
    if (e_buf_put(out, tag, 4) || w_le32(out, 0) || w_le64(out, body->n)
        || (body->n && e_buf_put(out, body->p, body->n)))
        return -1;
    (*nrec)++;
    return 0;
}

static int64_t gcd64(int64_t a, int64_t b)
{
    if (a < 0) a = -a;
    if (b < 0) b = -b;
    while (b) {
        int64_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

static int cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : x > y;
}

/* spazio per altri n byte (senza scriverli) */
static int st_reserve(e_buf *b, size_t n)
{
    if (b->n + n > b->cap) {
        size_t cap = b->cap ? b->cap : 4096;
        uint8_t *q;
        while (cap < b->n + n)
            cap *= 2;
        q = (uint8_t *)realloc(b->p, cap);
        if (!q)
            return -1;
        b->p = q;
        b->cap = cap;
    }
    return 0;
}

/* Il salvataggio in due fasi (rt.c): dt_engine_save_raw sul thread del motore e'
 * solo una fotografia -- le pagine di memoria copiate grezze in record "PAGR" --
 * e dt_state_finish, su qualunque thread, le comprime (zlib 6, salta quelle a
 * zero) nei "PAGE" del DTST e fa il DTSZ. Il risultato e' byte per byte quello di
 * prima; il motore si ferma per la copia (decine di ms), non per la compressione
 * (un buco nell'audio a ogni salvataggio). Dal Digitakt II. */
int dt_engine_save_raw(dt_engine *e, uint8_t **out_p, size_t *out_n, size_t *cap_io)
{
    /* out_p e cap_io: un buffer da riusare (gia' toccato: niente pagine nuove da
       azzerare ne' realloc a ogni salvataggio), o NULL/0 */
    e_buf out = {*out_p, 0, *cap_io}, b = {0};
    uint32_t nrec = 0, regs[18], *bases = NULL, nb = 0, i;
    uc_mem_region *rg = NULL;
    int rc = DT_ERR_NOMEM, k;
    dt_machine *m = e->m;

    *out_n = 0;
    if (e_buf_put(&out, "DTST", 4) || w_le32(&out, 1) || w_le32(&out, 0) || w_le32(&out, 0))
        goto fail;
    /* REGS (PC: quello del giro, come _save_session) */
    e_wr(e, UC_M68K_REG_PC, e->pc);
    dt_machine_regs(m, regs);
    b.n = 0;
    for (k = 0; k < 18; k++)
        w_le32(&b, regs[k]);
    if (w_rec(&out, &nrec, "REGS", &b))
        goto fail;
    /* EMAC: macsr, mask, acc0-3 (snapshot.EMAC_REGS) */
    b.n = 0;
    {
        static const int ids[6] = {UC_M68K_REG_MACSR, UC_M68K_REG_MAC_MASK, UC_M68K_REG_MACC0,
                                   UC_M68K_REG_MACC1, UC_M68K_REG_MACC2, UC_M68K_REG_MACC3};
        for (k = 0; k < 6; k++) {
            uint64_t v = 0;
            if (k < 2) {
                uint32_t v32 = 0;
                uc_reg_read(e->uc, ids[k], &v32);
                v = v32;
            } else
                uc_reg_read(e->uc, ids[k], &v);
            w_le32(&b, (uint32_t)ids[k]);
            w_le32(&b, 0);
            w_le64(&b, v);
        }
    }
    if (w_rec(&out, &nrec, "EMAC", &b))
        goto fail;
    /* CTLR, MMIO, CNTR */
    b.n = 0;
    for (i = 0; i < m->nctl; i++) {
        w_le32(&b, m->ctl[i].key);
        w_le32(&b, m->ctl[i].value);
    }
    if (w_rec(&out, &nrec, "CTLR", &b))
        goto fail;
    b.n = 0;
    for (i = 0; i < m->nmmio; i++) {
        w_le32(&b, m->mmio[i].addr);
        w_le32(&b, m->mmio[i].value);
    }
    if (w_rec(&out, &nrec, "MMIO", &b))
        goto fail;
    b.n = 0;
    w_le64(&b, m->ff1_count + (uint64_t)m->isa_counts[0]);   /* Machine.ff1_count: Python + C */
    w_le64(&b, m->movec_count);
    if (w_rec(&out, &nrec, "CNTR", &b))
        goto fail;
    /* MAPS e PAGE (pagine non nulle, zlib 6, in ordine di indirizzo) */
    if (uc_mem_regions(e->uc, &rg, &nb) != UC_ERR_OK)
        goto fail;
    bases = (uint32_t *)malloc((nb ? nb : 1) * sizeof *bases);
    if (!bases)
        goto fail;
    for (i = 0; i < nb; i++)
        bases[i] = (uint32_t)rg[i].begin;
    qsort(bases, nb, sizeof *bases, cmp_u32);
    b.n = 0;
    for (i = 0; i < nb; i++)
        w_le32(&b, bases[i]);
    if (w_rec(&out, &nrec, "MAPS", &b))
        goto fail;
    for (i = 0; i < nb; i++) {
        /* grezza (la compressione e' in dt_state_finish), letta dritta nel buffer
           d'uscita; le pagine a zero si tolgono subito */
        const size_t at = out.n;
        const uint64_t *w;
        size_t j;
        int nz = 0;
        if (st_reserve(&out, 24 + 0x100000))
            goto fail;
        {
            const uint8_t *h = dtm_host(m, bases[i]);
            if (h)
                memcpy(out.p + at + 24, h, 0x100000);
            else if (uc_mem_read(e->uc, bases[i], out.p + at + 24, 0x100000) != UC_ERR_OK)
                goto fail;
        }
        w = (const uint64_t *)(const void *)(out.p + at + 24);
        for (j = 0; j < 0x100000 / 8; j += 4)
            if (w[j] | w[j + 1] | w[j + 2] | w[j + 3]) {
                nz = 1;
                break;
            }
        if (!nz)
            continue;
        if (e_buf_put(&out, "PAGR", 4) || w_le32(&out, 0) || w_le64(&out, 8 + 0x100000)
            || w_le32(&out, bases[i]) || w_le32(&out, 0x100000))
            goto fail;
        out.n += 0x100000;           /* gia' letta al suo posto */
        nrec++;
    }
    /* FWID: lo SHA-256 del MAIN OS, gia' calcolato da dt_machine_new */
    {
        b.n = 0;
        e_buf_put(&b, m->main_sha256, 32);
        if (w_rec(&out, &nrec, "FWID", &b))
            goto fail;
    }
    if (e->boot_only)
        goto sdh;
    /* CUIN */
    b.n = 0;
    for (i = 0; i < e->uart_in.n; i++)
        w_u8(&b, e->uart_in.p[(e->uart_in.head + i) % e->uart_in.cap]);
    if (w_rec(&out, &nrec, "CUIN", &b))
        goto fail;
    /* CETX */
    b.n = 0;
    w_le32(&b, 35);
    w_le32(&b, 155);
    w_le64(&b, (uint64_t)(int64_t)e->etx.pending);
    w_le64(&b, (uint64_t)e->etx.bytes);
    w_le64(&b, (uint64_t)e->etx.transfers);
    if (w_rec(&out, &nrec, "CETX", &b))
        goto fail;
    /* CSSI */
    {
        sst_state *s = &e->sst;
        int64_t num = 0, den = 0;
        if (s->has_next) {
            int64_t g = gcd64(s->nextN, s->rh);
            num = s->nextN / g;
            den = s->rh / g;
        }
        b.n = 0;
        w_le64(&b, (uint64_t)s->rh);
        w_le64(&b, (uint64_t)s->ips);
        w_le64(&b, (uint64_t)s->now);
        w_le64(&b, (uint64_t)num);
        w_le64(&b, (uint64_t)den);
        w_le32(&b, (uint32_t)s->en);
        w_u8(&b, (uint8_t)!!s->int50_asserted);
        w_u8(&b, (uint8_t)!!s->int50_delivered);
        w_u8(&b, (uint8_t)!!s->force_asserted);
        w_u8(&b, (uint8_t)!!s->force_delivered);
        w_le64(&b, (uint64_t)s->requests);
        w_le64(&b, (uint64_t)s->major_rx);
        w_le64(&b, (uint64_t)s->major_tx);
        w_le64(&b, (uint64_t)s->sg_rx);
        w_le64(&b, (uint64_t)s->sg_tx);
        w_le64(&b, (uint64_t)s->tx_bytes);
        w_le32(&b, e->ssi_tx_crc);
        w_le64(&b, (uint64_t)s->vector170);
        w_le64(&b, (uint64_t)s->vector191);
        if (w_rec(&out, &nrec, "CSSI", &b))
            goto fail;
    }
    /* CSDH: overlay in ordine di settore. Qui grezzo ("CSDR", nell'ordine della
       tabella, scritto dritto nel buffer): l'ordinamento e' in dt_state_finish */
sdh:
    {
        e_esdhc *sd = &e->sd;
        size_t n = 0, j, hdr;
        uint64_t blen;
        for (j = 0; j < sd->ov.cap; j++)
            if (sd->ov.tab[j].used)
                n++;
        if (st_reserve(&out, 64 + n * (8 + 1 + 1024)))
            goto fail;
        hdr = out.n;
        if (e_buf_put(&out, "CSDR", 4) || w_le32(&out, 0) || w_le64(&out, 0))
            goto fail;
        w_le32(&out, sd->pattern);
        w_le32(&out, (uint32_t)sd->armed);
        w_le64(&out, (uint64_t)sd->dma_bytes);
        w_le64(&out, (uint64_t)sd->blocks);
        w_le32(&out, sd->rca);
        w_u8(&out, (uint8_t)!!sd->selected);
        w_le32(&out, (uint32_t)n);
        for (j = 0; j < sd->ov.cap; j++) {
            const e_sector *v = &sd->ov.tab[j];
            if (!v->used)
                continue;
            w_le64(&out, v->sec);
            w_u8(&out, v->mask ? 1 : 0);
            e_buf_put(&out, v->data, 512);
            if (v->mask)
                e_buf_put(&out, v->mask, 512);
        }
        blen = out.n - hdr - 16;
        for (j = 0; j < 8; j++)
            out.p[hdr + 8 + j] = (uint8_t)(blen >> (8 * j));
        nrec++;
    }
    if (e->boot_only)
        goto mani;
    /* CTIM */
    b.n = 0;
    w_le32(&b, (uint32_t)e->ntim);
    for (k = 0; k < e->ntim; k++) {
        e_tsrc *s = &e->tim[k];
        int c;
        w_le32(&b, (uint32_t)s->kind);
        w_le32(&b, (uint32_t)s->t.nch);
        for (c = 0; c < 4; c++)
            w_le32(&b, c < s->t.nch ? (uint32_t)s->t.ch[c] : 0);
        w_le64(&b, (uint64_t)s->ips);
        w_le64(&b, (uint64_t)s->now);
        w_u8(&b, (uint8_t)!!s->t.held);
        for (c = 0; c < 4; c++) {
            w_u8(&b, s->t.has[c] ? 1 : 0);
            w_f64(&b, s->t.has[c] ? s->t.next[c] : 0.0);
        }
        for (c = 0; c < 4; c++)
            w_le64(&b, (uint64_t)s->t.fired[c]);
        for (c = 0; c < 4; c++)
            w_le64(&b, (uint64_t)s->t.missed[c]);
        w_le32(&b, s->kind ? e->dtim_arm : 0);
        w_le32(&b, s->stale);
    }
    if (w_rec(&out, &nrec, "CTIM", &b))
        goto fail;
    /* CNOR: i blocchi scritti della flash SPI (Nor.blocks) */
    b.n = 0;
    {
        uint32_t nn = 0;
        for (i = 0; i < NOR_MAXBLK; i++)
            nn += e->nor.blk[i] != NULL;
        w_le32(&b, nn);
        for (i = 0; i < NOR_MAXBLK; i++)
            if (e->nor.blk[i]) {
                w_le32(&b, i * NOR_BLOCK);
                e_buf_put(&b, e->nor.blk[i], NOR_BLOCK);
            }
    }
    if (w_rec(&out, &nrec, "CNOR", &b))
        goto fail;
    /* CLED: RytmLedState.dump (registri, 0xFFFF = None; pad) */
    b.n = 0;
    {
        int r, l;
        for (r = 0; r < 4; r++)
            for (l = 0; l < 128; l++) {
                uint8_t x[2] = {(uint8_t)(e->leds.regs[r][l] & 0xFF), (uint8_t)((e->leds.regs[r][l] >> 8) & 0xFF)};
                e_buf_put(&b, x, 2);
            }
        for (l = 0; l < 12; l++)
            e_buf_put(&b, e->leds.pads[l], 3);
    }
    if (w_rec(&out, &nrec, "CLED", &b))
        goto fail;
    /* MANI (informativo) */
mani:
    {
        char mani[256];
        int n = snprintf(mani, sizeof mani, "{\"writer\": \"arcore %s\", \"instrs\": %lld}", ARCORE_VERSION,
                         (long long)e->instrs);
        b.n = 0;
        e_buf_put(&b, mani, (size_t)n);
        if (w_rec(&out, &nrec, "MANI", &b))
            goto fail;
    }
    memcpy(out.p + 8, &nrec, 4);
    *out_n = out.n;
    rc = DT_OK;
fail:
    *out_p = out.p;              /* il buffer resta del chiamante, anche in errore */
    *cap_io = out.cap;
    out.p = NULL;
    if (rg)
        uc_free(rg);
    free(bases);
    free(b.p);
    free(out.p);
    return rc;
}

static uint32_t rd_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

typedef struct {
    uint64_t key;
    size_t off, len;
} fin_ent;

static int cmp_ent(const void *a, const void *b)
{
    const fin_ent *x = (const fin_ent *)a, *y = (const fin_ent *)b;
    return x->key < y->key ? -1 : x->key > y->key;
}

/* CSDR -> CSDH: i settori in ordine (come cmp_sec) */
static int fin_csdh(e_buf *out, uint32_t *nrec, const uint8_t *p, size_t len)
{
    e_buf b = {0};
    fin_ent *v = NULL;
    uint32_t n, j;
    size_t at = 33;
    int rc = -1;
    if (len < 33)
        return -1;
    n = rd_le32(p + 29);
    v = (fin_ent *)malloc((n ? n : 1) * sizeof *v);
    if (!v)
        return -1;
    for (j = 0; j < n; j++) {
        size_t sz;
        if (at + 9 > len)
            goto done;
        sz = 9 + 512 + (p[at + 8] ? 512 : 0);
        if (at + sz > len)
            goto done;
        v[j].key = (uint64_t)rd_le32(p + at) | (uint64_t)rd_le32(p + at + 4) << 32;
        v[j].off = at;
        v[j].len = sz;
        at += sz;
    }
    qsort(v, n, sizeof *v, cmp_ent);
    if (st_reserve(&b, len) || e_buf_put(&b, p, 33))
        goto done;
    for (j = 0; j < n; j++)
        if (e_buf_put(&b, p + v[j].off, v[j].len))
            goto done;
    rc = w_rec(out, nrec, "CSDH", &b);
done:
    free(v);
    free(b.p);
    return rc;
}

int dt_state_finish(const uint8_t *raw, size_t n, int compress, uint8_t **out_p, size_t *out_n)
{
    e_buf out = {0}, b = {0};
    uint8_t *z = NULL;
    uint32_t nrec = 0, k;
    size_t at = 16;
    int rc = DT_ERR_NOMEM;
    *out_p = NULL;
    *out_n = 0;
    if (n < 16 || memcmp(raw, "DTST", 4))
        return DT_ERR_FORMAT;
    if (e_buf_put(&out, raw, 16))
        goto fail;
    z = (uint8_t *)malloc(compressBound(0x100000));
    if (!z)
        goto fail;
    for (k = rd_le32(raw + 8); k > 0; k--) {
        uint64_t len;
        if (at + 16 > n)
            goto bad;
        len = (uint64_t)rd_le32(raw + at + 8) | (uint64_t)rd_le32(raw + at + 12) << 32;
        if (len > n - at - 16)
            goto bad;
        if (!memcmp(raw + at, "PAGR", 4)) {
            const uint8_t *pg = raw + at + 24;
            uLongf zn = compressBound(0x100000);
            size_t j;
            int nz = 0;
            if (len != 8 + 0x100000)
                goto bad;
            for (j = 0; j < 0x100000; j++)
                if (pg[j]) {
                    nz = 1;
                    break;
                }
            if (nz) {
                if (compress2(z, &zn, pg, 0x100000, 6) != Z_OK)
                    goto fail;
                b.n = 0;
                if (e_buf_put(&b, raw + at + 16, 8) || e_buf_put(&b, z, zn) || w_rec(&out, &nrec, "PAGE", &b))
                    goto fail;
            }
        } else if (!memcmp(raw + at, "CSDR", 4)) {
            if (fin_csdh(&out, &nrec, raw + at + 16, (size_t)len))
                goto bad;
        } else {
            if (e_buf_put(&out, raw + at, 16 + (size_t)len))
                goto fail;
            nrec++;
        }
        at += 16 + (size_t)len;
    }
    memcpy(out.p + 8, &nrec, 4);
    if (compress) {
        uLongf zn = compressBound((uLong)out.n);
        uint8_t *zz = (uint8_t *)malloc(12 + zn);
        uint64_t rawn = out.n;
        if (!zz || compress2(zz + 12, &zn, out.p, (uLong)out.n, 1) != Z_OK) {
            free(zz);
            goto fail;
        }
        memcpy(zz, "DTSZ", 4);
        for (k = 0; k < 8; k++)
            zz[4 + k] = (uint8_t)(rawn >> (8 * k));
        free(out.p);
        out.p = zz;
        out.n = 12 + zn;
    }
    *out_p = out.p;
    *out_n = out.n;
    out.p = NULL;
    rc = DT_OK;
    goto fail;
bad:
    rc = DT_ERR_FORMAT;
fail:
    free(z);
    free(b.p);
    free(out.p);
    return rc;
}

int dt_engine_save_mem(dt_engine *e, int compress, uint8_t **out_p, size_t *out_n)
{
    uint8_t *raw = NULL;
    size_t n = 0, cap = 0;
    int r;
    *out_p = NULL;
    *out_n = 0;
    r = dt_engine_save_raw(e, &raw, &n, &cap);
    if (r == DT_OK)
        r = dt_state_finish(raw, n, compress, out_p, out_n);
    free(raw);
    return r;
}

int dt_engine_save(dt_engine *e, const char *path, int compress)
{
    uint8_t *buf;
    size_t n;
    FILE *f;
    int r = dt_engine_save_mem(e, compress, &buf, &n);
    char tmp[4096];
    if (r)
        return r;
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    f = fopen(tmp, "wb");
    if (!f || fwrite(buf, 1, n, f) != n) {
        if (f)
            fclose(f);
        free(buf);
        return DT_ERR_IO;
    }
    fclose(f);
    free(buf);
    return rename(tmp, path) ? DT_ERR_IO : DT_OK;
}
